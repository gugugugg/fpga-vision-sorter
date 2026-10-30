`timescale 1ns/1ps

// Lightweight board-only target detection/recognition.
// It segments red/green/blue targets, emits a frame-stable bbox/centroid, and
// classifies the dominant colour union as elongated/rectangle/round/irregular.
// This is deliberately not a connected-component detector: separated regions
// of the same colour are accumulated into one whole-frame union.
module color_shape_detector_4ppc #(
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600,
    parameter MIN_AREA   = 256
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_sof,
    input  wire        i_valid,
    input  wire [95:0] i_rgb,
    output reg         o_target_update,
    output reg         o_target_valid,
    output reg  [3:0]  o_target_class,
    output reg  [10:0] o_center_x,
    output reg  [9:0]  o_center_y,
    output reg  [10:0] o_bbox_x0,
    output reg  [10:0] o_bbox_x1,
    output reg  [9:0]  o_bbox_y0,
    output reg  [9:0]  o_bbox_y1,
    output reg  [19:0] o_area,
    output reg  [7:0]  o_quality
);
    localparam GROUPS_PER_LINE = IMG_WIDTH/4;

    function [1:0] pixel_color;
        input [23:0] p;
        reg [8:0] r;
        reg [8:0] g;
        reg [8:0] b;
        begin
            r = {1'b0,p[23:16]};
            g = {1'b0,p[15:8]};
            b = {1'b0,p[7:0]};
            if ((r > 140) && (r > g+40) && (r > b+40))
                pixel_color = 2'd1;
            else if ((g > 120) && (g > r+32) && (g > b+32))
                pixel_color = 2'd2;
            else if ((b > 120) && (b > r+32) && (b > g+32))
                pixel_color = 2'd3;
            else
                pixel_color = 2'd0;
        end
    endfunction

    function [19:0] pop4;
        input [3:0] bits;
        begin
            pop4 = {19'd0,bits[0]}+{19'd0,bits[1]}
                 + {19'd0,bits[2]}+{19'd0,bits[3]};
        end
    endfunction

    // Coordinate sums for the four pixels represented by one 4PPC mask.
    // Mask bit 3 corresponds to the left-most pixel (x_base + 0).
    function [31:0] sum_x4;
        input [3:0]  bits;
        input [10:0] base;
        begin
            sum_x4 = (bits[3] ? {21'd0,base}             : 32'd0)
                   + (bits[2] ? ({21'd0,base} + 32'd1)  : 32'd0)
                   + (bits[1] ? ({21'd0,base} + 32'd2)  : 32'd0)
                   + (bits[0] ? ({21'd0,base} + 32'd3)  : 32'd0);
        end
    endfunction

    function [31:0] sum_y4;
        input [3:0] bits;
        input [9:0] y;
        begin
            sum_y4 = (bits[3] ? {22'd0,y} : 32'd0)
                   + (bits[2] ? {22'd0,y} : 32'd0)
                   + (bits[1] ? {22'd0,y} : 32'd0)
                   + (bits[0] ? {22'd0,y} : 32'd0);
        end
    endfunction

    function [10:0] first4;
        input [3:0] bits;
        begin
            if (bits[3]) first4=11'd0;
            else if (bits[2]) first4=11'd1;
            else if (bits[1]) first4=11'd2;
            else first4=11'd3;
        end
    endfunction

    function [10:0] last4;
        input [3:0] bits;
        begin
            if (bits[0]) last4=11'd3;
            else if (bits[1]) last4=11'd2;
            else if (bits[2]) last4=11'd1;
            else last4=11'd0;
        end
    endfunction

    wire [1:0] c0 = pixel_color(i_rgb[95:72]);
    wire [1:0] c1 = pixel_color(i_rgb[71:48]);
    wire [1:0] c2 = pixel_color(i_rgb[47:24]);
    wire [1:0] c3 = pixel_color(i_rgb[23:0]);
    wire [3:0] red_mask   = {c0==1,c1==1,c2==1,c3==1};
    wire [3:0] green_mask = {c0==2,c1==2,c2==2,c3==2};
    wire [3:0] blue_mask  = {c0==3,c1==3,c2==3,c3==3};

    reg [10:0] x_group;
    reg [9:0]  y_pos;
    wire [10:0] x_base = x_group << 2;

    reg [19:0] area_r, area_g, area_b;
    // At 1024x600, max(sum_x) < 2^30 and max(sum_y) < 2^29.
    // Keep 32 bits so the centroid divisions cannot overflow.
    reg [31:0] sum_x_r, sum_x_g, sum_x_b;
    reg [31:0] sum_y_r, sum_y_g, sum_y_b;
    reg [10:0] xmin_r, xmin_g, xmin_b;
    reg [10:0] xmax_r, xmax_g, xmax_b;
    reg [9:0] ymin_r, ymin_g, ymin_b;
    reg [9:0] ymax_r, ymax_g, ymax_b;
    reg frame_seen;

    reg [1:0] dom_color;
    reg [19:0] dom_area;
    reg [10:0] dom_xmin, dom_xmax;
    reg [9:0] dom_ymin, dom_ymax;
    reg [31:0] dom_sum_x, dom_sum_y;

    // Frame-end work is deliberately split into three short cycles:
    // dominant-colour snapshot, bbox geometry, then occupancy classification.
    // This keeps the 20/32-bit comparisons and bbox multiply away from the
    // 112-MHz pixel accumulator path.
    reg [1:0] snap_color;
    reg [19:0] snap_area;
    reg [10:0] snap_xmin, snap_xmax;
    reg [9:0] snap_ymin, snap_ymax;
    reg [31:0] snap_sum_x, snap_sum_y;
    reg [11:0] pending_box_w, pending_box_h;
    wire [11:0] snap_box_w = snap_xmax - snap_xmin + 1'b1;
    wire [11:0] snap_box_h = {2'd0,snap_ymax} - {2'd0,snap_ymin} + 1'b1;
    wire [23:0] snap_box_area = snap_box_w * snap_box_h;

    // Frame-result calculation reuses one small sequential divider.  The
    // three divisions finish in about 120 clocks, far inside one video frame.
    localparam CALC_IDLE   = 4'd0;
    localparam CALC_GEOM   = 4'd1;
    localparam CALC_CLASS  = 4'd2;
    localparam CALC_X_REQ  = 4'd3;
    localparam CALC_X_WAIT = 4'd4;
    localparam CALC_Y_REQ  = 4'd5;
    localparam CALC_Y_WAIT = 4'd6;
    localparam CALC_Q_REQ  = 4'd7;
    localparam CALC_Q_WAIT = 4'd8;
    reg [3:0] calc_state;
    reg [3:0]  pending_class;
    reg [10:0] pending_bbox_x0, pending_bbox_x1;
    reg [9:0]  pending_bbox_y0, pending_bbox_y1;
    reg [19:0] pending_area;
    reg [31:0] pending_sum_x, pending_sum_y;
    reg [31:0] pending_quality_num;
    reg [23:0] pending_box_area;
    reg [10:0] pending_center_x;
    reg [9:0]  pending_center_y;

    wire [31:0] pending_area_w = {12'd0,pending_area};
    wire [31:0] pending_box_area_w = {8'd0,pending_box_area};
    wire [31:0] calc_quality_num = (pending_area_w << 8) - pending_area_w
                                 + (pending_box_area_w >> 1);
    wire [31:0] pending_area_x100 = (pending_area_w << 6) + (pending_area_w << 5)
                                   + (pending_area_w << 2);
    wire [31:0] pending_box_area_x90 = (pending_box_area_w << 6)
                                     + (pending_box_area_w << 4)
                                     + (pending_box_area_w << 3)
                                     + (pending_box_area_w << 1);
    wire [31:0] pending_box_area_x60 = (pending_box_area_w << 5)
                                     + (pending_box_area_w << 4)
                                     + (pending_box_area_w << 3)
                                     + (pending_box_area_w << 2);
    wire [1:0] pending_shape_class =
        ((pending_box_w > (pending_box_h << 1)) ||
         (pending_box_h > (pending_box_w << 1))) ? 2'd0 :
        (pending_area_x100 >= pending_box_area_x90) ? 2'd1 :
        (pending_area_x100 >= pending_box_area_x60) ? 2'd2 : 2'd3;

    wire div_start = (calc_state == CALC_X_REQ)
                   || (calc_state == CALC_Y_REQ)
                   || (calc_state == CALC_Q_REQ);
    wire [39:0] div_numerator = (calc_state == CALC_X_REQ)
                              ? {8'd0,pending_sum_x}
                              : (calc_state == CALC_Y_REQ)
                              ? {8'd0,pending_sum_y}
                              : {8'd0,pending_quality_num};
    wire [31:0] div_denominator = (calc_state == CALC_Q_REQ)
                                ? {8'd0,pending_box_area}
                                : {12'd0,pending_area};
    wire [39:0] div_quotient;
    wire div_busy;
    wire div_done;

    seq_div_u40_u32 u_result_divider (
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_start(div_start),
        .i_numerator(div_numerator),
        .i_denominator(div_denominator),
        .o_quotient(div_quotient),
        .o_busy(div_busy),
        .o_done(div_done)
    );

    always @(*) begin
        if ((area_r >= area_g) && (area_r >= area_b)) begin
            dom_color=1; dom_area=area_r; dom_xmin=xmin_r; dom_xmax=xmax_r;
            dom_ymin=ymin_r; dom_ymax=ymax_r;
            dom_sum_x=sum_x_r; dom_sum_y=sum_y_r;
        end else if (area_g >= area_b) begin
            dom_color=2; dom_area=area_g; dom_xmin=xmin_g; dom_xmax=xmax_g;
            dom_ymin=ymin_g; dom_ymax=ymax_g;
            dom_sum_x=sum_x_g; dom_sum_y=sum_y_g;
        end else begin
            dom_color=3; dom_area=area_b; dom_xmin=xmin_b; dom_xmax=xmax_b;
            dom_ymin=ymin_b; dom_ymax=ymax_b;
            dom_sum_x=sum_x_b; dom_sum_y=sum_y_b;
        end
    end

    task update_red;
        input [3:0] mask;
        begin
            if (mask != 0) begin
                area_r <= area_r + pop4(mask);
                sum_x_r <= sum_x_r + sum_x4(mask,x_base);
                sum_y_r <= sum_y_r + sum_y4(mask,y_pos);
                if (area_r == 0) begin
                    xmin_r <= x_base + first4(mask); xmax_r <= x_base + last4(mask);
                    ymin_r <= y_pos; ymax_r <= y_pos;
                end else begin
                    if (x_base + first4(mask) < xmin_r) xmin_r <= x_base + first4(mask);
                    if (x_base + last4(mask)  > xmax_r) xmax_r <= x_base + last4(mask);
                    if (y_pos < ymin_r) ymin_r <= y_pos;
                    if (y_pos > ymax_r) ymax_r <= y_pos;
                end
            end
        end
    endtask

    task update_green;
        input [3:0] mask;
        begin
            if (mask != 0) begin
                area_g <= area_g + pop4(mask);
                sum_x_g <= sum_x_g + sum_x4(mask,x_base);
                sum_y_g <= sum_y_g + sum_y4(mask,y_pos);
                if (area_g == 0) begin
                    xmin_g <= x_base + first4(mask); xmax_g <= x_base + last4(mask);
                    ymin_g <= y_pos; ymax_g <= y_pos;
                end else begin
                    if (x_base + first4(mask) < xmin_g) xmin_g <= x_base + first4(mask);
                    if (x_base + last4(mask)  > xmax_g) xmax_g <= x_base + last4(mask);
                    if (y_pos < ymin_g) ymin_g <= y_pos;
                    if (y_pos > ymax_g) ymax_g <= y_pos;
                end
            end
        end
    endtask

    task update_blue;
        input [3:0] mask;
        begin
            if (mask != 0) begin
                area_b <= area_b + pop4(mask);
                sum_x_b <= sum_x_b + sum_x4(mask,x_base);
                sum_y_b <= sum_y_b + sum_y4(mask,y_pos);
                if (area_b == 0) begin
                    xmin_b <= x_base + first4(mask); xmax_b <= x_base + last4(mask);
                    ymin_b <= y_pos; ymax_b <= y_pos;
                end else begin
                    if (x_base + first4(mask) < xmin_b) xmin_b <= x_base + first4(mask);
                    if (x_base + last4(mask)  > xmax_b) xmax_b <= x_base + last4(mask);
                    if (y_pos < ymin_b) ymin_b <= y_pos;
                    if (y_pos > ymax_b) ymax_b <= y_pos;
                end
            end
        end
    endtask

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            x_group<=0; y_pos<=0; frame_seen<=0;
            area_r<=0; area_g<=0; area_b<=0;
            sum_x_r<=0; sum_x_g<=0; sum_x_b<=0;
            sum_y_r<=0; sum_y_g<=0; sum_y_b<=0;
            xmin_r<=IMG_WIDTH-1; xmin_g<=IMG_WIDTH-1; xmin_b<=IMG_WIDTH-1;
            xmax_r<=0; xmax_g<=0; xmax_b<=0;
            ymin_r<=IMG_HEIGHT-1; ymin_g<=IMG_HEIGHT-1; ymin_b<=IMG_HEIGHT-1;
            ymax_r<=0; ymax_g<=0; ymax_b<=0;
            calc_state<=CALC_IDLE;
            snap_color<=0; snap_area<=0; snap_xmin<=0; snap_xmax<=0;
            snap_ymin<=0; snap_ymax<=0; snap_sum_x<=0; snap_sum_y<=0;
            pending_box_w<=0; pending_box_h<=0;
            pending_class<=0;
            pending_bbox_x0<=0; pending_bbox_x1<=0;
            pending_bbox_y0<=0; pending_bbox_y1<=0;
            pending_area<=0; pending_sum_x<=0; pending_sum_y<=0;
            pending_quality_num<=0; pending_box_area<=0;
            pending_center_x<=0; pending_center_y<=0;
            o_target_update<=0; o_target_valid<=0; o_target_class<=0;
            o_center_x<=0; o_center_y<=0;
            o_bbox_x0<=0; o_bbox_x1<=0; o_bbox_y0<=0; o_bbox_y1<=0;
            o_area<=0; o_quality<=0;
        end else begin
            o_target_update <= 1'b0;

            // Complete the frame result away from the 4PPC pixel datapath.
            // Outputs change atomically only when all three quotients exist.
            case (calc_state)
                CALC_GEOM: begin
                    pending_bbox_x0 <= snap_xmin;
                    pending_bbox_x1 <= snap_xmax;
                    pending_bbox_y0 <= snap_ymin;
                    pending_bbox_y1 <= snap_ymax;
                    pending_area <= snap_area;
                    pending_sum_x <= snap_sum_x;
                    pending_sum_y <= snap_sum_y;
                    pending_box_w <= snap_box_w;
                    pending_box_h <= snap_box_h;
                    pending_box_area <= snap_box_area;
                    calc_state <= CALC_CLASS;
                end
                CALC_CLASS: begin
                    pending_class <= {snap_color,pending_shape_class};
                    pending_quality_num <= calc_quality_num;
                    calc_state <= CALC_X_REQ;
                end
                CALC_X_REQ: calc_state <= CALC_X_WAIT;
                CALC_X_WAIT: if (div_done) begin
                    pending_center_x <= div_quotient[10:0];
                    calc_state <= CALC_Y_REQ;
                end
                CALC_Y_REQ: calc_state <= CALC_Y_WAIT;
                CALC_Y_WAIT: if (div_done) begin
                    pending_center_y <= div_quotient[9:0];
                    calc_state <= CALC_Q_REQ;
                end
                CALC_Q_REQ: calc_state <= CALC_Q_WAIT;
                CALC_Q_WAIT: if (div_done) begin
                    o_target_valid <= 1'b1;
                    o_target_class <= pending_class;
                    o_center_x <= pending_center_x;
                    o_center_y <= pending_center_y;
                    o_bbox_x0 <= pending_bbox_x0;
                    o_bbox_x1 <= pending_bbox_x1;
                    o_bbox_y0 <= pending_bbox_y0;
                    o_bbox_y1 <= pending_bbox_y1;
                    o_area <= pending_area;
                    o_quality <= (div_quotient > 40'd255)
                               ? 8'hff : div_quotient[7:0];
                    o_target_update <= 1'b1;
                    calc_state <= CALC_IDLE;
                end
                default: calc_state <= CALC_IDLE;
            endcase

            if (i_valid) begin
                if (i_sof) begin
                    if (frame_seen && dom_area >= MIN_AREA) begin
                        // Snapshot the completed frame before its accumulators
                        // are reused. Geometry/classification and publication
                        // continue in the background over subsequent cycles.
                        snap_color <= dom_color;
                        snap_area <= dom_area;
                        snap_xmin <= dom_xmin;
                        snap_xmax <= dom_xmax;
                        snap_ymin <= dom_ymin;
                        snap_ymax <= dom_ymax;
                        snap_sum_x <= dom_sum_x;
                        snap_sum_y <= dom_sum_y;
                        calc_state <= CALC_GEOM;
                    end else if (frame_seen) begin
                        // An empty/undersized frame is also a new atomic result.
                        o_target_valid <= 1'b0;
                        o_target_class <= 0;
                        o_center_x <= 0; o_center_y <= 0;
                        o_bbox_x0 <= 0; o_bbox_x1 <= 0;
                        o_bbox_y0 <= 0; o_bbox_y1 <= 0;
                        o_area <= 0;
                        o_quality <= 0;
                        o_target_update <= 1'b1;
                        calc_state <= CALC_IDLE;
                    end
                frame_seen<=1'b1;
                x_group<=1; y_pos<=0;
                area_r<=pop4(red_mask); area_g<=pop4(green_mask); area_b<=pop4(blue_mask);
                sum_x_r<=sum_x4(red_mask,11'd0);
                sum_x_g<=sum_x4(green_mask,11'd0);
                sum_x_b<=sum_x4(blue_mask,11'd0);
                sum_y_r<=0; sum_y_g<=0; sum_y_b<=0;
                xmin_r<=first4(red_mask); xmax_r<=last4(red_mask);
                xmin_g<=first4(green_mask); xmax_g<=last4(green_mask);
                xmin_b<=first4(blue_mask); xmax_b<=last4(blue_mask);
                ymin_r<=0; ymin_g<=0; ymin_b<=0;
                ymax_r<=0; ymax_g<=0; ymax_b<=0;
                end else begin
                    update_red(red_mask);
                    update_green(green_mask);
                    update_blue(blue_mask);
                    if (x_group == GROUPS_PER_LINE-1) begin
                        x_group<=0;
                        if (y_pos == IMG_HEIGHT-1) y_pos<=0; else y_pos<=y_pos+1'b1;
                    end else x_group<=x_group+1'b1;
                end
            end
        end
    end
endmodule
