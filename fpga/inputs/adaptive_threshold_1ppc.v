`timescale 1ns/1ps

// Rectangular 15x15 Bernsen threshold. Low-contrast windows are assigned as a
// whole using the frame-committed global threshold; they do not turn every
// uniform dark pixel white as local-mean-minus-C does. Contrast 32 rejects the
// measured paper/sensor texture which fragmented objects at contrast 15.
// A centre 3x3 vote suppresses weak flat-region specks only when that 3x3
// grayscale range is below 24. Strong thin strokes retain the centre test.
// Two registered stages match the gallery's statistics stages. Spatially the
// centre is (x-7,y-7); the first fourteen rows/columns are masked while filling.
module adaptive_threshold_1ppc #(
    parameter integer IMG_WIDTH=1024,
    parameter integer X_BITS=11,
    parameter integer Y_BITS=11,
    parameter [7:0] CONTRAST_THRESHOLD=8'd32,
    parameter integer ENABLE_FLAT_VOTE=1,
    parameter integer ENABLE_LOW_CONTRAST_MEAN=0,
    parameter integer WINDOW_RADIUS=7,
    parameter [7:0] FLAT_RANGE_THRESHOLD=8'd24
)(
    input wire i_clk, i_rst_n, i_frame_start, i_de,
    input wire [7:0] i_y,
    input wire [9:0] i_global_threshold,
    output wire [23:0] o_rgb,
    output wire [30:0] o_decision_info
);
    localparam integer ADDR_BITS=(IMG_WIDTH>1)?$clog2(IMG_WIDTH):1;
    localparam integer WIN=2*WINDOW_RADIUS+1;
    reg [X_BITS-1:0] x_pos;
    reg [Y_BITS-1:0] y_pos;
    reg de_d;
    wire [X_BITS-1:0] read_column=(!i_rst_n || i_frame_start || (!i_de && de_d)) ?
        {X_BITS{1'b0}} : i_de ? x_pos+1'b1 : x_pos;
    wire [7:0] row_tap [0:WIN-1];
    assign row_tap[0]=i_y;
    genvar line;
    generate for(line=0;line<WIN-1;line=line+1) begin: g_line
        (* ram_style="bram" *) reg [7:0] memory [0:IMG_WIDTH-1];
        reg [7:0] read_q;
        assign row_tap[line+1]=read_q;
        // No reset on RAM data outputs. Frame/window validity hides old data;
        // each active row overwrites it before that row can become valid.
        always @(posedge i_clk) begin
            read_q<=memory[read_column[ADDR_BITS-1:0]];
            if(i_rst_n && !i_frame_start && i_de && x_pos<IMG_WIDTH)
                memory[x_pos[ADDR_BITS-1:0]]<=row_tap[line];
        end
    end endgenerate

    // Balanced reductions: four comparator levels for fifteen values.
    function [7:0] reduce_min;
        input [119:0] taps;
        reg [7:0] tree [0:30]; integer n;
        begin
            for(n=0;n<15;n=n+1) tree[15+n]=n<WIN?taps[n*8+:8]:8'd255;
            tree[30]=255;
            for(n=14;n>=0;n=n-1)
                tree[n]=tree[n*2+1]<tree[n*2+2]?tree[n*2+1]:tree[n*2+2];
            reduce_min=tree[0];
        end
    endfunction
    function [7:0] reduce_max;
        input [119:0] taps;
        reg [7:0] tree [0:30]; integer n;
        begin
            for(n=0;n<15;n=n+1) tree[15+n]=n<WIN?taps[n*8+:8]:8'd0;
            tree[30]=0;
            for(n=14;n>=0;n=n-1)
                tree[n]=tree[n*2+1]>tree[n*2+2]?tree[n*2+1]:tree[n*2+2];
            reduce_max=tree[0];
        end
    endfunction
    wire [119:0] vertical_taps;
    genvar tap;
    generate for(tap=0;tap<15;tap=tap+1) begin: g_vertical_tap
        if(tap<WIN)assign vertical_taps[tap*8+:8]=row_tap[tap];
        else assign vertical_taps[tap*8+:8]=8'd0;
    end endgenerate
    reg [7:0] column_min,column_max,column_center;
    reg column_de,column_first,column_valid;
    reg [7:0] threshold_stage1;
    reg [7:0] min_left [0:WIN-2],max_left [0:WIN-2];
    reg [7:0] center_left [0:WINDOW_RADIUS-1];
    // Reuse the fourteen existing grey line buffers. Each column holds the
    // three rows around the eventual centre; no extra frame/line storage.
    reg [23:0] center_rows_column;
    reg [23:0] center_rows_left [0:WINDOW_RADIUS];
    wire [119:0] horizontal_min_taps,horizontal_max_taps;
    assign horizontal_min_taps[7:0]=column_min;
    assign horizontal_max_taps[7:0]=column_max;
    generate for(tap=0;tap<14;tap=tap+1) begin: g_horizontal_tap
        if(tap<WIN-1)begin
            assign horizontal_min_taps[(tap+1)*8+:8]=min_left[tap];
            assign horizontal_max_taps[(tap+1)*8+:8]=max_left[tap];
        end else begin
            assign horizontal_min_taps[(tap+1)*8+:8]=8'd255;
            assign horizontal_max_taps[(tap+1)*8+:8]=8'd0;
        end
    end endgenerate
    wire [7:0] window_min=reduce_min(horizontal_min_taps);
    wire [7:0] window_max=reduce_max(horizontal_max_taps);
    reg [8:0] mid_sum_stage2;
    reg [7:0] contrast_stage2,center_stage2,threshold_stage2;
    reg valid_stage2;
    reg [71:0] center_window_stage2;
    wire [71:0] center_window={center_rows_left[WINDOW_RADIUS],center_rows_left[WINDOW_RADIUS-1],center_rows_left[WINDOW_RADIUS-2]};
    // A lone min/max sample affects a 15x15 footprint. In the experimental
    // low-contrast branch, use the actual centre 3x3 mean instead; it shares
    // existing taps and preserves the spatial centre and registered latency.
    wire [9:0] centre_sum0={2'd0,center_window[7:0]}+{2'd0,center_window[15:8]}+{2'd0,center_window[23:16]};
    wire [9:0] centre_sum1={2'd0,center_window[31:24]}+{2'd0,center_window[39:32]}+{2'd0,center_window[47:40]};
    wire [9:0] centre_sum2={2'd0,center_window[55:48]}+{2'd0,center_window[63:56]}+{2'd0,center_window[71:64]};
    reg [11:0] centre_sum_stage2;
    reg [7:0] flat_range_stage2;
    function [7:0] min9;
        input [71:0] values;
        reg [119:0] padded;
        begin padded={48'hffffffffffff,values};min9=reduce_min(padded);end
    endfunction
    function [7:0] max9;
        input [71:0] values;
        reg [119:0] padded;
        begin padded={48'd0,values};max9=reduce_max(padded);end
    endfunction
    wire [8:0] centre_votes;
    genvar vote;
    generate for(vote=0;vote<9;vote=vote+1) begin: g_vote
        assign centre_votes[vote]={center_window_stage2[vote*8+:8],1'b0}>=mid_sum_stage2;
    end endgenerate
    wire [1:0] votes0={1'b0,centre_votes[0]}+{1'b0,centre_votes[1]}+{1'b0,centre_votes[2]};
    wire [1:0] votes1={1'b0,centre_votes[3]}+{1'b0,centre_votes[4]}+{1'b0,centre_votes[5]};
    wire [1:0] votes2={1'b0,centre_votes[6]}+{1'b0,centre_votes[7]}+{1'b0,centre_votes[8]};
    wire [3:0] votes_sum={2'd0,votes0}+{2'd0,votes1}+{2'd0,votes2};
    wire centre_white=(ENABLE_FLAT_VOTE && flat_range_stage2<FLAT_RANGE_THRESHOLD) ?
        votes_sum>=5 : {center_stage2,1'b0}>=mid_sum_stage2;
    wire [11:0] nine_threshold={1'b0,threshold_stage2,3'b0}+{4'd0,threshold_stage2};
    wire white=(contrast_stage2<CONTRAST_THRESHOLD) ?
        (ENABLE_LOW_CONTRAST_MEAN ? centre_sum_stage2>=nine_threshold :
         mid_sum_stage2>={threshold_stage2,1'b0}) :
        centre_white;
    assign o_rgb=valid_stage2?{24{white}}:24'd0;
    // Signed score in 1/18-luma units. Avoid division or a rounded midpoint:
    // flat windows use a centre mean; strong fine detail uses its centre tap.
    wire [13:0] local_mean_target=({5'd0,mid_sum_stage2}<<3)+{5'd0,mid_sum_stage2};
    wire [7:0] local_cut=contrast_stage2<CONTRAST_THRESHOLD?threshold_stage2:mid_sum_stage2[8:1];
    reg signed [13:0] decision_score,centre_difference;
    always @* begin
        centre_difference=$signed({5'd0,center_stage2,1'b0})-$signed({5'd0,mid_sum_stage2});
        if(contrast_stage2<CONTRAST_THRESHOLD)
            decision_score=$signed({1'b0,centre_sum_stage2,1'b0})-$signed({1'b0,nine_threshold,1'b0});
        else if(flat_range_stage2<FLAT_RANGE_THRESHOLD)
            decision_score=$signed({1'b0,centre_sum_stage2,1'b0})-$signed(local_mean_target);
        else decision_score=(centre_difference<<<3)+centre_difference;
    end
    assign o_decision_info={valid_stage2,center_stage2,local_cut,decision_score};
    integer n;
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            x_pos<=0;y_pos<=0;de_d<=0;
            column_min<=0;column_max<=0;column_center<=0;
            column_de<=0;column_first<=0;column_valid<=0;threshold_stage1<=128;
            mid_sum_stage2<=0;contrast_stage2<=0;center_stage2<=0;
            threshold_stage2<=128;valid_stage2<=0;
            center_rows_column<=0;center_window_stage2<=0;flat_range_stage2<=0;centre_sum_stage2<=0;
            for(n=0;n<WIN-1;n=n+1) begin min_left[n]<=0;max_left[n]<=0;end
            for(n=0;n<WINDOW_RADIUS;n=n+1) center_left[n]<=0;
            for(n=0;n<WINDOW_RADIUS+1;n=n+1) center_rows_left[n]<=0;
        end else begin
            de_d<=i_de;
            column_de<=i_de && !i_frame_start;
            column_first<=x_pos==0;
            column_valid<=i_de && !i_frame_start && x_pos>=14 && y_pos>=14 && x_pos<IMG_WIDTH;
            column_min<=reduce_min(vertical_taps);column_max<=reduce_max(vertical_taps);
            column_center<=row_tap[WINDOW_RADIUS];
            center_rows_column<={row_tap[WINDOW_RADIUS+1],row_tap[WINDOW_RADIUS],row_tap[WINDOW_RADIUS-1]};
            threshold_stage1<=i_global_threshold>255?8'd255:i_global_threshold[7:0];
            mid_sum_stage2<={1'b0,window_min}+{1'b0,window_max};
            contrast_stage2<=window_max-window_min;
            center_stage2<=center_left[WINDOW_RADIUS-1];threshold_stage2<=threshold_stage1;
            valid_stage2<=column_valid;
            center_window_stage2<=center_window;
            centre_sum_stage2<={2'd0,centre_sum0}+{2'd0,centre_sum1}+{2'd0,centre_sum2};
            flat_range_stage2<=max9(center_window)-min9(center_window);
            if(column_de) begin
                min_left[0]<=column_min;max_left[0]<=column_max;center_left[0]<=column_center;
                for(n=1;n<WIN-1;n=n+1) begin
                    min_left[n]<=column_first?8'd0:min_left[n-1];
                    max_left[n]<=column_first?8'd0:max_left[n-1];
                end
                for(n=1;n<WINDOW_RADIUS;n=n+1) center_left[n]<=column_first?8'd0:center_left[n-1];
                center_rows_left[0]<=center_rows_column;
                for(n=1;n<WINDOW_RADIUS+1;n=n+1) center_rows_left[n]<=column_first?24'd0:center_rows_left[n-1];
            end
            if(i_frame_start) begin x_pos<=0;y_pos<=0;end
            else if(i_de) x_pos<=x_pos+1'b1;
            else if(de_d) begin x_pos<=0;y_pos<=y_pos+1'b1;end
        end
    end
endmodule
