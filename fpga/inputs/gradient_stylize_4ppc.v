`timescale 1ns/1ps

// True 1024-pixel, four-lane edge extraction. Unlike the official Lab 2,
// pixels are never averaged into 256 groups. Four banked line memories allow
// one previous-row sample per lane and infer block RAM without array reset.
module gradient_stylize_4ppc #(
    parameter IMG_WIDTH      = 1024,
    parameter IMG_HEIGHT     = 600,
    parameter EDGE_THRESHOLD = 10'd72,
    parameter BIN_THRESHOLD  = 8'd128
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_sof,
    input  wire        i_valid,
    input  wire [95:0] i_rgb,
    input  wire [9:0]  i_edge_threshold,
    output wire        o_sof,
    output wire        o_valid,
    output wire [95:0] o_raw,
    output wire [95:0] o_gray,
    output wire [95:0] o_binary,
    output wire [95:0] o_edge,
    output wire [95:0] o_ukiyo
);
    localparam GROUPS_PER_LINE = IMG_WIDTH / 4;
    localparam GROUP_ADDR_W = (GROUPS_PER_LINE <= 2) ? 1 : $clog2(GROUPS_PER_LINE);
    localparam [GROUP_ADDR_W-1:0] LAST_GROUP = GROUPS_PER_LINE-1;

    wire [31:0] gray_now;
    rgb4_to_gray4 u_gray(.i_rgb(i_rgb), .o_gray(gray_now));

    // Keep both common attribute spellings. The synchronous read-before-write
    // coding style below must be confirmed as READ_FIRST in TangDynasty.
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [7:0] line0 [0:GROUPS_PER_LINE-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [7:0] line1 [0:GROUPS_PER_LINE-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [7:0] line2 [0:GROUPS_PER_LINE-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [7:0] line3 [0:GROUPS_PER_LINE-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [23:0] rgb_line0 [0:GROUPS_PER_LINE-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [23:0] rgb_line1 [0:GROUPS_PER_LINE-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [23:0] rgb_line2 [0:GROUPS_PER_LINE-1];
    (* ram_style = "block", syn_ramstyle = "block_ram" *) reg [23:0] rgb_line3 [0:GROUPS_PER_LINE-1];

    reg [GROUP_ADDR_W-1:0] x_group;
    reg [9:0]  y_pos;
    reg [7:0]  last_gray;

    // One accepted-edge bit per pixel is enough for causal dual-threshold
    // connectivity. The 4-bit bank keeps the 4PPC lane mapping explicit.
    (* ram_style = "distributed", syn_ramstyle = "distributed_ram" *)
    reg [3:0] edge_line [0:GROUPS_PER_LINE-1];
    reg [3:0] prev_edge_d;
    reg [GROUP_ADDR_W-1:0] edge_addr_d;
    reg left_edge_d;

    reg         valid_d;
    reg         sof_d;
    reg [95:0]  rgb_d;
    reg [31:0]  gray_d;
    reg [7:0]   left0_d;
    reg [7:0]   prev0_d;
    reg [7:0]   prev1_d;
    reg [7:0]   prev2_d;
    reg [7:0]   prev3_d;
    reg [23:0]  prev_rgb0_d;
    reg [23:0]  prev_rgb1_d;
    reg [23:0]  prev_rgb2_d;
    reg [23:0]  prev_rgb3_d;
    reg [23:0]  left_rgb0_d;
    reg [23:0]  last_rgb;
    reg         first_line_d;
    reg         first_group_d;
    reg         last_group_d;
    reg [1:0]   y_mod4_d;
    // SOF is also a hard resynchronisation point. Selecting address zero here
    // prevents a truncated/corrupt preceding frame from poisoning the first
    // group of the next frame.
    wire [GROUP_ADDR_W-1:0] line_addr = i_sof ? {GROUP_ADDR_W{1'b0}} : x_group;

    function [8:0] abs_diff;
        input [7:0] a;
        input [7:0] b;
        begin
            abs_diff = (a >= b) ? (a-b) : (b-a);
        end
    endfunction

    wire [7:0] g0 = gray_d[31:24];
    wire [7:0] g1 = gray_d[23:16];
    wire [7:0] g2 = gray_d[15:8];
    wire [7:0] g3 = gray_d[7:0];

    // Preserve the original inexpensive gradient for the ukiyo-e branch.
    wire [9:0] raw_mag0 = abs_diff(g0, left0_d) + abs_diff(g0, prev0_d);
    wire [9:0] raw_mag1 = abs_diff(g1, g0)      + abs_diff(g1, prev1_d);
    wire [9:0] raw_mag2 = abs_diff(g2, g1)      + abs_diff(g2, prev2_d);
    wire [9:0] raw_mag3 = abs_diff(g3, g2)      + abs_diff(g3, prev3_d);

    // The enhanced edge branch deliberately reuses the short Manhattan
    // gradient above.  A previous 2x2 Prewitt form improved the picture, but
    // placed two add/subtract levels plus a four-lane feedback chain on the
    // 112 MHz video path.  Dual-threshold connectivity retains weak contour
    // segments while keeping the arithmetic identical to the proven path.
    wire [9:0] mag0 = raw_mag0;
    wire [9:0] mag1 = raw_mag1;
    wire [9:0] mag2 = raw_mag2;
    wire [9:0] mag3 = raw_mag3;
    wire [9:0] active_threshold = (i_edge_threshold == 0)
                                  ? EDGE_THRESHOLD : i_edge_threshold;
    wire [9:0] half_threshold = {1'b0,active_threshold[9:1]};
    wire [9:0] low_threshold = (half_threshold < 10'd16)
                               ? 10'd16 : half_threshold;
    wire high_hit0 = (mag0 >= active_threshold);
    wire high_hit1 = (mag1 >= active_threshold);
    wire high_hit2 = (mag2 >= active_threshold);
    wire high_hit3 = (mag3 >= active_threshold);
    wire low_hit0 = (mag0 >= low_threshold);
    wire low_hit1 = (mag1 >= low_threshold);
    wire low_hit2 = (mag2 >= low_threshold);
    wire low_hit3 = (mag3 >= low_threshold);

    // Causal hysteresis: weak pixels survive only when connected to an
    // accepted edge in the previous row or a strong edge on the left. Lane 0
    // may also use the accepted result from the preceding 4-pixel group.
    // Importantly, lanes 1..3 never depend recursively on e0/e1/e2, so the
    // four-pixel datapath has no same-cycle ripple chain.
    wire e0 = !first_line_d && !first_group_d
              && (high_hit0 || (low_hit0 && (left_edge_d || prev_edge_d[3])));
    wire e1 = !first_line_d
              && (high_hit1 || (low_hit1 && (high_hit0 || prev_edge_d[2])));
    wire e2 = !first_line_d
              && (high_hit2 || (low_hit2 && (high_hit1 || prev_edge_d[1])));
    wire e3 = !first_line_d
              && (high_hit3 || (low_hit3 && (high_hit2 || prev_edge_d[0])));

    // Keep only dominant key lines in the ukiyo-e branch.  The normal EDGE
    // output retains the user-selected threshold.
    wire [10:0] ukiyo_threshold_wide = {1'b0,active_threshold} + 11'd16;
    wire [9:0] ukiyo_threshold = ukiyo_threshold_wide[10]
                                 ? 10'd1023 : ukiyo_threshold_wide[9:0];
    wire ue0 = !first_line_d && !first_group_d && (raw_mag0 >= ukiyo_threshold);
    wire ue1 = !first_line_d && (raw_mag1 >= ukiyo_threshold);
    wire ue2 = !first_line_d && (raw_mag2 >= ukiyo_threshold);
    wire ue3 = !first_line_d && (raw_mag3 >= ukiyo_threshold);

    // Causal 2D low-pass: (2*current + left + previous-row + 2) / 4.
    // It uses one RGB line buffer, keeps one group/cycle throughput and avoids
    // the full-frame storage of a software bilateral filter.
    function [23:0] causal_smooth_rgb;
        input [23:0] cur;
        input [23:0] left;
        input [23:0] up;
        reg [9:0] rs;
        reg [9:0] gs;
        reg [9:0] bs;
        begin
            rs = ({2'b0,cur[23:16]}<<1) + {2'b0,left[23:16]}
                 + {2'b0,up[23:16]} + 10'd2;
            gs = ({2'b0,cur[15:8]}<<1) + {2'b0,left[15:8]}
                 + {2'b0,up[15:8]} + 10'd2;
            bs = ({2'b0,cur[7:0]}<<1) + {2'b0,left[7:0]}
                 + {2'b0,up[7:0]} + 10'd2;
            causal_smooth_rgb = {rs[9:2],gs[9:2],bs[9:2]};
        end
    endfunction

    function [7:0] causal_smooth_gray;
        input [7:0] cur;
        input [7:0] left;
        input [7:0] up;
        reg [9:0] total;
        begin
            total = ({2'b0,cur}<<1) + {2'b0,left} + {2'b0,up} + 10'd2;
            causal_smooth_gray = total[9:2];
        end
    endfunction

    wire [23:0] smooth0 = causal_smooth_rgb(
        rgb_d[95:72],left_rgb0_d,first_line_d ? rgb_d[95:72] : prev_rgb0_d);
    wire [23:0] smooth1 = causal_smooth_rgb(
        rgb_d[71:48],rgb_d[95:72],first_line_d ? rgb_d[71:48] : prev_rgb1_d);
    wire [23:0] smooth2 = causal_smooth_rgb(
        rgb_d[47:24],rgb_d[71:48],first_line_d ? rgb_d[47:24] : prev_rgb2_d);
    wire [23:0] smooth3 = causal_smooth_rgb(
        rgb_d[23:0],rgb_d[47:24],first_line_d ? rgb_d[23:0] : prev_rgb3_d);
    wire [7:0] smooth_g0 = causal_smooth_gray(g0,left0_d,first_line_d ? g0 : prev0_d);
    wire [7:0] smooth_g1 = causal_smooth_gray(g1,g0,first_line_d ? g1 : prev1_d);
    wire [7:0] smooth_g2 = causal_smooth_gray(g2,g1,first_line_d ? g2 : prev2_d);
    wire [7:0] smooth_g3 = causal_smooth_gray(g3,g2,first_line_d ? g3 : prev3_d);

    wire [23:0] gray0_rgb = {g0,g0,g0};
    wire [23:0] gray1_rgb = {g1,g1,g1};
    wire [23:0] gray2_rgb = {g2,g2,g2};
    wire [23:0] gray3_rgb = {g3,g3,g3};
    wire [23:0] bin0_rgb = (g0 >= BIN_THRESHOLD) ? 24'hffffff : 24'h000000;
    wire [23:0] bin1_rgb = (g1 >= BIN_THRESHOLD) ? 24'hffffff : 24'h000000;
    wire [23:0] bin2_rgb = (g2 >= BIN_THRESHOLD) ? 24'hffffff : 24'h000000;
    wire [23:0] bin3_rgb = (g3 >= BIN_THRESHOLD) ? 24'hffffff : 24'h000000;
    wire [23:0] edge0_rgb = e0 ? 24'hffffff : 24'h000000;
    wire [23:0] edge1_rgb = e1 ? 24'hffffff : 24'h000000;
    wire [23:0] edge2_rgb = e2 ? 24'hffffff : 24'h000000;
    wire [23:0] edge3_rgb = e3 ? 24'hffffff : 24'h000000;

    // Hardware-friendly ukiyo-e palette.  It deliberately uses a small,
    // fixed set of woodblock-inspired colours rather than independently
    // posterising R/G/B.  Strong local gradients become a dark indigo key
    // line; otherwise hue dominance and luma select one palette entry.
    // A sparse 4x4 diagonal texture slightly varies paper/sand regions to
    // avoid an unnaturally flat digital fill.
    function [23:0] ukiyo_pixel;
        input [23:0] p;
        input [7:0] luma;
        input edge_flag;
        input texture_dot;
        reg [8:0] rv;
        reg [8:0] gv;
        reg [8:0] bv;
        reg [7:0] max_c;
        reg [7:0] min_c;
        begin
            rv = {1'b0,p[23:16]};
            gv = {1'b0,p[15:8]};
            bv = {1'b0,p[7:0]};
            max_c = (p[23:16] >= p[15:8])
                    ? ((p[23:16] >= p[7:0]) ? p[23:16] : p[7:0])
                    : ((p[15:8] >= p[7:0]) ? p[15:8] : p[7:0]);
            min_c = (p[23:16] <= p[15:8])
                    ? ((p[23:16] <= p[7:0]) ? p[23:16] : p[7:0])
                    : ((p[15:8] <= p[7:0]) ? p[15:8] : p[7:0]);

            if (edge_flag)
                ukiyo_pixel = 24'h142330; // dark indigo woodblock outline
            else if ((luma > 8'd210) && ((max_c-min_c) < 8'd40))
                ukiyo_pixel = texture_dot ? 24'hebd9b2 : 24'hf2e4be;
            else if ((rv > gv+9'd36) && (rv > bv+9'd28)) begin
                // Warm, moderately saturated skin/wood tones need a softer
                // block colour than genuinely red paint, signs or feathers.
                if ((p[15:8] > 8'd30) && (p[7:0] > 8'd20)
                    && (rv < gv+9'd100) && (gv < bv+9'd80)
                    && ((gv > bv+9'd4)
                        || ((luma < 8'd110) && (gv+9'd4 > bv)))) begin
                    if (luma > 8'd145)
                        ukiyo_pixel = 24'he8d3a6;
                    else if (luma > 8'd85)
                        ukiyo_pixel = 24'hc49a6c;
                    else
                        ukiyo_pixel = 24'h806a5a;
                end
                else
                    ukiyo_pixel = 24'hc64a3a; // vermilion
            end
            else if ((bv > rv+9'd24) && (bv > gv+9'd12))
                ukiyo_pixel = (luma > 8'd135) ? 24'h5e88a1 : 24'h2b4c77;
            else if ((gv > rv+9'd20) && (gv > bv+9'd12))
                ukiyo_pixel = 24'h4f7255; // pine green
            else if ((p[23:16] > 8'd155) && (p[15:8] > 8'd125)
                     && (p[7:0] < 8'd125))
                ukiyo_pixel = 24'hd2a448; // ochre
            else if (luma > 8'd200)
                ukiyo_pixel = texture_dot ? 24'he1c99b : 24'he8d3a6;
            else if (luma > 8'd145)
                ukiyo_pixel = texture_dot ? 24'hbd9063 : 24'hc49a6c;
            else if (luma > 8'd85)
                ukiyo_pixel = 24'h806a5a; // muted brown
            else
                ukiyo_pixel = 24'h1b2a35; // deep blue-black
        end
    endfunction

    assign o_sof     = sof_d;
    assign o_valid   = valid_d;
    assign o_raw     = rgb_d;
    assign o_gray    = {gray0_rgb,gray1_rgb,gray2_rgb,gray3_rgb};
    assign o_binary  = {bin0_rgb,bin1_rgb,bin2_rgb,bin3_rgb};
    assign o_edge    = {edge0_rgb,edge1_rgb,edge2_rgb,edge3_rgb};
    assign o_ukiyo = {ukiyo_pixel(smooth0,smooth_g0,ue0,(y_mod4_d==2'd0)),
                      ukiyo_pixel(smooth1,smooth_g1,ue1,(y_mod4_d==2'd1)),
                      ukiyo_pixel(smooth2,smooth_g2,ue2,(y_mod4_d==2'd2)),
                      ukiyo_pixel(smooth3,smooth_g3,ue3,(y_mod4_d==2'd3))};

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            x_group      <= 0;
            y_pos        <= 0;
            last_gray    <= 0;
            prev_edge_d  <= 0;
            edge_addr_d  <= 0;
            left_edge_d  <= 1'b0;
            valid_d      <= 1'b0;
            sof_d        <= 1'b0;
            rgb_d        <= 0;
            gray_d       <= 0;
            left0_d      <= 0;
            prev0_d      <= 0;
            prev1_d      <= 0;
            prev2_d      <= 0;
            prev3_d      <= 0;
            prev_rgb0_d  <= 0;
            prev_rgb1_d  <= 0;
            prev_rgb2_d  <= 0;
            prev_rgb3_d  <= 0;
            left_rgb0_d  <= 0;
            last_rgb     <= 0;
            first_line_d <= 1'b1;
            first_group_d<= 1'b1;
            last_group_d <= 1'b0;
            y_mod4_d     <= 2'd0;
        end else begin
            valid_d <= i_valid;
            sof_d   <= i_sof;
            if (valid_d && !i_sof) begin
                edge_line[edge_addr_d] <= {e0,e1,e2,e3};
                if (last_group_d) begin
                    left_edge_d <= 1'b0;
                end else begin
                    left_edge_d <= e3;
                end
            end
            if (i_valid) begin
                rgb_d         <= i_rgb;
                gray_d        <= gray_now;
                prev0_d       <= line0[line_addr];
                prev1_d       <= line1[line_addr];
                prev2_d       <= line2[line_addr];
                prev3_d       <= line3[line_addr];
                prev_rgb0_d   <= rgb_line0[line_addr];
                prev_rgb1_d   <= rgb_line1[line_addr];
                prev_rgb2_d   <= rgb_line2[line_addr];
                prev_rgb3_d   <= rgb_line3[line_addr];
                prev_edge_d   <= edge_line[line_addr];
                edge_addr_d   <= line_addr;
                line0[line_addr]<= gray_now[31:24];
                line1[line_addr]<= gray_now[23:16];
                line2[line_addr]<= gray_now[15:8];
                line3[line_addr]<= gray_now[7:0];
                rgb_line0[line_addr]<= i_rgb[95:72];
                rgb_line1[line_addr]<= i_rgb[71:48];
                rgb_line2[line_addr]<= i_rgb[47:24];
                rgb_line3[line_addr]<= i_rgb[23:0];
                left0_d       <= (x_group == 0) ? gray_now[31:24] : last_gray;
                last_gray     <= gray_now[7:0];
                left_rgb0_d   <= (x_group == 0) ? i_rgb[95:72] : last_rgb;
                last_rgb      <= i_rgb[23:0];
                first_line_d  <= (y_pos == 0);
                first_group_d <= (x_group == 0);
                last_group_d  <= (x_group == LAST_GROUP);
                y_mod4_d      <= y_pos[1:0];

                if (i_sof) begin
                    x_group   <= 1;
                    y_pos     <= 0;
                    left0_d   <= gray_now[31:24];
                    left_rgb0_d <= i_rgb[95:72];
                    first_line_d  <= 1'b1;
                    first_group_d <= 1'b1;
                    last_group_d  <= (LAST_GROUP == 0);
                    y_mod4_d      <= 2'd0;
                    left_edge_d   <= 1'b0;
                end else if (x_group == LAST_GROUP) begin
                    x_group   <= 0;
                    last_gray <= 0;
                    if (y_pos == IMG_HEIGHT-1)
                        y_pos <= 0;
                    else
                        y_pos <= y_pos + 1'b1;
                end else begin
                    x_group <= x_group + 1'b1;
                end
            end
        end
    end
endmodule
