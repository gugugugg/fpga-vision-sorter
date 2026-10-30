`timescale 1ns/1ps

// Hardware-oriented shared Ukiyo-e / ink-wash stream processor.
//
// The implementation keeps four pixels/cycle and uses only causal neighbours:
// two fixed-point smoothing passes, a five-sample palette mode filter, dual
// threshold connected line work, warm-surface simplification and sparse 16x16
// print texture tiles.  All rendered colours use a 4-bit fixed palette index.
module ukiyo_stylize_4ppc_v2 #(
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_sof,
    input  wire        i_valid,
    input  wire [95:0] i_rgb,
    input  wire [9:0]  i_edge_threshold,
    input  wire        i_ink_style,
    output reg         o_sof,
    output reg         o_valid,
    output reg  [95:0] o_rgb
);
    localparam GROUPS_PER_LINE = IMG_WIDTH / 4;
    localparam GROUP_ADDR_W = (GROUPS_PER_LINE <= 2) ? 1 : $clog2(GROUPS_PER_LINE);
    localparam [GROUP_ADDR_W-1:0] LAST_GROUP = GROUPS_PER_LINE-1;

    function [23:0] smooth_pixel;
        input [23:0] cur;
        input [23:0] left;
        input [23:0] up;
        reg [9:0] rs, gs, bs;
        begin
            rs = ({2'b0,cur[23:16]}<<1) + {2'b0,left[23:16]} + {2'b0,up[23:16]} + 10'd2;
            gs = ({2'b0,cur[15:8]} <<1) + {2'b0,left[15:8]}  + {2'b0,up[15:8]}  + 10'd2;
            bs = ({2'b0,cur[7:0]}  <<1) + {2'b0,left[7:0]}   + {2'b0,up[7:0]}   + 10'd2;
            smooth_pixel = {rs[9:2],gs[9:2],bs[9:2]};
        end
    endfunction

    function [7:0] pixel_luma;
        input [23:0] p;
        reg [15:0] r16;
        reg [15:0] g16;
        reg [15:0] b16;
        reg [15:0] sum;
        begin
            r16 = {8'd0,p[23:16]};
            g16 = {8'd0,p[15:8]};
            b16 = {8'd0,p[7:0]};
            sum = (r16 << 6) + (r16 << 3) + (r16 << 2) + r16
                + (g16 << 7) + (g16 << 4) + (g16 << 2) + (g16 << 1)
                + (b16 << 4) + (b16 << 3) + (b16 << 2) + b16;
            pixel_luma = sum[15:8];
        end
    endfunction

    function [8:0] abs_diff;
        input [7:0] a;
        input [7:0] b;
        begin
            abs_diff = (a >= b) ? (a-b) : (b-a);
        end
    endfunction

    function warm_surface;
        input [23:0] p;
        reg [8:0] r, g, b;
        begin
            r = {1'b0,p[23:16]};
            g = {1'b0,p[15:8]};
            b = {1'b0,p[7:0]};
            warm_surface = (r > g+9'd8) && (r > b+9'd14) && (g+9'd8 > b)
                           && (r > 9'd35) && (g > 9'd20) && (b > 9'd12)
                           && (r < g+9'd100) && (g < b+9'd70)
                           && !((r > 9'd165) && (g < 9'd90) && (b < 9'd80));
        end
    endfunction

    function skin_fill_candidate;
        input [23:0] p;
        input [7:0] luma;
        reg [8:0] r,g,b;
        begin
            r={1'b0,p[23:16]};g={1'b0,p[15:8]};b={1'b0,p[7:0]};
            skin_fill_candidate=(r>g+9'd36)&&(r>b+9'd28)
                &&(g>9'd30)&&(b>9'd20)&&(r<g+9'd100)&&(g<b+9'd80)
                &&((g>b+9'd4)||((luma<8'd110)&&(g+9'd4>b)));
        end
    endfunction

    function [3:0] palette_index;
        input [23:0] p;
        input [7:0] luma;
        reg [8:0] r, g, b;
        reg [7:0] max_c, min_c;
        reg skin_fill;
        begin
            r = {1'b0,p[23:16]};
            g = {1'b0,p[15:8]};
            b = {1'b0,p[7:0]};
            max_c = (p[23:16] >= p[15:8])
                    ? ((p[23:16] >= p[7:0]) ? p[23:16] : p[7:0])
                    : ((p[15:8] >= p[7:0]) ? p[15:8] : p[7:0]);
            min_c = (p[23:16] <= p[15:8])
                    ? ((p[23:16] <= p[7:0]) ? p[23:16] : p[7:0])
                    : ((p[15:8] <= p[7:0]) ? p[15:8] : p[7:0]);
            skin_fill = skin_fill_candidate(p,luma);
            if ((luma > 8'd210) && ((max_c-min_c) < 8'd40))
                palette_index = 4'd2;
            else if (skin_fill && (luma > 8'd145))
                palette_index = 4'd9;
            else if (skin_fill && (luma > 8'd85))
                palette_index = 4'd11;
            else if (skin_fill)
                palette_index = 4'd13;
            else if ((r > g+9'd36) && (r > b+9'd28))
                palette_index = 4'd4;
            else if ((b > r+9'd24) && (b > g+9'd12))
                palette_index = (luma > 8'd135) ? 4'd5 : 4'd6;
            else if ((g > r+9'd20) && (g > b+9'd12))
                palette_index = 4'd7;
            else if ((r > 9'd155) && (g > 9'd125) && (b < 9'd125))
                palette_index = 4'd8;
            else if (luma > 8'd200)
                palette_index = 4'd9;
            else if (luma > 8'd145)
                palette_index = 4'd11;
            else if (luma > 8'd85)
                palette_index = 4'd13;
            else
                palette_index = 4'd14;
        end
    endfunction

    // Seven achromatic ink levels reuse legal 4-bit palette indices.  The
    // non-uniform thresholds deliberately leave large quiet tone regions,
    // which reads as layered washes instead of an ordinary gray conversion.
    function [3:0] ink_palette_index;
        input [7:0] luma;
        begin
            if (luma > 8'd235)      ink_palette_index = 4'd2;
            else if (luma > 8'd214) ink_palette_index = 4'd3;
            else if (luma > 8'd180) ink_palette_index = 4'd9;
            else if (luma > 8'd145) ink_palette_index = 4'd10;
            else if (luma > 8'd108) ink_palette_index = 4'd11;
            else if (luma > 8'd72)  ink_palette_index = 4'd13;
            else                    ink_palette_index = 4'd14;
        end
    endfunction

    function [3:0] palette_family;
        input [3:0] index;
        input ink_style;
        begin
            if (ink_style) begin
                case (index)
                    4'd2,4'd3,4'd9: palette_family = 4'd0;
                    4'd10,4'd11:     palette_family = 4'd1;
                    4'd12,4'd13:     palette_family = 4'd2;
                    default:         palette_family = 4'd3;
                endcase
            end else begin
                case (index)
                    4'd2,4'd3,4'd9,4'd10: palette_family = 4'd0;
                    4'd11,4'd12,4'd13:    palette_family = 4'd1;
                    4'd6,4'd14:           palette_family = 4'd2;
                    4'd4:                 palette_family = 4'd3;
                    4'd5:                 palette_family = 4'd4;
                    4'd7:                 palette_family = 4'd5;
                    4'd8:                 palette_family = 4'd6;
                    default:              palette_family = 4'd7;
                endcase
            end
        end
    endfunction

    function [3:0] majority5;
        input [3:0] a,b,c,d,e;
        reg [2:0] count;
        begin
            count = 3'd1 + (a==b) + (a==c) + (a==d) + (a==e);
            if (count >= 3) majority5 = a;
            else begin
                count = 3'd1 + (b==c) + (b==d) + (b==e);
                if (count >= 3) majority5 = b;
                else begin
                    count = 3'd1 + (c==d) + (c==e);
                    if (count >= 3) majority5 = c;
                    else majority5 = a;
                end
            end
        end
    endfunction

    function majority5_bit;
        input a,b,c,d,e;
        reg [2:0] count;
        begin
            count = a+b+c+d+e;
            majority5_bit = (count >= 3);
        end
    endfunction

    function [23:0] palette_rgb;
        input [3:0] index;
        begin
            case (index)
                4'd0:  palette_rgb = 24'h142330;
                4'd1:  palette_rgb = 24'h34424a;
                4'd2:  palette_rgb = 24'hf2e4be;
                4'd3:  palette_rgb = 24'hebd9b2;
                4'd4:  palette_rgb = 24'hc64a3a;
                4'd5:  palette_rgb = 24'h5e88a1;
                4'd6:  palette_rgb = 24'h2b4c77;
                4'd7:  palette_rgb = 24'h4f7255;
                4'd8:  palette_rgb = 24'hd2a448;
                4'd9:  palette_rgb = 24'he8d3a6;
                4'd10: palette_rgb = 24'he1c99b;
                4'd11: palette_rgb = 24'hc49a6c;
                4'd12: palette_rgb = 24'hbd9063;
                4'd13: palette_rgb = 24'h806a5a;
                default: palette_rgb = 24'h1b2a35;
            endcase
        end
    endfunction

    function [23:0] ink_palette_rgb;
        input [3:0] index;
        begin
            case (index)
                4'd0:  ink_palette_rgb = 24'h141f22; // concentrated ink
                4'd1:  ink_palette_rgb = 24'h4b595b; // connected weak stroke
                4'd2:  ink_palette_rgb = 24'hf6efdc; // warm rice paper
                4'd3:  ink_palette_rgb = 24'he6e1d3; // clear wash / fibre
                4'd9:  ink_palette_rgb = 24'hcfd0c7; // pale wash
                4'd10: ink_palette_rgb = 24'habb0ab; // light wash
                4'd11: ink_palette_rgb = 24'h7e8886; // middle wash
                4'd12: ink_palette_rgb = 24'h64706f; // granulated middle wash
                4'd13: ink_palette_rgb = 24'h4b595b; // heavy wash
                default: ink_palette_rgb = 24'h28383b; // deep wash
            endcase
        end
    endfunction

    function fibre_seed;
        input [10:0] x;
        input [9:0] y;
        reg [3:0] tx, ty;
        reg [15:0] row_bits;
        begin
            tx = x[3:0] ^ {4{y[4]}};
            ty = y[3:0] ^ {4{x[4]}};
            case (ty)
                4'd1: row_bits=16'h0200; 4'd3: row_bits=16'h0040;
                4'd5: row_bits=16'h2000; 4'd7: row_bits=16'h0004;
                4'd9: row_bits=16'h0100; 4'd11:row_bits=16'h4000;
                4'd13:row_bits=16'h0010; 4'd15:row_bits=16'h0800;
                default: row_bits=16'h0000;
            endcase
            fibre_seed = row_bits[tx];
        end
    endfunction

    function hatch_seed;
        input [10:0] x;
        input [9:0] y;
        reg [3:0] tx, ty;
        reg [15:0] row_bits;
        begin
            tx = x[3:0] ^ {4{y[4]}};
            ty = y[3:0] ^ {4{x[4]}};
            case (ty)
                4'd1: row_bits=16'h0008; 4'd5: row_bits=16'h0800;
                4'd9: row_bits=16'h0020; 4'd13:row_bits=16'h2000;
                default: row_bits=16'h0000;
            endcase
            hatch_seed = row_bits[tx];
        end
    endfunction

    // Stage A: raw stream line neighbourhood and first smoothing pass.
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] raw_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] raw_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] raw_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] raw_line3[0:GROUPS_PER_LINE-1];
    reg [GROUP_ADDR_W-1:0] x_group;
    reg [9:0] y_pos;
    wire [GROUP_ADDR_W-1:0] input_addr = i_sof ? {GROUP_ADDR_W{1'b0}} : x_group;
    wire [9:0] input_y = i_sof ? 10'd0 : y_pos;
    reg a_valid,a_sof,a_first_line,a_first_group;
    reg [GROUP_ADDR_W-1:0] a_addr;
    reg [9:0] a_y;
    reg [95:0] a_rgb;
    reg [23:0] a_up0,a_up1,a_up2,a_up3,a_left0,last_raw;
    wire [23:0] a_s10=smooth_pixel(a_rgb[95:72],a_left0,a_first_line?a_rgb[95:72]:a_up0);
    wire [23:0] a_s11=smooth_pixel(a_rgb[71:48],a_rgb[95:72],a_first_line?a_rgb[71:48]:a_up1);
    wire [23:0] a_s12=smooth_pixel(a_rgb[47:24],a_rgb[71:48],a_first_line?a_rgb[47:24]:a_up2);
    wire [23:0] a_s13=smooth_pixel(a_rgb[23:0],a_rgb[47:24],a_first_line?a_rgb[23:0]:a_up3);

    // Stage B: line neighbourhood of pass one and second smoothing pass.
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] s1_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] s1_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] s1_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [23:0] s1_line3[0:GROUPS_PER_LINE-1];
    reg b_valid,b_sof,b_first_line,b_first_group;
    reg [GROUP_ADDR_W-1:0] b_addr;
    reg [9:0] b_y;
    reg [95:0] b_s1;
    reg [23:0] b_up0,b_up1,b_up2,b_up3,b_left0,last_s1;
    wire [23:0] b_s20=smooth_pixel(b_s1[95:72],b_left0,b_first_line?b_s1[95:72]:b_up0);
    wire [23:0] b_s21=smooth_pixel(b_s1[71:48],b_s1[95:72],b_first_line?b_s1[71:48]:b_up1);
    wire [23:0] b_s22=smooth_pixel(b_s1[47:24],b_s1[71:48],b_first_line?b_s1[47:24]:b_up2);
    wire [23:0] b_s23=smooth_pixel(b_s1[23:0],b_s1[47:24],b_first_line?b_s1[23:0]:b_up3);

    // The original V2 prototype evaluated smoothing, luma and palette
    // classification in one 112-MHz MIPI-byte-clock cycle.  Keep the exact
    // arithmetic, but register each part so the board implementation has a
    // realistic timing path rather than a simulation-only combinational cone.
    reg b2_valid,b2_sof,b2_first_line,b2_first_group;
    reg [GROUP_ADDR_W-1:0] b2_addr;
    reg [9:0] b2_y;
    reg [95:0] b2_s2;
    wire [7:0] b2_l0=pixel_luma(b2_s2[95:72]), b2_l1=pixel_luma(b2_s2[71:48]);
    wire [7:0] b2_l2=pixel_luma(b2_s2[47:24]), b2_l3=pixel_luma(b2_s2[23:0]);

    reg a_ink_style,b_ink_style,b2_ink_style,b3_ink_style,b4_ink_style;
    reg c_ink_style,d_ink_style,e_ink_style,f_ink_style;
    reg b3_valid,b3_sof,b3_first_line,b3_first_group;
    reg [GROUP_ADDR_W-1:0] b3_addr;
    reg [9:0] b3_y;
    reg [95:0] b3_s2;
    reg [31:0] b3_luma;
    wire [3:0] b3_p0=b3_ink_style?ink_palette_index(b3_luma[31:24]):palette_index(b3_s2[95:72],b3_luma[31:24]);
    wire [3:0] b3_p1=b3_ink_style?ink_palette_index(b3_luma[23:16]):palette_index(b3_s2[71:48],b3_luma[23:16]);
    wire [3:0] b3_p2=b3_ink_style?ink_palette_index(b3_luma[15:8]):palette_index(b3_s2[47:24],b3_luma[15:8]);
    wire [3:0] b3_p3=b3_ink_style?ink_palette_index(b3_luma[7:0]):palette_index(b3_s2[23:0],b3_luma[7:0]);
    wire b3_w0=warm_surface(b3_s2[95:72]), b3_w1=warm_surface(b3_s2[71:48]);
    wire b3_w2=warm_surface(b3_s2[47:24]), b3_w3=warm_surface(b3_s2[23:0]);

    reg b4_valid,b4_sof,b4_first_line,b4_first_group;
    reg [GROUP_ADDR_W-1:0] b4_addr;
    reg [9:0] b4_y;
    reg [31:0] b4_luma;
    reg [15:0] b4_pal;
    reg [3:0] b4_warm;

    // Stage C: luma gradient and provisional palette five-sample neighbourhood.
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [7:0] luma_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [7:0] luma_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [7:0] luma_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [7:0] luma_line3[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] pal_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] pal_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] pal_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] pal_line3[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg warm_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg warm_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg warm_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg warm_line3[0:GROUPS_PER_LINE-1];
    reg c_valid,c_sof,c_first_line,c_first_group;
    reg [GROUP_ADDR_W-1:0] c_addr;
    reg [9:0] c_y;
    reg [31:0] c_luma,c_up_luma;
    reg [15:0] c_pal,c_up_pal;
    reg [3:0] c_warm,c_up_warm;
    reg [7:0] c_left_luma0,c_up_left_luma0,last_luma,last_up_luma3;
    reg [3:0] c_leftp1_0,c_leftp2_0,c_up_leftp0,last_p2,last_p3,last_up_p3;
    reg c_leftw1_0,c_leftw2_0,c_up_leftw0,last_w2,last_w3,last_up_w3;
    wire [7:0] c_l0=c_luma[31:24],c_l1=c_luma[23:16],c_l2=c_luma[15:8],c_l3=c_luma[7:0];
    wire [7:0] c_u0=c_first_line?c_l0:c_up_luma[31:24];
    wire [7:0] c_u1=c_first_line?c_l1:c_up_luma[23:16];
    wire [7:0] c_u2=c_first_line?c_l2:c_up_luma[15:8];
    wire [7:0] c_u3=c_first_line?c_l3:c_up_luma[7:0];
    wire [7:0] c_left0=c_first_group?c_l0:c_left_luma0;
    wire [9:0] c_g0=(c_first_line||c_first_group)?10'd0:(abs_diff(c_l0,c_left0)+abs_diff(c_l0,c_u0));
    wire [9:0] c_g1=c_first_line?10'd0:(abs_diff(c_l1,c_l0)+abs_diff(c_l1,c_u1));
    wire [9:0] c_g2=c_first_line?10'd0:(abs_diff(c_l2,c_l1)+abs_diff(c_l2,c_u2));
    wire [9:0] c_g3=c_first_line?10'd0:(abs_diff(c_l3,c_l2)+abs_diff(c_l3,c_u3));
    wire [9:0] active_threshold=(i_edge_threshold==0)?10'd72:i_edge_threshold;
    wire [9:0] v2_strong_threshold=(active_threshold>10'd24)?(active_threshold-10'd24):10'd1;
    wire [9:0] v2_weak_threshold=(v2_strong_threshold>10'd24)?(v2_strong_threshold>>1):10'd12;
    wire c_s0=!c_first_line&&!c_first_group&&(c_g0>=v2_strong_threshold);
    wire c_s1=!c_first_line&&(c_g1>=v2_strong_threshold);
    wire c_s2=!c_first_line&&(c_g2>=v2_strong_threshold);
    wire c_s3=!c_first_line&&(c_g3>=v2_strong_threshold);
    wire [3:0] c_p0=c_pal[15:12],c_p1=c_pal[11:8],c_p2=c_pal[7:4],c_p3=c_pal[3:0];
    wire [3:0] c_up0=c_first_line?c_p0:c_up_pal[15:12];
    wire [3:0] c_up1=c_first_line?c_p1:c_up_pal[11:8];
    wire [3:0] c_up2=c_first_line?c_p2:c_up_pal[7:4];
    wire [3:0] c_up3=c_first_line?c_p3:c_up_pal[3:0];
    wire [3:0] c_m0=(c_first_line||c_first_group||c_s0||c_warm[3])?c_p0:majority5(c_p0,c_leftp1_0,c_leftp2_0,c_up0,c_up_leftp0);
    wire [3:0] c_m1=(c_first_line||c_first_group||c_s1||c_warm[2])?c_p1:majority5(c_p1,c_p0,c_leftp1_0,c_up1,c_up0);
    wire [3:0] c_m2=(c_first_line||c_s2||c_warm[1])?c_p2:majority5(c_p2,c_p1,c_p0,c_up2,c_up1);
    wire [3:0] c_m3=(c_first_line||c_s3||c_warm[0])?c_p3:majority5(c_p3,c_p2,c_p1,c_up3,c_up2);
    wire c_sw0=(c_first_line||c_first_group)?c_warm[3]:majority5_bit(c_warm[3],c_leftw1_0,c_leftw2_0,c_up_warm[3],c_up_leftw0);
    wire c_sw1=(c_first_line||c_first_group)?c_warm[2]:majority5_bit(c_warm[2],c_warm[3],c_leftw1_0,c_up_warm[2],c_up_warm[3]);
    wire c_sw2=c_first_line?c_warm[1]:majority5_bit(c_warm[1],c_warm[2],c_warm[3],c_up_warm[1],c_up_warm[2]);
    wire c_sw3=c_first_line?c_warm[0]:majority5_bit(c_warm[0],c_warm[1],c_warm[2],c_up_warm[0],c_up_warm[1]);

    // Stage D: cleaned palette/skin/strong-edge line neighbourhood.
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] clean_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] clean_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] clean_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] clean_line3[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg skin_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg skin_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg skin_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg skin_line3[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg strong_line0[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg strong_line1[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg strong_line2[0:GROUPS_PER_LINE-1];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg strong_line3[0:GROUPS_PER_LINE-1];
    reg d_valid,d_sof,d_first_line,d_first_group;
    reg [GROUP_ADDR_W-1:0] d_addr;
    reg [9:0] d_y;
    reg [15:0] d_clean,d_up_clean;
    reg [3:0] d_skin,d_up_skin,d_strong,d_up_strong;
    reg [39:0] d_grad;
    reg [31:0] d_luma,d_up_luma;
    reg [7:0] d_left_luma0,d_up_left_luma0;
    reg [3:0] d_left_clean0,d_up_left_clean0,last_clean3,last_up_clean3;
    reg d_left_skin0,d_up_left_skin0,last_skin3,last_up_skin3;
    reg d_left_strong0,d_up_left_strong0,last_strong3,last_up_strong3;
    wire [3:0] d_p0=d_clean[15:12],d_p1=d_clean[11:8],d_p2=d_clean[7:4],d_p3=d_clean[3:0];
    wire [3:0] d_u0=d_first_line?d_p0:d_up_clean[15:12];
    wire [3:0] d_u1=d_first_line?d_p1:d_up_clean[11:8];
    wire [3:0] d_u2=d_first_line?d_p2:d_up_clean[7:4];
    wire [3:0] d_u3=d_first_line?d_p3:d_up_clean[3:0];
    wire [3:0] d_lpi0=d_first_group?d_p0:d_left_clean0;
    wire d_b0=!d_first_line&&!d_first_group&&((d_p0!=d_lpi0)||(d_p0!=d_u0));
    wire d_b1=!d_first_line&&((d_p1!=d_p0)||(d_p1!=d_u1));
    wire d_b2=!d_first_line&&((d_p2!=d_p1)||(d_p2!=d_u2));
    wire d_b3=!d_first_line&&((d_p3!=d_p2)||(d_p3!=d_u3));
    wire d_mb0=d_b0&&((palette_family(d_p0,d_ink_style)!=palette_family(d_lpi0,d_ink_style))||(palette_family(d_p0,d_ink_style)!=palette_family(d_u0,d_ink_style)));
    wire d_mb1=d_b1&&((palette_family(d_p1,d_ink_style)!=palette_family(d_p0,d_ink_style))||(palette_family(d_p1,d_ink_style)!=palette_family(d_u1,d_ink_style)));
    wire d_mb2=d_b2&&((palette_family(d_p2,d_ink_style)!=palette_family(d_p1,d_ink_style))||(palette_family(d_p2,d_ink_style)!=palette_family(d_u2,d_ink_style)));
    wire d_mb3=d_b3&&((palette_family(d_p3,d_ink_style)!=palette_family(d_p2,d_ink_style))||(palette_family(d_p3,d_ink_style)!=palette_family(d_u3,d_ink_style)));
    wire [9:0] d_g0=d_grad[39:30],d_g1=d_grad[29:20],d_g2=d_grad[19:10],d_g3=d_grad[9:0];
    wire [7:0] d_l0=d_luma[31:24],d_l1=d_luma[23:16],d_l2=d_luma[15:8],d_l3=d_luma[7:0];
    wire [7:0] d_ul0=d_first_line?d_l0:d_up_luma[31:24];
    wire [7:0] d_ul1=d_first_line?d_l1:d_up_luma[23:16];
    wire [7:0] d_ul2=d_first_line?d_l2:d_up_luma[15:8];
    wire [7:0] d_ul3=d_first_line?d_l3:d_up_luma[7:0];
    wire [7:0] d_ll0=d_first_group?d_l0:d_left_luma0;
    wire [9:0] d_mean0=({2'b0,d_l0}+{2'b0,d_ll0}+{2'b0,d_ul0}+{2'b0,(d_first_line?d_ll0:d_up_left_luma0)}+10'd2)>>2;
    wire [9:0] d_mean1=({2'b0,d_l1}+{2'b0,d_l0}+{2'b0,d_ul1}+{2'b0,d_ul0}+10'd2)>>2;
    wire [9:0] d_mean2=({2'b0,d_l2}+{2'b0,d_l1}+{2'b0,d_ul2}+{2'b0,d_ul1}+10'd2)>>2;
    wire [9:0] d_mean3=({2'b0,d_l3}+{2'b0,d_l2}+{2'b0,d_ul3}+{2'b0,d_ul2}+10'd2)>>2;
    wire d_ls0=d_first_group?d_skin[3]:d_left_skin0;
    wire d_us0=d_first_line?d_skin[3]:d_up_skin[3];
    wire d_us1=d_first_line?d_skin[2]:d_up_skin[2];
    wire d_us2=d_first_line?d_skin[1]:d_up_skin[1];
    wire d_us3=d_first_line?d_skin[0]:d_up_skin[0];
    wire d_ctx0=d_skin[3]||d_ls0||d_us0;
    wire d_ctx1=d_skin[2]||d_skin[3]||d_us1;
    wire d_ctx2=d_skin[1]||d_skin[2]||d_us2;
    wire d_ctx3=d_skin[0]||d_skin[1]||d_us3;
    wire d_skinb0=!d_first_line&&!d_first_group&&((d_skin[3]!=d_ls0)||(d_skin[3]!=d_us0));
    wire d_skinb1=!d_first_line&&((d_skin[2]!=d_skin[3])||(d_skin[2]!=d_us1));
    wire d_skinb2=!d_first_line&&((d_skin[1]!=d_skin[2])||(d_skin[1]!=d_us2));
    wire d_skinb3=!d_first_line&&((d_skin[0]!=d_skin[1])||(d_skin[0]!=d_us3));
    wire d_face0=d_ctx0&&(d_mean0>=d_l0+10'd18)&&(d_l0<8'd175);
    wire d_face1=d_ctx1&&(d_mean1>=d_l1+10'd18)&&(d_l1<8'd175);
    wire d_face2=d_ctx2&&(d_mean2>=d_l2+10'd18)&&(d_l2<8'd175);
    wire d_face3=d_ctx3&&(d_mean3>=d_l3+10'd18)&&(d_l3<8'd175);
    wire d_lstrong0=d_first_group?d_strong[3]:d_left_strong0;
    wire d_ustrong0=d_first_line?1'b0:d_up_strong[3];
    wire d_ustrong1=d_first_line?1'b0:d_up_strong[2];
    wire d_ustrong2=d_first_line?1'b0:d_up_strong[1];
    wire d_ustrong3=d_first_line?1'b0:d_up_strong[0];
    wire d_link0=(d_g0>=v2_weak_threshold)&&(d_lstrong0||d_ustrong0||(d_first_line?1'b0:d_up_left_strong0));
    wire d_link1=(d_g1>=v2_weak_threshold)&&(d_strong[3]||d_ustrong1||d_ustrong0);
    wire d_link2=(d_g2>=v2_weak_threshold)&&(d_strong[2]||d_ustrong2||d_ustrong1);
    wire d_link3=(d_g3>=v2_weak_threshold)&&(d_strong[1]||d_ustrong3||d_ustrong2);
    wire d_ink0=d_strong[3]||(d_mb0&&!d_ctx0&&(d_g0>=25))||(d_skinb0&&(d_g0>=24));
    wire d_ink1=d_strong[2]||(d_mb1&&!d_ctx1&&(d_g1>=25))||(d_skinb1&&(d_g1>=24));
    wire d_ink2=d_strong[1]||(d_mb2&&!d_ctx2&&(d_g2>=25))||(d_skinb2&&(d_g2>=24));
    wire d_ink3=d_strong[0]||(d_mb3&&!d_ctx3&&(d_g3>=25))||(d_skinb3&&(d_g3>=24));
    wire d_wink0=!d_ink0&&(d_link0||(d_b0&&!d_ctx0&&(d_g0>=12))||d_face0);
    wire d_wink1=!d_ink1&&(d_link1||(d_b1&&!d_ctx1&&(d_g1>=12))||d_face1);
    wire d_wink2=!d_ink2&&(d_link2||(d_b2&&!d_ctx2&&(d_g2>=12))||d_face2);
    wire d_wink3=!d_ink3&&(d_link3||(d_b3&&!d_ctx3&&(d_g3>=12))||d_face3);
    wire [10:0] d_x0={{(11-GROUP_ADDR_W-2){1'b0}},d_addr,2'b00};
    wire [10:0] d_x1=d_x0+1'b1,d_x2=d_x0+2'd2,d_x3=d_x0+2'd3;
    wire d_flat0=(d_g0<12)&&!d_b0&&!d_ctx0,d_flat1=(d_g1<12)&&!d_b1&&!d_ctx1;
    wire d_flat2=(d_g2<12)&&!d_b2&&!d_ctx2,d_flat3=(d_g3<12)&&!d_b3&&!d_ctx3;
    wire d_fib0=fibre_seed(d_x0,d_y)||((d_x0!=0)&&fibre_seed(d_x0-1'b1,d_y));
    wire d_fib1=fibre_seed(d_x1,d_y)||fibre_seed(d_x0,d_y);
    wire d_fib2=fibre_seed(d_x2,d_y)||fibre_seed(d_x1,d_y);
    wire d_fib3=fibre_seed(d_x3,d_y)||fibre_seed(d_x2,d_y);
    wire d_hat0=hatch_seed(d_x0,d_y)||((d_x0!=0)&&(d_y!=0)&&hatch_seed(d_x0-1'b1,d_y-1'b1));
    wire d_hat1=hatch_seed(d_x1,d_y)||((d_y!=0)&&hatch_seed(d_x0,d_y-1'b1));
    wire d_hat2=hatch_seed(d_x2,d_y)||((d_y!=0)&&hatch_seed(d_x1,d_y-1'b1));
    wire d_hat3=hatch_seed(d_x3,d_y)||((d_y!=0)&&hatch_seed(d_x2,d_y-1'b1));
    function dark_fill;
        input [3:0] p;
        begin dark_fill=(p==4'd6)||(p==4'd7)||(p==4'd13)||(p==4'd14); end
    endfunction
    function [3:0] textured_index;
        input [3:0] p;
        input flat,fibre,hatch,strong_ink,weak_ink;
        begin
            if(strong_ink) textured_index=4'd0;
            else if(weak_ink||(flat&&hatch&&dark_fill(p))) textured_index=4'd1;
            else if(flat&&fibre&&p==4'd2) textured_index=4'd3;
            else if(flat&&fibre&&p==4'd9) textured_index=4'd10;
            else if(flat&&fibre&&p==4'd11) textured_index=4'd12;
            else textured_index=p;
        end
    endfunction
    wire [3:0] d_r0=textured_index(d_p0,d_flat0,d_fib0,d_hat0,d_ink0,d_wink0);
    wire [3:0] d_r1=textured_index(d_p1,d_flat1,d_fib1,d_hat1,d_ink1,d_wink1);
    wire [3:0] d_r2=textured_index(d_p2,d_flat2,d_fib2,d_hat2,d_ink2,d_wink2);
    wire [3:0] d_r3=textured_index(d_p3,d_flat3,d_fib3,d_hat3,d_ink3,d_wink3);
    reg e_valid,e_sof;
    reg [15:0] e_palette;
    reg [3:0] e_flat,e_fibre,e_hatch,e_strong,e_weak;
    wire [3:0] e_r0=textured_index(e_palette[15:12],e_flat[3],e_fibre[3],e_hatch[3],e_strong[3],e_weak[3]);
    wire [3:0] e_r1=textured_index(e_palette[11:8],e_flat[2],e_fibre[2],e_hatch[2],e_strong[2],e_weak[2]);
    wire [3:0] e_r2=textured_index(e_palette[7:4],e_flat[1],e_fibre[1],e_hatch[1],e_strong[1],e_weak[1]);
    wire [3:0] e_r3=textured_index(e_palette[3:0],e_flat[0],e_fibre[0],e_hatch[0],e_strong[0],e_weak[0]);
    reg f_valid,f_sof;
    reg [15:0] f_index;

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            x_group<=0; y_pos<=0; last_raw<=0; last_s1<=0;
            a_valid<=0; b_valid<=0; b2_valid<=0; b3_valid<=0; b4_valid<=0;
            c_valid<=0; d_valid<=0; e_valid<=0; f_valid<=0; o_valid<=0;
            a_ink_style<=0;b_ink_style<=0;b2_ink_style<=0;b3_ink_style<=0;b4_ink_style<=0;
            c_ink_style<=0;d_ink_style<=0;e_ink_style<=0;f_ink_style<=0;
            a_sof<=0; b_sof<=0; b2_sof<=0; b3_sof<=0; b4_sof<=0;
            c_sof<=0; d_sof<=0; e_sof<=0; f_sof<=0; o_sof<=0; o_rgb<=0;
            e_palette<=0; e_flat<=0; e_fibre<=0; e_hatch<=0; e_strong<=0; e_weak<=0; f_index<=0;
            last_luma<=0; last_up_luma3<=0; last_p2<=0; last_p3<=0; last_up_p3<=0;
            last_w2<=0; last_w3<=0; last_up_w3<=0;
            last_clean3<=0; last_up_clean3<=0; last_skin3<=0; last_up_skin3<=0;
            last_strong3<=0; last_up_strong3<=0;
        end else begin
            a_valid<=i_valid; a_sof<=i_sof; a_ink_style<=i_ink_style;
            b_valid<=a_valid; b_sof<=a_sof; b_ink_style<=a_ink_style;
            b2_valid<=b_valid; b2_sof<=b_sof; b2_ink_style<=b_ink_style;
            b3_valid<=b2_valid; b3_sof<=b2_sof; b3_ink_style<=b2_ink_style;
            b4_valid<=b3_valid; b4_sof<=b3_sof; b4_ink_style<=b3_ink_style;
            c_valid<=b4_valid; c_sof<=b4_sof; c_ink_style<=b4_ink_style;
            d_valid<=c_valid; d_sof<=c_sof; d_ink_style<=c_ink_style;
            e_valid<=d_valid; e_sof<=d_sof; e_ink_style<=d_ink_style;
            f_valid<=e_valid; f_sof<=e_sof; f_ink_style<=e_ink_style;
            o_valid<=f_valid; o_sof<=f_sof;
            if(d_valid) begin
                e_palette<={d_p0,d_p1,d_p2,d_p3};
                e_flat<={d_flat0,d_flat1,d_flat2,d_flat3};
                e_fibre<={d_fib0,d_fib1,d_fib2,d_fib3};
                e_hatch<={d_hat0,d_hat1,d_hat2,d_hat3};
                e_strong<={d_ink0,d_ink1,d_ink2,d_ink3};
                e_weak<={d_wink0,d_wink1,d_wink2,d_wink3};
            end
            if(e_valid)
                f_index<={e_r0,e_r1,e_r2,e_r3};
            if(f_valid) begin
                if(f_ink_style)
                    o_rgb<={ink_palette_rgb(f_index[15:12]),ink_palette_rgb(f_index[11:8]),
                             ink_palette_rgb(f_index[7:4]),ink_palette_rgb(f_index[3:0])};
                else
                    o_rgb<={palette_rgb(f_index[15:12]),palette_rgb(f_index[11:8]),
                             palette_rgb(f_index[7:4]),palette_rgb(f_index[3:0])};
            end

            if(i_valid) begin
                a_addr<=input_addr; a_y<=input_y; a_rgb<=i_rgb;
                a_first_line<=(input_y==0); a_first_group<=(input_addr==0);
                a_up0<=raw_line0[input_addr]; a_up1<=raw_line1[input_addr];
                a_up2<=raw_line2[input_addr]; a_up3<=raw_line3[input_addr];
                raw_line0[input_addr]<=i_rgb[95:72]; raw_line1[input_addr]<=i_rgb[71:48];
                raw_line2[input_addr]<=i_rgb[47:24]; raw_line3[input_addr]<=i_rgb[23:0];
                a_left0<=(input_addr==0)?i_rgb[95:72]:last_raw; last_raw<=i_rgb[23:0];
                if(i_sof) begin x_group<=1; y_pos<=0; end
                else if(input_addr==LAST_GROUP) begin
                    x_group<=0; last_raw<=0; y_pos<=(input_y==IMG_HEIGHT-1)?0:input_y+1'b1;
                end else x_group<=input_addr+1'b1;
            end
            if(a_valid) begin
                b_addr<=a_addr; b_y<=a_y; b_s1<={a_s10,a_s11,a_s12,a_s13};
                b_first_line<=a_first_line; b_first_group<=a_first_group;
                b_up0<=s1_line0[a_addr]; b_up1<=s1_line1[a_addr]; b_up2<=s1_line2[a_addr]; b_up3<=s1_line3[a_addr];
                s1_line0[a_addr]<=a_s10; s1_line1[a_addr]<=a_s11; s1_line2[a_addr]<=a_s12; s1_line3[a_addr]<=a_s13;
                b_left0<=a_first_group?a_s10:last_s1; last_s1<=a_s13;
                if(a_addr==LAST_GROUP) last_s1<=0;
            end
            if(b_valid) begin
                b2_addr<=b_addr; b2_y<=b_y; b2_first_line<=b_first_line; b2_first_group<=b_first_group;
                b2_s2<={b_s20,b_s21,b_s22,b_s23};
            end
            if(b2_valid) begin
                b3_addr<=b2_addr; b3_y<=b2_y; b3_first_line<=b2_first_line; b3_first_group<=b2_first_group;
                b3_s2<=b2_s2; b3_luma<={b2_l0,b2_l1,b2_l2,b2_l3};
            end
            if(b3_valid) begin
                b4_addr<=b3_addr; b4_y<=b3_y; b4_first_line<=b3_first_line; b4_first_group<=b3_first_group;
                b4_luma<=b3_luma; b4_pal<={b3_p0,b3_p1,b3_p2,b3_p3};
                b4_warm<=b3_ink_style?4'b0000:{b3_w0,b3_w1,b3_w2,b3_w3};
            end
            if(b4_valid) begin
                c_addr<=b4_addr; c_y<=b4_y; c_first_line<=b4_first_line; c_first_group<=b4_first_group;
                c_luma<=b4_luma; c_pal<=b4_pal; c_warm<=b4_warm;
                c_up_luma<={luma_line0[b4_addr],luma_line1[b4_addr],luma_line2[b4_addr],luma_line3[b4_addr]};
                c_up_pal<={pal_line0[b4_addr],pal_line1[b4_addr],pal_line2[b4_addr],pal_line3[b4_addr]};
                c_up_warm<={warm_line0[b4_addr],warm_line1[b4_addr],warm_line2[b4_addr],warm_line3[b4_addr]};
                luma_line0[b4_addr]<=b4_luma[31:24]; luma_line1[b4_addr]<=b4_luma[23:16];
                luma_line2[b4_addr]<=b4_luma[15:8]; luma_line3[b4_addr]<=b4_luma[7:0];
                pal_line0[b4_addr]<=b4_pal[15:12]; pal_line1[b4_addr]<=b4_pal[11:8];
                pal_line2[b4_addr]<=b4_pal[7:4]; pal_line3[b4_addr]<=b4_pal[3:0];
                warm_line0[b4_addr]<=b4_warm[3]; warm_line1[b4_addr]<=b4_warm[2];
                warm_line2[b4_addr]<=b4_warm[1]; warm_line3[b4_addr]<=b4_warm[0];
                c_left_luma0<=b4_first_group?b4_luma[31:24]:last_luma;
                c_up_left_luma0<=b4_first_line?b4_luma[31:24]:(b4_first_group?luma_line0[b4_addr]:last_up_luma3);
                last_luma<=b4_luma[7:0]; last_up_luma3<=luma_line3[b4_addr];
                c_leftp1_0<=b4_first_group?b4_pal[15:12]:last_p3; c_leftp2_0<=b4_first_group?b4_pal[15:12]:last_p2;
                c_up_leftp0<=b4_first_group?b4_pal[15:12]:last_up_p3;
                last_p2<=b4_pal[7:4]; last_p3<=b4_pal[3:0]; last_up_p3<=pal_line3[b4_addr];
                c_leftw1_0<=b4_first_group?b4_warm[3]:last_w3; c_leftw2_0<=b4_first_group?b4_warm[3]:last_w2;
                c_up_leftw0<=b4_first_group?b4_warm[3]:last_up_w3;
                last_w2<=b4_warm[1]; last_w3<=b4_warm[0]; last_up_w3<=warm_line3[b4_addr];
                if(b4_addr==LAST_GROUP) begin last_luma<=0; last_up_luma3<=0; last_p2<=0; last_p3<=0; last_up_p3<=0; last_w2<=0; last_w3<=0; last_up_w3<=0; end
            end
            if(c_valid) begin
                d_addr<=c_addr; d_y<=c_y; d_first_line<=c_first_line; d_first_group<=c_first_group;
                d_clean<={c_m0,c_m1,c_m2,c_m3}; d_skin<={c_sw0,c_sw1,c_sw2,c_sw3};
                d_strong<={c_s0,c_s1,c_s2,c_s3}; d_grad<={c_g0,c_g1,c_g2,c_g3};
                d_luma<=c_luma; d_up_luma<=c_up_luma; d_left_luma0<=c_left_luma0; d_up_left_luma0<=c_up_left_luma0;
                d_up_clean<={clean_line0[c_addr],clean_line1[c_addr],clean_line2[c_addr],clean_line3[c_addr]};
                d_up_skin<={skin_line0[c_addr],skin_line1[c_addr],skin_line2[c_addr],skin_line3[c_addr]};
                d_up_strong<={strong_line0[c_addr],strong_line1[c_addr],strong_line2[c_addr],strong_line3[c_addr]};
                clean_line0[c_addr]<=c_m0; clean_line1[c_addr]<=c_m1; clean_line2[c_addr]<=c_m2; clean_line3[c_addr]<=c_m3;
                skin_line0[c_addr]<=c_sw0; skin_line1[c_addr]<=c_sw1; skin_line2[c_addr]<=c_sw2; skin_line3[c_addr]<=c_sw3;
                strong_line0[c_addr]<=c_s0; strong_line1[c_addr]<=c_s1; strong_line2[c_addr]<=c_s2; strong_line3[c_addr]<=c_s3;
                d_left_clean0<=c_first_group?c_m0:last_clean3; d_up_left_clean0<=c_first_group?c_m0:last_up_clean3;
                d_left_skin0<=c_first_group?c_sw0:last_skin3; d_up_left_skin0<=c_first_group?c_sw0:last_up_skin3;
                d_left_strong0<=c_first_group?c_s0:last_strong3; d_up_left_strong0<=c_first_group?c_s0:last_up_strong3;
                last_clean3<=c_m3; last_up_clean3<=clean_line3[c_addr];
                last_skin3<=c_sw3; last_up_skin3<=skin_line3[c_addr];
                last_strong3<=c_s3; last_up_strong3<=strong_line3[c_addr];
                if(c_addr==LAST_GROUP) begin
                    last_clean3<=0;last_up_clean3<=0;last_skin3<=0;last_up_skin3<=0;last_strong3<=0;last_up_strong3<=0;
                end
            end
        end
    end
endmodule
