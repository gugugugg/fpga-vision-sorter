`timescale 1ns/1ps

// Frame statistics shared by automatic exposure and automatic white balance.
// No back-pressure is used. A frame is exactly IMG_WIDTH*IMG_HEIGHT pixels.
module vision_stats_4ppc #(
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600,
    parameter PREFER_NEUTRAL_AWB = 0,
    parameter MIN_NEUTRAL_PIXELS = 4096
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_sof,
    input  wire        i_valid,
    input  wire [95:0] i_rgb,
    // Already balanced reference, aligned to the same four raw RGB pixels.
    // Used only for selection; gain targets still come from pre-AWB sums.
    input  wire [95:0] i_neutral_rgb,

    output reg         o_stats_valid,
    output reg  [31:0] o_luma_sum,
    output reg  [19:0] o_luma_count,
    output reg  [31:0] o_r_sum,
    output reg  [31:0] o_g_sum,
    output reg  [31:0] o_b_sum,
    output reg  [19:0] o_awb_count
);
    localparam GROUPS_PER_LINE = IMG_WIDTH / 4;

    wire [23:0] p0 = i_rgb[95:72];
    wire [23:0] p1 = i_rgb[71:48];
    wire [23:0] p2 = i_rgb[47:24];
    wire [23:0] p3 = i_rgb[23:0];
    wire [31:0] gray;

    rgb4_to_gray4 u_gray(.i_rgb(i_rgb), .o_gray(gray));

    function accept_awb;
        input [23:0] p;
        reg [7:0] maxc;
        reg [7:0] minc;
        begin
            maxc = (p[23:16] > p[15:8]) ? p[23:16] : p[15:8];
            if (p[7:0] > maxc) maxc = p[7:0];
            minc = (p[23:16] < p[15:8]) ? p[23:16] : p[15:8];
            if (p[7:0] < minc) minc = p[7:0];
            // Reject black-level and saturated pixels.
            accept_awb = (maxc < 8'd245) && (minc > 8'd8);
        end
    endfunction

    function accept_neutral;
        input [23:0] p;
        reg [7:0] maxc,minc;
        begin
            maxc=p[23:16]>p[15:8]?p[23:16]:p[15:8];
            maxc=p[7:0]>maxc?p[7:0]:maxc;
            minc=p[23:16]<p[15:8]?p[23:16]:p[15:8];
            minc=p[7:0]<minc?p[7:0]:minc;
            accept_neutral=maxc>=64 && maxc<245 && maxc-minc<=(maxc>>3);
        end
    endfunction

    wire a0 = accept_awb(p0);
    wire a1 = accept_awb(p1);
    wire a2 = accept_awb(p2);
    wire a3 = accept_awb(p3);

    wire n0=PREFER_NEUTRAL_AWB && a0 && accept_neutral(i_neutral_rgb[95:72]);
    wire n1=PREFER_NEUTRAL_AWB && a1 && accept_neutral(i_neutral_rgb[71:48]);
    wire n2=PREFER_NEUTRAL_AWB && a2 && accept_neutral(i_neutral_rgb[47:24]);
    wire n3=PREFER_NEUTRAL_AWB && a3 && accept_neutral(i_neutral_rgb[23:0]);
    // Selection and addition occupy separate clocks. The former placement
    // combined min/max, colour-range subtraction and three serial additions
    // in the 112-MHz byte clock, failing routed setup by 55 ps.
    reg [95:0] p_rgb;
    reg [3:0] p_awb_accept,p_neutral_accept;
    function [10:0] selected_sum4;
        input [7:0] v0,v1,v2,v3;
        input [3:0] accept;
        reg [8:0] pair01,pair23;
        begin
            pair01={1'b0,(accept[0]?v0:8'd0)}+{1'b0,(accept[1]?v1:8'd0)};
            pair23={1'b0,(accept[2]?v2:8'd0)}+{1'b0,(accept[3]?v3:8'd0)};
            selected_sum4={2'd0,pair01}+{2'd0,pair23};
        end
    endfunction
    wire [10:0] neutral_r_add=selected_sum4(p_rgb[95:88],p_rgb[71:64],p_rgb[47:40],p_rgb[23:16],p_neutral_accept);
    wire [10:0] neutral_g_add=selected_sum4(p_rgb[87:80],p_rgb[63:56],p_rgb[39:32],p_rgb[15:8],p_neutral_accept);
    wire [10:0] neutral_b_add=selected_sum4(p_rgb[79:72],p_rgb[55:48],p_rgb[31:24],p_rgb[7:0],p_neutral_accept);
    wire [2:0] neutral_n_add={2'd0,p_neutral_accept[0]}+{2'd0,p_neutral_accept[1]}
                           +{2'd0,p_neutral_accept[2]}+{2'd0,p_neutral_accept[3]};
    wire [10:0] awb_r_add=selected_sum4(p_rgb[95:88],p_rgb[71:64],p_rgb[47:40],p_rgb[23:16],p_awb_accept);
    wire [10:0] awb_g_add=selected_sum4(p_rgb[87:80],p_rgb[63:56],p_rgb[39:32],p_rgb[15:8],p_awb_accept);
    wire [10:0] awb_b_add=selected_sum4(p_rgb[79:72],p_rgb[55:48],p_rgb[31:24],p_rgb[7:0],p_awb_accept);
    wire [2:0] awb_n_add={2'd0,p_awb_accept[0]}+{2'd0,p_awb_accept[1]}
                       +{2'd0,p_awb_accept[2]}+{2'd0,p_awb_accept[3]};
    reg        p_valid;
    reg        p_sof;
    reg        p_ae_sample;
    reg [31:0] p_gray;
    wire [9:0] p_luma_add = {2'd0,p_gray[31:24]} + {2'd0,p_gray[23:16]}
                           + {2'd0,p_gray[15:8]} + {2'd0,p_gray[7:0]};

    reg [10:0] x_group;
    reg [9:0]  y_pos;
    reg        frame_seen;
    reg [31:0] luma_sum;
    reg [19:0] luma_count;
    reg [31:0] r_sum;
    reg [31:0] g_sum;
    reg [31:0] b_sum;
    reg [19:0] awb_count;
    reg [31:0] neutral_r_sum,neutral_g_sum,neutral_b_sum;
    reg [19:0] neutral_count;
    wire use_neutral=PREFER_NEUTRAL_AWB && neutral_count>=MIN_NEUTRAL_PIXELS;

    // Register the four-pixel reductions before the 32-bit frame
    // accumulators. This isolates RGB/luma arithmetic from the carry chain
    // and is required by the board's 112-MHz MIPI byte clock.
    reg        s_valid;
    reg        s_sof;
    reg        s_ae_sample;
    reg [9:0]  s_luma_add;
    reg [10:0] s_awb_r_add, s_awb_g_add, s_awb_b_add;
    reg [2:0]  s_awb_n_add;
    reg [10:0] s_neutral_r_add,s_neutral_g_add,s_neutral_b_add;
    reg [2:0] s_neutral_n_add;

    // Sample one group out of four and one row out of four for AE.
    wire ae_sample = (x_group[1:0] == 2'b00) && (y_pos[1:0] == 2'b00);

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            x_group      <= 0;
            y_pos        <= 0;
            frame_seen   <= 1'b0;
            luma_sum     <= 0;
            luma_count   <= 0;
            r_sum        <= 0;
            g_sum        <= 0;
            b_sum        <= 0;
            awb_count    <= 0;
            neutral_r_sum<=0;neutral_g_sum<=0;neutral_b_sum<=0;neutral_count<=0;
            p_rgb<=0;p_awb_accept<=0;p_neutral_accept<=0;
            s_neutral_r_add<=0;s_neutral_g_add<=0;s_neutral_b_add<=0;s_neutral_n_add<=0;
            p_valid      <= 1'b0;
            p_sof        <= 1'b0;
            p_ae_sample  <= 1'b0;
            p_gray       <= 0;
            s_valid      <= 1'b0;
            s_sof        <= 1'b0;
            s_ae_sample  <= 1'b0;
            s_luma_add   <= 0;
            s_awb_r_add  <= 0;
            s_awb_g_add  <= 0;
            s_awb_b_add  <= 0;
            s_awb_n_add  <= 0;
            o_stats_valid<= 1'b0;
            o_luma_sum   <= 0;
            o_luma_count <= 0;
            o_r_sum      <= 0;
            o_g_sum      <= 0;
            o_b_sum      <= 0;
            o_awb_count  <= 0;
        end else begin
            o_stats_valid <= 1'b0;

            p_valid <= i_valid;
            p_sof <= i_sof;
            if (i_valid) begin
                p_gray <= gray;
                p_rgb<=i_rgb;
                p_awb_accept<={a3,a2,a1,a0};
                p_neutral_accept<={n3,n2,n1,n0};
                p_ae_sample <= ae_sample;

                if (i_sof) begin
                    x_group <= 1;
                    y_pos <= 0;
                end else if (x_group == GROUPS_PER_LINE-1) begin
                    x_group <= 0;
                    if (y_pos == IMG_HEIGHT-1)
                        y_pos <= 0;
                    else
                        y_pos <= y_pos + 1'b1;
                end else begin
                    x_group <= x_group + 1'b1;
                end
            end

            s_valid <= p_valid;
            s_sof <= p_sof;
            if (p_valid) begin
                s_luma_add <= p_luma_add;
                s_awb_r_add <= awb_r_add;
                s_awb_g_add <= awb_g_add;
                s_awb_b_add <= awb_b_add;
                s_awb_n_add <= awb_n_add;
                s_neutral_r_add<=neutral_r_add;s_neutral_g_add<=neutral_g_add;
                s_neutral_b_add<=neutral_b_add;s_neutral_n_add<=neutral_n_add;
                s_ae_sample <= p_ae_sample;
            end

            if (s_valid) begin
                if (s_sof) begin
                    if (frame_seen) begin
                        o_luma_sum    <= luma_sum;
                        o_luma_count  <= luma_count;
                        o_r_sum       <= use_neutral?neutral_r_sum:r_sum;
                        o_g_sum       <= use_neutral?neutral_g_sum:g_sum;
                        o_b_sum       <= use_neutral?neutral_b_sum:b_sum;
                        o_awb_count   <= use_neutral?neutral_count:awb_count;
                        o_stats_valid <= 1'b1;
                    end
                    frame_seen <= 1'b1;
                    luma_sum   <= {22'd0,s_luma_add};
                    luma_count <= 4;
                    r_sum      <= {21'd0,s_awb_r_add};
                    g_sum      <= {21'd0,s_awb_g_add};
                    b_sum      <= {21'd0,s_awb_b_add};
                    awb_count  <= {17'd0,s_awb_n_add};
                    neutral_r_sum<={21'd0,s_neutral_r_add};
                    neutral_g_sum<={21'd0,s_neutral_g_add};
                    neutral_b_sum<={21'd0,s_neutral_b_add};
                    neutral_count<={17'd0,s_neutral_n_add};
                end else begin
                    if (s_ae_sample) begin
                        luma_sum   <= luma_sum + {22'd0,s_luma_add};
                        luma_count <= luma_count + 4;
                    end
                    r_sum     <= r_sum + {21'd0,s_awb_r_add};
                    g_sum     <= g_sum + {21'd0,s_awb_g_add};
                    b_sum     <= b_sum + {21'd0,s_awb_b_add};
                    awb_count <= awb_count + {17'd0,s_awb_n_add};
                    neutral_r_sum<=neutral_r_sum+{21'd0,s_neutral_r_add};
                    neutral_g_sum<=neutral_g_sum+{21'd0,s_neutral_g_add};
                    neutral_b_sum<=neutral_b_sum+{21'd0,s_neutral_b_add};
                    neutral_count<=neutral_count+{17'd0,s_neutral_n_add};
                end
            end
        end
    end
endmodule
