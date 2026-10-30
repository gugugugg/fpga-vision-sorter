`timescale 1ns/1ps

// Applies one frame-stable R/B gain to all four lanes (no per-lane colour stripes).
module awb_apply_4ppc (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_sof,
    input  wire        i_valid,
    input  wire [95:0] i_rgb,
    input  wire [9:0]  i_r_gain_q8,
    input  wire [9:0]  i_b_gain_q8,
    output reg         o_sof,
    output reg         o_valid,
    output reg  [95:0] o_rgb
);
    reg [9:0] active_r_gain;
    reg [9:0] active_b_gain;
    wire [9:0] use_r_gain = i_sof ? i_r_gain_q8 : active_r_gain;
    wire [9:0] use_b_gain = i_sof ? i_b_gain_q8 : active_b_gain;
    function [7:0] scale_sat;
        input [7:0] channel;
        input [9:0] gain;
        reg [17:0] product;
        begin
            product = channel * gain;
            if (product[17:8] > 10'd255)
                scale_sat = 8'hff;
            else
                scale_sat = product[15:8];
        end
    endfunction

    function [23:0] apply_pixel;
        input [23:0] p;
        input [9:0] rg;
        input [9:0] bg;
        begin
            apply_pixel = {scale_sat(p[23:16], rg), p[15:8],
                           scale_sat(p[7:0], bg)};
        end
    endfunction

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            active_r_gain <= 10'd256;
            active_b_gain <= 10'd256;
            o_sof   <= 1'b0;
            o_valid <= 1'b0;
            o_rgb   <= 0;
        end else begin
            if (i_sof) begin
                active_r_gain <= i_r_gain_q8;
                active_b_gain <= i_b_gain_q8;
            end
            o_sof   <= i_sof;
            o_valid <= i_valid;
            if (i_valid) begin
                o_rgb <= {apply_pixel(i_rgb[95:72], use_r_gain, use_b_gain),
                          apply_pixel(i_rgb[71:48], use_r_gain, use_b_gain),
                          apply_pixel(i_rgb[47:24], use_r_gain, use_b_gain),
                          apply_pixel(i_rgb[23:0],  use_r_gain, use_b_gain)};
            end
        end
    end
endmodule
