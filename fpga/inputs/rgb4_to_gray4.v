`timescale 1ns/1ps

// Four RGB888 pixels to four BT.601-like luma samples.
// Pixel order follows the official 96-bit stream: pixel 0 is [95:72].
module rgb4_to_gray4 (
    input  wire [95:0] i_rgb,
    output wire [31:0] o_gray
);
    function [7:0] rgb_to_gray;
        input [23:0] rgb;
        reg [15:0] r16;
        reg [15:0] g16;
        reg [15:0] b16;
        reg [15:0] weighted;
        begin
            // Exact BT.601 integer coefficients, written as shifts/adds so
            // constant luma conversion does not consume scarce DSP blocks.
            r16 = {8'd0,rgb[23:16]};
            g16 = {8'd0,rgb[15:8]};
            b16 = {8'd0,rgb[7:0]};
            weighted = (r16 << 6) + (r16 << 3) + (r16 << 2) + r16
                     + (g16 << 7) + (g16 << 4) + (g16 << 2) + (g16 << 1)
                     + (b16 << 4) + (b16 << 3) + (b16 << 2) + b16;
            rgb_to_gray = weighted[15:8];
        end
    endfunction

    assign o_gray[31:24] = rgb_to_gray(i_rgb[95:72]);
    assign o_gray[23:16] = rgb_to_gray(i_rgb[71:48]);
    assign o_gray[15:8]  = rgb_to_gray(i_rgb[47:24]);
    assign o_gray[7:0]   = rgb_to_gray(i_rgb[23:0]);
endmodule
