`timescale 1ns/1ps

// Crop a 1280x720 RAW10 stream to the existing 1024x600 4PPC ISP raster.
// Both offsets are even Bayer coordinates; horizontal offset is 32 beats.
// The source has no backpressure, so valid/tuser/tlast are gated together.
module raw10_center_crop_4ppc #(
    parameter integer INPUT_WIDTH = 1280,
    parameter integer INPUT_HEIGHT = 720,
    parameter integer OUTPUT_WIDTH = 1024,
    parameter integer OUTPUT_HEIGHT = 600
)(
    input wire i_clk,
    input wire i_rst_n,
    input wire i_valid,
    input wire i_sof,
    input wire i_last,
    input wire [39:0] i_data,
    output wire o_valid,
    output wire o_sof,
    output wire o_last,
    output wire [39:0] o_data
);
    localparam integer INPUT_BEATS = INPUT_WIDTH / 4;
    localparam integer OUTPUT_BEATS = OUTPUT_WIDTH / 4;
    localparam integer X_OFFSET = (INPUT_BEATS - OUTPUT_BEATS) / 2;
    localparam integer Y_OFFSET = (INPUT_HEIGHT - OUTPUT_HEIGHT) / 2;
    reg [9:0] x_beat;
    reg [9:0] y_line;
    wire [9:0] x_now = i_sof ? 10'd0 : x_beat;
    wire [9:0] y_now = i_sof ? 10'd0 : y_line;
    wire in_window = (x_now >= X_OFFSET) && (x_now < X_OFFSET + OUTPUT_BEATS) &&
                     (y_now >= Y_OFFSET) && (y_now < Y_OFFSET + OUTPUT_HEIGHT);

    assign o_valid = i_valid && in_window;
    assign o_sof = o_valid && (x_now == X_OFFSET) && (y_now == Y_OFFSET);
    assign o_last = o_valid && (x_now == X_OFFSET + OUTPUT_BEATS - 1);
    assign o_data = i_data;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            x_beat <= 10'd0;
            y_line <= 10'd0;
        end else if (i_valid) begin
            if (i_last) begin
                x_beat <= 10'd0;
                y_line <= i_sof ? 10'd1 :
                          (y_now == INPUT_HEIGHT - 1 ? 10'd0 : y_now + 1'b1);
            end else begin
                x_beat <= x_now + 1'b1;
                if (i_sof)
                    y_line <= 10'd0;
            end
        end else if (i_sof) begin
            x_beat <= 10'd0;
            y_line <= 10'd0;
        end
    end
endmodule
