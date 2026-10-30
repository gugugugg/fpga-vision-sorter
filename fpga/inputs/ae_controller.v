`timescale 1ns/1ps

// Sensor-independent, low-bandwidth exposure-first control policy.
// Board integration supplies sensor-specific exposure and gain limits.
module ae_controller #(
    parameter TARGET_LUMA   = 110,
    parameter DEAD_BAND     = 10,
    parameter UPDATE_FRAMES = 4,
    parameter EXP_INIT      = 1500,
    parameter EXP_MIN       = 32,
    parameter EXP_MAX       = 2246,
    parameter EXP_STEP      = 32,
    parameter GAIN_INIT     = 16,
    parameter GAIN_MIN      = 16,
    parameter GAIN_MAX      = 89
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_enable,
    input  wire        i_stats_valid,
    input  wire [31:0] i_luma_sum,
    input  wire [19:0] i_luma_count,
    output reg  [15:0] o_exposure,
    output reg  [15:0] o_gain_index,
    output reg         o_update,
    output reg         o_stable
);
    reg [3:0] frame_div;
    wire [31:0] low_limit  = i_luma_count * (TARGET_LUMA-DEAD_BAND);
    wire [31:0] high_limit = i_luma_count * (TARGET_LUMA+DEAD_BAND);

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            frame_div    <= 0;
            o_exposure   <= EXP_INIT;
            o_gain_index <= GAIN_INIT;
            o_update     <= 1'b0;
            o_stable     <= 1'b0;
        end else begin
            o_update <= 1'b0;
            if (!i_enable) begin
                frame_div <= 0;
                o_stable  <= 1'b0;
            end else if (i_stats_valid && i_luma_count != 0) begin
                if (frame_div == UPDATE_FRAMES-1) begin
                    frame_div <= 0;
                    if (i_luma_sum < low_limit) begin
                        o_stable <= 1'b0;
                        if (o_exposure < EXP_MAX) begin
                            if (o_exposure + EXP_STEP > EXP_MAX)
                                o_exposure <= EXP_MAX;
                            else
                                o_exposure <= o_exposure + EXP_STEP;
                            o_update <= 1'b1;
                        end else if (o_gain_index < GAIN_MAX) begin
                            o_gain_index <= o_gain_index + 1'b1;
                            o_update <= 1'b1;
                        end
                    end else if (i_luma_sum > high_limit) begin
                        o_stable <= 1'b0;
                        if (o_gain_index > GAIN_MIN) begin
                            o_gain_index <= o_gain_index - 1'b1;
                            o_update <= 1'b1;
                        end else if (o_exposure > EXP_MIN) begin
                            if (o_exposure < EXP_MIN+EXP_STEP)
                                o_exposure <= EXP_MIN;
                            else
                                o_exposure <= o_exposure - EXP_STEP;
                            o_update <= 1'b1;
                        end
                    end else begin
                        o_stable <= 1'b1;
                    end
                end else begin
                    frame_div <= frame_div + 1'b1;
                end
            end
        end
    end
endmodule
