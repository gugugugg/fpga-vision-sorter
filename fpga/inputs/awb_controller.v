`timescale 1ns/1ps

// Frame-latched gray-world AWB. One divider is time-shared by R and B.
module awb_controller #(
    parameter MIN_PIXELS = 4096
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_stats_valid,
    input  wire [31:0] i_r_sum,
    input  wire [31:0] i_g_sum,
    input  wire [31:0] i_b_sum,
    input  wire [19:0] i_pixel_count,
    output reg  [9:0]  o_r_gain_q8,
    output reg  [9:0]  o_b_gain_q8,
    output reg         o_gain_valid
);
    localparam ST_IDLE    = 3'd0;
    localparam ST_START_R = 3'd1;
    localparam ST_WAIT_R  = 3'd2;
    localparam ST_START_B = 3'd3;
    localparam ST_WAIT_B  = 3'd4;

    reg [2:0] state;
    reg [31:0] r_sum;
    reg [31:0] g_sum;
    reg [31:0] b_sum;
    reg [9:0] target_r;
    reg div_start;
    reg [39:0] div_num;
    reg [31:0] div_den;
    wire [39:0] div_q;
    wire div_done;

    function [9:0] clamp_gain;
        input [39:0] q;
        begin
            if (q < 40'd128)
                clamp_gain = 10'd128;
            else if (q > 40'd512)
                clamp_gain = 10'd512;
            else
                clamp_gain = q[9:0];
        end
    endfunction

    function [9:0] approach_gain;
        input [9:0] current,target;
        reg [10:0] distance;
        reg [9:0] step;
        begin
            // Symmetric 1/4 IIR with a minimum one-code step. The former
            // floor((3*current+target)/4) stalls up to three codes below
            // target and biases neutral subjects towards green.
            distance=current>target?current-target:target-current;
            step=(distance+11'd3)>>2;
            approach_gain=current>target?current-step:current+step;
        end
    endfunction

    seq_div_u40_u32 u_div(
        .i_clk(i_clk), .i_rst_n(i_rst_n), .i_start(div_start),
        .i_numerator(div_num), .i_denominator(div_den),
        .o_quotient(div_q), .o_busy(), .o_done(div_done)
    );

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state        <= ST_IDLE;
            r_sum        <= 0;
            g_sum        <= 0;
            b_sum        <= 0;
            target_r     <= 10'd256;
            div_start    <= 1'b0;
            div_num      <= 0;
            div_den      <= 1;
            o_r_gain_q8  <= 10'd256;
            o_b_gain_q8  <= 10'd256;
            o_gain_valid <= 1'b0;
        end else begin
            div_start    <= 1'b0;
            o_gain_valid <= 1'b0;
            case (state)
                ST_IDLE: begin
                    if (i_stats_valid && i_pixel_count >= MIN_PIXELS
                        && i_r_sum != 0 && i_g_sum != 0 && i_b_sum != 0) begin
                        r_sum <= i_r_sum;
                        g_sum <= i_g_sum;
                        b_sum <= i_b_sum;
                        state <= ST_START_R;
                    end
                end
                ST_START_R: begin
                    div_num   <= {g_sum, 8'b0};
                    div_den   <= r_sum;
                    div_start <= 1'b1;
                    state     <= ST_WAIT_R;
                end
                ST_WAIT_R: begin
                    if (div_done) begin
                        target_r <= clamp_gain(div_q);
                        state    <= ST_START_B;
                    end
                end
                ST_START_B: begin
                    div_num   <= {g_sum, 8'b0};
                    div_den   <= b_sum;
                    div_start <= 1'b1;
                    state     <= ST_WAIT_B;
                end
                ST_WAIT_B: begin
                    if (div_done) begin
                        // 1/4 IIR update suppresses frame-to-frame colour pumping.
                        o_r_gain_q8  <= approach_gain(o_r_gain_q8,target_r);
                        o_b_gain_q8  <= approach_gain(o_b_gain_q8,clamp_gain(div_q));
                        o_gain_valid <= 1'b1;
                        state        <= ST_IDLE;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule
