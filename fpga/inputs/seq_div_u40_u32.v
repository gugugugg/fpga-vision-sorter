`timescale 1ns/1ps

// One small sequential divider reused for frame-level control calculations.
// The 40-cycle latency is negligible in the vertical blanking interval.
module seq_div_u40_u32 (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_start,
    input  wire [39:0] i_numerator,
    input  wire [31:0] i_denominator,
    output reg  [39:0] o_quotient,
    output reg         o_busy,
    output reg         o_done
);
    reg [39:0] numerator_shift;
    reg [39:0] quotient_work;
    reg [32:0] remainder;
    reg [31:0] denominator;
    reg [5:0]  count;

    wire [32:0] remainder_shift = {remainder[31:0], numerator_shift[39]};
    wire ge_denominator = remainder_shift >= {1'b0, denominator};
    wire [32:0] remainder_next = ge_denominator
                                ? remainder_shift - {1'b0, denominator}
                                : remainder_shift;
    wire [39:0] quotient_next = {quotient_work[38:0], ge_denominator};

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            numerator_shift <= 0;
            quotient_work   <= 0;
            remainder       <= 0;
            denominator     <= 0;
            count           <= 0;
            o_quotient      <= 0;
            o_busy          <= 1'b0;
            o_done          <= 1'b0;
        end else begin
            o_done <= 1'b0;
            if (i_start && !o_busy) begin
                if (i_denominator == 0) begin
                    o_quotient <= {40{1'b1}};
                    o_done     <= 1'b1;
                end else begin
                    numerator_shift <= i_numerator;
                    quotient_work   <= 0;
                    remainder       <= 0;
                    denominator     <= i_denominator;
                    count           <= 6'd40;
                    o_busy          <= 1'b1;
                end
            end else if (o_busy) begin
                numerator_shift <= {numerator_shift[38:0], 1'b0};
                remainder       <= remainder_next;
                quotient_work   <= quotient_next;
                count           <= count - 1'b1;
                if (count == 1) begin
                    o_quotient <= quotient_next;
                    o_busy     <= 1'b0;
                    o_done     <= 1'b1;
                end
            end
        end
    end
endmodule

