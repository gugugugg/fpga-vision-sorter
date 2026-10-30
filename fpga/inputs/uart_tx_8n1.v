`timescale 1ns/1ps

// Minimal 8-N-1 UART transmitter.  The baud divider is rounded to the
// nearest system-clock cycle, which gives 434 clocks/bit at 50 MHz/115200.
module uart_tx_8n1 #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD   = 115200
)(
    input  wire       i_clk,
    input  wire       i_rst_n,
    input  wire       i_valid,
    input  wire [7:0] i_data,
    output wire       o_ready,
    output reg        o_busy,
    output reg        o_tx
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + (BAUD/2)) / BAUD;

    reg [9:0] shift_reg;
    reg [3:0] bit_index;
    integer bit_counter;

    assign o_ready = ~o_busy;

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            shift_reg  <= 10'h3ff;
            bit_index  <= 4'd0;
            bit_counter <= 0;
            o_busy <= 1'b0;
            o_tx   <= 1'b1;
        end else if(!o_busy) begin
            o_tx <= 1'b1;
            if(i_valid) begin
                // LSB first: start, eight data bits, stop.
                shift_reg <= {1'b1,i_data,1'b0};
                bit_index <= 4'd0;
                bit_counter <= CLKS_PER_BIT-1;
                o_busy <= 1'b1;
                o_tx <= 1'b0;
            end
        end else if(bit_counter == 0) begin
            if(bit_index == 4'd9) begin
                o_busy <= 1'b0;
                o_tx <= 1'b1;
            end else begin
                bit_index <= bit_index + 1'b1;
                bit_counter <= CLKS_PER_BIT-1;
                o_tx <= shift_reg[bit_index+1'b1];
            end
        end else begin
            bit_counter <= bit_counter-1;
        end
    end
endmodule
