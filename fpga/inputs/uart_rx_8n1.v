`timescale 1ns/1ps

// Mid-bit sampled 8-N-1 UART receiver.  The input is synchronized internally;
// the external DAT pin must already be translated to the FPGA 3.3 V domain.
module uart_rx_8n1 #(
    parameter integer CLK_HZ = 50000000,
    parameter integer BAUD   = 115200
)(
    input  wire       i_clk,
    input  wire       i_rst_n,
    input  wire       i_rx,
    output reg        o_valid,
    output reg [7:0]  o_data,
    output reg        o_frame_error
);
    localparam integer CLKS_PER_BIT = (CLK_HZ + (BAUD/2)) / BAUD;
    localparam integer HALF_BIT     = CLKS_PER_BIT/2;
    localparam [1:0] RX_IDLE  = 2'd0;
    localparam [1:0] RX_START = 2'd1;
    localparam [1:0] RX_DATA  = 2'd2;
    localparam [1:0] RX_STOP  = 2'd3;

    reg rx_meta, rx_sync;
    reg [1:0] state;
    reg [2:0] bit_index;
    reg [7:0] shift_reg;
    integer bit_counter;

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= i_rx;
            rx_sync <= rx_meta;
        end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            state <= RX_IDLE;
            bit_index <= 3'd0;
            shift_reg <= 8'd0;
            bit_counter <= 0;
            o_valid <= 1'b0;
            o_data <= 8'd0;
            o_frame_error <= 1'b0;
        end else begin
            o_valid <= 1'b0;
            o_frame_error <= 1'b0;
            case(state)
                RX_IDLE: begin
                    if(!rx_sync) begin
                        bit_counter <= HALF_BIT-1;
                        state <= RX_START;
                    end
                end
                RX_START: begin
                    if(bit_counter == 0) begin
                        if(!rx_sync) begin
                            bit_index <= 3'd0;
                            bit_counter <= CLKS_PER_BIT-1;
                            state <= RX_DATA;
                        end else begin
                            state <= RX_IDLE;
                        end
                    end else begin
                        bit_counter <= bit_counter-1;
                    end
                end
                RX_DATA: begin
                    if(bit_counter == 0) begin
                        shift_reg[bit_index] <= rx_sync;
                        bit_counter <= CLKS_PER_BIT-1;
                        if(bit_index == 3'd7)
                            state <= RX_STOP;
                        else
                            bit_index <= bit_index + 1'b1;
                    end else begin
                        bit_counter <= bit_counter-1;
                    end
                end
                default: begin // RX_STOP
                    if(bit_counter == 0) begin
                        if(rx_sync) begin
                            o_data <= shift_reg;
                            o_valid <= 1'b1;
                        end else begin
                            o_frame_error <= 1'b1;
                        end
                        state <= RX_IDLE;
                    end else begin
                        bit_counter <= bit_counter-1;
                    end
                end
            endcase
        end
    end
endmodule
