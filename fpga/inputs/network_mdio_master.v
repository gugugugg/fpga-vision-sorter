`timescale 1ns/1ps
// Original Clause 22 controller. Full 32-bit preamble on every transaction.
// MDC <= 1 MHz at default 50 MHz sys clock; MDIO is open drain.
module network_mdio_master #(
    parameter integer HALF_PERIOD = 25
)(
    input i_clk, i_rst_n, i_start, i_read,
    input [4:0] i_phy_addr, i_reg_addr,
    input [15:0] i_write_data,
    output reg o_busy, o_done, o_error,
    output reg [15:0] o_read_data,
    output reg o_mdc,
    inout io_mdio
);
    reg [63:0] frame;
    reg read_transaction;
    reg [5:0] bit_number;
    integer divider;
    reg mdio_meta, mdio_sample;
    wire release_read = read_transaction && bit_number >= 46;
    wire pull_low = o_busy && !release_read && !frame[63-bit_number];
    assign io_mdio = pull_low ? 1'b0 : 1'bz;
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin mdio_meta <= 1'b1; mdio_sample <= 1'b1; end
        else begin mdio_meta <= io_mdio; mdio_sample <= mdio_meta; end
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_busy <= 0; o_done <= 0; o_error <= 0; o_mdc <= 0;
            o_read_data <= 0; frame <= 0; read_transaction <= 0;
            bit_number <= 0; divider <= 0;
        end else begin
            o_done <= 0;
            if (!o_busy) begin
                o_mdc <= 0; divider <= 0;
                if (i_start) begin
                    frame <= {32'hffffffff, 2'b01, (i_read ? 2'b10 : 2'b01),
                              i_phy_addr, i_reg_addr, 2'b10, i_write_data};
                    read_transaction <= i_read; bit_number <= 0;
                    o_read_data <= 0; o_error <= 0; o_busy <= 1;
                end
            end else if (divider == HALF_PERIOD-1) begin
                divider <= 0;
                o_mdc <= !o_mdc;
                if (!o_mdc) begin
                    if (read_transaction && bit_number == 47 && mdio_sample != 1'b0)
                        o_error <= 1;
                    if (read_transaction && bit_number >= 48)
                        o_read_data <= {o_read_data[14:0], mdio_sample};
                end else if (bit_number == 63) begin
                    o_busy <= 0; o_done <= 1;
                end else bit_number <= bit_number + 1'b1;
            end else divider <= divider + 1;
        end
    end
endmodule
