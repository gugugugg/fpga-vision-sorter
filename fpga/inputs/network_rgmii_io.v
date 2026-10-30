`timescale 1ns/1ps
// HD-bank DDR conversion for all fixed PH1P35MDG324 bank22 network pins.
// TX_ER is zero: frozen GMII TX has no error input.
module network_rgmii_io(
    input i_tx_clk, i_tx_clk90, i_rx_clk, i_tx_rst_n, i_txc_rst_n, i_rx_rst_n,
    input i_tx_valid, input [7:0] i_tx_data,
    input [3:0] i_rgmii_rxd, input i_rgmii_rx_ctl,
    output [3:0] o_rgmii_txd, output o_rgmii_tx_ctl, o_rgmii_txc,
    output [7:0] o_rx_data, output o_rx_valid, o_rx_error
);
    wire [4:0] rx_rise, rx_fall;
    genvar n;
    generate for (n=0;n<4;n=n+1) begin: g_data
        PH1P_LOGIC_HD_ODDR #(.ASYNCRST("ENABLE")) tx_ddr(
            .clk(i_tx_clk), .rst(!i_tx_rst_n),
            .d0(i_tx_rst_n && i_tx_valid ? i_tx_data[n] : 1'b0),
            .d1(i_tx_rst_n && i_tx_valid ? i_tx_data[n+4] : 1'b0), .q(o_rgmii_txd[n]));
        PH1P_LOGIC_HD_IDDR #(.ASYNCRST("ENABLE"), .MODE("SAME_EDGE_PIPE")) rx_ddr(
            .clk(i_rx_clk), .rst(!i_rx_rst_n), .d(i_rgmii_rxd[n]),
            .q0(rx_rise[n]), .q1(rx_fall[n]));
    end endgenerate
    PH1P_LOGIC_HD_ODDR #(.ASYNCRST("ENABLE")) tx_ctl(
        .clk(i_tx_clk), .rst(!i_tx_rst_n), .d0(i_tx_valid && i_tx_rst_n),
        .d1(i_tx_valid && i_tx_rst_n), .q(o_rgmii_tx_ctl));
    PH1P_LOGIC_HD_ODDR #(.ASYNCRST("ENABLE")) tx_clock(
        .clk(i_tx_clk90), .rst(!i_txc_rst_n), .d0(1'b1), .d1(1'b0), .q(o_rgmii_txc));
    PH1P_LOGIC_HD_IDDR #(.ASYNCRST("ENABLE"), .MODE("SAME_EDGE_PIPE")) rx_ctl(
        .clk(i_rx_clk), .rst(!i_rx_rst_n), .d(i_rgmii_rx_ctl),
        .q0(rx_rise[4]), .q1(rx_fall[4]));
    // The HD IDDR output and fabric RX clock use different clock routes.
    // Capture the completed SAME_EDGE_PIPE pair on the falling fabric edge,
    // then register it on the rising edge. Both transfers have a half-cycle
    // timing budget; the MAC still has a full cycle for CRC and frame parsing.
    // Data and both control samples advance together, including idle/error.
    (* syn_preserve=1 *) reg [9:0] rx_half,rx_byte;
    always @(negedge i_rx_clk or negedge i_rx_rst_n)
        if(!i_rx_rst_n) rx_half<=0;
        else rx_half<={rx_fall,rx_rise};
    always @(posedge i_rx_clk or negedge i_rx_rst_n)
        if(!i_rx_rst_n) rx_byte<=0;
        else rx_byte<=rx_half;
    assign o_rx_data = i_rx_rst_n ? {rx_byte[8:5],rx_byte[3:0]} : 8'b0;
    assign o_rx_valid = i_rx_rst_n && rx_byte[4];
    assign o_rx_error = i_rx_rst_n && (rx_byte[4] ^ rx_byte[9]);
endmodule
