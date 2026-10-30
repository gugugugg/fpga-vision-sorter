`timescale 1ns/1ps
// Frozen contract. o_ready is qualified gigabit full-duplex link in o_clk
// domain, AFTER initialization/readback; never just PLL lock or init complete.
module network_rgmii_phy #(
    parameter [4:0] PHY_ADDR = 5'd4,
    parameter integer RESET_CYCLES = 1000000,
    parameter integer SETTLE_CYCLES = 1000000,
    parameter integer POLL_CYCLES = 500000,
    parameter integer MDIO_HALF_PERIOD = 25
)(
    input i_sys_clk, i_rst_n,
    input [3:0] i_rgmii_rxd,
    input i_rgmii_rx_ctl, i_rgmii_rxc,
    output [3:0] o_rgmii_txd,
    output o_rgmii_tx_ctl, o_rgmii_txc, o_phy_rst_n, o_mdc,
    inout io_mdio,
    output o_clk, o_rx_clk, o_ready,
    output o_rx_valid, o_rx_error,
    output [7:0] o_rx_data,
    input i_tx_valid,
    input [7:0] i_tx_data
);
    wire clk90, pll_locked, sys_rst_n, init_done, link_gigabit, fault;
    wire tx_rst_n, txc_rst_n, rx_rst_n;
    network_phy_pll u_pll(.i_refclk(i_sys_clk), .i_rst_n(i_rst_n),
        .o_clk(o_clk), .o_clk90(clk90), .o_locked(pll_locked));
    network_reset_sync sys_reset(.i_clk(i_sys_clk),
        .i_async_rst_n(i_rst_n && pll_locked), .o_rst_n(sys_rst_n));
    network_phy_init #(.PHY_ADDR(PHY_ADDR), .RESET_CYCLES(RESET_CYCLES),
        .SETTLE_CYCLES(SETTLE_CYCLES), .POLL_CYCLES(POLL_CYCLES),
        .MDIO_HALF_PERIOD(MDIO_HALF_PERIOD)) u_init(
        .i_clk(i_sys_clk), .i_rst_n(sys_rst_n), .o_phy_rst_n(o_phy_rst_n),
        .o_mdc(o_mdc), .io_mdio(io_mdio), .o_init_done(init_done),
        .o_link_gigabit(link_gigabit), .o_fault(fault));
    wire qualified = sys_rst_n && init_done && link_gigabit && !fault;
    network_reset_sync tx_reset(.i_clk(o_clk), .i_async_rst_n(qualified), .o_rst_n(tx_rst_n));
    // Forward clock before link-up; its release is synchronized in the 90deg domain.
    network_reset_sync txc_reset(.i_clk(clk90),
        .i_async_rst_n(i_rst_n && pll_locked), .o_rst_n(txc_rst_n));
    network_reset_sync rx_reset(.i_clk(i_rgmii_rxc), .i_async_rst_n(qualified), .o_rst_n(rx_rst_n));
    // Leave RX on its dedicated clock-capable input; Integration must confirm
    // TD chooses the HR IO clock route and a global route for the RX stack.
    assign o_rx_clk = i_rgmii_rxc;
    assign o_ready = tx_rst_n;
    network_rgmii_io u_io(.i_tx_clk(o_clk), .i_tx_clk90(clk90), .i_rx_clk(o_rx_clk),
        .i_tx_rst_n(tx_rst_n), .i_txc_rst_n(txc_rst_n), .i_rx_rst_n(rx_rst_n),
        .i_tx_valid(i_tx_valid), .i_tx_data(i_tx_data),
        .i_rgmii_rxd(i_rgmii_rxd), .i_rgmii_rx_ctl(i_rgmii_rx_ctl),
        .o_rgmii_txd(o_rgmii_txd), .o_rgmii_tx_ctl(o_rgmii_tx_ctl), .o_rgmii_txc(o_rgmii_txc),
        .o_rx_data(o_rx_data), .o_rx_valid(o_rx_valid), .o_rx_error(o_rx_error));
endmodule
