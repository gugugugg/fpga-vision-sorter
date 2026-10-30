`timescale 1ns/1ps
// 50 -> 125 MHz, 0/90 degrees. Analog/divider settings from local TD-generated
// 33_ethernet_udp_led pll (TD 6.1.155545, 2025-06-05); no protocol RTL reused.
module network_phy_pll(input i_refclk, i_rst_n, output o_clk, o_clk90, o_locked);
`ifdef NETWORK_PHY_SIM
    network_phy_sim_pll model(.i_refclk(i_refclk), .i_rst_n(i_rst_n),
        .o_clk(o_clk), .o_clk90(o_clk90), .o_locked(o_locked));
`else
    wire [6:0] clkc, clkcb;
    PH1P_LOGIC_BUFG feedback_buf(.i(clkc[0]), .o(o_clk));
    PH1P_LOGIC_BUFG phase_buf(.i(clkc[1]), .o(o_clk90));
    PH1P_PHY_PLL_V2 #(
        .FBKCLK("CLKC0_EXT"), .FBKCLK_INT("CLKC0_EXT"), .FIN("50.000000000"),
        .REFCLK_DIV(2), .FBCLK_DIV(5),
        .CLKC0_ENABLE("ENABLE"), .CLKC0_DIV(7), .CLKC0_CPHASE(6),
        .CLKC0_FPHASE(0), .CLKC0_FPHASE_RSTSEL(0), .CLKC0_DUTY_INT(4),
        .CLKC1_ENABLE("ENABLE"), .CLKC1_DIV(7), .CLKC1_CPHASE(0),
        .CLKC1_FPHASE(6), .CLKC1_FPHASE_RSTSEL(1), .CLKC1_DUTY_INT(4),
        .CLKC2_ENABLE("ENABLE"), .CLKC2_DIV(35), .CLKC2_CPHASE(34),
        .CLKC2_FPHASE(0), .CLKC2_FPHASE_RSTSEL(0), .CLKC2_DUTY_INT(18),
        .PLL_USR_RST("ENABLE"), .PLL_FEED_TYPE("EXTERNAL"), .PLL_FASTLOOP("ENABLE"),
        .LPF_RES(0), .LPF_CAP(13), .ICP_CUR(29), .PHASE_PATH_SEL(0),
        .DYN_PHASE_PATH_SEL("DISABLE"), .DYN_FPHASE_EN("DISABLE"),
        .PI_OUT_SEL("NORMAL"), .PI_FRAC_EN("DISABLE"), .CLKC0_PI_SHIFT_EN("DISABLE"),
        .FEEDBK_MODE("NORMAL"), .DYN_CPHASE_EN("DISABLE")
    ) u_pll(
        .refclk(i_refclk), .fbclk(o_clk), .clkc(clkc), .clkcb(clkcb), .lock(o_locked),
        .pllreset(!i_rst_n), .clkc_rst(2'b00), .clkc_en(7'b0000111),
        .refclk_rst(1'b0), .wakeup(1'b1), .cps_step(1'b0), .pllpd(1'b0),
        .psclk(1'b0), .psclksel(3'b000), .psstep(1'b0), .psdown(1'b0), .psdone(),
        .drp_clk(1'b0), .drp_rstn(1'b1), .drp_sel(1'b0), .drp_rd(1'b0), .drp_wr(1'b0),
        .drp_addr(8'b0), .drp_wdata(8'b0), .drp_err(), .drp_rdy(), .drp_rdata(),
        .ssc_reset(1'b0), .scan_clk(1'b1), .scan_enable(1'b1), .scan_mode(1'b0), .scan_resetn(1'b1));
`endif
endmodule
