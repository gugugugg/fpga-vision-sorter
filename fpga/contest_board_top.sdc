create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 2.222 -waveform {0.000 1.111} [get_nets {IO_rx_clk_pad_p}]

derive_clocks
rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]
rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]

# uii2c CLK_DIV=241: half-period=121 camera clock cycles. The free-running
# divider starts at clkdiv=0/scl_clk=0, so its first rise is source cycle 120.
create_generated_clock -name {CAM_I2C_CLK} \
    -source [get_pins {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}] \
    -edges {241 483 725} [get_nets {u_camera_cfg/u_i2c/scl_clk}]
# Active SCL edges reach the output 91 camera clocks after the internal edge
# (OFFSET=90, tested before the divider updates). Idle/start/stop gate SCL high.
create_generated_clock -name {CAM_I2C_BUS} \
    -source [get_pins {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}] \
    -edges {423 665 907} [get_nets {O_cam_scl}]
# O_cam_24m is the same direct net as pll_clk_24m; do not redefine/override
# its clock with an additional generated clock on that shared net.
# SC520CS V1.6 Table 1-4: standard-mode data/ACK valid <=3.45 us after
# falling SCL, input hold >=0 ns and setup >=250 ns before rising SCL.
# These are SENSOR-PIN requirements. Board traces/level-shifter delays remain
# external facts to be added after the physical interface is confirmed.
set_input_delay -clock [get_clocks {CAM_I2C_BUS}] -clock_fall -min 0.000 [get_ports {IO_cam_sda}]
set_input_delay -clock [get_clocks {CAM_I2C_BUS}] -clock_fall -max 3450.000 [get_ports {IO_cam_sda}]
set_output_delay -clock [get_clocks {CAM_I2C_BUS}] -min 0.000 [get_ports {IO_cam_sda}]
set_output_delay -clock [get_clocks {CAM_I2C_BUS}] -max 250.000 [get_ports {IO_cam_sda}]

# Do not group entire camera/DDR/system domains: that would override Gray
# distribution bounds. Relax only the actual receiver first stages below.
# External reset/PLL lock assert asynchronously only into reset-release chains.
set_false_path -from [get_ports {I_rst_n}]
# SW4 drives only asynchronous reset assertion and the menu FIRST-stage
# synchronizer. The same reset release chains still time their DATA paths.
set_false_path -from [get_ports {I_sw4}] -through [get_nets {board_reset_n}]
set_false_path -to [get_regs -nowarn {u_vision/u_mode/menu_sync0}]
set_false_path -through [get_nets {core_rst_n}]

# Except only receiver FIRST stages; second stages and algorithm paths remain
# timed to their destination clock. Mailbox data stays held through the ack.
# The LP HS detector and sensor byte clock are independent. The board adapter
# now has two HS synchronizer flops; only hs_meta receives the asynchronous
# level. hs_meta -> hs_sync and the downstream offset counter stay timed.
set_false_path -to [get_regs -nowarn {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_hs_fabric_sync/hs_meta}]
set_false_path -to [get_regs -nowarn {*/ack_sync0 */ready_sync0 */req_sync0 */src_reset_sync0}]
# Toggle mailboxes hold all bits until acknowledgement. Bound their data
# distribution to one period of the fastest destination (133.333 MHz DDR).
set_max_delay -datapath_only 7.500 -from [get_regs -nowarn {*/tx_hold[*]}] -to [get_regs -nowarn {*/data_sync0[*]}]
set_false_path -hold -to [get_regs -nowarn {*/data_sync0[*]}]
set_false_path -to [get_regs -nowarn {u_vision/u_display_rate/vs0}]
set_false_path -to [get_regs -nowarn {u_capture_rate/counter/vs0}]
set_false_path -to [get_regs -nowarn {u_vision/u_mode/key_sync0[*] u_capture_uart/key_sync[0] u_capture_uart/rx_sync[0]}]
set_false_path -to [get_regs -nowarn {lane_fault0 status0[*] u_video_in/enable0 u_video_in/overflow0 u_video_out/S_video_vsync_1d}]
set_false_path -to [get_regs -nowarn {u_capture_uart/ack0 u_capture_uart/req0 u_capture_uart/key_cdc[0] u_capture_uart/resend_cdc[0]}]
# UART mailbox data is held until the byte finishes and acknowledge returns.
set_max_delay -datapath_only 20.000 -from [get_regs -nowarn {u_capture_uart/hold[*]}] -to [get_regs -nowarn {u_capture_uart/hold0[*]}]
set_false_path -hold -to [get_regs -nowarn {u_capture_uart/hold0[*]}]
# AE data is held until the I2C transaction completes and the ack returns.
set_max_delay -datapath_only 41.538 -from [get_regs -nowarn {u_vision/u_ae_bridge/src_exposure_hold[*] u_vision/u_ae_bridge/src_gain_hold[*]}] -to [get_regs -nowarn {u_vision/u_ae_bridge/exp_sync0[*] u_vision/u_ae_bridge/gain_sync0[*]}]
set_false_path -hold -to [get_regs -nowarn {u_vision/u_ae_bridge/exp_sync0[*] u_vision/u_ae_bridge/gain_sync0[*]}]
# These are asynchronous RESET inputs of reset-release chains, not algorithm
# or pointer data paths. Their second-stage DATA paths remain fully timed.
set_false_path -from [get_regs -nowarn {u_lp_reset/release_pipe[*]}] -to [get_regs -nowarn {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_fabric_reset/release_pipe[*]}]
set_false_path -from [get_regs -nowarn {u_video_out/S_fifo_rst}] -to [get_regs -nowarn {u_video_out/U_w128_d512_fifo/soft_fifo_al_4057d6b76aa6_Inst/asy_r_rst0 u_video_out/U_w128_d512_fifo/soft_fifo_al_4057d6b76aa6_Inst/asy_r_rst1}]
# Source period bounds for registered Gray FIFO pointers: camera 112.5 MHz,
# corrected DDR user 133.3 MHz. Keep both second synchronizer stages timed.
set_max_delay -datapath_only 8.888 -from [get_regs -nowarn {u_video_in/u_fifo/wg[*]}] -to [get_regs -nowarn {u_video_in/u_fifo/wg0[*]}]
set_max_delay -datapath_only 7.500 -from [get_regs -nowarn {u_video_in/u_fifo/rg[*]}] -to [get_regs -nowarn {u_video_in/u_fifo/rg0[*]}]
set_false_path -hold -to [get_regs -nowarn {u_video_in/u_fifo/wg0[*] u_video_in/u_fifo/rg0[*] */sync_r1[*]}]
# Registered Gray timebase changes one bit per microsecond. Bound distribution
# to one source clock. Keep dg0->dg1 and sg0->sg1 timing active.
set_max_delay -datapath_only 20.000 -from [get_regs -nowarn {u_frame_latency/time_gray[*]}] -to [get_regs -nowarn {u_frame_latency/dg0[*] u_frame_latency/sg0[*]}]
set_false_path -hold -to [get_regs -nowarn {u_frame_latency/dg0[*] u_frame_latency/sg0[*]}]
# Common cartoon history FIFOs: registered Gray pointers, bounded to ONE
# source period. Only first-stage hold checks are excepted; stage2 is timed.
# These constraints remain present when the complete network chain is pruned.
set_false_path -to [get_regs -nowarn {g_cartoon_history$u_store/fault_sync0}]
set_max_delay -datapath_only 7.500 -from [get_regs -nowarn {g_cartoon_history$u_store/u_read_fifo/wg[*]}] -to [get_regs -nowarn {g_cartoon_history$u_store/u_read_fifo/wg0[*]}]
set_max_delay -datapath_only 19.231 -from [get_regs -nowarn {g_cartoon_history$u_store/u_read_fifo/rg[*]}] -to [get_regs -nowarn {g_cartoon_history$u_store/u_read_fifo/rg0[*]}]
set_max_delay -datapath_only 19.231 -from [get_regs -nowarn {g_cartoon_history$u_store/u_write_fifo/wg[*]}] -to [get_regs -nowarn {g_cartoon_history$u_store/u_write_fifo/wg0[*]}]
set_max_delay -datapath_only 7.500 -from [get_regs -nowarn {g_cartoon_history$u_store/u_write_fifo/rg[*]}] -to [get_regs -nowarn {g_cartoon_history$u_store/u_write_fifo/rg0[*]}]
set_false_path -hold -to [get_regs -nowarn {g_cartoon_history$u_store/u_read_fifo/wg0[*] g_cartoon_history$u_store/u_read_fifo/rg0[*] g_cartoon_history$u_store/u_write_fifo/wg0[*] g_cartoon_history$u_store/u_write_fifo/rg0[*]}]
# Transaction reset is asserted only after all accepted reads have drained.
# Exceptions apply ONLY to asynchronous assertion pins of reset-release chains.
set_false_path -to [get_pins -nowarn {g_cartoon_history$u_store/*/release_pipe_reg[*]_syn_2.asr}]


# Reset notification chains: asynchronous assertion pins only.
set_false_path -to [get_pins -nowarn {*/src_channel_reset_sync0_reg.sr */src_channel_reset_sync1_reg.sr */src_reset_sync0_reg.sr */src_reset_sync1_reg.sr */src_channel_reset_sync0_reg_syn_*.asr */src_channel_reset_sync1_reg_syn_*.asr */src_reset_sync0_reg_syn_*.asr */src_reset_sync1_reg_syn_*.asr}]

# Only asynchronous UART inputs; all controller internal timing stays checked.
set_false_path -from [get_ports {I_uart_rx I_arm_uart_rx}]
set_false_path -to [get_regs -nowarn {u_teach_console/k4/sync[0]}]
set_output_delay -clock [get_clocks {sys_clk_50m}] -max 10.000 [get_ports {O_uart_tx O_arm_uart_tx}]
set_output_delay -clock [get_clocks {sys_clk_50m}] -min 0.000 [get_ports {O_uart_tx O_arm_uart_tx}]

# Only the first synchronizer stages of the new physical inputs.
set_false_path -from [get_ports {I_sw1}] -to [get_regs -nowarn {u_capture_controls/capture_sync0}]
set_false_path -from [get_ports {I_sw4}] -to [get_regs -nowarn {u_capture_controls/sw4_sync0}]
set_false_path -from [get_ports {I_template_flash_miso}] -to [get_regs -nowarn {u_template_video/u_flash/miso_sync0}]
