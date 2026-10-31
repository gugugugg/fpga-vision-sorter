`timescale 1ns/1ps
`default_nettype none

// Self-developed board integration top for HX1P35A / PH1P35MDG324.
// Vendor sources are used only as BSP/IP blocks. Camera ISP output enters the
// contest vision subsystem before DDR, while frame-aligned OSD is inserted
// after DDR in the HDMI pixel domain.
module contest_board_top #(
    parameter ENABLE_NETWORK_DEBUG = 0,
    parameter ENABLE_HDMI_MENU = 1,
    parameter ENABLE_DETECTION = 0,
    parameter ENABLE_HEAVY_FILTERS = 1,
    parameter ENABLE_EVA = 0,
    parameter ENABLE_CARTOON = 1,
    parameter ENABLE_CARTOON_TEMPORAL = 1,
    parameter ENABLE_MOTION = 0,
    parameter ENABLE_ARM = 0,
    parameter ENABLE_COLOR_SHADING = 1,
    parameter NORMAL_IMAGE_AWB = 1,
    parameter ENABLE_QC_3D = 0,
    parameter ENABLE_QC_COMPONENTS = 0,
    parameter ENABLE_QC_VOXELS = 0,
    parameter [63:0] MODE_BITMAP = 64'h1f0db7
)(
    input  wire        I_sys_clk,
    input  wire        I_rst_n,
    input wire I_sw4,
    input wire I_sw1,
    output wire O_template_flash_cs_n,O_template_flash_clk,O_template_flash_mosi,
    input wire I_template_flash_miso,
    output wire O_template_flash_wp_n,O_template_flash_hold_n,

    input wire I_arm_uart_rx,
    output wire O_arm_uart_tx,
    output wire O_buzzer_n,
    output wire        O_cam_scl,
    inout  wire        IO_cam_sda,
    output wire        O_cam_24m,
    output wire        O_cam_rst,
    inout  wire [1:0]  I_button,
    input  wire        I_capture_key_n,
    input  wire        I_uart_rx,
    output wire        O_uart_tx,

    input wire [3:0] I_net_rgmii_rxd,
    input wire I_net_rgmii_rx_ctl,I_net_rgmii_rxc,
    output wire [3:0] O_net_rgmii_txd,
    output wire O_net_rgmii_tx_ctl,O_net_rgmii_txc,O_net_phy_rst_n,O_net_mdc,
    inout wire IO_net_mdio,
    output wire        O_tmds_ch0_p,
    output wire        O_tmds_ch1_p,
    output wire        O_tmds_ch2_p,
    output wire        O_tmds_clk_p,

    inout  wire        IO_rx_clk_pad_n,
    inout  wire        IO_rx_clk_pad_p,
    inout  wire [3:0]  IO_rx_data_pad_n,
    inout  wire [3:0]  IO_rx_data_pad_p,

    output wire [12:0] ddr_addr,
    output wire [1:0]  ddr_ba,
    output wire [0:0]  ddr_cke,
    output wire [0:0]  ddr_odt,
    output wire [0:0]  ddr_cs_n,
    output wire        ddr_ras_n,
    output wire        ddr_cas_n,
    output wire        ddr_we_n,
    output wire [0:0]  ddr_ck_p,
    output wire [0:0]  ddr_ck_n,
    inout  wire [1:0]  ddr_dm,
    inout  wire [15:0] ddr_dq,
    inout  wire [1:0]  ddr_dqs_p,
    inout  wire [1:0]  ddr_dqs_n
);
    wire clk_100m, clk_24m, hdmi_pixel_clk, hdmi_serial_clk;
    wire csi_rx_clk,ddr_clk,ddr_init_done;
    wire pll_lock;
    wire board_reset_n,menu_enabled,capture_key_n;
    wire capture_mode,capture_command;wire [15:0] capture_code;
    wire [3:0] capture_keys;
    template_capture_controls u_capture_controls(
      .clk(I_sys_clk),.rst_n(sys_rst_n),.sw1(I_sw1),.sw4(I_sw4),.key_press(capture_keys),
      .capture_mode(capture_mode),.command_update(capture_command),.command_code(capture_code));
    assign O_template_flash_wp_n=1'b1;
    assign O_template_flash_hold_n=1'b1;
    wire [3:0] menu_keys_n;
    board_key_router #(.ENABLE_MENU(ENABLE_HDMI_MENU)) u_key_router(
        .i_sw4(I_sw4),.i_key3_n(I_rst_n),.i_key4_n(I_capture_key_n),.i_keys12_n(I_button),
        .o_reset_n(board_reset_n),.o_menu_enabled(menu_enabled),
        .o_capture_key_n(capture_key_n),.o_keys_n(menu_keys_n));
    wire core_rst_n = board_reset_n & pll_lock;
    wire sys_rst_n,cam_rst_n,csi_rst_n,ddr_rst_n,pixel_rst_n,lp_rst_n,serial_rst_n;
    reset_release u_sys_reset(I_sys_clk,core_rst_n,sys_rst_n);
    reset_release u_cam_reset(clk_24m,core_rst_n,cam_rst_n);
    reset_release u_csi_reset(csi_rx_clk,core_rst_n,csi_rst_n);
    reset_release u_ddr_reset(ddr_clk,core_rst_n,ddr_rst_n);
    reset_release u_pixel_reset(hdmi_pixel_clk,core_rst_n,pixel_rst_n);
    reset_release u_lp_reset(clk_100m,core_rst_n,lp_rst_n);
    reset_release u_serial_reset(hdmi_serial_clk,core_rst_n,serial_rst_n);

    wire hs_rx_valid;
    wire [15:0] hs_rx_data;
    wire [1:0] lane_error;
    wire camera_packet_fault;
    wire csi_diag_update;
    wire [63:0] csi_diag,csi_diag_hdmi;
    cdc_bus_toggle #(.WIDTH(64)) u_csi_diag_to_hdmi(
        .i_src_clk(csi_rx_clk),.i_src_rst_n(csi_rst_n),
        .i_src_update(csi_diag_update),.i_src_data(csi_diag),.o_src_busy(),
        .i_dst_clk(hdmi_pixel_clk),.i_dst_rst_n(pixel_rst_n),
        .o_dst_update(),.o_dst_data(csi_diag_hdmi));
    wire demosaic_fault;
    wire csi_frame_start, csi_frame_end, csi_valid;
    wire [31:0] csi_data;
    wire raw10_frame_start, raw10_frame_end, raw10_valid;
    wire [39:0] raw10_data;
    wire axis_last, axis_user, axis_valid;
    wire [39:0] axis_data;
    wire raw_last, raw_user, raw_valid;
    wire [39:0] raw_data;

    wire demosaic_ready;
    wire demosaic_last, demosaic_user, demosaic_valid;
    wire [127:0] demosaic_data;
    wire [95:0] demosaic_rgb;
    wire vision_sof, vision_eol, vision_valid;
    wire [95:0] vision_rgb;
    wire packed_frame_start, packed_valid;
    wire [127:0] packed_data;

    wire cam_cfg_done, cam_ae_cfg_done, cam_ae_req;
    wire [15:0] cam_exposure, cam_gain_index;
    wire [15:0] cam_sensor_id;
    wire cam_sensor_id_ok, cam_iic_error;
    reg [17:0] cam_power_cnt;
    reg cam_pwdnb, cam_i2c_ready;
    // PLL output is approximately 24.074 MHz. Keep XSHUTDN low for >=1 ms
    // and start I2C >=5 ms after release, as in the SC520CS reference.
    localparam integer CAM_PWDNB_LOW_CYCLES = 24075;
    localparam integer CAM_I2C_START_CYCLES = 144450;

    wire video_out_rd_busy, video_in_wr_busy;
    wire [1:0] video_out_rp;
    wire vi_ddr_wr_en, vo_ddr_rd_en;
    wire [24:0] vi_ddr_wr_addr, vo_ddr_rd_addr;
    wire [127:0] vi_ddr_wr_data;
    wire ddr_user_wr_en, ddr_user_rd_en;
    wire [24:0] ddr_user_addr;
    wire [127:0] ddr_user_wr_data;
    wire ddr_user_ready, ddr_user_rd_valid;
    wire [127:0] ddr_user_rd_data;
    wire vi_sof, vi_overflow;
    wire [37:0] stream_meta,write_meta;
    wire frame_meta_update,ddr_meta_update;
    wire [123:0] frame_meta,ddr_frame_meta;
    wire selected_frame_start,bound_meta_ddr_update,bound_meta_update;
    wire [91:0] bound_meta_ddr,bound_meta;
    wire [31:0] bound_id_ddr,bound_id;
    wire [123:0] bound_bundle={bound_id_ddr,bound_meta_ddr};
    wire [123:0] bound_bundle_hdmi;
    assign {bound_id,bound_meta}=bound_bundle_hdmi;
    wire view_cmd,manual_view_cmd,arm_view_cmd;wire [2:0] view_id,manual_view_id,arm_view_id;
    wire arm_packet_active,command_byte_valid;wire [7:0] command_byte;
    wire [7:0] arm_status_hdmi;wire [8:0] arm_points_hdmi;
    wire [5:0] qc_views;wire qc_hull_valid,qc_overrun;
    wire [31:0] target_frame_id;
    wire teach_2d,clear_2d;
    wire coeff_valid,coeff_clear;wire [2:0] coeff_view;wire signed [9:0] coeff_cos,coeff_sin;
    wire [5:0] qc_coefficients;
    // DAT pin is deliberately unbound until the user selects an expansion pin.
    // Logical TX remains observable in ARM status; no actuator pin is driven.
    wire arm_dat_drive_low,arm_scanning_sys;
    assign view_cmd=arm_view_cmd | (manual_view_cmd && !arm_scanning_sys);
    assign view_id=arm_view_cmd?arm_view_id:manual_view_id;
    wire latency_valid;wire [15:0] latency_ms;
    wire capture_req, capture_resend, capture_busy, capture_error;
    wire capture_block_reads, display_ready, display_rd_valid;
    wire capture_byte_valid, capture_byte_ready;
    wire [7:0] capture_byte;
    wire [3:0] capture_saved, capture_sent;
    wire [2:0] capture_batch;
    (* async_reg="true",syn_preserve=1 *) reg lane_fault0, lane_fault1;
    wire [7:0] capture_fps;
    // Each flag is independent; this is not an atomic cross-domain data bus.
    (* async_reg="true",syn_preserve=1 *) reg [7:0] status0,status1;
    always @(posedge hdmi_pixel_clk or negedge pixel_rst_n)
      if(!pixel_rst_n) begin status0<=0;status1<=0;end
      else begin
        status0<={vi_overflow,demosaic_fault,camera_packet_fault,|lane_error,
                  ddr_init_done,cam_iic_error,cam_sensor_id_ok,cam_cfg_done};
        status1<=status0;
      end
    capture_frame_rate u_capture_rate(
      .source_clk(csi_rx_clk),.source_rst_n(csi_rst_n),
      .source_sof(demosaic_user && demosaic_valid),
      .sys_clk(I_sys_clk),.sys_rst_n(sys_rst_n),
      .display_clk(hdmi_pixel_clk),.display_rst_n(pixel_rst_n),.fps(capture_fps));
    reg [1:0] saved_display_rp;
    reg capture_busy_d;
    always @(posedge ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin
            lane_fault0<=0;lane_fault1<=0;
            saved_display_rp<=0;capture_busy_d<=0;
        end else begin
            lane_fault0<=camera_packet_fault | demosaic_fault;lane_fault1<=lane_fault0;
            capture_busy_d<=capture_busy;
            if(!capture_busy) saved_display_rp<=video_out_rp;
        end
    end

    wire [24:0] mc_app_addr;
    wire [2:0] mc_app_cmd;
    wire mc_app_en, mc_app_wdf_end, mc_app_wdf_wren;
    wire [127:0] mc_app_wdf_data, mc_app_rd_data;
    wire [15:0] mc_app_wdf_mask;
    wire mc_app_rd_data_end, mc_app_rd_data_valid;
    wire mc_app_rdy, mc_app_wdf_rdy;

    wire hdmi_vsync, hdmi_hsync, hdmi_de, hdmi_user, hdmi_last;
    wire hdmi_window_rd_en;
    wire [23:0] video_rd_data;
    wire mixer_vsync, mixer_hsync, mixer_de;
    wire [23:0] mixer_rgb;
    wire osd_vsync, osd_hsync, osd_de;
    wire [23:0] osd_rgb;

    wire target_valid;
    wire [3:0] target_class;
    wire [10:0] target_center_x, target_bbox_x0, target_bbox_x1;
    wire [9:0] target_center_y, target_bbox_y0, target_bbox_y1;

    wire net_clk125,net_rx_clk,net_ready,net_rx_valid,net_rx_error,net_tx_valid;
    wire [7:0] net_rx_data,net_tx_data;
    wire remote_mode_valid,remote_threshold_valid,remote_rejected;
    wire [5:0] remote_mode,sys_mode,display_mode;
    wire menu_open_sys;
    wire [15:0] sort_status_sys,sort_status_pixel;
    wire [9:0] remote_threshold,sys_threshold;
    wire legacy_ready,legacy_rd_valid,net_mc_wr,net_mc_rd;
    wire [127:0] legacy_rd_data;
    wire history_bus_wr,history_bus_rd,history_bus_ready,history_bus_rd_valid;
    wire [24:0] history_bus_addr;wire [127:0] history_bus_data,history_bus_rd_data;
    wire history_read_pixel,history_write_pixel,history_valid;
    wire [31:0] history_old,history_new;wire [9:0] history_configuration;wire [7:0] history_status;
    wire [127:0] history_diagnostics;
    generate if(ENABLE_CARTOON && ENABLE_CARTOON_TEMPORAL)begin:g_cartoon_history
        wire wr,rd,ready,rd_valid,arbiter_fault;wire [24:0] addr;wire [127:0] data,rd_data;
        cartoon_history_store u_store(
            .rst_n(core_rst_n),.pixel_clk(hdmi_pixel_clk),.ddr_clk(ddr_clk),
            .start(bound_meta_update),.frame_id(bound_id),.configuration({bound_meta[91:86],history_configuration}),
            .read_pixel(history_read_pixel),.write_pixel(history_write_pixel),.new_state(history_new),
            .old_valid(history_valid),.old_state(history_old),.status(history_status),.diagnostics(history_diagnostics),
            .i_fault(arbiter_fault),.wr(wr),.rd(rd),.addr(addr),.data(data),.ready(ready),.rd_valid(rd_valid),.rd_data(rd_data));
        history_ddr_arbiter #(.NET_STROBES_ACCEPTED(1)) u_arbiter(
            .i_clk(ddr_clk),.i_rst_n(ddr_rst_n),
            .i_legacy_wr(ddr_user_wr_en),.i_legacy_rd(ddr_user_rd_en),
            .i_legacy_reserved(video_in_wr_busy || video_out_rd_busy || capture_busy),
            .i_legacy_addr(ddr_user_addr),.i_legacy_data(ddr_user_wr_data),
            .o_legacy_ready(legacy_ready),.o_legacy_rd_valid(legacy_rd_valid),.o_legacy_rd_data(legacy_rd_data),
            .i_net_wr(wr),.i_net_rd(rd),.i_net_addr(addr),.i_net_data(data),
            .o_net_ready(ready),.o_net_rd_valid(rd_valid),.o_net_rd_data(rd_data),
            .o_wr(history_bus_wr),.o_rd(history_bus_rd),.o_addr(history_bus_addr),.o_data(history_bus_data),
            .i_ready(history_bus_ready),.i_rd_valid(history_bus_rd_valid),.i_rd_data(history_bus_rd_data),
            .o_fault(arbiter_fault),.o_outstanding());
    end else begin:g_no_cartoon_history
        assign history_bus_wr=ddr_user_wr_en;assign history_bus_rd=ddr_user_rd_en;
        assign history_bus_addr=ddr_user_addr;assign history_bus_data=ddr_user_wr_data;
        assign legacy_ready=history_bus_ready;assign legacy_rd_valid=history_bus_rd_valid;
        assign legacy_rd_data=history_bus_rd_data;assign history_valid=0;assign history_old=0;assign history_status=0;
        assign history_diagnostics=0;
    end endgenerate
    wire [24:0] net_mc_addr;
    wire [127:0] net_mc_data;
    wire [231:0] hdmi_status,status_snapshot;
    wire [255:0] debug_telemetry_hdmi,debug_telemetry_net;
    wire diagnostic_mailbox_ready;
    generate if(ENABLE_NETWORK_DEBUG) begin: g_network
    reg osd_vs_d;
    network_mailbox #(.WIDTH(256)) u_diagnostics_to_net(
        .i_rst_n(core_rst_n),.i_src_clk(hdmi_pixel_clk),
        .i_src_valid(osd_vsync && !osd_vs_d && diagnostic_mailbox_ready),
        .i_src_data(debug_telemetry_hdmi),.o_src_ready(diagnostic_mailbox_ready),
        .i_dst_clk(net_clk125),.o_dst_valid(),.o_dst_data(debug_telemetry_net),.i_dst_ready(1'b1));
    wire sys_frame_boundary;
    always @(posedge hdmi_pixel_clk or negedge pixel_rst_n)
        if(!pixel_rst_n) osd_vs_d<=0;else osd_vs_d<=osd_vsync;
    wire status_mailbox_ready;
    network_mailbox #(.WIDTH(232)) u_status_to_sys(
        .i_rst_n(core_rst_n),.i_src_clk(hdmi_pixel_clk),
        .i_src_valid(osd_vsync && !osd_vs_d && status_mailbox_ready),
        .i_src_data(hdmi_status),.o_src_ready(status_mailbox_ready),
        .i_dst_clk(I_sys_clk),.o_dst_valid(sys_frame_boundary),
        .o_dst_data(status_snapshot),.i_dst_ready(1'b1));
    wire [231:0] sys_status={{2'd0,sys_mode},{6'd0,sys_threshold},status_snapshot[207:0]};
    network_rgmii_phy u_network_phy(
        .i_sys_clk(I_sys_clk),.i_rst_n(core_rst_n),
        .i_rgmii_rxd(I_net_rgmii_rxd),.i_rgmii_rx_ctl(I_net_rgmii_rx_ctl),
        .i_rgmii_rxc(I_net_rgmii_rxc),.o_rgmii_txd(O_net_rgmii_txd),
        .o_rgmii_tx_ctl(O_net_rgmii_tx_ctl),.o_rgmii_txc(O_net_rgmii_txc),
        .o_phy_rst_n(O_net_phy_rst_n),.o_mdc(O_net_mdc),.io_mdio(IO_net_mdio),
        .o_clk(net_clk125),.o_rx_clk(net_rx_clk),.o_ready(net_ready),
        .o_rx_valid(net_rx_valid),.o_rx_error(net_rx_error),.o_rx_data(net_rx_data),
        .i_tx_valid(net_tx_valid),.i_tx_data(net_tx_data));
    wire camera_heartbeat;
    cdc_bus_toggle #(.WIDTH(1)) u_camera_heartbeat(
        .i_src_clk(csi_rx_clk),.i_src_rst_n(csi_rst_n),
        .i_src_update(frame_meta_update),.i_src_data(1'b1),.o_src_busy(),
        .i_dst_clk(hdmi_pixel_clk),.i_dst_rst_n(pixel_rst_n),
        .o_dst_update(camera_heartbeat),.o_dst_data());
    reg bound_seen,camera_seen;reg [22:0] camera_age;
    always @(posedge hdmi_pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin bound_seen<=0;camera_seen<=0;camera_age<=0;end
        else begin
            if(bound_meta_update) bound_seen<=1;
            if(camera_heartbeat) begin camera_age<=0;camera_seen<=1;end
            else if(camera_age<5200000) camera_age<=camera_age+1'b1;
        end
    end
    wire source_ok=status1[0] && status1[1] && !status1[2] && status1[3] &&
        !(|status1[7:4]) && bound_seen && camera_seen && camera_age<5200000;
    wire network_ddr_fault;wire [9:0] network_outstanding;
    board_network_integration #(.MODE_BITMAP(MODE_BITMAP)) u_network(
        .i_core_rst_n(core_rst_n),.i_net_clk(net_clk125),.i_rx_clk(net_rx_clk),
        .i_csi_clk(csi_rx_clk),.i_csi_sample({8'd0,demosaic_fault,camera_packet_fault,
            csi_valid,csi_frame_end,csi_frame_start,lane_error,hs_rx_valid,hs_rx_data}),
        .i_telemetry(debug_telemetry_net),
        .i_sys_clk(I_sys_clk),.i_pixel_clk(hdmi_pixel_clk),.i_ddr_clk(ddr_clk),
        .i_net_enable(net_ready),.i_rx_valid(net_rx_valid),.i_rx_error(net_rx_error),
        .i_rx_data(net_rx_data),.o_tx_valid(net_tx_valid),.o_tx_data(net_tx_data),
        .i_vsync(osd_vsync),.i_de(osd_de),.i_rgb(osd_rgb),.i_display_mode(display_mode),
        .i_source_ok(source_ok),.i_sys_frame_boundary(sys_frame_boundary),.i_sys_status(sys_status),
        .i_remote_rejected(remote_rejected),.o_remote_mode_valid(remote_mode_valid),
        .o_remote_mode(remote_mode),.o_remote_threshold_valid(remote_threshold_valid),
        .o_remote_threshold(remote_threshold),.i_capture_busy(capture_busy),
        .i_legacy_wr(history_bus_wr),.i_legacy_rd(history_bus_rd),
        .i_legacy_reserved(video_in_wr_busy || video_out_rd_busy || capture_busy),
        .i_legacy_addr(history_bus_addr),.i_legacy_data(history_bus_data),
        .o_legacy_ready(history_bus_ready),.o_legacy_rd_valid(history_bus_rd_valid),
        .o_legacy_rd_data(history_bus_rd_data),.o_wr(net_mc_wr),.o_rd(net_mc_rd),
        .o_addr(net_mc_addr),.o_data(net_mc_data),.i_ready(ddr_user_ready),
        .i_rd_valid(ddr_user_rd_valid),.i_rd_data(ddr_user_rd_data),
        .o_fault(network_ddr_fault),.o_outstanding(network_outstanding));
    end else begin: g_no_network
        // Remove PHY/UDP/trace/HDMI tap/frame store/DDR debug arbiter together.
        assign O_net_rgmii_txd=0;assign O_net_rgmii_tx_ctl=0;
        assign O_net_rgmii_txc=0;assign O_net_phy_rst_n=0;
        assign O_net_mdc=0;assign IO_net_mdio=1'bz;
        assign remote_mode_valid=0;assign remote_mode=0;
        assign remote_threshold_valid=0;assign remote_threshold=0;
        assign history_bus_ready=ddr_user_ready;
        assign history_bus_rd_valid=ddr_user_rd_valid;
        assign history_bus_rd_data=ddr_user_rd_data;
        assign net_mc_wr=history_bus_wr;assign net_mc_rd=history_bus_rd;
        assign net_mc_addr=history_bus_addr;assign net_mc_data=history_bus_data;
    end endgenerate
    assign O_cam_24m = clk_24m;
    assign O_cam_rst = cam_pwdnb;

    // Release the BSP I2C reset half a camera cycle away from divider edges.
    always @(negedge clk_24m or negedge cam_rst_n) begin
        if (!cam_rst_n) begin
            cam_power_cnt <= 18'd0;
            cam_pwdnb <= 1'b0;
            cam_i2c_ready <= 1'b0;
        end else begin
            if (cam_power_cnt < CAM_I2C_START_CYCLES)
                cam_power_cnt <= cam_power_cnt + 1'b1;
            if (cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
                cam_pwdnb <= 1'b1;
            if (cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
                cam_i2c_ready <= 1'b1;
        end
    end

    PLL u_PLL (
        .refclk(I_sys_clk), .reset(~board_reset_n),
        .clk0_out(clk_100m), .clk1_out(clk_24m),
        .clk4_out(hdmi_pixel_clk), .clk5_out(hdmi_serial_clk),
        .lock(pll_lock)
    );

    // Official camera-register support is BSP, not a contest image algorithm.
    // uii2c divides by 2*(floor(CLK_DIV/2)+1): 242 gives 99.48 kHz,
    // below the sensor's 100 kHz standard-mode maximum at 24.074074 MHz.
    sc520_camera_cfg #(.CLK_DIV(241),.CAM_CLK_HZ(24074074)) u_camera_cfg (
        .I_clk(clk_24m), .I_rst_n(cam_i2c_ready),
        .I_ae_req(cam_ae_req), .I_ae(cam_exposure), .I_ag(cam_gain_index),
        .O_cam_scl(O_cam_scl), .IO_cam_sda(IO_cam_sda),
        .O_cfg_done(cam_cfg_done), .O_ae_cfg_done(cam_ae_cfg_done),
        .O_sensor_id(cam_sensor_id), .O_sensor_id_ok(cam_sensor_id_ok),
        .O_iic_error(cam_iic_error)
    );

    board_mipi_dphy_rx #(
        .DPHY_RX_LOCATION("DPHY0"), .HS_EQUALIZER("3dB"),
        .HS_VGA_GAIN("8dB"), .LANE_NUM(2), .BYTE_NUM(1)
    ) u_mipi_dphy_rx_ph1p_mipiio_wrapper (
        .I_lp_clk(clk_100m), .I_rst(~lp_rst_n),
        .I_clk_lane_in_delay(6'd0), .I_data_lane0_in_delay(6'd0),
        .I_data_lane1_in_delay(6'd0), .I_data_lane2_in_delay(6'd0),
        .I_data_lane3_in_delay(6'd0), .I_lane_invert(2'b00),
        .O_hs_rx_clk(csi_rx_clk), .O_hs_rx_valid(hs_rx_valid),
        .O_hs_rx_data(hs_rx_data), .O_lp_rx_lane0_p(), .O_lp_rx_lane0_n(),
        .I_lp_tx_en(1'b0), .I_lp_tx_lane0_p(1'b1), .I_lp_tx_lane0_n(1'b1),
        .O_lane_match_error(), .O_lane_error(lane_error),
        .IO_rx_clk_pad_n(IO_rx_clk_pad_n), .IO_rx_clk_pad_p(IO_rx_clk_pad_p),
        .IO_rx_data_pad_n(IO_rx_data_pad_n), .IO_rx_data_pad_p(IO_rx_data_pad_p)
    );

    csi_packet_guard u_camera_packet_guard(
        .i_clk(csi_rx_clk),.i_rst_n(csi_rst_n),
        .i_hs_valid(hs_rx_valid),.i_hs_data(hs_rx_data),
        .i_lane_fault(|lane_error),.o_fault(camera_packet_fault),
        .o_diag_update(csi_diag_update),.o_diag(csi_diag));

    csi_unpacket_2lane u_csi_unpack (
        .I_clk(csi_rx_clk), .I_rst_n(csi_rst_n),
        .I_hs_valid(hs_rx_valid), .I_hs_data(hs_rx_data),
        .O_csi_frame_start(csi_frame_start), .O_csi_frame_end(csi_frame_end),
        .O_csi_valid(csi_valid), .O_csi_data(csi_data)
    );

    raw10_unpacket_2lane u_raw10_unpack (
        .I_clk(csi_rx_clk), .I_rst_n(csi_rst_n),
        .I_csi_frame_start(csi_frame_start), .I_csi_frame_end(csi_frame_end),
        .I_csi_valid(csi_valid), .I_csi_data(csi_data),
        .I_camera_black_level(10'd0),
        .O_raw10_frame_start(raw10_frame_start),
        .O_raw10_frame_end(raw10_frame_end), .O_raw10_valid(raw10_valid),
        .O_raw10_data(raw10_data)
    );

    uial2axis #(.IMG_WIDTH(1280),.IMG_HEIGHT(720),.INPUT_DATA_WIDTH(40))
    u_native_to_axis (
        .I_native_clk(csi_rx_clk), .I_rst_n(csi_rst_n),
        .I_data(raw10_data), .I_data_valid(raw10_valid),
        .I_data_start(raw10_frame_start), .I_data_end(raw10_frame_end),
        .axis_tvalid(axis_valid), .axis_tdata(axis_data),
        .axis_tuser(axis_user), .axis_tlast(axis_last)
    );

    raw10_center_crop_4ppc u_image_crop (
        .i_clk(csi_rx_clk), .i_rst_n(csi_rst_n),
        .i_data(axis_data), .i_last(axis_last),
        .i_valid(axis_valid), .i_sof(axis_user),
        .o_data(raw_data), .o_last(raw_last),
        .o_valid(raw_valid), .o_sof(raw_user)
    );

    wire [39:0] calibrated_raw;
    wire calibrated_valid,calibrated_sof,calibrated_last;
    generate if(ENABLE_COLOR_SHADING) begin:g_color_calibration
        // Covered-lens green median 16 RGB8 => nominal RAW10 pedestal 64.
        // Subtract before spatial gains; the uncorrected diagnostic bypass
        // above/below retains the original sensor data.
        raw_color_shading_4ppc #(.BLACK_LEVEL(10'd64)) u_color_shading(
            .i_clk(csi_rx_clk),.i_rst_n(csi_rst_n),.i_raw(raw_data),
            .i_valid(raw_valid),.i_sof(raw_user),.i_last(raw_last),
            .o_raw(calibrated_raw),.o_valid(calibrated_valid),
            .o_sof(calibrated_sof),.o_last(calibrated_last));
    end else begin:g_uncorrected_camera
        assign calibrated_raw=raw_data;assign calibrated_valid=raw_valid;
        assign calibrated_sof=raw_user;assign calibrated_last=raw_last;
    end endgenerate
    bayer_bilinear_4ppc #(.HEIGHT(600),.WIDTH(1024),.BAYER_MODE("BGGR")) u_demosaic (
        .i_clk(csi_rx_clk),.i_rst_n(csi_rst_n),
        .i_raw(calibrated_raw),.i_valid(calibrated_valid),.i_sof(calibrated_sof),.i_last(calibrated_last),
        .o_last(demosaic_last),.o_sof(demosaic_user),.o_rgb(demosaic_rgb),
        .o_valid(demosaic_valid),.o_fault(demosaic_fault));
    assign demosaic_ready=1'b1;
    assign demosaic_data=128'd0;

    capture_display_latency u_frame_latency(
      .sys_clk(I_sys_clk),.sys_rst_n(sys_rst_n),.source_clk(csi_rx_clk),
      .source_rst_n(csi_rst_n),.csi_start(csi_frame_start),
      .source_sof(demosaic_user && demosaic_valid),.next_source_id(stream_meta[37:6]+32'd1),
      .display_clk(hdmi_pixel_clk),.display_rst_n(pixel_rst_n),
      .display_vsync(osd_vsync),.display_de(osd_de),.display_id(bound_id),
      .valid(latency_valid),.milliseconds(latency_ms));
    contest_vision_subsystem #(.IMG_WIDTH(1024),.IMG_HEIGHT(600),
        .NORMAL_IMAGE_AWB(NORMAL_IMAGE_AWB),
        .ENABLE_HDMI_MENU(ENABLE_HDMI_MENU),
        .SYS_CLK_HZ(50000000),.AE_EXP_INIT(1991),.AE_EXP_MAX(1991),.AE_GAIN_MAX(176),
        .DDR_META_FRAME_DELAY(1),.BUFFER_BOUND_META(1),.ENABLE_REMOTE(ENABLE_NETWORK_DEBUG),
        .ENABLE_DETECTION(ENABLE_DETECTION),.ENABLE_HEAVY_FILTERS(ENABLE_HEAVY_FILTERS),
        .ENABLE_EVA(ENABLE_EVA),
        .ENABLE_CARTOON(ENABLE_CARTOON),.ENABLE_CARTOON_TEMPORAL(ENABLE_CARTOON_TEMPORAL),
        .ENABLE_MOTION(ENABLE_MOTION),.ENABLE_ARM(ENABLE_ARM),
        .ENABLE_QC_3D(ENABLE_QC_3D),.ENABLE_QC_COMPONENTS(ENABLE_QC_COMPONENTS),
        .ENABLE_QC_VOXELS(ENABLE_QC_VOXELS),.MODE_BITMAP(MODE_BITMAP)) u_vision (
        .i_sys_clk(I_sys_clk), .i_sys_rst_n(sys_rst_n),
        .i_key_n(menu_keys_n),.i_menu_enabled(menu_enabled && !capture_mode),
        .i_capture_mode(capture_mode),.o_key_press(capture_keys),
        .i_remote_mode_valid(remote_mode_valid),.i_remote_mode(remote_mode),
        .i_remote_threshold_valid(remote_threshold_valid),.i_remote_threshold(remote_threshold),
        .o_menu_open(menu_open_sys),.o_remote_rejected(remote_rejected),.o_sys_mode(sys_mode),.o_sys_threshold(sys_threshold),
        .o_display_mode(display_mode),.o_status_hdmi(hdmi_status),.o_debug_telemetry(debug_telemetry_hdmi),
        .i_isp_clk(csi_rx_clk), .i_isp_rst_n(csi_rst_n),
        .i_isp_sof(demosaic_user), .i_isp_eof(demosaic_last),
        .i_isp_valid(demosaic_valid), .i_isp_rgb(demosaic_rgb),
        .o_isp_sof(vision_sof), .o_isp_eof(vision_eol),
        .o_isp_valid(vision_valid), .o_isp_rgb(vision_rgb),
        .i_cam_cfg_clk(clk_24m), .i_cam_cfg_rst_n(cam_rst_n),
        .i_cam_cfg_done(cam_cfg_done), .i_cam_ae_cfg_done(cam_ae_cfg_done),
        .o_cam_ae_req(cam_ae_req), .o_cam_exposure(cam_exposure),
        .o_cam_gain_index(cam_gain_index),
        .i_hdmi_clk(hdmi_pixel_clk), .i_hdmi_rst_n(pixel_rst_n),
        .i_hdmi_vsync(mixer_vsync), .i_hdmi_hsync(mixer_hsync),
        .i_hdmi_de(mixer_de), .i_hdmi_rgb(mixer_rgb),
        .i_bound_meta_update(bound_meta_update),.i_bound_meta(bound_meta),
        .i_history_valid(history_valid),.i_history(history_old),
        .o_history_read(history_read_pixel),.o_history_write(history_write_pixel),.o_history(history_new),
        .o_history_configuration(history_configuration),
        .i_history_status(history_status),
        .i_history_diagnostics(history_diagnostics),
        .i_bound_frame_id(bound_id),.i_view_cmd(view_cmd),.i_view_id(view_id),
        .i_teach_2d(teach_2d),.i_clear_2d(clear_2d),
        .i_coeff_valid(coeff_valid),.i_coeff_clear(coeff_clear),.i_coeff_view(coeff_view),.i_coeff_cos(coeff_cos),.i_coeff_sin(coeff_sin),
        .i_latency_valid(latency_valid),.i_latency_ms(latency_ms),
        .i_capture_fps(capture_fps),.i_board_status(status1),.i_csi_diag(csi_diag_hdmi),
        .i_arm_status(arm_status_hdmi),.i_arm_points(arm_points_hdmi),
        .o_qc_views_loaded(qc_views),.o_qc_hull_valid(qc_hull_valid),.o_qc_overrun(qc_overrun),
        .o_qc_coefficients_loaded(qc_coefficients),
        .o_target_frame_id(target_frame_id),
        .o_stream_meta(stream_meta),.o_frame_meta_update(frame_meta_update),.o_frame_meta(frame_meta),
        .o_hdmi_vsync(osd_vsync), .o_hdmi_hsync(osd_hsync),
        .o_hdmi_de(osd_de), .o_hdmi_rgb(osd_rgb),
        .o_target_valid(target_valid), .o_target_class(target_class),
        .o_target_center_x(target_center_x), .o_target_center_y(target_center_y),
        .o_target_bbox_x0(target_bbox_x0), .o_target_bbox_x1(target_bbox_x1),
        .o_target_bbox_y0(target_bbox_y0), .o_target_bbox_y1(target_bbox_y1)
    );

    wire template_vs,template_hs,template_de;
    wire [23:0] template_rgb;
    wire vision_command_sys,vision_command_pixel,vision_command_busy;
    wire [15:0] vision_code_sys,vision_code_pixel;
    wire [239:0] vision_status_pixel,vision_status_sys;
    wire vision_status_update;
    cdc_bus_toggle #(.WIDTH(16)) u_template_command(
      .i_src_clk(I_sys_clk),.i_src_rst_n(sys_rst_n),.i_src_update(vision_command_sys || capture_command),.i_src_data(capture_command?capture_code:vision_code_sys),.o_src_busy(vision_command_busy),
      .i_dst_clk(hdmi_pixel_clk),.i_dst_rst_n(pixel_rst_n),.o_dst_update(vision_command_pixel),.o_dst_data(vision_code_pixel));
    cdc_bus_toggle #(.WIDTH(240)) u_template_status(
      .i_src_clk(hdmi_pixel_clk),.i_src_rst_n(pixel_rst_n),.i_src_update(vision_status_update),.i_src_data(vision_status_pixel),.o_src_busy(),
      .i_dst_clk(I_sys_clk),.i_dst_rst_n(sys_rst_n),.o_dst_update(),.o_dst_data(vision_status_sys));
    cdc_bus_toggle #(.WIDTH(16)) u_sort_status_cdc(
      .i_src_clk(I_sys_clk),.i_src_rst_n(sys_rst_n),.i_src_update(1'b1),.i_src_data({capture_mode,sort_status_sys[14:0]}),.o_src_busy(),
      .i_dst_clk(hdmi_pixel_clk),.i_dst_rst_n(pixel_rst_n),.o_dst_update(),.o_dst_data(sort_status_pixel));
    part_template_hdmi #(.ROI_X(64),.ROI_Y(37),.ROI_W(896),.ROI_H(525)) u_template_video(
      .clk(hdmi_pixel_clk),.rst_n(pixel_rst_n),.vsync(osd_vsync),.hsync(osd_hsync),.de(osd_de),.rgb(osd_rgb),
      .analysis_vsync(mixer_vsync),.analysis_de(mixer_de),.analysis_rgb(mixer_rgb),
      .source_frame_id(bound_id),.command_update(vision_command_pixel),.command_code(vision_code_pixel),
      .out_vsync(template_vs),.out_hsync(template_hs),.out_de(template_de),.out_rgb(template_rgb),
      .show_overlay(display_mode==19),.capture_mode(sort_status_pixel[15]),
      .flash_cs_n(O_template_flash_cs_n),.flash_clk(O_template_flash_clk),.flash_mosi(O_template_flash_mosi),.flash_miso(I_template_flash_miso),
      .sort_status(sort_status_pixel),.status_bus(vision_status_pixel),.status_update(vision_status_update));
    rgb96_to_ddr128_packer u_ddr_width (
        .i_clk(csi_rx_clk), .i_rst_n(csi_rst_n),
        .i_frame_start(vision_sof), .i_valid(vision_valid),
        .i_data(vision_rgb), .o_frame_start(packed_frame_start),
        .o_valid(packed_valid), .o_data(packed_data)
    );

    board_frame_writer u_video_in (
        .i_rst_n(core_rst_n), .i_camera_clk(csi_rx_clk),
        .i_camera_sof(packed_frame_start),
        .i_camera_valid(packed_valid), .i_camera_data(packed_data),
        .i_camera_meta(stream_meta),.o_meta(write_meta),
        .i_ddr_clk(ddr_clk), .i_ready(legacy_ready),.i_enable(ddr_init_done),
        .i_read_busy(video_out_rd_busy), .o_busy(video_in_wr_busy),
        .o_display_rp(video_out_rp), .o_write(vi_ddr_wr_en),
        .o_sof(vi_sof), .o_overflow(vi_overflow),
        .o_addr(vi_ddr_wr_addr), .o_data(vi_ddr_wr_data)
    );

    // USB UART belongs exclusively to the qualified arm packet console.
    // Camera menu keys cannot issue a physical jog; K4 remains a stop input.
    robot_arm_zlink_console #(.ALLOW_MULTI_MANUAL(1),.SIGNATURE(16'h3924)) u_teach_console(
      .I_sys_clk(I_sys_clk),.I_rst_n(sys_rst_n),.I_usb_uart_rx(I_uart_rx),
      .I_arm_uart_rx(I_arm_uart_rx),.I_key1_n(1'b1),.I_key2_n(1'b1),
      .I_key4_n(I_capture_key_n),.I_sw4(1'b1),
      .I_sort_enable(sys_mode==19 && !menu_open_sys && !capture_mode),.I_sort_start_n(I_button[1]),.I_template_key_n(1'b1),
      .O_buzzer_n(O_buzzer_n),.O_sort_status(sort_status_sys),
      .O_usb_uart_tx(O_uart_tx),.O_arm_uart_tx(O_arm_uart_tx),.O_manual_mask(),.O_vision_update(vision_command_sys),.O_vision_code(vision_code_sys),
      .I_vision_status(vision_status_sys),.I_vision_busy(vision_command_busy || vision_status_sys[230]));
    assign command_byte_valid=0;assign command_byte=0;
    assign capture_req=0;assign capture_resend=0;assign capture_byte_ready=1;
    assign manual_view_cmd=0;assign manual_view_id=0;assign teach_2d=0;assign clear_2d=0;
    generate if(ENABLE_ARM) begin:g_arm
      arm_vision_bridge u_arm_bridge(
        .sys_clk(I_sys_clk),.sys_rst_n(sys_rst_n),.pixel_clk(hdmi_pixel_clk),.pixel_rst_n(pixel_rst_n),
        .vsync(mixer_vsync),.de(mixer_de),.mode(display_mode),.target_valid(target_valid),
        .frame_id(target_frame_id),.x(target_center_x),.y(target_center_y),
        .byte_valid(command_byte_valid),.byte_data(command_byte),.packet_active(arm_packet_active),
        .dat(1'b1),.dat_drive_low(arm_dat_drive_low),.views_loaded(qc_views),.hull_valid(qc_hull_valid),
        .qc_overrun(qc_overrun),.coefficients_loaded(qc_coefficients),.view_valid(arm_view_cmd),.view_id(arm_view_id),
        .status_hdmi(arm_status_hdmi),.points_hdmi(arm_points_hdmi),.scanning_sys(arm_scanning_sys),
        .coeff_valid(coeff_valid),.coeff_clear(coeff_clear),.coeff_view(coeff_view),.coeff_cos(coeff_cos),.coeff_sin(coeff_sin));
    end else begin:g_no_arm
      assign arm_packet_active=0;assign arm_view_cmd=0;assign arm_view_id=0;
      assign arm_dat_drive_low=0;assign arm_status_hdmi=0;assign arm_points_hdmi=0;
      assign arm_scanning_sys=0;
      assign coeff_valid=0;assign coeff_clear=0;assign coeff_view=0;assign coeff_cos=0;assign coeff_sin=0;
    end endgenerate
    board_capture_export u_capture (
        .i_clk(ddr_clk),.i_rst_n(ddr_rst_n),.i_capture(capture_req),
        .i_resend(capture_resend),.i_init_done(ddr_init_done),
        .i_fault(vi_overflow | lane_fault1),
        .i_wr(vi_ddr_wr_en),.i_sof(vi_sof),.i_wr_busy(video_in_wr_busy),
        .i_wr_addr(vi_ddr_wr_addr),.i_wr_data(vi_ddr_wr_data),
        .i_rd(vo_ddr_rd_en),.i_rd_busy(video_out_rd_busy),.i_rd_addr(vo_ddr_rd_addr),
        .i_ready(legacy_ready),.i_rd_valid(legacy_rd_valid),
        .i_rd_data(legacy_rd_data),.o_live_ready(display_ready),
        .o_live_rd_valid(display_rd_valid),.o_block_reads(capture_block_reads),
        .o_wr(ddr_user_wr_en),.o_rd(ddr_user_rd_en),.o_addr(ddr_user_addr),
        .o_wr_data(ddr_user_wr_data),.o_byte_valid(capture_byte_valid),
        .o_byte(capture_byte),.i_byte_ready(capture_byte_ready),
        .o_busy(capture_busy),.o_error(capture_error),.o_saved(capture_saved),
        .o_sent(capture_sent),.o_batch(capture_batch));

    cdc_bus_toggle #(.WIDTH(124)) u_frame_meta_to_ddr(
        .i_src_clk(csi_rx_clk),.i_src_rst_n(csi_rst_n),
        .i_src_update(frame_meta_update),.i_src_data(frame_meta),.o_src_busy(),
        .i_dst_clk(ddr_clk),.i_dst_rst_n(ddr_rst_n),
        .o_dst_update(ddr_meta_update),.o_dst_data(ddr_frame_meta));
    frame_metadata_binder #(.PIPELINE_INPUT(1)) u_frame_metadata(
        .i_clk(ddr_clk),.i_rst_n(ddr_rst_n),.i_fault(vi_overflow | lane_fault1),
        .i_write(ddr_user_wr_en),.i_sof(vi_sof),.i_addr(ddr_user_addr),
        .i_stream_meta(write_meta),.i_meta_update(ddr_meta_update),.i_meta(ddr_frame_meta),
        .i_display_start(selected_frame_start),
        .i_display_slot(capture_busy?saved_display_rp:video_out_rp),
        .o_update(bound_meta_ddr_update),.o_meta(bound_meta_ddr),.o_frame_id(bound_id_ddr));
    cdc_bus_toggle #(.WIDTH(124)) u_selected_meta_to_hdmi(
        .i_src_clk(ddr_clk),.i_src_rst_n(ddr_rst_n),
        .i_src_update(bound_meta_ddr_update),.i_src_data(bound_bundle),.o_src_busy(),
        .i_dst_clk(hdmi_pixel_clk),.i_dst_rst_n(pixel_rst_n),
        .o_dst_update(bound_meta_update),.o_dst_data(bound_bundle_hdmi));
    board_frame_reader u_video_out (
        .o_frame_start(selected_frame_start),
        .I_rst_n(core_rst_n), .I_ddr_clk(ddr_clk),
        .O_video_out_rd_busy(video_out_rd_busy),
        .I_video_in_wr_busy(video_in_wr_busy | capture_block_reads),
        .I_video_out_rp(capture_busy ? saved_display_rp : video_out_rp),
        .O_ddr_user_rd_en(vo_ddr_rd_en), .O_ddr_user_addr(vo_ddr_rd_addr),
        .I_ddr_user_ready(display_ready),
        .I_ddr_user_rd_valid(display_rd_valid),
        .I_ddr_user_rd_data(legacy_rd_data),
        .I_dsi_clk(hdmi_pixel_clk), .I_video_vsync(~hdmi_vsync),
        .I_video_rd_en(hdmi_window_rd_en), .O_vdieo_data(video_rd_data)
    );

    mc_to_user_interface u_mc_to_user_interface (
        .I_clk(ddr_clk), .I_rst_n(ddr_rst_n),
        .I_ddr_user_wr_en(net_mc_wr), .I_ddr_user_rd_en(net_mc_rd),
        .I_ddr_user_addr(net_mc_addr), .I_ddr_user_wr_data(net_mc_data),
        .O_ddr_user_ready(ddr_user_ready),
        .O_ddr_user_rd_valid(ddr_user_rd_valid),
        .O_ddr_user_rd_data(ddr_user_rd_data),
        .O_mc_app_en(mc_app_en), .O_mc_app_addr(mc_app_addr),
        .O_mc_app_cmd(mc_app_cmd), .I_mc_app_rdy(mc_app_rdy),
        .O_mc_app_wdf_wren(mc_app_wdf_wren),
        .O_mc_app_wdf_data(mc_app_wdf_data),
        .O_mc_app_wdf_end(mc_app_wdf_end), .O_mc_app_wdf_mask(mc_app_wdf_mask),
        .I_mc_app_wdf_rdy(mc_app_wdf_rdy), .I_mc_app_rd_data(mc_app_rd_data),
        .I_mc_app_rd_data_end(mc_app_rd_data_end),
        .I_mc_app_rd_data_valid(mc_app_rd_data_valid)
    );

    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper (
        // The retained IP sets PLL0_REFCLK_FREQ=100 and tCK=1876 ps.
        .I_sys_clk(clk_100m), .I_sys_rst_n(lp_rst_n), .O_ddr_clk(ddr_clk),
        .O_init_calib_complete(ddr_init_done), .I_mc_app_addr(mc_app_addr),
        .I_mc_app_cmd(mc_app_cmd), .I_mc_app_en(mc_app_en),
        .I_mc_app_wdf_data(mc_app_wdf_data), .I_mc_app_wdf_end(mc_app_wdf_end),
        .I_mc_app_wdf_mask(mc_app_wdf_mask), .I_mc_app_wdf_wren(mc_app_wdf_wren),
        .O_mc_app_rd_data(mc_app_rd_data),
        .O_mc_app_rd_data_end(mc_app_rd_data_end),
        .O_mc_app_rd_data_valid(mc_app_rd_data_valid),
        .O_mc_app_rdy(mc_app_rdy), .O_mc_app_wdf_rdy(mc_app_wdf_rdy),
        .ddr_addr(ddr_addr), .ddr_ba(ddr_ba), .ddr_cke(ddr_cke),
        .ddr_odt(ddr_odt), .ddr_cs_n(ddr_cs_n), .ddr_ras_n(ddr_ras_n),
        .ddr_cas_n(ddr_cas_n), .ddr_we_n(ddr_we_n), .ddr_ck_p(ddr_ck_p),
        .ddr_ck_n(ddr_ck_n), .ddr_dm(ddr_dm), .ddr_dq(ddr_dq),
        .ddr_dqs_p(ddr_dqs_p), .ddr_dqs_n(ddr_dqs_n)
    );

    uivtc #(.H_ActiveSize(1024),.H_FrameSize(1344),.H_SyncStart(1184),
            .H_SyncEnd(1208),.V_ActiveSize(600),.V_FrameSize(635),
            .V_SyncStart(612),.V_SyncEnd(614)) u_hdmi_vtc (
        .I_vtc_rstn(pixel_rst_n), .I_vtc_clk(hdmi_pixel_clk),
        .O_vtc_vs(hdmi_vsync), .O_vtc_hs(hdmi_hsync),
        .O_vtc_de_valid(hdmi_de), .O_vtc_user(hdmi_user), .O_vtc_last(hdmi_last)
    );

    board_hdmi_mixer u_hdmi_mixer (
        .clk(hdmi_pixel_clk), .rst_n(pixel_rst_n),
        .vsync(hdmi_vsync), .hsync(hdmi_hsync), .de(hdmi_de),
        .read_enable(hdmi_window_rd_en), .reader_rgb(video_rd_data),
        .out_vsync(mixer_vsync), .out_hsync(mixer_hsync),
        .out_de(mixer_de), .out_rgb(mixer_rgb)
    );

    board_hdmi_tx u_hdmi_tx (
        .I_serial_rst_n(serial_rst_n),
        .I_pixel_clk(hdmi_pixel_clk), .I_serial_clk(hdmi_serial_clk),
        .I_rst(~pixel_rst_n), .I_key_in(1'b0), .I_edid_read_trig(1'b0),
        .O_edid_read_valid(), .O_edid_read_data(), .I_video_rgb_enable(1'b1),
        .I_video_in_vs(template_vs), .I_video_in_de(template_de),
        .I_video_in_user(1'b0), .I_video_in_valid(1'b0),
        .I_video_in_last(1'b0), .O_video_in_ready(), .I_video_in_data(template_rgb),
        .I_audio_valid(1'b0), .I_audio_left_data(24'd0),
        .I_audio_right_data(24'd0), .I_i2s_BCLK(1'b0),
        .I_i2s_LRCK(1'b0), .I_i2s_DOUT(1'b0), .O_ddc_scl(),
        .O_hdmi_clk_p(), .O_hdmi_tx_p(), .O_tmds_ch0_p(O_tmds_ch0_p),
        .O_tmds_ch1_p(O_tmds_ch1_p), .O_tmds_ch2_p(O_tmds_ch2_p),
        .O_tmds_clk_p(O_tmds_clk_p)
    );

    wire _unused = ^{vision_eol,target_class,target_center_x,target_center_y,
                     target_bbox_x0,target_bbox_x1,target_bbox_y0,target_bbox_y1,
                     demosaic_ready,cam_sensor_id,capture_error,capture_saved,
                     capture_sent,capture_batch,capture_busy_d};
endmodule
`default_nettype wire
