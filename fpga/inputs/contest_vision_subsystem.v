`timescale 1ns/1ps

// Self-developed contest subsystem. Board-vendor MIPI/DDR/HDMI IP connects only
// at these documented ports and is not copied into this source tree.
module contest_vision_subsystem #(
    parameter NORMAL_IMAGE_AWB = 1,
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600,
    parameter SYS_CLK_HZ = 50000000,
    parameter AE_EXP_MAX = 2246,
    parameter AE_EXP_INIT = 1500,
    parameter AE_GAIN_MAX = 89,
    // Number of HDMI frame boundaries by which detection metadata trails the
    // newly received ISP result.  Set this to the measured DDR frame-buffer
    // latency (0..3) after board integration.
    parameter DDR_META_FRAME_DELAY = 1,
    parameter BUFFER_BOUND_META = 0,
    parameter ENABLE_REMOTE = 0,
    parameter ENABLE_HDMI_MENU = 0,
    parameter ENABLE_DETECTION = 1,
    parameter ENABLE_HEAVY_FILTERS = 1,
    parameter ENABLE_EVA = 0,
    parameter ENABLE_CARTOON = 0,
    parameter ENABLE_CARTOON_TEMPORAL = 0,
    parameter ENABLE_MOTION = 0,
    parameter ENABLE_ARM = 0,
    parameter ENABLE_LEGACY_STYLES = 0,
    parameter ENABLE_QC_3D = 1,
    parameter ENABLE_QC_COMPONENTS = 1,
    parameter ENABLE_QC_VOXELS = 1,
    parameter [63:0] MODE_BITMAP = 64'h7ffff
)(
    input  wire        i_sys_clk,
    input  wire        i_sys_rst_n,
    input  wire [3:0]  i_key_n,
    input wire i_menu_enabled,
    input wire i_capture_mode,output wire [3:0] o_key_press,
    input wire i_raw_calibration,
    input wire i_remote_mode_valid,input wire [5:0] i_remote_mode,
    input wire i_remote_threshold_valid,input wire [9:0] i_remote_threshold,
    output wire o_remote_rejected,
    output wire o_menu_open,
    output wire [5:0] o_sys_mode,o_display_mode,
    output wire [9:0] o_sys_threshold,
    output wire [231:0] o_status_hdmi,
    output wire [255:0] o_debug_telemetry,

    // Demosaic RGB888 4PPC domain.
    input  wire        i_isp_clk,
    input  wire        i_isp_rst_n,
    output wire [5:0] o_requested_capture_mode,
    input wire [5:0] i_capture_frame_mode,
    input  wire        i_isp_sof,
    // Legacy name: this is the vendor stream's TLAST/end-of-line marker, not
    // a unique end-of-frame pulse.
    input  wire        i_isp_eof,
    input  wire        i_isp_valid,
    input  wire [95:0] i_isp_rgb,
    output wire        o_isp_sof,
    output wire        o_isp_eof, // delayed end-of-line/TLAST marker
    output wire        o_isp_valid,
    output wire [95:0] o_isp_rgb,

    // Camera I2C configuration module interface (approximately 24 MHz domain).
    input  wire        i_cam_cfg_clk,
    input  wire        i_cam_cfg_rst_n,
    input  wire        i_cam_cfg_done,
    input  wire        i_cam_ae_cfg_done,
    output wire        o_cam_ae_req,
    output wire [15:0] o_cam_exposure,
    output wire [15:0] o_cam_gain_index,

    // DDR video-out / HDMI 1PPC domain.
    input  wire        i_hdmi_clk,
    input  wire        i_hdmi_rst_n,
    input  wire        i_hdmi_vsync,
    input  wire        i_hdmi_hsync,
    input  wire        i_hdmi_de,
    input  wire [23:0] i_hdmi_rgb,
    input wire i_bound_meta_update,
    input wire [91:0] i_bound_meta,
    input wire [31:0] i_bound_frame_id,
    input wire i_history_valid,input wire [31:0] i_history,
    output wire o_history_read,o_history_write,output wire [31:0] o_history,
    output wire [9:0] o_history_configuration,
    input wire [7:0] i_history_status,
    input wire [127:0] i_history_diagnostics,
    input wire i_view_cmd,input wire [2:0] i_view_id,
    input wire i_teach_2d,i_clear_2d,
    input wire i_coeff_valid,i_coeff_clear,input wire [2:0] i_coeff_view,
    input wire signed [9:0] i_coeff_cos,i_coeff_sin,
    input wire i_latency_valid,input wire [15:0] i_latency_ms,
    input wire [7:0] i_capture_fps,i_board_status,
    input wire [63:0] i_csi_diag,
    input wire [7:0] i_arm_status,input wire [8:0] i_arm_points,
    output wire [5:0] o_qc_views_loaded,output wire o_qc_hull_valid,o_qc_overrun,
    output wire [5:0] o_qc_coefficients_loaded,
    output wire [31:0] o_target_frame_id,
    output wire [37:0] o_stream_meta,
    output wire o_frame_meta_update,
    output wire [123:0] o_frame_meta,
    output wire        o_hdmi_vsync,
    output wire        o_hdmi_hsync,
    output wire        o_hdmi_de,
    output wire [23:0] o_hdmi_rgb,

    // HDMI-frame-aligned metadata.  A later UART block in another clock
    // domain must cross this bus with its own handshake (or consume the ISP
    // publication event before this display-delay stage).
    output wire        o_target_valid,
    output wire [3:0]  o_target_class,
    output wire [10:0] o_target_center_x,
    output wire [9:0]  o_target_center_y,
    output wire [10:0] o_target_bbox_x0,
    output wire [10:0] o_target_bbox_x1,
    output wire [9:0]  o_target_bbox_y0,
    output wire [9:0]  o_target_bbox_y1
);
    wire [5:0] mode_sys;
    wire [9:0] threshold_sys;
    wire menu_open_sys;
    assign o_menu_open=menu_open_sys;
    wire [5:0] menu_cursor_sys,menu_first_sys;
    reg mode_ctrl_init;
    reg [15:0] mode_ctrl_last;
    reg mode_ctrl_update;
    wire [5:0] capture_mode_sys=mode_sys==0 && !i_raw_calibration ? 6'd14 : mode_sys;
    wire [15:0] mode_ctrl_sys = {capture_mode_sys,threshold_sys};
    wire [15:0] mode_ctrl_isp_rx;
    wire mode_ctrl_busy;
    wire mode_ctrl_isp_update;
    reg [15:0] mode_ctrl_isp;
    wire [5:0] mode_isp = mode_ctrl_isp[15:10];
    assign o_requested_capture_mode=mode_isp;
    wire [2:0] core_mode_isp = i_capture_frame_mode==14 ? 3'd7 :
        (!NORMAL_IMAGE_AWB && i_capture_frame_mode==0) ? 3'd0 : 3'd5;
    wire [9:0] threshold_isp = mode_ctrl_isp[9:0];

    assign o_sys_mode=mode_sys;assign o_sys_threshold=threshold_sys;
    vision_mode_ctrl #(.CLK_HZ(SYS_CLK_HZ),.MAX_MODE(23),.ENABLE_REMOTE(ENABLE_REMOTE),
        .ENABLE_MENU(ENABLE_HDMI_MENU),
        .MODE_BITMAP(MODE_BITMAP)) u_mode(
        .i_clk(i_sys_clk),.i_rst_n(i_sys_rst_n),.i_key_n(i_key_n),
        .i_menu_enabled(i_menu_enabled),.i_capture_mode(i_capture_mode),.o_key_press(o_key_press),.o_menu_open(menu_open_sys),
        .o_menu_cursor(menu_cursor_sys),.o_menu_first(menu_first_sys),
        .o_mode(mode_sys),.o_edge_threshold(threshold_sys),
        .i_remote_mode_valid(i_remote_mode_valid),.i_remote_mode(i_remote_mode),
        .i_remote_threshold_valid(i_remote_threshold_valid),.i_remote_threshold(i_remote_threshold),
        .o_remote_rejected(o_remote_rejected)
    );

    // Generate a transaction for the initial value and every subsequent
    // change.  cdc_bus_toggle keeps the newest pending value if ISP is busy or
    // independently reset.
    always @(posedge i_sys_clk or negedge i_sys_rst_n) begin
        if(!i_sys_rst_n) begin
            mode_ctrl_init <= 1'b0;
            mode_ctrl_last <= {6'd0,10'd72};
            mode_ctrl_update <= 1'b0;
        end else begin
            mode_ctrl_update <= 1'b0;
            if(!mode_ctrl_init || mode_ctrl_sys != mode_ctrl_last) begin
                mode_ctrl_init <= 1'b1;
                mode_ctrl_last <= mode_ctrl_sys;
                mode_ctrl_update <= 1'b1;
            end
        end
    end

    cdc_bus_toggle #(.WIDTH(16)) u_mode_cdc(
        .i_src_clk(i_sys_clk),.i_src_rst_n(i_sys_rst_n),
        .i_src_update(mode_ctrl_update),.i_src_data(mode_ctrl_last),
        .o_src_busy(mode_ctrl_busy),
        .i_dst_clk(i_isp_clk),.i_dst_rst_n(i_isp_rst_n),
        .o_dst_update(mode_ctrl_isp_update),.o_dst_data(mode_ctrl_isp_rx)
    );

    always @(posedge i_isp_clk or negedge i_isp_rst_n) begin
        if(!i_isp_rst_n)
            mode_ctrl_isp <= {6'd0,10'd72};
        else if(mode_ctrl_isp_update)
            mode_ctrl_isp <= mode_ctrl_isp_rx;
    end

    wire [15:0] ae_exposure;
    wire [15:0] ae_gain;
    wire ae_update,ae_stable,awb_stable;
    wire [9:0] awb_r_gain,awb_b_gain;
    wire [15:0] pipe_cycles;
    wire target_valid;
    wire [3:0] target_class;
    wire [10:0] center_x,bbox_x0,bbox_x1;
    wire [9:0] center_y,bbox_y0,bbox_y1;
    wire [19:0] target_area;
    wire [7:0] target_quality;
    wire target_update;

    vision_core_4ppc #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
        .AE_EXP_INIT(AE_EXP_INIT),.AE_EXP_MAX(AE_EXP_MAX),.AE_GAIN_MAX(AE_GAIN_MAX),
        .ENABLE_DETECTION(ENABLE_DETECTION),.ENABLE_LEGACY_STYLES(ENABLE_LEGACY_STYLES)) u_core(
        .i_clk(i_isp_clk),.i_rst_n(i_isp_rst_n),
        .i_sof(i_isp_sof),.i_eof(i_isp_eof),.i_valid(i_isp_valid),.i_rgb(i_isp_rgb),
        .i_mode(core_mode_isp),.i_edge_threshold(threshold_isp),
        .o_sof(o_isp_sof),.o_eof(o_isp_eof),.o_valid(o_isp_valid),.o_rgb(o_isp_rgb),
        .o_exposure(ae_exposure),.o_gain_index(ae_gain),.o_ae_update(ae_update),
        .o_ae_stable(ae_stable),.o_awb_r_gain_q8(awb_r_gain),
        .o_awb_b_gain_q8(awb_b_gain),.o_awb_stable(awb_stable),
        .o_pipeline_cycles(pipe_cycles),.o_target_valid(target_valid),
        .o_target_update(target_update),
        .o_target_class(target_class),.o_center_x(center_x),.o_center_y(center_y),
        .o_bbox_x0(bbox_x0),.o_bbox_x1(bbox_x1),.o_bbox_y0(bbox_y0),.o_bbox_y1(bbox_y1),
        .o_target_area(target_area),.o_target_quality(target_quality)
    );

    wire ae_src_busy;
    ae_i2c_cdc_bridge u_ae_bridge(
        .i_src_clk(i_isp_clk),.i_src_rst_n(i_isp_rst_n),.i_src_update(ae_update),
        .i_src_exposure(ae_exposure),.i_src_gain(ae_gain),.o_src_busy(ae_src_busy),
        .i_dst_clk(i_cam_cfg_clk),.i_dst_rst_n(i_cam_cfg_rst_n),
        .i_cam_cfg_done(i_cam_cfg_done),.i_ae_cfg_done(i_cam_ae_cfg_done),
        .o_ae_req(o_cam_ae_req),.o_ae(o_cam_exposure),.o_ag(o_cam_gain_index)
    );

    // Detector metadata is published after all frame-end divisions have
    // completed.  Transfer that explicit publication event as one atomic
    // bundled-data transaction; never sample the 68-bit bus independently.
    wire [67:0] target_bus_isp = {target_valid,target_class,center_x,center_y,
                                  bbox_x0,bbox_x1,bbox_y0,bbox_y1};
    reg [31:0] source_frame_id,completed_frame_id;
    reg [5:0] source_mode,completed_mode;
    always @(posedge i_isp_clk or negedge i_isp_rst_n) begin
        if(!i_isp_rst_n) begin
            source_frame_id<=0;completed_frame_id<=0;source_mode<=0;completed_mode<=0;
        end else if(i_isp_sof && i_isp_valid) begin
            completed_frame_id<=source_frame_id;completed_mode<=source_mode;
            source_frame_id<=source_frame_id+1'b1;source_mode<=i_capture_frame_mode;
        end
    end
    assign o_stream_meta={source_frame_id,source_mode};
    assign o_frame_meta_update=target_update;
    wire [91:0] osd_meta_isp = {completed_mode,pipe_cycles,ae_stable,awb_stable,
                                target_bus_isp};
    assign o_frame_meta={completed_frame_id,osd_meta_isp};
    wire [91:0] osd_meta_hdmi_rx;
    wire osd_meta_hdmi_update;
    wire osd_meta_busy;

    cdc_bus_toggle #(.WIDTH(92)) u_osd_meta_cdc(
        .i_src_clk(i_isp_clk),.i_src_rst_n(i_isp_rst_n),
        .i_src_update(target_update),.i_src_data(osd_meta_isp),
        .o_src_busy(osd_meta_busy),
        .i_dst_clk(i_hdmi_clk),.i_dst_rst_n(i_hdmi_rst_n),
        .o_dst_update(osd_meta_hdmi_update),.o_dst_data(osd_meta_hdmi_rx)
    );

    reg hdmi_vsync_d;
    reg [91:0] osd_meta_pending;
    reg [91:0] osd_meta_d0,osd_meta_d1,osd_meta_d2;
    reg [91:0] osd_meta_commit;
    wire hdmi_frame_start = i_hdmi_vsync & ~hdmi_vsync_d;

    always @(posedge i_hdmi_clk or negedge i_hdmi_rst_n) begin
        if(!i_hdmi_rst_n) begin
            hdmi_vsync_d <= 1'b0;
            osd_meta_pending <= 92'd0;
            osd_meta_d0 <= 92'd0;
            osd_meta_d1 <= 92'd0;
            osd_meta_d2 <= 92'd0;
            osd_meta_commit <= 92'd0;
        end else begin
            hdmi_vsync_d <= i_hdmi_vsync;
            if(osd_meta_hdmi_update)
                osd_meta_pending <= osd_meta_hdmi_rx;

            if(BUFFER_BOUND_META) begin
                // Sent by the DDR reader's ACTUAL buffer selection event,
                // during blanking. No guessed fixed frame delay on the board.
                if(i_bound_meta_update) osd_meta_commit<=i_bound_meta;
            end else if(hdmi_frame_start) begin
                osd_meta_d0 <= osd_meta_pending;
                osd_meta_d1 <= osd_meta_d0;
                osd_meta_d2 <= osd_meta_d1;
                case(DDR_META_FRAME_DELAY)
                    0: osd_meta_commit <= osd_meta_pending;
                    1: osd_meta_commit <= osd_meta_d0;
                    2: osd_meta_commit <= osd_meta_d1;
                    default: osd_meta_commit <= osd_meta_d2;
                endcase
            end
        end
    end

    reg [31:0] displayed_frame_id;
    always @(posedge i_hdmi_clk or negedge i_hdmi_rst_n)
      if(!i_hdmi_rst_n) displayed_frame_id<=0;
      else if(i_bound_meta_update) displayed_frame_id<=i_bound_frame_id;
    wire [5:0] mode_hdmi = osd_meta_commit[91:86];
    assign o_display_mode=mode_hdmi;
    wire [15:0] latency_hdmi = osd_meta_commit[85:70];
    wire ae_hdmi = osd_meta_commit[69];
    wire awb_hdmi = osd_meta_commit[68];
    wire [67:0] target_hdmi = osd_meta_commit[67:0];

    assign o_target_valid    = target_hdmi[67];
    assign o_target_class    = target_hdmi[66:63];
    assign o_target_center_x = target_hdmi[62:52];
    assign o_target_center_y = target_hdmi[51:42];
    assign o_target_bbox_x0  = target_hdmi[41:31];
    assign o_target_bbox_x1  = target_hdmi[30:20];
    assign o_target_bbox_y0  = target_hdmi[19:10];
    assign o_target_bbox_y1  = target_hdmi[9:0];

    wire filter_vsync,filter_hsync,filter_de;
    wire [23:0] filter_rgb;
    // Threshold commands cross as a coherent bus, then commit in blanking.
    // A 10-bit comparison stays black for thresholds greater than 255.
    wire threshold_hdmi_update;wire [9:0] threshold_hdmi_rx;
    reg [9:0] threshold_hdmi_pending,threshold_hdmi;
    cdc_bus_toggle #(.WIDTH(10)) u_threshold_hdmi_cdc(
      .i_src_clk(i_sys_clk),.i_src_rst_n(i_sys_rst_n),.i_src_update(mode_ctrl_update),
      .i_src_data(threshold_sys),.o_src_busy(),
      .i_dst_clk(i_hdmi_clk),.i_dst_rst_n(i_hdmi_rst_n),
      .o_dst_update(threshold_hdmi_update),.o_dst_data(threshold_hdmi_rx));
    always @(posedge i_hdmi_clk or negedge i_hdmi_rst_n)
      if(!i_hdmi_rst_n) begin threshold_hdmi_pending<=10'd72;threshold_hdmi<=10'd72;end
      else begin
        if(threshold_hdmi_update)threshold_hdmi_pending<=threshold_hdmi_rx;
        if(hdmi_frame_start)threshold_hdmi<=threshold_hdmi_pending;
      end
    display_filter_1ppc #(.LATCH_MODE(0),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
        .MODE_BITMAP(MODE_BITMAP),.USE_CONFIGURED_THRESHOLD(1),.ENABLE_BINARY_DENOISE(1),.ENABLE_ADPT_STABLE(0),
        .ENABLE_HEAVY_FILTERS(ENABLE_HEAVY_FILTERS),.ENABLE_EVA(ENABLE_EVA),
        .ENABLE_CARTOON(ENABLE_CARTOON),.ENABLE_CARTOON_TEMPORAL(ENABLE_CARTOON_TEMPORAL)) u_display_filter(
        .i_history_valid(i_history_valid),.i_history(i_history),
        .o_history_read(o_history_read),.o_history_write(o_history_write),.o_history(o_history),
        .i_clk(i_hdmi_clk),.i_rst_n(i_hdmi_rst_n),
        .i_vsync(i_hdmi_vsync),.i_hsync(i_hdmi_hsync),.i_de(i_hdmi_de),
        .i_rgb(i_hdmi_rgb),.i_mode({2'd0,mode_hdmi}),
        .i_threshold(threshold_hdmi),
        .o_vsync(filter_vsync),.o_hsync(filter_hsync),.o_de(filter_de),.o_rgb(filter_rgb)
    );

    wire view_update;wire [2:0] view_hdmi;
    wire coeff_update;wire [23:0] coeff_hdmi;
    cdc_bus_toggle #(.WIDTH(24)) u_coeff_cdc(
      .i_src_clk(i_sys_clk),.i_src_rst_n(i_sys_rst_n),.i_src_update(i_coeff_valid || i_coeff_clear),
      .i_src_data({i_coeff_clear,i_coeff_view,i_coeff_cos,i_coeff_sin}),.o_src_busy(),
      .i_dst_clk(i_hdmi_clk),.i_dst_rst_n(i_hdmi_rst_n),.o_dst_update(coeff_update),.o_dst_data(coeff_hdmi));
    wire qc_command_update;wire [1:0] qc_command;
    cdc_bus_toggle #(.WIDTH(2)) u_qc_command_cdc(
      .i_src_clk(i_sys_clk),.i_src_rst_n(i_sys_rst_n),.i_src_update(i_teach_2d || i_clear_2d),
      .i_src_data({i_teach_2d,i_clear_2d}),.o_src_busy(),
      .i_dst_clk(i_hdmi_clk),.i_dst_rst_n(i_hdmi_rst_n),.o_dst_update(qc_command_update),.o_dst_data(qc_command));
    cdc_bus_toggle #(.WIDTH(3)) u_view_cdc(
      .i_src_clk(i_sys_clk),.i_src_rst_n(i_sys_rst_n),.i_src_update(i_view_cmd),
      .i_src_data(i_view_id),.o_src_busy(),.i_dst_clk(i_hdmi_clk),
      .i_dst_rst_n(i_hdmi_rst_n),.o_dst_update(view_update),.o_dst_data(view_hdmi));
    wire [23:0] qc_rgb;reg [23:0] qc_rgb_d,qc_rgb_d2;
    wire qc_valid,hull_valid,qc_defect,qc_reobserve,qc_overrun,qc_reference_valid;
    wire [31:0] qc_id;wire [10:0] qc_components,qc_holes,qc_area;
    wire [4:0] qc_x,qc_y,hole_x,hole_y;wire signed [5:0] vel_x,vel_y;
    wire [15:0] volume,missing,excess;wire [5:0] dim_x,dim_y,dim_z,views;
    wire [2:0] suggested_view;
    wire [31:0] hull_id;
    wire qc_2d_reference,qc_2d_defect;wire [5:0] qc_width,qc_height;
    assign o_qc_views_loaded=views;assign o_qc_hull_valid=hull_valid;assign o_qc_overrun=qc_overrun;
    assign o_target_frame_id=displayed_frame_id;
    assign o_history_configuration=threshold_hdmi;
    wire [23:0] motion_rgb;wire motion_valid;
    wire [31:0] motion_id;wire [10:0] motion_cells;
    generate if(ENABLE_MOTION) begin: g_motion
      motion_regions_1ppc #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT)) u_motion(
        .clk(i_hdmi_clk),.rst_n(i_hdmi_rst_n),.vsync(i_hdmi_vsync),.hsync(i_hdmi_hsync),
        .de(i_hdmi_de),.rgb(i_hdmi_rgb),.frame_id(displayed_frame_id),
        .overlay_rgb(motion_rgb),.result_valid(motion_valid),
        .result_frame_id(motion_id),.changed_cells(motion_cells));
    end else begin: g_no_motion
      assign motion_rgb=0;assign motion_valid=0;assign motion_id=0;assign motion_cells=0;
    end endgenerate
    generate if(ENABLE_QC_3D) begin: g_qc_3d
    qc_inspection_1ppc #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),
      .ENABLE_COMPONENTS(ENABLE_QC_COMPONENTS),.ENABLE_VOXELS(ENABLE_QC_VOXELS),
      .ENABLE_INTERNAL_MOTION(!ENABLE_MOTION)) u_inspection(
      .clk(i_hdmi_clk),.rst_n(i_hdmi_rst_n),.vsync(i_hdmi_vsync),.de(i_hdmi_de),
      .rgb(i_hdmi_rgb),.mode(mode_hdmi),.frame_id(displayed_frame_id),
      .view_valid(view_update),.view_id(view_hdmi),.diagnostic_rgb(qc_rgb),
      .teach_2d(qc_command_update && qc_command[1]),.clear_2d(qc_command_update && qc_command[0]),
      .coeff_valid(coeff_update && !coeff_hdmi[23]),.coeff_clear(coeff_update && coeff_hdmi[23]),
      .coeff_view(coeff_hdmi[22:20]),.coeff_cos(coeff_hdmi[19:10]),.coeff_sin(coeff_hdmi[9:0]),
      .result_valid(qc_valid),.result_frame_id(qc_id),.components(qc_components),
      .holes(qc_holes),.area(qc_area),.cx(qc_x),.cy(qc_y),.hole_x(hole_x),.hole_y(hole_y),
      .velocity_x(vel_x),.velocity_y(vel_y),.volume(volume),.missing(missing),.excess(excess),
      .size_x(dim_x),.size_y(dim_y),.size_z(dim_z),.hull_valid(hull_valid),
      .defect(qc_defect),.reobserve(qc_reobserve),.suggested_view(suggested_view),
      .views_loaded(views),.hull_result_frame_id(hull_id),.overrun(qc_overrun),.reference_valid(qc_reference_valid),
      .reference_2d_valid(qc_2d_reference),.defect_2d(qc_2d_defect),.width_2d(qc_width),.height_2d(qc_height),
      .coefficients_loaded(o_qc_coefficients_loaded));
    end else begin: g_no_qc_3d
        assign qc_rgb=0;assign qc_valid=motion_valid;assign qc_id=motion_id;
        assign qc_components=motion_cells;assign qc_holes=0;assign qc_area=0;
        assign qc_x=0;assign qc_y=0;assign hole_x=0;assign hole_y=0;
        assign vel_x=0;assign vel_y=0;
        assign volume=0;assign missing=0;assign excess=0;
        assign dim_x=0;assign dim_y=0;assign dim_z=0;assign views=0;
        assign hull_valid=0;assign qc_defect=0;assign qc_reobserve=0;
        assign suggested_view=0;assign qc_overrun=0;assign qc_reference_valid=0;
        assign hull_id=0;
        assign qc_2d_reference=0;assign qc_2d_defect=0;assign qc_width=0;assign qc_height=0;
        assign o_qc_coefficients_loaded=0;
    end endgenerate
    always @(posedge i_hdmi_clk or negedge i_hdmi_rst_n)
      if(!i_hdmi_rst_n) begin qc_rgb_d<=0;qc_rgb_d2<=0;end
      else begin qc_rgb_d<=qc_rgb;qc_rgb_d2<=qc_rgb_d;end
    wire diagnostic_mode=(ENABLE_MOTION && mode_hdmi==15) || (ENABLE_QC_3D &&
      ((ENABLE_QC_COMPONENTS && ((mode_hdmi==12)||(mode_hdmi==13)||(mode_hdmi==14)||(mode_hdmi==15))) ||
       (ENABLE_QC_VOXELS && mode_hdmi==18)));
    wire [23:0] display_rgb=ENABLE_MOTION && mode_hdmi==15?motion_rgb:
       diagnostic_mode?qc_rgb_d2:filter_rgb;
    wire micro_vsync,micro_hsync,micro_de;
    wire [23:0] micro_rgb;
    micro_motion_roi u_micro_motion(
      .clk(i_hdmi_clk),.rst_n(i_hdmi_rst_n),
      .vsync(filter_vsync),.hsync(filter_hsync),.de(filter_de),.rgb(display_rgb),
      .enable(mode_hdmi==18),.source_frame_id(displayed_frame_id),
      .out_vsync(micro_vsync),.out_hsync(micro_hsync),.out_de(micro_de),.out_rgb(micro_rgb));
    wire [7:0] display_fps;
    wire motion_mode=ENABLE_MOTION && mode_hdmi==15;
    wire analysis_valid=motion_mode?motion_valid:mode_hdmi==18?hull_valid:qc_valid;
    wire [31:0] analysis_id=motion_mode?motion_id:mode_hdmi==18?hull_id:qc_id;
    wire [10:0] analysis_components=motion_mode?motion_cells:qc_components;
    assign o_debug_telemetry={i_board_status,i_csi_diag[63:56],i_csi_diag[55:48],
        i_capture_fps,display_fps,{2'd0,mode_hdmi},
        {ENABLE_CARTOON_TEMPORAL!=0,3'd0,i_latency_valid,target_hdmi[67],awb_hdmi,ae_hdmi},8'd1,
        (ENABLE_CARTOON_TEMPORAL?i_history_diagnostics[127:80]:i_csi_diag[47:0]),i_latency_ms,displayed_frame_id,
        (ENABLE_CARTOON_TEMPORAL?i_history_diagnostics[79:48]:analysis_id),
        (ENABLE_CARTOON_TEMPORAL?i_history_diagnostics[47:32]:{5'd0,analysis_components}),
        (ENABLE_CARTOON_TEMPORAL?i_history_diagnostics[31:16]:{5'd0,qc_holes}),
        (ENABLE_CARTOON_TEMPORAL?i_history_diagnostics[15:0]:volume),
        (ENABLE_CARTOON_TEMPORAL?i_history_status:
        {3'd0,hull_valid,qc_reference_valid,qc_overrun,qc_reobserve,qc_defect}),{5'd0,view_hdmi}};
    // Legacy VX status: focus and arm fields are unused (zero). Current
    // SYS mode/threshold replace the first 24 bits after atomic HDMI->SYS CDC.
    assign o_status_hdmi={24'd0,{5'd0,target_hdmi[67],awb_hdmi,ae_hdmi},
        {4'd0,target_hdmi[66:63]},
        {5'd0,target_hdmi[41:31]},{6'd0,target_hdmi[19:10]},
        {5'd0,target_hdmi[30:20]},{6'd0,target_hdmi[9:0]},
        {5'd0,target_hdmi[62:52]},{6'd0,target_hdmi[51:42]},
        32'd0,{16'd0,volume},{15'd0,qc_defect},8'd0,{5'd0,view_hdmi}};
    display_frame_rate #(.SYS_HZ(SYS_CLK_HZ)) u_display_rate(
      .sys_clk(i_sys_clk),.sys_rst_n(i_sys_rst_n),.display_clk(i_hdmi_clk),
      .display_rst_n(i_hdmi_rst_n),.display_vsync(i_hdmi_vsync),.fps(display_fps));
    wire osd_vsync,osd_hsync,osd_de;
    wire [23:0] osd_rgb;
    vision_osd_1ppc #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
        .NORMAL_IMAGE_AWB(NORMAL_IMAGE_AWB),
        .STAGE2_INFO(ENABLE_EVA || ENABLE_MOTION),.QC_INFO(ENABLE_QC_3D),.ARM_INFO(ENABLE_ARM)) u_osd(
        .i_clk(i_hdmi_clk),.i_rst_n(i_hdmi_rst_n),
        .i_vsync(micro_vsync),.i_hsync(micro_hsync),.i_de(micro_de),.i_rgb(micro_rgb),
        .i_mode(mode_hdmi),.i_pipeline_cycles(latency_hdmi),
        .i_latency_valid(i_latency_valid),.i_latency_ms(i_latency_ms),
        .i_qc_valid(analysis_valid),.i_qc_frame_id(analysis_id),.i_qc_components(analysis_components),
        .i_qc_holes(qc_holes),.i_qc_x(qc_x),.i_qc_y(qc_y),
        .i_hole_x(hole_x),.i_hole_y(hole_y),.i_velocity_x(vel_x),.i_velocity_y(vel_y),
        .i_hull_valid(mode_hdmi==18?hull_valid:qc_valid && qc_2d_reference),
        .i_volume(mode_hdmi==18?volume:{5'd0,qc_area}),
        .i_dim_x(mode_hdmi==18?dim_x:qc_width),.i_dim_y(mode_hdmi==18?dim_y:qc_height),.i_dim_z(dim_z),
        .i_reference_valid(mode_hdmi==18?qc_reference_valid:qc_2d_reference),
        .i_display_fps(display_fps),
        .i_capture_fps(i_capture_fps),.i_board_status(i_board_status),.i_csi_diag(i_csi_diag),
        .i_source_frame_id(displayed_frame_id),
        .i_arm_status(i_arm_status),.i_arm_points(i_arm_points),
        .i_defect(mode_hdmi==18?qc_defect:qc_2d_defect),.i_reobserve(qc_reobserve),.i_suggested_view(suggested_view),
        .i_ae_stable(ae_hdmi),.i_awb_valid(awb_hdmi),
        .i_target_valid(!diagnostic_mode && target_hdmi[67]),.i_target_class(target_hdmi[66:63]),
        .i_center_x(target_hdmi[62:52]),.i_center_y(target_hdmi[51:42]),
        .i_bbox_x0(target_hdmi[41:31]),.i_bbox_x1(target_hdmi[30:20]),
        .i_bbox_y0(target_hdmi[19:10]),.i_bbox_y1(target_hdmi[9:0]),
        .o_vsync(osd_vsync),.o_hsync(osd_hsync),.o_de(osd_de),.o_rgb(osd_rgb)
    );
    generate if(ENABLE_HDMI_MENU) begin:g_menu
        wire [18:0] state_sys={menu_open_sys,menu_cursor_sys,menu_first_sys,mode_sys};
        reg [18:0] state_last;
        reg state_init,state_update;
        wire [18:0] state_hdmi;
        always @(posedge i_sys_clk or negedge i_sys_rst_n) begin
            if(!i_sys_rst_n) begin state_last<=0;state_init<=0;state_update<=0;end
            else begin
                state_update<=0;
                if(!state_init || state_last!=state_sys) begin
                    state_init<=1;state_last<=state_sys;state_update<=1;
                end
            end
        end
        cdc_bus_toggle #(.WIDTH(19)) u_menu_cdc(
            .i_src_clk(i_sys_clk),.i_src_rst_n(i_sys_rst_n),
            .i_src_update(state_update),.i_src_data(state_last),.o_src_busy(),
            .i_dst_clk(i_hdmi_clk),.i_dst_rst_n(i_hdmi_rst_n),
            .o_dst_update(),.o_dst_data(state_hdmi));
        vision_menu_overlay_1ppc #(.MODE_BITMAP(MODE_BITMAP)) u_menu_overlay(
            .i_clk(i_hdmi_clk),.i_rst_n(i_hdmi_rst_n),
            .i_vsync(osd_vsync),.i_hsync(osd_hsync),.i_de(osd_de),.i_rgb(osd_rgb),
            .i_menu_open(state_hdmi[18]),.i_cursor(state_hdmi[17:12]),
            .i_first(state_hdmi[11:6]),.i_applied_mode(mode_hdmi==14 ? 6'd0 : mode_hdmi),
            .o_vsync(o_hdmi_vsync),.o_hsync(o_hdmi_hsync),.o_de(o_hdmi_de),.o_rgb(o_hdmi_rgb));
    end else begin:g_no_menu
        assign o_hdmi_vsync=osd_vsync;assign o_hdmi_hsync=osd_hsync;
        assign o_hdmi_de=osd_de;assign o_hdmi_rgb=osd_rgb;
    end endgenerate

    wire _unused_meta = ^{awb_r_gain,awb_b_gain,target_area,target_quality,
                          ae_src_busy,mode_ctrl_busy,mode_ctrl_isp_update,
                          osd_meta_busy};
endmodule
