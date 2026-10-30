`timescale 1ns/1ps
// Production glue, shared by board top and the real-module integration test.
// PHY is external to this block. No pixel or protocol fixture is synthesized.
module board_network_integration #(
    parameter WIDTH=1024, HEIGHT=600, CLK_HZ=125000000,
    parameter [31:0] LOCAL_IP=32'ha9fe125b,PEER_IP=32'ha9fe125a,
    parameter [63:0] MODE_BITMAP=64'h7ffff,
    parameter BASIC_CATALOG=1,
    parameter TIMEOUT_CYCLES=125000000
)(
    input wire i_core_rst_n,i_net_clk,i_rx_clk,i_sys_clk,i_pixel_clk,i_ddr_clk,
    input wire i_csi_clk,input wire [31:0] i_csi_sample,
    input wire [255:0] i_telemetry,
    input wire i_net_enable,i_rx_valid,i_rx_error,input wire [7:0] i_rx_data,
    output wire o_tx_valid,output wire [7:0] o_tx_data,
    input wire i_vsync,i_de,input wire [23:0] i_rgb,
    input wire [5:0] i_display_mode,input wire i_source_ok,
    input wire i_sys_frame_boundary,input wire [231:0] i_sys_status,
    input wire i_remote_rejected,
    output wire o_remote_mode_valid,output wire [5:0] o_remote_mode,
    output wire o_remote_threshold_valid,output wire [9:0] o_remote_threshold,
    input wire i_capture_busy,i_legacy_wr,i_legacy_rd,i_legacy_reserved,
    input wire [24:0] i_legacy_addr,input wire [127:0] i_legacy_data,
    output wire o_legacy_ready,o_legacy_rd_valid,
    output wire [127:0] o_legacy_rd_data,
    output wire o_wr,o_rd,output wire [24:0] o_addr,
    output wire [127:0] o_data,input wire i_ready,i_rd_valid,
    input wire [127:0] i_rd_data,output wire o_fault,
    output wire [9:0] o_outstanding
);
    wire net_rst_n,ddr_rst_n,pixel_rst_n;
    network_reset_sync nr(i_net_clk,i_core_rst_n & i_net_enable,net_rst_n);
    network_reset_sync dr(i_ddr_clk,i_core_rst_n,ddr_rst_n);
    network_reset_sync pr(i_pixel_clk,i_core_rst_n,pixel_rst_n);
    (* async_reg="true" *) reg source0,source1,k4_0,k4_1;
    always @(posedge i_net_clk or negedge net_rst_n) begin
        if(!net_rst_n) begin source0<=0;source1<=0;k4_0<=1;k4_1<=1;end
        else begin source0<=i_source_ok;source1<=source0;k4_0<=i_capture_busy;k4_1<=k4_0;end
    end
    wire udp_ready,udp_valid,udp_last;wire [7:0] udp_data;
    wire [15:0] udp_length,udp_src,udp_dst;wire [47:0] peer_mac;
    wire [31:0] peer_ip;wire start,ready,payload_ready,payload_valid,abort_tx,done,error;
    wire [47:0] dest_mac;wire [31:0] dest_ip;
    wire [15:0] src_port,dest_port;wire [10:0] length;wire [7:0] payload_data;
    wire app_rx_ready,debug_rx_ready,debug_request,debug_start,debug_abort,debug_valid;
    wire [47:0] debug_mac;wire [15:0] debug_port;wire [10:0] debug_length;
    wire [7:0] debug_data;
    reg tx_active,tx_owner;
    wire debug_select=tx_active?tx_owner:debug_request;
    wire selected_start=debug_select?debug_start:start;
    assign udp_ready=udp_dst==6012?debug_rx_ready:app_rx_ready;
    always @(posedge i_net_clk or negedge net_rst_n) begin
        if(!net_rst_n) begin tx_active<=0;tx_owner<=0;end
        else if(!tx_active && selected_start) begin tx_active<=1;tx_owner<=debug_select;end
        else if(tx_active && (done || error)) tx_active<=0;
    end
    network_udp_stack #(.LOCAL_IP(LOCAL_IP),.PEER_IP(PEER_IP)) stack(
        .i_clk(i_net_clk),.i_rx_clk(i_rx_clk),.i_rst_n(net_rst_n),
        .i_rx_valid(i_rx_valid),.i_rx_error(i_rx_error),.i_rx_data(i_rx_data),
        .o_tx_valid(o_tx_valid),.o_tx_data(o_tx_data),.i_udp_ready(udp_ready),
        .o_udp_valid(udp_valid),.o_udp_last(udp_last),.o_udp_data(udp_data),
        .o_udp_length(udp_length),.o_udp_src_port(udp_src),.o_udp_dest_port(udp_dst),
        .o_peer_mac(peer_mac),.o_peer_ip(peer_ip),.i_start(selected_start),
        .i_dest_mac(debug_select?debug_mac:dest_mac),.i_dest_ip(debug_select?PEER_IP:dest_ip),
        .i_src_port(debug_select?16'd6012:src_port),.i_dest_port(debug_select?debug_port:dest_port),
        .i_length(debug_select?debug_length:length),.o_ready(ready),.o_payload_ready(payload_ready),
        .i_payload_valid(debug_select?debug_valid:payload_valid),
        .i_payload_data(debug_select?debug_data:payload_data),.i_abort(debug_select?debug_abort:abort_tx),
        .o_done(done),.o_error(error),.o_bad_frames(),.o_busy_drops(),.o_rejected(),.o_accepted());
    wire capture,original,capture_ready,frame_valid,frame_original,release_frame;
    wire [31:0] frame_id,frame_crc,frame_bytes;wire [15:0] frame_w,frame_h;
    wire [5:0] frame_mode;wire video_valid,video_ready,video_error;wire [7:0] video_data;
    wire control_valid,control_threshold,control_ready,control_done,control_error;
    wire [15:0] control_value;wire [231:0] status;
    network_debug_app #(.MODE_BITMAP(MODE_BITMAP),.PEER_IP(PEER_IP),.BASIC_CATALOG(BASIC_CATALOG),.FRAME_WIDTH(WIDTH),
        .FRAME_HEIGHT(HEIGHT),.CLK_HZ(CLK_HZ),.TIMEOUT_CYCLES(TIMEOUT_CYCLES)) app(
        .i_clk(i_net_clk),.i_rst_n(net_rst_n),.o_udp_ready(app_rx_ready),
        .i_udp_valid(udp_valid && udp_dst!=6012),.i_udp_last(udp_last),.i_udp_data(udp_data),
        .i_udp_length(udp_length),.i_udp_src_port(udp_src),.i_udp_dest_port(udp_dst),
        .i_peer_mac(peer_mac),.i_peer_ip(peer_ip),.o_start(start),.o_dest_mac(dest_mac),
        .o_dest_ip(dest_ip),.o_src_port(src_port),.o_dest_port(dest_port),.o_length(length),
        .i_ready(ready && !debug_select),.i_payload_ready(payload_ready && !debug_select),.o_payload_valid(payload_valid),
        .o_payload_data(payload_data),.o_abort(abort_tx),
        .i_done(done && tx_active && !tx_owner),.i_error(error && tx_active && !tx_owner),
        .i_source_ready(source1),.i_capture_ready(capture_ready && !k4_1),
        .o_capture(capture),.o_original(original),.i_frame_valid(frame_valid),
        .i_frame_id(frame_id),.i_frame_crc(frame_crc),.i_frame_width(frame_w),
        .i_frame_height(frame_h),.i_frame_bytes(frame_bytes),.i_frame_mode(frame_mode),
        .i_frame_original(frame_original),.o_frame_release(release_frame),
        .i_video_valid(video_valid),.i_video_data(video_data),.o_video_ready(video_ready),
        .i_video_error(video_error),.o_control_valid(control_valid),
        .o_control_threshold(control_threshold),.o_control_value(control_value),
        .i_control_ready(control_ready),.i_control_done(control_done),
        .i_control_error(control_error),.i_status(status),.o_streaming(),
        .o_rejected(),.o_bad_packets(),.o_frames());
    wire trace_arm,trace_ready,trace_valid,trace_busy,trace_read;
    wire [10:0] trace_addr;wire [31:0] trace_data;
    network_csi_trace trace(
        .i_rst_n(i_core_rst_n),.i_net_clk(i_net_clk),.i_csi_clk(i_csi_clk),
        .i_arm(trace_arm),.o_arm_ready(trace_ready),.o_valid(trace_valid),.o_busy(trace_busy),
        .i_sample(i_csi_sample),.i_read(trace_read),.i_addr(trace_addr),.o_data(trace_data));
    network_diagnostic_endpoint #(.PEER_IP(PEER_IP)) diagnostics(
        .i_clk(i_net_clk),.i_rst_n(net_rst_n),.i_udp_valid(udp_valid && udp_dst==6012),
        .i_udp_last(udp_last),.i_udp_data(udp_data),.i_udp_length(udp_length),
        .i_udp_src_port(udp_src),.i_peer_mac(peer_mac),.i_peer_ip(peer_ip),.o_udp_ready(debug_rx_ready),
        .o_request(debug_request),.o_start(debug_start),.o_dest_mac(debug_mac),.o_dest_port(debug_port),
        .o_length(debug_length),.i_ready(ready && debug_select),
        .i_payload_ready(payload_ready && debug_select),
        .i_done(done && tx_active && tx_owner),.i_error(error && tx_active && tx_owner),
        .o_payload_valid(debug_valid),.o_payload_data(debug_data),.o_abort(debug_abort),
        .i_telemetry(i_telemetry),.o_arm(trace_arm),.i_arm_ready(trace_ready),
        .i_trace_valid(trace_valid),.i_trace_busy(trace_busy),.o_trace_read(trace_read),
        .o_trace_addr(trace_addr),.i_trace_data(trace_data));
    network_control_cdc control(
        .i_rst_n(i_core_rst_n),.i_net_clk(i_net_clk),.i_sys_clk(i_sys_clk),
        .i_valid(control_valid),.i_threshold(control_threshold),.i_value(control_value),
        .i_net_enable(i_net_enable),.o_ready(control_ready),.o_done(control_done),
        .o_error(control_error),.o_status(status),.i_sys_frame_boundary(i_sys_frame_boundary),
        .i_sys_status(i_sys_status),.i_remote_rejected(i_remote_rejected),
        .o_remote_mode_valid(o_remote_mode_valid),.o_remote_mode(o_remote_mode),
        .o_remote_threshold_valid(o_remote_threshold_valid),.o_remote_threshold(o_remote_threshold));
    wire sof,eof,pvalid;wire [23:0] rgb;wire [5:0] mode;
    network_hdmi_tap tap(.i_clk(i_pixel_clk),.i_rst_n(pixel_rst_n),.i_vsync(i_vsync),
        .i_de(i_de),.i_rgb(i_rgb),.i_display_mode(i_display_mode),
        .o_sof(sof),.o_frame_end(eof),.o_valid(pvalid),.o_rgb(rgb),.o_mode(mode));
    wire net_wr,net_rd,net_ready,net_rd_valid;wire [24:0] net_addr;
    wire [127:0] net_data,net_rd_data;
    network_video_capture #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.TIMEOUT_CYCLES(TIMEOUT_CYCLES)) store(
        .i_rst_n(i_core_rst_n),.i_net_clk(i_net_clk),.i_pixel_clk(i_pixel_clk),
        .i_ddr_clk(i_ddr_clk),.i_capture(capture),.i_original(original),
        .i_net_enable(i_net_enable),.o_capture_ready(capture_ready),
        .i_sof(sof),.i_frame_end(eof),.i_pixel_valid(pvalid),.i_rgb(rgb),
        .i_display_mode(mode),.i_source_ok(i_source_ok),.o_frame_valid(frame_valid),
        .o_frame_original(frame_original),.o_frame_mode(frame_mode),.o_frame_id(frame_id),
        .o_frame_crc(frame_crc),.o_frame_bytes(frame_bytes),.o_frame_width(frame_w),
        .o_frame_height(frame_h),.i_release(release_frame),.o_video_valid(video_valid),
        .o_video_data(video_data),.i_video_ready(video_ready),.o_video_error(video_error),
        .o_wr(net_wr),.o_rd(net_rd),.o_addr(net_addr),.o_wr_data(net_data),
        .i_ddr_ready(net_ready),.i_rd_valid(net_rd_valid),.i_rd_data(net_rd_data),
        .o_busy(),.o_failed());
    // Capture strobes include net_ready; raw held-request clients use default 0.
    network_ddr_arbiter #(.NET_STROBES_ACCEPTED(1)) arbiter(.i_clk(i_ddr_clk),.i_rst_n(ddr_rst_n),
        .i_legacy_wr(i_legacy_wr),.i_legacy_rd(i_legacy_rd),
        .i_legacy_reserved(i_legacy_reserved),.i_legacy_addr(i_legacy_addr),
        .i_legacy_data(i_legacy_data),.o_legacy_ready(o_legacy_ready),
        .o_legacy_rd_valid(o_legacy_rd_valid),.o_legacy_rd_data(o_legacy_rd_data),
        .i_net_wr(net_wr),.i_net_rd(net_rd),.i_net_addr(net_addr),.i_net_data(net_data),
        .o_net_ready(net_ready),.o_net_rd_valid(net_rd_valid),.o_net_rd_data(net_rd_data),
        .o_wr(o_wr),.o_rd(o_rd),.o_addr(o_addr),.o_data(o_data),.i_ready(i_ready),
        .i_rd_valid(i_rd_valid),.i_rd_data(i_rd_data),.o_fault(o_fault),.o_outstanding(o_outstanding));
endmodule
