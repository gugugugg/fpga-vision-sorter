`timescale 1ns/1ps
// Shared endpoint for board logical preview and the later DAT pin wrapper.
// Sample the first displayed pixel after frame metadata has been committed,
// then cross frame ID, validity and coordinates atomically to SYS.
module arm_vision_bridge #(parameter CLK_HZ=50000000,IMG_WIDTH=1024,IMG_HEIGHT=600)(
 input wire sys_clk,sys_rst_n,pixel_clk,pixel_rst_n,vsync,de,
 input wire [5:0] mode,input wire target_valid,input wire [31:0] frame_id,
 input wire [10:0] x,input wire [9:0] y,
 input wire byte_valid,input wire [7:0] byte_data,output wire packet_active,
 input wire dat,output wire dat_drive_low,
 input wire [5:0] views_loaded,coefficients_loaded,input wire hull_valid,qc_overrun,
 output wire view_valid,output wire [2:0] view_id,
 output wire [7:0] status_hdmi,output wire [8:0] points_hdmi,output wire scanning_sys,
 output wire coeff_valid,coeff_clear,output wire [2:0] coeff_view,
 output wire signed [9:0] coeff_cos,coeff_sin
);
 wire command_valid,command_error;wire [7:0] kind,address;wire [95:0] payload;
 wire [7:0] status;wire [8:0] points;
 wire signed [15:0] requested_cos=payload[95:80],requested_sin=payload[79:64];
 wire coefficient_packet=command_valid && kind==6;
 wire coefficient_ok=address<6 && !status[6] && requested_cos>=-256 && requested_cos<=256 &&
   requested_sin>=-256 && requested_sin<=256 && (requested_cos!=0 || requested_sin!=0);
 assign coeff_valid=coefficient_packet && coefficient_ok;
 assign coeff_clear=command_valid && kind==3 && address==7 && !status[6];
 assign coeff_view=address[2:0];assign coeff_cos=requested_cos[9:0];assign coeff_sin=requested_sin[9:0];
 arm_packet_decoder #(.TIMEOUT_CYCLES(CLK_HZ/200)) u_arm_decode(
  .clk(sys_clk),.rst_n(sys_rst_n),.byte_valid(byte_valid),.byte_data(byte_data),
  .valid(command_valid),.error(command_error),.kind(kind),.address(address),
  .payload(payload),.active(packet_active));
 reg posted,update;reg [53:0] bundle;
 always @(posedge pixel_clk or negedge pixel_rst_n)
  if(!pixel_rst_n) begin posted<=0;update<=0;bundle<=0;end
  else begin
   update<=0;
   if(vsync) posted<=0;
   else if(de && !posted) begin
    posted<=1;update<=1;bundle<={frame_id,target_valid && mode==2,x,y};
   end
  end
 wire target_update;wire [53:0] target;
 cdc_bus_toggle #(.WIDTH(54)) u_arm_target_cdc(
  .i_src_clk(pixel_clk),.i_src_rst_n(pixel_rst_n),.i_src_update(update),.i_src_data(bundle),.o_src_busy(),
  .i_dst_clk(sys_clk),.i_dst_rst_n(sys_rst_n),.o_dst_update(target_update),.o_dst_data(target));
 reg qc_init;reg [13:0] qc_last;reg qc_update;
 always @(posedge pixel_clk or negedge pixel_rst_n)
  if(!pixel_rst_n) begin qc_init<=0;qc_last<=0;qc_update<=0;end
  else begin
   qc_update<=0;
   if(!qc_init || qc_last!={coefficients_loaded,qc_overrun,hull_valid,views_loaded}) begin
    qc_init<=1;qc_last<={coefficients_loaded,qc_overrun,hull_valid,views_loaded};qc_update<=1;
   end
  end
 wire [13:0] qc_sys;
 cdc_bus_toggle #(.WIDTH(14)) u_arm_qc_cdc(
  .i_src_clk(pixel_clk),.i_src_rst_n(pixel_rst_n),.i_src_update(qc_update),.i_src_data(qc_last),.o_src_busy(),
  .i_dst_clk(sys_clk),.i_dst_rst_n(sys_rst_n),.o_dst_update(),.o_dst_data(qc_sys));
 assign scanning_sys=status[2];
 arm_vision_controller #(.CLK_HZ(CLK_HZ),.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_arm_control(
  .clk(sys_clk),.rst_n(sys_rst_n),.packet_valid(command_valid && kind!=6),
  .packet_error(command_error || (coefficient_packet && !coefficient_ok)),.calibrated_views(qc_sys[13:8]),
  .kind(kind),.address(address),.payload(payload),.target_update(target_update),
  .target_valid(target[21]),.frame_id(target[53:22]),.target_x(target[20:10]),.target_y(target[9:0]),
  .dat(dat),.dat_drive_low(dat_drive_low),.views_loaded(qc_sys[5:0]),.hull_valid(qc_sys[6]),
  .qc_overrun(qc_sys[7]),.view_valid(view_valid),.view_id(view_id),.status(status),.points(points),.last_target_id());
 reg status_init;reg [16:0] last_status;reg status_update;
 always @(posedge sys_clk or negedge sys_rst_n)
  if(!sys_rst_n) begin status_init<=0;last_status<=0;status_update<=0;end
  else begin
   status_update<=0;
   if(!status_init || last_status!={status,points}) begin
    status_init<=1;last_status<={status,points};status_update<=1;
   end
  end
 cdc_bus_toggle #(.WIDTH(17)) u_arm_status_cdc(
  .i_src_clk(sys_clk),.i_src_rst_n(sys_rst_n),.i_src_update(status_update),.i_src_data(last_status),.o_src_busy(),
  .i_dst_clk(pixel_clk),.i_dst_rst_n(pixel_rst_n),.o_dst_update(),.o_dst_data({status_hdmi,points_hdmi}));
endmodule
