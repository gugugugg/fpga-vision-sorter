`timescale 1ns/1ps
// HDMI experiment adapter. Commands:1 empty background,2 cuboid,3 wedge.
// Capture completion is a generation counter, not merely a command ACK.
module part_template_hdmi #(
 parameter WIDTH=1024,HEIGHT=600,ROI_X=256,ROI_Y=150,ROI_W=512,ROI_H=300,
 parameter CLK_HZ=52002000,GW=128,GH=75
)(input wire clk,rst_n,vsync,hsync,de,input wire [23:0] rgb,
 input wire analysis_vsync,analysis_de,input wire [23:0] analysis_rgb,
 input wire [31:0] source_frame_id,input wire command_update,input wire [15:0] command_code,
 output wire out_vsync,out_hsync,out_de,output wire [23:0] out_rgb,
 input wire show_overlay,capture_mode,
 output wire flash_cs_n,flash_clk,flash_mosi,input wire flash_miso,
 input wire [15:0] sort_status,
 output wire [239:0] status_bus,output wire status_update);
 reg vs_d,de_d,frame_active;reg [23:0] rgb_d;reg [31:0] active_id;
 wire source_start=analysis_de && !de_d && !frame_active;
 reg bg_pending,bg_busy,bg_saw_invalid,record_request,record_class,last_error;
 reg [1:0] operation;reg [15:0] generation;
 reg [31:0] last_capture_id;
 wire gs,gv;wire [15:0] pixel;wire [31:0] gid;
 wire bg_valid;wire [1:0] loaded,cls,sc;
 wire pred,stable,busy,dropped,rb,rd,re,mu,mv,tv,gc;
 wire [31:0] pid,rid,mid;wire [10:0] ce,we;
 wire [13:0] mc;wire [3:0] reject;
 wire [7:0] x0,x1,y0,y1,mx0,mx1,my0,my1;
 wire [63:0] diagnostic;
 wire capture_sync1=capture_mode;
 wire cancel_capture=command_update && command_code==4;
 wire store_busy,store_present;wire [2:0] stored;wire [2:0] store_error;
 wire load_begin,load_class,load_we,load_bit,load_commit;
 wire [9:0] load_addr;
 wire cap_begin,cap_we,cap_bit,cap_commit;wire [9:0] cap_addr;
 wire background_store_active,background_load_begin,background_load_we,background_load_done;
 wire [13:0] background_addr;wire [15:0] background_pixel,background_q;
 wire bg_capture_done=bg_busy && bg_saw_invalid && bg_valid && !bg_pending;
 wire save_request=bg_capture_done || (rd && !re && loaded[record_class]);
 reg [2:0] capture_notice;
 reg [1:0] capture_kind;
 reg [4:0] capture_start_frames;
 reg save_pending,store_busy_d;
 wire [2:0] shown_notice=capture_start_frames!=0?{1'b0,capture_kind}:capture_notice;
 wire selected_saved=capture_kind==1?stored[2]:capture_kind==2?stored[0]:stored[1];
 wire [15:0] storage_status={4'd0,shown_notice,capture_sync1,store_present,store_busy,store_error,stored};
 template_flash_store #(.CLK_HZ(CLK_HZ)) u_flash(
  .clk(clk),.rst_n(rst_n),.cancel(cancel_capture),
  .capture_begin(cap_begin),.capture_we(cap_we),.capture_bit(cap_bit),.capture_addr(cap_addr),
  .save_request(save_request),.save_class(bg_capture_done?2'd2:{1'b0,record_class}),
  .background_active(background_store_active),.background_load_begin(background_load_begin),.background_load_we(background_load_we),.background_load_done(background_load_done),
  .background_addr(background_addr),.background_pixel(background_pixel),.background_q(background_q),
  .load_begin(load_begin),.load_class(load_class),.load_we(load_we),.load_bit(load_bit),.load_commit(load_commit),.load_addr(load_addr),
  .busy(store_busy),.present(store_present),.saved(stored),.error(store_error),
  .flash_cs_n(flash_cs_n),.flash_clk(flash_clk),.flash_mosi(flash_mosi),.flash_miso(flash_miso));
 wire config_command=command_update && command_code[15] && !bg_busy && !rb && operation==0;
 wire [15:0] flags={bg_valid,loaded,stable,sc,rb,bg_busy,last_error,reject[2:0],mv,tv,gc,1'b0};
 // Keep the original ten status words in the low160 bits for compatibility.
 assign status_bus={storage_status,diagnostic,flags,{5'd0,ce},{5'd0,we},pid,last_capture_id,generation,{2'd0,mc},active_id[15:0]};
 assign status_update=pred || mu || rd || (analysis_vsync && !vs_d);
 part_visual_sampler_1ppc #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.ROI_X(ROI_X),.ROI_Y(ROI_Y),
  .ROI_W(ROI_W),.ROI_H(ROI_H),.GW(GW),.GH(GH)) u_sampler(.clk(clk),.rst_n(rst_n),.sof(source_start),
  .de(de_d),.rgb(rgb_d),.frame_id(source_frame_id),.grid_start(gs),.grid_valid(gv),
  .grid_rgb565(pixel),.grid_frame_id(gid));
 part_visual_grid #(.W(GW),.H(GH),.CLK_HZ(CLK_HZ)) u_grid(.clk(clk),.rst_n(rst_n),.frame_start(gs),
.cancel(cancel_capture),.capture_begin(cap_begin),.capture_we(cap_we),.capture_bit(cap_bit),.capture_addr(cap_addr),.capture_commit(cap_commit),
  .storage_active(background_store_active),.storage_load_begin(background_load_begin),.storage_load_we(background_load_we),.storage_load_done(background_load_done),
  .storage_addr(background_addr),.storage_pixel(background_pixel),.storage_q(background_q),
  .sample_valid(gv),.teach_background(bg_pending),.frame_id(gid),.rgb565(pixel),
  .threshold_update(config_command),.threshold_rb(command_code[6:2]),.threshold_g(command_code[12:7]),.diagnostic(diagnostic),
  .teach_begin(load_begin),.teach_class(load_class),.teach_we(load_we),.teach_bit(load_bit),.teach_commit(load_commit),.teach_addr(load_addr),
  .record_request(record_request),.record_class(record_class),.record_busy(rb),.record_done(rd),.record_error(re),.record_frame_id(rid),
  .background_valid(bg_valid),.templates_loaded(loaded),.prediction_update(pred),.predicted_class(cls),
  .prediction_frame_id(pid),.cuboid_errors(ce),.wedge_errors(we),.reject_reason(reject),.stable(stable),.stable_class(sc),
  .target_x0(x0),.target_x1(x1),.target_y0(y0),.target_y1(y1),
  .motion_update(mu),.motion_valid(mv),.track_valid(tv),.global_change(gc),.motion_frame_id(mid),.motion_cells(mc),
  .motion_x0(mx0),.motion_x1(mx1),.motion_y0(my0),.motion_y1(my1),.busy(busy),.dropped(dropped));
 part_template_overlay #(.WIDTH(WIDTH),.ROI_X(ROI_X),.ROI_Y(ROI_Y),.ROI_W(ROI_W),.ROI_H(ROI_H),.CLK_HZ(CLK_HZ),.GW(GW),.GH(GH)) u_overlay(
  .clk(clk),.rst_n(rst_n),.vsync(vsync),.hsync(hsync),.de(de),.rgb(rgb),
  .frame_id(source_frame_id),.prediction_id(pid),.capture_id(last_capture_id),
  .predicted_class(cls),.foreground_valid(diagnostic[51]),.reject_reason(reject[2:0]),
  .show_overlay(show_overlay),.capture_mode(capture_sync1),.capture_notice(shown_notice),.store_busy(store_busy),.store_error(store_error),.stored(stored),.sort_status(sort_status),.background_valid(bg_valid),.loaded(loaded),.stable(stable),.class_id(sc),.operation(operation),.last_error(last_error),
  .cuboid_errors(ce),.wedge_errors(we),.motion_cells(mc),.track_valid(tv),
  .x0(x0),.x1(x1),.y0(y0),.y1(y1),.mx0(mx0),.mx1(mx1),.my0(my0),.my1(my1),
  .out_vsync(out_vsync),.out_hsync(out_hsync),.out_de(out_de),.out_rgb(out_rgb));
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
   vs_d<=0;de_d<=0;frame_active<=0;rgb_d<=0;active_id<=0;
   bg_pending<=0;bg_busy<=0;bg_saw_invalid<=0;record_request<=0;record_class<=0;
   last_error<=0;operation<=0;generation<=0;last_capture_id<=0;
   capture_notice<=0;capture_kind<=0;capture_start_frames<=0;save_pending<=0;store_busy_d<=0;
  end else begin
   vs_d<=analysis_vsync;de_d<=analysis_de;rgb_d<=analysis_rgb;record_request<=0;
   store_busy_d<=store_busy;
   if(analysis_vsync && !vs_d && capture_start_frames!=0)capture_start_frames<=capture_start_frames-1'b1;
   if(save_request)begin capture_notice<=4;save_pending<=1;end
   if(store_busy_d && !store_busy && save_pending)begin
    capture_notice<=store_error==0 && selected_saved?5:6;save_pending<=0;
   end
   if(analysis_vsync && !vs_d)frame_active<=0;
   if(source_start)begin frame_active<=1;active_id<=source_frame_id;end
   if(gs && bg_pending)bg_pending<=0;
   if(bg_busy && !bg_valid)bg_saw_invalid<=1;
   if(bg_busy && bg_saw_invalid && bg_valid && !bg_pending)begin
    bg_busy<=0;bg_saw_invalid<=0;operation<=0;generation<=generation+1'b1;last_capture_id<=gid;
   end
   if(cancel_capture)begin bg_pending<=0;bg_busy<=0;bg_saw_invalid<=0;operation<=0;last_error<=0;
    capture_notice<=7;capture_start_frames<=0;save_pending<=0;end
   if(command_update && store_busy && command_code!=4)last_error<=1;
   if(rd)begin
    operation<=0;last_error<=re;generation<=generation+1'b1;last_capture_id<=rid;
    if(re)begin capture_notice<=6;capture_start_frames<=0;save_pending<=0;end
   end
   if(command_update && command_code!=4 && !bg_busy && !rb && !store_busy && operation==0)begin
    last_error<=0;
    if(command_code==1)begin bg_pending<=1;bg_busy<=1;bg_saw_invalid<=0;operation<=1;
     capture_kind<=1;capture_notice<=1;capture_start_frames<=30;end
    else if(command_code==2 || command_code==3)begin
     record_request<=1;record_class<=command_code==3;operation<=command_code;
     capture_kind<=command_code[1:0];capture_notice<=command_code[2:0];capture_start_frames<=30;
    end
   end
  end
endmodule
