`timescale 1ns/1ps
// Fully local grid path: taught RGB565 background -> binary silhouette ->
// bbox-normalised template match, alongside gray frame-difference tracking.
// Input is a complete row-major W*H sample grid from ONE fixed camera ROI.
// Detection result coordinates remain in grid units. No actuator outputs.
module part_visual_grid #(parameter W=128,H=75,CLK_HZ=50000000)(
 input wire clk,rst_n,cancel,frame_start,sample_valid,teach_background,
 output wire capture_begin,capture_we,capture_bit,capture_commit,output wire [9:0] capture_addr,
 input wire [31:0] frame_id,input wire [15:0] rgb565,
 input wire storage_active,storage_load_begin,storage_load_we,storage_load_done,
 input wire [13:0] storage_addr,input wire [15:0] storage_pixel,output wire [15:0] storage_q,
 input wire threshold_update,input wire [4:0] threshold_rb,input wire [5:0] threshold_g,
 output wire [63:0] diagnostic,
 input wire teach_begin,teach_class,teach_we,teach_bit,teach_commit,
 input wire [9:0] teach_addr,
 input wire record_request,record_class,
 output wire record_busy,output reg record_done,record_error,
 output reg [31:0] record_frame_id,
 output wire background_valid,output wire [1:0] templates_loaded,
 output reg prediction_update,output wire [1:0] predicted_class,
 output wire [31:0] prediction_frame_id,output wire [10:0] cuboid_errors,wedge_errors,
 output wire [3:0] reject_reason,output wire stable,output wire [1:0] stable_class,
 output reg [7:0] target_x0,target_x1,target_y0,target_y1,
 output wire motion_update,motion_valid,track_valid,global_change,
 output wire [31:0] motion_frame_id,output wire [13:0] motion_cells,
 output wire [7:0] motion_x0,motion_x1,motion_y0,motion_y1,
 output wire busy,dropped
);
 wire gs,gv,fg;wire [7:0] gray;wire [31:0] gid;
 wire bg_start=frame_start && teach_background;
 wire config_update=threshold_update===1'b1;
 wire clear_job=cancel || storage_load_begin || bg_start || teach_begin || record_request || config_update;
 wire [4:0] applied_rb;wire [5:0] applied_g;wire [63:0] mask_diagnostic;
 assign diagnostic={1'b0,applied_rb,applied_g,mask_diagnostic[51:0]};
 part_background_mask #(.W(W),.H(H)) u_background(
  .clk(clk),.rst_n(rst_n),.cancel(cancel),.frame_start(frame_start),.teach_background(teach_background),
  .sample_valid(sample_valid),.rgb565(rgb565),.frame_id(frame_id),
  .storage_active(storage_active),.storage_load_begin(storage_load_begin),.storage_load_we(storage_load_we),.storage_load_done(storage_load_done),
  .storage_addr(storage_addr),.storage_pixel(storage_pixel),.storage_q(storage_q),
  .threshold_update(config_update),.threshold_rb(threshold_rb),.threshold_g(threshold_g),
  .applied_rb(applied_rb),.applied_g(applied_g),
  .background_valid(background_valid),.out_start(gs),.out_valid(gv),.foreground(fg),.gray(gray),.out_id(gid));
 wire pb,pw,ps,pbit,normal_busy,ready,class_busy,class_error,class_done;
 wire [9:0] pa;wire [31:0] pid;
 wire [7:0] bx0,bx1,by0,by1;wire object_valid;
 reg [7:0] saved_x0,saved_x1,saved_y0,saved_y1;
 wire vote_stable;wire [1:0] vote_class;
 reg record_pending,recording,record_slot;
 reg [31:0] request_source;
 wire [31:0] source_distance=pid-request_source;
 wire fresh_patch=source_distance!=0 && !source_distance[31];
 wire record_begin=pb && record_pending && fresh_patch && object_valid && background_valid;
 wire local_teach_begin=teach_begin || record_begin;
 assign capture_begin=record_begin;assign capture_we=recording && pw;
 assign capture_bit=pbit;assign capture_addr=pa;assign capture_commit=recording && ps;
 assign record_busy=record_pending || recording;
 assign busy=normal_busy || class_busy;
 assign stable=vote_stable && background_valid && templates_loaded==3 && !storage_active;
 assign stable_class=stable?vote_class:0;
 part_mask_normalizer #(.W(W),.H(H)) u_normalizer(
  .clk(clk),.rst_n(rst_n),.cancel(clear_job),.frame_start(gs),.sample_valid(gv),
  .foreground(fg),.consumer_ready(ready || recording || record_begin),.frame_id(gid),.busy(normal_busy),.dropped(dropped),
  .patch_begin(pb),.patch_we(pw),.patch_bit(pbit),.patch_start(ps),.patch_addr(pa),.patch_id(pid),
  .bbox_x0(bx0),.bbox_x1(bx1),.bbox_y0(by0),.bbox_y1(by1),.object_valid(object_valid),.diagnostic(mask_diagnostic));
 part_binary_template u_templates(
  .clk(clk),.rst_n(rst_n),.cancel((cancel || storage_load_begin || bg_start || record_request || config_update) && !teach_begin),.clear_templates(config_update),
  .teach_begin(local_teach_begin),.teach_class(record_begin?record_slot:teach_class),
  .teach_we(recording?pw:teach_we),.teach_bit(recording?pbit:teach_bit),
  .teach_commit(recording?ps:teach_commit),.teach_addr(recording?pa:teach_addr),
  .patch_begin(pb && !record_begin && !recording),.patch_we(pw && !recording),
  .patch_bit(pbit),.start(ps && !recording),.patch_addr(pa),.frame_id(pid),
  .ready(ready),.busy(class_busy),.done(class_done),.error(class_error),
  .templates_loaded(templates_loaded),.result_class(predicted_class),
  .cuboid_errors(cuboid_errors),.wedge_errors(wedge_errors),
  .result_frame_id(prediction_frame_id),.reject_reason(reject_reason));
 part_template_vote #(.CLK_HZ(CLK_HZ)) u_vote(
  .clk(clk),.rst_n(rst_n),.clear(clear_job),.result_update(prediction_update),
  .frame_id(prediction_frame_id),.class_id(predicted_class),.stable(vote_stable),.stable_class(vote_class));
 part_frame_difference #(.W(W),.H(H)) u_motion(
  .clk(clk),.rst_n(rst_n),.clear(clear_job),.frame_start(gs),.sample_valid(gv),.frame_id(gid),.gray(gray),.foreground(fg),
  .update(motion_update),.comparison_valid(),.motion_valid(motion_valid),.track_valid(track_valid),
  .global_change(global_change),.result_frame_id(motion_frame_id),.changed(motion_cells),
  .x0(motion_x0),.x1(motion_x1),.y0(motion_y0),.y1(motion_y1));
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
   prediction_update<=0;
   saved_x0<=0;saved_x1<=0;saved_y0<=0;saved_y1<=0;
   target_x0<=0;target_x1<=0;target_y0<=0;target_y1<=0;
   record_pending<=0;recording<=0;record_slot<=0;request_source<=0;
   record_done<=0;record_error<=0;record_frame_id<=0;
  end else begin
   record_done<=0;
   prediction_update<=class_done && !clear_job;
   if(ps)begin saved_x0<=bx0;saved_x1<=bx1;saved_y0<=by0;saved_y1<=by1;end
   if(class_done)begin
    target_x0<=saved_x0;target_x1<=saved_x1;target_y0<=saved_y0;target_y1<=saved_y1;
   end
   if(record_request)begin
    record_pending<=1;recording<=0;record_slot<=record_class;request_source<=frame_id;record_error<=0;
   end
   if(record_begin)begin record_pending<=0;recording<=1;record_frame_id<=pid;end
   if(pb && record_pending && fresh_patch && !object_valid)begin
    record_pending<=0;record_error<=1;record_done<=1;record_frame_id<=pid;
   end
   if(ps && recording)begin recording<=0;record_done<=1;end
   if(bg_start || cancel)begin record_pending<=0;recording<=0;record_error<=0;end
  end
endmodule
