`timescale 1ns/1ps
// Ordered 32x32 silhouette patches. 1=cuboid/left, 2=wedge/right, 0=unknown.
// Teach both masks from this camera; no fabricated default recognition.
// Configuration and patch ports share a clock. No robot commands are emitted.
module part_binary_template #(
 parameter MIN_AREA=64
)(
 input wire clk,rst_n,cancel,clear_templates,
 input wire teach_begin,teach_class,teach_we,teach_bit,teach_commit,
 input wire [9:0] teach_addr,
 input wire patch_begin,patch_we,patch_bit,start,
 input wire [9:0] patch_addr,input wire [31:0] frame_id,
 output wire ready,output reg busy,done,error,
 output reg [1:0] templates_loaded,result_class,
 output reg [10:0] cuboid_errors,wedge_errors,
 output reg [31:0] result_frame_id,output reg [3:0] reject_reason
);
 (* ram_style="bram" *) reg cube[0:1023],wedge[0:1023],patch[0:1023];
 reg teaching,teach_slot,teach_bad,loading,patch_bad,patch_complete;
 reg [10:0] teach_count,teach_area,patch_count,patch_area;
 reg [31:0] patch_id;
 reg [9:0] index;reg phase;
 reg cube_q,wedge_q,patch_q;
 reg [10:0] cube_sum,wedge_sum;
 wire [10:0] cube_next=cube_sum+{10'd0,(cube_q^patch_q)};
 wire [10:0] wedge_next=wedge_sum+{10'd0,(wedge_q^patch_q)};
 assign ready=!busy && !teaching;
 always @(posedge clk) begin
  cube_q<=cube[index];wedge_q<=wedge[index];patch_q<=patch[index];
  if(rst_n && !cancel && teach_we && teaching && !busy &&
     teach_count<1024 && teach_addr==teach_count) begin
   if(teach_slot)wedge[teach_addr]<=teach_bit;else cube[teach_addr]<=teach_bit;
  end
  if(rst_n && !cancel && patch_we && loading && ready &&
     patch_count<1024 && patch_addr==patch_count)patch[patch_addr]<=patch_bit;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   teaching<=0;teach_slot<=0;teach_bad<=0;loading<=0;patch_bad<=0;patch_complete<=0;
   teach_count<=0;teach_area<=0;patch_count<=0;patch_area<=0;patch_id<=0;
   index<=0;phase<=0;cube_sum<=0;wedge_sum<=0;
   busy<=0;done<=0;error<=0;templates_loaded<=0;result_class<=0;
   cuboid_errors<=0;wedge_errors<=0;result_frame_id<=0;reject_reason<=0;
  end else begin
   done<=0;error<=0;
   if(teach_begin && !busy && !loading)begin
    teaching<=1;teach_slot<=teach_class;teach_count<=0;teach_area<=0;teach_bad<=0;
    templates_loaded[teach_class]<=0;result_class<=0;
   end
   if(teach_we && teaching && !busy)begin
    if(teach_count>=1024 || teach_addr!=teach_count)teach_bad<=1;
    else begin teach_count<=teach_count+1'b1;teach_area<=teach_area+teach_bit;end
   end
   if(teach_commit && teaching && !busy)begin
    teaching<=0;
    if(teach_count==1024 && !teach_bad && teach_area>=MIN_AREA)
      templates_loaded[teach_slot]<=1;
    else begin error<=1;reject_reason<=1;end
   end
   if(patch_begin && ready)begin
    loading<=1;patch_complete<=0;patch_count<=0;patch_area<=0;patch_bad<=0;patch_id<=frame_id;
   end
   if(patch_we && loading && ready)begin
    if(patch_count>=1024 || patch_addr!=patch_count)patch_bad<=1;
    else begin
     patch_count<=patch_count+1'b1;patch_area<=patch_area+patch_bit;
     if(patch_count==1023)patch_complete<=1;
    end
   end
   if(start && ready)begin
    loading<=0;result_frame_id<=patch_id;result_class<=0;
    cuboid_errors<=1024;wedge_errors<=1024;
    if(!patch_complete || patch_bad || patch_count!=1024)begin
     done<=1;error<=1;reject_reason<=2;
    end else if(templates_loaded!=3)begin done<=1;reject_reason<=3;end
    else if(patch_area<MIN_AREA)begin done<=1;reject_reason<=4;end
    else begin
     busy<=1;index<=0;phase<=0;cube_sum<=0;wedge_sum<=0;reject_reason<=0;
    end
   end
   if(busy)begin
    if(!phase)phase<=1;
    else begin
     phase<=0;cube_sum<=cube_next;wedge_sum<=wedge_next;
     if(index==1023)begin
      busy<=0;done<=1;patch_complete<=0;cuboid_errors<=cube_next;wedge_errors<=wedge_next;
      result_class<=cube_next<=wedge_next?1:2;reject_reason<=0;
     end else index<=index+1'b1;
    end
   end
   if(cancel)begin
    busy<=0;done<=0;error<=0;loading<=0;patch_complete<=0;result_class<=0;
    // Partial template replacement must never publish a valid mask.
    teaching<=0;reject_reason<=7;
   end
   // Explicit re-teach supersedes any in-flight classification. Never let an
   // old result regain validity while its template bank is being replaced.
   if(teach_begin && !cancel)begin
    busy<=0;done<=0;loading<=0;patch_complete<=0;result_class<=0;
    teaching<=1;teach_slot<=teach_class;teach_count<=0;teach_area<=0;teach_bad<=0;
    templates_loaded[teach_class]<=0;
   end
   if(clear_templates)begin
    templates_loaded<=0;teaching<=0;busy<=0;done<=0;loading<=0;patch_complete<=0;result_class<=0;
   end
  end
 end
endmodule
