`timescale 1ns/1ps
// Row-major low-resolution samples, e.g. 128x75 in a fixed workstation ROI.
// Foreground gates old-position ghosts; full-frame changes invalidate tracking.
// One target only: multiple moving foregrounds are an ROI union, not CCL.
module part_frame_difference #(
 parameter W=128,H=75,THRESHOLD=20,MIN_CHANGED=4,HOLD_FRAMES=3
)(
 input wire clk,rst_n,clear,frame_start,sample_valid,
 input wire [31:0] frame_id,input wire [7:0] gray,input wire foreground,
 output reg update,comparison_valid,motion_valid,track_valid,global_change,
 output reg [31:0] result_frame_id,output reg [13:0] changed,
 output reg [7:0] x0,x1,y0,y1
);
 localparam P=W*H;
 (* ram_style="bram" *) reg [7:0] history[0:P-1];
 reg active,compare,history_valid,seen;
 reg [31:0] current_id,previous_id;
 reg [13:0] count,all_changed,fg_changed;
 reg [7:0] x,y,min_x,max_x,min_y,max_y,misses;
 reg pending,pcompare,pforeground;reg [13:0] pindex;
 reg [7:0] old_gray,pgray,px,py;
 wire [7:0] difference=pgray>old_gray?pgray-old_gray:old_gray-pgray;
 wire moving=pcompare && difference>=THRESHOLD;
 wire target_moving=moving && pforeground;
 wire [13:0] total=all_changed+moving,target_total=fg_changed+target_moving;
 wire [7:0] final_x0=target_moving && px<min_x?px:min_x;
 wire [7:0] final_x1=target_moving && px>max_x?px:max_x;
 wire [7:0] final_y0=target_moving && py<min_y?py:min_y;
 wire [7:0] final_y1=target_moving && py>max_y?py:max_y;
 always @(posedge clk)begin
  if(rst_n && active && sample_valid && !frame_start && count<P)begin
   old_gray<=history[count];history[count]<=gray;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   active<=0;compare<=0;history_valid<=0;seen<=0;current_id<=0;previous_id<=0;
   count<=0;all_changed<=0;fg_changed<=0;x<=0;y<=0;
   min_x<=W-1;max_x<=0;min_y<=H-1;max_y<=0;misses<=0;
   pending<=0;pcompare<=0;pforeground<=0;pindex<=0;pgray<=0;px<=0;py<=0;
   update<=0;comparison_valid<=0;motion_valid<=0;track_valid<=0;global_change<=0;
   result_frame_id<=0;changed<=0;x0<=0;x1<=0;y0<=0;y1<=0;
  end else begin
   update<=0;pending<=0;
   if(active && sample_valid && !frame_start && count<P)begin
    pending<=1;pcompare<=compare;pforeground<=foreground;pindex<=count;
    pgray<=gray;px<=x;py<=y;count<=count+1'b1;
    if(x==W-1)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;
   end
   if(pending)begin
    all_changed<=total;fg_changed<=target_total;
    if(target_moving)begin min_x<=final_x0;max_x<=final_x1;min_y<=final_y0;max_y<=final_y1;end
    if(pindex==P-1)begin
     active<=0;history_valid<=1;previous_id<=current_id;seen<=1;
     update<=1;result_frame_id<=current_id;comparison_valid<=pcompare;
     changed<=pcompare?target_total:0;global_change<=pcompare && total>P*3/4;
     motion_valid<=pcompare && total<=P*3/4 && target_total>=MIN_CHANGED;
     if(pcompare && total<=P*3/4 && target_total>=MIN_CHANGED)begin
      track_valid<=1;misses<=0;x0<=final_x0;x1<=final_x1;y0<=final_y0;y1<=final_y1;
     end else if(!pcompare || total>P*3/4 || misses+1>=HOLD_FRAMES)begin
      track_valid<=0;misses<=0;
     end else misses<=misses+1'b1;
    end
   end
   if(frame_start)begin
    // A partial frame has overwritten history; next complete frame is baseline.
    if(active)begin history_valid<=0;track_valid<=0;end
    active<=!seen || frame_id!=previous_id;
    compare<=history_valid && !active && frame_id!=previous_id;
    current_id<=frame_id;count<=0;all_changed<=0;fg_changed<=0;x<=0;y<=0;
    min_x<=W-1;max_x<=0;min_y<=H-1;max_y<=0;pending<=0;
   end
   if(clear)begin
    active<=0;pending<=0;history_valid<=0;seen<=0;track_valid<=0;
    motion_valid<=0;comparison_valid<=0;global_change<=0;update<=0;changed<=0;misses<=0;
   end
  end
 end
endmodule
