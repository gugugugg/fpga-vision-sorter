`timescale 1ns/1ps
// Coarse RGB565 subtraction against an explicitly taught empty workstation.
// Teach is latched at frame start. Only a COMPLETE teach frame becomes valid.
// R/B thresholds are in 5-bit codes; G in 6-bit codes (4/8 ~=32 of 255).
module part_background_mask #(parameter W=128,H=75,RB_THRESHOLD=6,G_THRESHOLD=12)(
 input wire clk,rst_n,cancel,frame_start,teach_background,sample_valid,
 input wire storage_active,storage_load_begin,storage_load_we,storage_load_done,
 input wire [13:0] storage_addr,input wire [15:0] storage_pixel,output wire [15:0] storage_q,
 input wire [15:0] rgb565,input wire [31:0] frame_id,
 input wire threshold_update,input wire [4:0] threshold_rb,input wire [5:0] threshold_g,
 output reg [4:0] applied_rb,output reg [5:0] applied_g,
 output reg background_valid,output reg out_start,out_valid,foreground,
 output reg [7:0] gray,output reg [31:0] out_id
);
 localparam P=W*H;
 (* ram_style="bram" *) reg [15:0] background[0:P-1];
 reg active,teaching,pending,pteach,pvalid;
 reg config_pending;reg [4:0] pending_rb;reg [5:0] pending_g;
 reg [13:0] count,pindex;reg [15:0] old_pixel,pixel;
 assign storage_q=old_pixel;
 wire [4:0] dr=pixel[15:11]>old_pixel[15:11]?pixel[15:11]-old_pixel[15:11]:old_pixel[15:11]-pixel[15:11];
 wire [5:0] dg=pixel[10:5]>old_pixel[10:5]?pixel[10:5]-old_pixel[10:5]:old_pixel[10:5]-pixel[10:5];
 wire [4:0] db=pixel[4:0]>old_pixel[4:0]?pixel[4:0]-old_pixel[4:0]:old_pixel[4:0]-pixel[4:0];
 // Shift/add luminance approximates (R+2G+B)/4, all channels expanded to8 bits.
 wire [9:0] lum={pixel[15:11],3'b0}+{pixel[10:5],3'b0}+{pixel[4:0],3'b0};
 wire normal_access=active && sample_valid && !frame_start && count<P;
 wire [13:0] memory_addr=storage_active?storage_addr:count;
 wire [15:0] memory_data=storage_active?storage_pixel:rgb565;
 wire memory_write=storage_active?storage_load_we:teaching;
 always @(posedge clk)begin
  if(rst_n && (storage_active || normal_access))begin
   old_pixel<=background[memory_addr];
   if(memory_write)background[memory_addr]<=memory_data;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   active<=0;teaching<=0;pending<=0;pteach<=0;pvalid<=0;count<=0;pindex<=0;pixel<=0;
   background_valid<=0;out_start<=0;out_valid<=0;foreground<=0;gray<=0;out_id<=0;
   applied_rb<=RB_THRESHOLD;applied_g<=G_THRESHOLD;config_pending<=0;pending_rb<=RB_THRESHOLD;pending_g<=G_THRESHOLD;
  end else begin
   pending<=0;out_start<=0;out_valid<=0;
   if(active && sample_valid && !frame_start && !storage_active && count<P)begin
    pending<=1;pixel<=rgb565;pteach<=teaching;pvalid<=background_valid;
    pindex<=count;count<=count+1'b1;
   end
   if(pending)begin
    out_valid<=!pteach && pvalid;gray<=lum[9:2];
    foreground<=!pteach && pvalid && (dr>=applied_rb || dg>=applied_g || db>=applied_rb);
    if(pindex==P-1)begin active<=0;if(pteach)background_valid<=1;end
   end
   if(frame_start)begin
    if(config_pending)begin applied_rb<=pending_rb;applied_g<=pending_g;config_pending<=0;end
    active<=1;teaching<=teach_background;count<=0;pending<=0;
    // An interrupted replacement may already have overwritten reference RAM.
    if(teach_background || (active && teaching))background_valid<=0;
    out_start<=!teach_background && background_valid && !(active && teaching);out_id<=frame_id;
   end
   if(storage_active)begin active<=0;pending<=0;teaching<=0;end
   if(storage_load_begin)background_valid<=0;
   if(storage_load_done)background_valid<=1;
   if(cancel)begin active<=0;pending<=0;if(teaching)background_valid<=0;teaching<=0;end
   if(threshold_update && threshold_rb!=0 && threshold_g!=0)begin
    pending_rb<=threshold_rb;pending_g<=threshold_g;config_pending<=1;
   end
  end
 end
endmodule
