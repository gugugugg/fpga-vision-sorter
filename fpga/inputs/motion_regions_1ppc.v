`timescale 1ns/1ps
// Frame differences on 32x32 bin-centre samples. No CCL, holes or 3D engine.
// The completed mask is swapped only at VS; the red overlay and CELLS count
// describe the preceding source frame. Duplicate DDR frames are not motion.
module motion_regions_1ppc #(parameter WIDTH=1024,HEIGHT=600,B=5,THRESHOLD=24)(
 input wire clk,rst_n,vsync,hsync,de,input wire [23:0] rgb,
 input wire [31:0] frame_id,
 output reg [23:0] overlay_rgb,output reg result_valid,
 output reg [31:0] result_frame_id,output reg [2*B:0] changed_cells
);
 localparam N=1<<B,P=1<<(2*B);
 reg vs_d,seen,started,history_valid,unique_frame,compare_frame;
 reg [10:0] x;reg [9:0] y;reg [B-1:0] col,row;
 reg [31:0] id,previous_id;
 reg [2*B:0] samples,count;
 reg sample_d,end_d;reg [2*B-1:0] sample_addr;reg [7:0] sample_luma,old_luma;
 (* ram_style="bram" *) reg [7:0] history[0:P-1];
 (* ram_style="bram" *) reg mask0[0:P-1],mask1[0:P-1];
 reg front,pending;reg [31:0] pending_id;reg [2*B:0] pending_count;
 wire start=vsync && !vs_d;
 wire [15:0] lum=rgb[23:16]*16'd77+rgb[15:8]*16'd150+rgb[7:0]*16'd29;
 wire sample_now=seen && unique_frame && de && !start &&
   x==((2*col+1)*WIDTH)/(2*N) && y==((2*row+1)*HEIGHT)/(2*N);
 wire [2*B-1:0] addr={row,col};
 wire [7:0] delta=sample_luma>old_luma?sample_luma-old_luma:old_luma-sample_luma;
 wire moved=compare_frame && delta>=THRESHOLD;
 reg mask_q,mask_d;reg [23:0] rgb_d,rgb_d2;reg de_d,de_d2;
 // Synchronous history and mask reads; RAM contents need no reset because
 // flags hide all uninitialised words until a complete frame was captured.
 always @(posedge clk) begin
  old_luma<=history[addr];
  mask_q<=front?mask1[addr]:mask0[addr];
  if(rst_n && sample_d) begin
   history[sample_addr]<=sample_luma;
   if(front) mask0[sample_addr]<=moved;else mask1[sample_addr]<=moved;
  end
 end
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin
   vs_d<=0;seen<=0;started<=0;history_valid<=0;unique_frame<=0;compare_frame<=0;
   x<=0;y<=0;col<=0;row<=0;id<=0;previous_id<=0;samples<=0;count<=0;
   sample_d<=0;end_d<=0;sample_addr<=0;sample_luma<=0;front<=0;pending<=0;
   pending_id<=0;pending_count<=0;result_valid<=0;result_frame_id<=0;changed_cells<=0;
   mask_d<=0;rgb_d<=0;rgb_d2<=0;de_d<=0;de_d2<=0;overlay_rgb<=0;
  end else begin
   vs_d<=vsync;sample_d<=sample_now;sample_addr<=addr;sample_luma<=lum[15:8];
   end_d<=de && seen && x==WIDTH-1 && y==HEIGHT-1 && unique_frame;
   rgb_d<=rgb;rgb_d2<=rgb_d;de_d<=de;de_d2<=de_d;
   mask_d<=mask_q && result_valid;
   overlay_rgb<=!de_d2?24'd0:mask_d?
      {8'hff,{1'b0,rgb_d2[15:9]},{1'b0,rgb_d2[7:1]}}:rgb_d2;
   if(sample_now) samples<=samples+1'b1;
   if(sample_d && moved) count<=count+1'b1;
   if(end_d) begin
    if(samples==P) begin
     pending<=1;pending_id<=id;pending_count<=count+(sample_d && moved);
     history_valid<=1;previous_id<=id;
    end else begin history_valid<=0;result_valid<=0;end
   end
   if(start) begin
    seen<=1;started<=0;x<=0;y<=0;col<=0;row<=0;samples<=0;count<=0;
    unique_frame<=0;sample_d<=0;
    if(pending) begin
     front<=~front;result_valid<=1;result_frame_id<=pending_id;changed_cells<=pending_count;
    end
    pending<=0;
   end else if(de && seen) begin
    if(!started) begin
     started<=1;id<=frame_id;
     unique_frame<=!history_valid || frame_id!=previous_id;
     compare_frame<=history_valid && frame_id==previous_id+1'b1;
    end
    if(x==WIDTH-1) begin
     x<=0;col<=0;y<=y+1'b1;
     if(y==((row+1)*HEIGHT)/N-1) row<=row+1'b1;
    end else begin
     x<=x+1'b1;
     if(x==((col+1)*WIDTH)/N-1) col<=col+1'b1;
    end
   end
  end
endmodule
