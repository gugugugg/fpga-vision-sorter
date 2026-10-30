`timescale 1ns/1ps
// Bin-centre sampler for the grid core. SOF precedes the first active pixel;
// DE samples arrive in row-major raster order, independent of line blanking.
// Fixed ROI is an experiment parameter, not calibrated camera coordinates.
module part_visual_sampler_1ppc #(
 parameter WIDTH=1024,HEIGHT=600,ROI_X=256,ROI_Y=150,ROI_W=512,ROI_H=300,GW=128,GH=75
)(input wire clk,rst_n,sof,de,input wire [23:0] rgb,input wire [31:0] frame_id,
 output reg grid_start,grid_valid,output reg [15:0] grid_rgb565,output reg [31:0] grid_frame_id);
 reg active;reg [10:0] x;reg [9:0] y;reg [7:0] col,row;
 wire [19:0] x_product=(2*{1'b0,col}+1)*ROI_W;
 wire [19:0] y_product=(2*{1'b0,row}+1)*ROI_H;
 // TD otherwise maps the seven-pixel bin stride to scarce DSP multipliers.
 function [10:0] x_step(input [10:0] i);
  begin x_step=(ROI_W/GW==7)?(i<<3)-i:i*(ROI_W/GW);end
 endfunction
 function [9:0] y_step(input [9:0] i);
  begin y_step=(ROI_H/GH==7)?(i<<3)-i:i*(ROI_H/GH);end
 endfunction
 wire [10:0] sample_x=(ROI_W%GW==0)?ROI_X+x_step({3'd0,col})+(ROI_W/GW)/2:ROI_X+x_product/(2*GW);
 wire [9:0] sample_y=(ROI_H%GH==0)?ROI_Y+y_step({2'd0,row})+(ROI_H/GH)/2:ROI_Y+y_product/(2*GH);
 wire sample_now=active && de && !sof && col<GW && row<GH && x==sample_x && y==sample_y;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
   active<=0;x<=0;y<=0;col<=0;row<=0;
   grid_start<=0;grid_valid<=0;grid_rgb565<=0;grid_frame_id<=0;
  end else begin
   grid_start<=0;grid_valid<=0;
   if(sof)begin
    active<=1;x<=0;y<=0;col<=0;row<=0;grid_start<=1;grid_frame_id<=frame_id;
   end else if(active && de)begin
    if(sample_now)begin
     grid_valid<=1;grid_rgb565<={rgb[23:19],rgb[15:10],rgb[7:3]};
     col<=col+1'b1;
    end
    if(x==WIDTH-1)begin
     x<=0;y<=y+1'b1;col<=0;
     if(y==sample_y && row<GH)row<=row+1'b1;
     if(y==HEIGHT-1)active<=0;
    end else x<=x+1'b1;
   end
  end
endmodule
