`timescale 1ns/1ps
// Causal, per-channel horizontal impulse suppression. The two preceding
// samples must agree; real high-frequency texture is otherwise retained.
// Combinational output enters the LUT read register, adding no stream delay.
module eva_denoise_1ppc(input wire clk,rst_n,frame_start,de,
 input wire [23:0] rgb,output wire [23:0] clean_rgb);
 reg [23:0] left1,left2;reg [1:0] count;
 function [7:0] diff;
  input [7:0] a,b;begin diff=a>b?a-b:b-a;end
 endfunction
 function [7:0] suppress;
  input [7:0] current,a,b;
  reg [9:0] sum;
  begin
   sum={2'd0,current,1'b0}+{3'd0,a}+{3'd0,b}+10'd2;
   suppress=(diff(a,b)<12 && diff(current,a)>=24 && diff(current,b)>=24)
       ?sum[9:2]:current;
  end
 endfunction
 assign clean_rgb=count<2?rgb:
  {suppress(rgb[23:16],left1[23:16],left2[23:16]),
   suppress(rgb[15:8],left1[15:8],left2[15:8]),
   suppress(rgb[7:0],left1[7:0],left2[7:0])};
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin left1<=0;left2<=0;count<=0;end
  else if(frame_start || !de) begin count<=0;left1<=0;left2<=0;end
  else begin left2<=left1;left1<=rgb;if(count<2) count<=count+1'b1;end
endmodule
