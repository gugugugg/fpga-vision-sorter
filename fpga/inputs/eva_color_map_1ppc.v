`timescale 1ns/1ps
// 9^3 RGB LUT, tetrahedral interpolation, two registered stages, 1 pixel/clk.
// ROM addresses and weights use the same sorted fractional axes. Four ROM
// read ports are explicit so synthesis can replicate ROMs without mux clocks.
// The existing display output register completes the three-clock contract.
module eva_color_map_1ppc #(parameter LUT_FILE="eva_grade_9.hex")(
 input wire clk,rst_n,input wire [23:0] rgb,output wire [23:0] mapped_rgb
);
 (* ram_style="bram" *) reg [23:0] rom0[0:728],rom1[0:728],rom2[0:728],rom3[0:728];
 initial begin
  $readmemh(LUT_FILE,rom0);$readmemh(LUT_FILE,rom1);
  $readmemh(LUT_FILE,rom2);$readmemh(LUT_FILE,rom3);
 end
 reg [4:0] fr,fg,fb;
 wire [9:0] base={7'd0,rgb[23:21]}*10'd81+
                  {7'd0,rgb[15:13]}*10'd9+{7'd0,rgb[7:5]};
 reg [9:0] off1,off2;
 reg [4:0] f1,f2,f3;
 always @* begin
  fr=rgb[20:16];fg=rgb[12:8];fb=rgb[4:0];
  if(fr>=fg && fg>=fb) begin off1=81;off2=90;f1=fr;f2=fg;f3=fb;end
  else if(fr>=fb && fb>=fg) begin off1=81;off2=82;f1=fr;f2=fb;f3=fg;end
  else if(fb>=fr && fr>=fg) begin off1=1;off2=82;f1=fb;f2=fr;f3=fg;end
  else if(fg>=fr && fr>=fb) begin off1=9;off2=90;f1=fg;f2=fr;f3=fb;end
  else if(fg>=fb && fb>=fr) begin off1=9;off2=10;f1=fg;f2=fb;f3=fr;end
  else begin off1=1;off2=10;f1=fb;f2=fg;f3=fr;end
 end
 reg [23:0] p0,p1,p2,p3;
 reg [4:0] w1,w2,w3;
 // No reset on ROM outputs, permitting native synchronous ROM inference.
 always @(posedge clk) begin
  p0<=rom0[base];p1<=rom1[base+off1];p2<=rom2[base+off2];p3<=rom3[base+91];
 end
 function signed [16:0] mix;
  input [7:0] a,b,c,d;input [4:0] x,y,z;
  reg signed [8:0] ab,bc,cd;
  reg signed [14:0] mx,my,mz;
  begin
   ab=$signed({1'b0,b})-$signed({1'b0,a});
   bc=$signed({1'b0,c})-$signed({1'b0,b});
   cd=$signed({1'b0,d})-$signed({1'b0,c});
   mx=ab*$signed({1'b0,x});my=bc*$signed({1'b0,y});mz=cd*$signed({1'b0,z});
   mix=$signed({4'd0,a,5'd0})+17'sd16+mx+my+mz;
  end
 endfunction
 reg signed [16:0] red,green,blue;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin w1<=0;w2<=0;w3<=0;red<=0;green<=0;blue<=0;end
  else begin
   w1<=f1;w2<=f2;w3<=f3;
   red<=mix(p0[23:16],p1[23:16],p2[23:16],p3[23:16],w1,w2,w3);
   green<=mix(p0[15:8],p1[15:8],p2[15:8],p3[15:8],w1,w2,w3);
   blue<=mix(p0[7:0],p1[7:0],p2[7:0],p3[7:0],w1,w2,w3);
  end
 function [7:0] clip;
  input signed [16:0] v;
  begin
   if(v < 0) clip=8'd0;
   else if(v > 17'sd8160) clip=8'd255;
   else clip=v[12:5];
  end
 endfunction
 assign mapped_rgb={clip(red),clip(green),clip(blue)};
endmodule
