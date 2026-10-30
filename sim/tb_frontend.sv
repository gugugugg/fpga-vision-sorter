`timescale 1ns/1ps
module tb;
reg clk=0,rst=0,valid=0,sof=0,last=0;
reg [39:0] raw=0;reg [5:0] requested=0;
wire [39:0] result;wire ov,os,ol;wire [5:0] frame_mode;
always #5 clk=~clk;
raw_color_shading_4ppc #(.WIDTH(8),.HEIGHT(3),.BLACK_LEVEL(64),.TABLE_FILE("gains.hex")) dut(
 .i_clk(clk),.i_rst_n(rst),.i_raw(raw),.i_valid(valid),.i_sof(sof),.i_last(last),
 .i_mode(requested),.o_frame_mode(frame_mode),.o_raw(result),.o_valid(ov),.o_sof(os),.o_last(ol));
reg [39:0] expected[0:63];reg es[0:63],el[0:63];
integer sent=0,received=0,row=0,chosen=0,f,k;
function [39:0] golden(input [39:0] value,input integer y,mode);
 integer lane,v,g;reg [39:0] r;
 begin
 r=value;
 if(mode!=14)for(lane=0;lane<4;lane=lane+1)begin
  v=value[lane*10+:10];v=v>64?v-64:0;
  if((y%2==0 && lane%2==0)||(y%2==1 && lane%2==1))begin
   g=y%2==0?1536:1280;v=(v*g+512)>>10;if(v>1023)v=1023;
  end
  r[lane*10+:10]=v;
 end
 golden=r;
 end
endfunction
always @(posedge clk)if(rst && valid)begin
 if(sof)begin chosen=requested;row=0;end
 expected[sent]=golden(raw,row,chosen);es[sent]=sof;el[sent]=last;sent=sent+1;
 if(last)row=row+1;
end
always @(negedge clk)if(rst && ov)begin
 if(received>=sent || result!==expected[received] || os!==es[received] || ol!==el[received])
  $fatal(1,"frontend beat %0d result=%h expected=%h",received,result,expected[received]);
 received=received+1;
end
initial begin
 repeat(3)@(negedge clk);rst=1;
 for(f=0;f<3;f=f+1)begin
  requested=f==1?14:0;
  for(k=0;k<6;k=k+1)begin
   @(negedge clk);valid=1;sof=k==0;last=k%2==1;
   raw={10'd1023,10'd64,10'd32,10'(96+k*9)};
   if(k==2)requested=f==1?0:14;
   @(negedge clk);valid=0;sof=0;last=0;
   if(frame_mode!=(f==1?14:0))$fatal(1,"mid-frame request changed active mode");
  end
  repeat(8)@(negedge clk);
 end
 if(received!=18)$fatal(1,"missing frontend beats %0d",received);
 $display("PASS frontend: original RAW10 exact, corrected arithmetic unchanged, frame-held switch, SOF/TLAST/gaps aligned");$finish;
end
endmodule
