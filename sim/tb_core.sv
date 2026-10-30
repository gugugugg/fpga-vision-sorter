`timescale 1ns/1ps
module tb;
reg clk=0,rst=0,sof=0,eol=0,valid=0;
reg [95:0] rgb=0;reg [2:0] mode=5;
wire os,ov,oe;wire [95:0] result;wire [15:0] exposure,gain;wire ae_update;
always #5 clk=~clk;
vision_core_4ppc #(.IMG_WIDTH(16),.IMG_HEIGHT(8),.AE_EXP_INIT(400),.AE_EXP_MAX(1000),
 .ENABLE_DETECTION(0),.ENABLE_LEGACY_STYLES(0)) dut(
 .i_clk(clk),.i_rst_n(rst),.i_sof(sof),.i_eof(eol),.i_valid(valid),.i_rgb(rgb),
 .i_mode(mode),.i_edge_threshold(10'd72),.o_sof(os),.o_eof(oe),.o_valid(ov),.o_rgb(result),
 .o_exposure(exposure),.o_gain_index(gain),.o_ae_update(ae_update));
reg [95:0] expected[0:1023];integer sent=0,received=0,k,f,lane,v;
integer updates=0,updates_before;reg [15:0] held_exposure;
reg completed_reference=0;
function [23:0] balanced(input [23:0] value);
 integer r,b;
 begin r=value[23:16]*2;if(r>255)r=255;b=value[7:0]/2;
 balanced={8'(r),value[15:8],8'(b)};end
endfunction
always @(posedge clk)if(rst)begin
 if(ae_update)updates=updates+1;
 if(dut.stats_valid && completed_reference &&
   (dut.u_awb_ctrl.i_stats_valid || dut.u_ae_ctrl.i_stats_valid))
  $fatal(1,"reference frame statistics reached normal controls");
 if(valid)begin
  for(integer n=0;n<4;n=n+1)
   expected[sent][n*24+:24]=mode==7?rgb[n*24+:24]:balanced(rgb[n*24+:24]);
  sent=sent+1;
 end
end
always @(negedge clk)if(rst && ov)begin
 if(received>=sent || result!==expected[received])
  $fatal(1,"core beat %0d result=%h expected=%h",received,result,expected[received]);
 received=received+1;
end
task frame(input [2:0] selected);
 begin
 @(negedge clk);completed_reference=mode==7;mode=selected;
 for(k=0;k<32;k=k+1)begin
  valid=1;sof=k==0;eol=k%4==3;
  for(lane=0;lane<4;lane=lane+1)rgb[lane*24+:24]={8'(24+k%12+lane),8'd40,8'd60};
  @(negedge clk);
 end
 valid=0;sof=0;eol=0;repeat(64)@(negedge clk);
 end
endtask
initial begin
 force dut.u_awb_ctrl.o_r_gain_q8=10'd512;
 force dut.u_awb_ctrl.o_b_gain_q8=10'd128;
 repeat(3)@(negedge clk);rst=1;
 for(f=0;f<6;f=f+1)frame(5);
 if(updates==0)$fatal(1,"normal AE did not update");
 updates_before=updates;held_exposure=exposure;
 for(f=0;f<6;f=f+1)frame(7);
 if(updates!=updates_before || exposure!=held_exposure)$fatal(1,"reference changed exposure");
 for(f=0;f<6;f=f+1)frame(5);
 if(updates<=updates_before)$fatal(1,"AE did not resume");
 if(received!=576)$fatal(1,"missing core beats %0d",received);
 $display("PASS core: original RGB exact, AWB retained in normal mode, AE held and resumed, common output alignment");$finish;
end
endmodule
