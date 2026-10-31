`timescale 1ns/1ps
module tb_motion_noise_threshold;
 reg clk=0,rst_n=0,frame_start=0,sample_valid=0;
 reg [31:0] frame_id=0;
 reg [7:0] gray=100;
 wire motion_valid,track_valid;
 wire [13:0] changed;
 always #5 clk=~clk;
 part_frame_difference #(.W(16),.H(8)) dut(
  .clk(clk),.rst_n(rst_n),.clear(1'b0),.frame_start(frame_start),.sample_valid(sample_valid),
  .frame_id(frame_id),.gray(gray),.foreground(1'b1),.motion_valid(motion_valid),.track_valid(track_valid),.changed(changed));
 task frame(input integer changes);
 begin
  @(negedge clk);frame_id=frame_id+1;frame_start=1;
  @(negedge clk);frame_start=0;
  for(integer i=0;i<128;i=i+1)begin
   sample_valid=1;gray=i<changes?130:100;
   @(negedge clk);sample_valid=0;@(negedge clk);
  end
  repeat(5)@(negedge clk);
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  frame(0);
  frame(4);if(motion_valid || changed!=4)$fatal(1,"four noisy cells triggered motion");
  frame(0);
  frame(9);if(motion_valid || changed!=9)$fatal(1,"nine noisy cells triggered motion");
  frame(0);
  frame(11);if(motion_valid || changed!=11)$fatal(1,"eleven cells triggered motion");
  frame(0);
  frame(12);if(!motion_valid || !track_valid || changed!=12)$fatal(1,"twelve moving cells were not detected");
  frame(12);frame(12);frame(12);
  if(motion_valid || track_valid || changed!=0)$fatal(1,"stationary target kept tracking active");
  $display("PASS motion threshold: 4/9/11 changed cells ignored, 12 detected, stationary hold expired");
  $finish;
 end
 initial begin #1000000;$fatal(1,"simulation did not complete");end
endmodule
