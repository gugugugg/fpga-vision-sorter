`timescale 1ns/1ps
module tb_manual_template_capture;
 reg clk=0,rst_n=0,frame_start=0,sample_valid=0,teach_background=0;
 reg record_request=0,record_class=0;
 reg [31:0] frame_id=0;
 reg [15:0] rgb565=16'h001f;
 wire record_done,record_error,background_valid,capture_begin,capture_we,capture_bit;
 wire [1:0] loaded,predicted_class;
 wire [9:0] capture_addr;
 wire [63:0] diagnostic;
 integer completed=0,writes=0,ones=0;
 always #5 clk=~clk;
 part_visual_grid #(.W(16),.H(12)) dut(
  .clk(clk),.rst_n(rst_n),.cancel(1'b0),.frame_start(frame_start),.sample_valid(sample_valid),
  .teach_background(teach_background),.frame_id(frame_id),.rgb565(rgb565),
  .storage_active(1'b0),.storage_load_begin(1'b0),.storage_load_we(1'b0),.storage_load_done(1'b0),
  .storage_addr(14'd0),.storage_pixel(16'd0),.threshold_update(1'b0),.threshold_rb(5'd6),.threshold_g(6'd12),
  .teach_begin(1'b0),.teach_class(1'b0),.teach_we(1'b0),.teach_bit(1'b0),.teach_commit(1'b0),.teach_addr(10'd0),
  .record_request(record_request),.record_class(record_class),.record_done(record_done),.record_error(record_error),
  .background_valid(background_valid),.templates_loaded(loaded),.predicted_class(predicted_class),.diagnostic(diagnostic),
  .capture_begin(capture_begin),.capture_we(capture_we),.capture_bit(capture_bit),.capture_addr(capture_addr));
 always @(posedge clk)begin
  if(capture_begin)begin writes=0;ones=0;end
  if(capture_we)begin
   if(capture_addr!=writes)$fatal(1,"template pixels out of order");
   writes=writes+1;ones=ones+capture_bit;
  end
  if(record_done)begin
   if(record_error || writes!=1024)$fatal(1,"capture failed or incomplete");
   completed=completed+1;
  end
 end
 task frame(input integer shape,input integer background);
 begin
  @(negedge clk);frame_id=frame_id+1;frame_start=1;teach_background=background;
  @(negedge clk);frame_start=0;teach_background=0;
  for(integer y=0;y<12;y=y+1)begin
   for(integer x=0;x<16;x=x+1)begin
    sample_valid=1;rgb565=16'h001f;
    if((shape==1 && x>=3 && x<=12 && y>=3 && y<=8) ||
       (shape==2 && x<=4 && y>=3 && y<=8) || shape==3 ||
       (shape==4 && x>=3 && x<=12 && y>=2 && y<=9 && x-3<=y-2) ||
       (shape==5 && ((x>=3 && x<=12 && y>=3 && y<=8) || (x>=6 && x<=9 && y>=9))))rgb565=16'hffff;
    @(negedge clk);sample_valid=0;@(negedge clk);
   end
  end
  repeat(10000)@(negedge clk);
 end endtask
 task capture(input integer slot,input integer shape);
 integer before_count;
 begin
  before_count=completed;
  @(negedge clk);record_class=slot;record_request=1;
  @(negedge clk);record_request=0;
  frame(shape,0);
  if(completed!=before_count+1 || !loaded[slot])$fatal(1,"selected template not committed: slot=%0d shape=%0d",slot,shape);
  if(shape!=0 && ones==0)$fatal(1,"foreground lost during unchecked capture");
  if(shape==0 && ones!=0)$fatal(1,"empty capture was not empty");
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  frame(0,1);if(!background_valid)$fatal(1,"background not captured");
  capture(0,1);capture(1,4);
  capture(0,5);frame(5,0);
  if(!diagnostic[51] || diagnostic[50:48]!=0 || diagnostic[7:0]!=11 || predicted_class!=1)
   $fatal(1,"shadow touching the ROI bottom: diagnostic=%h class=%0d scores=%0d/%0d",diagnostic,predicted_class,dut.cuboid_errors,dut.wedge_errors);
  capture(0,2);capture(1,3);
  capture(0,0);
  frame(0,0);if(predicted_class!=0)$fatal(1,"empty workstation classified as a part");
  @(negedge clk);rst_n=0;repeat(4)@(negedge clk);rst_n=1;
  capture(1,0);if(background_valid)$fatal(1,"capture fabricated a background");
  $display("PASS manual capture: cuboid, wedge, bottom shadow recognition, border capture, large foreground, empty, no background; empty recognition retained");
  $finish;
 end
 initial begin #10000000;$fatal(1,"simulation did not complete");end
endmodule
