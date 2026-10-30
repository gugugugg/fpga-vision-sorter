`timescale 1ns/1ps
// Decoded camera SOF is a one-source-clock pulse. Transfer its toggle so that
// the slower 50 MHz counter cannot miss it. Display repetitions are excluded.
module capture_frame_rate #(parameter SYS_HZ=50000000)(
 input wire source_clk,source_rst_n,source_sof,
 input wire sys_clk,sys_rst_n,display_clk,display_rst_n,
 output wire [7:0] fps
);
 reg frame_toggle;
 always @(posedge source_clk or negedge source_rst_n)
  if(!source_rst_n) frame_toggle<=0;
  else if(source_sof) frame_toggle<=~frame_toggle;
 display_frame_rate #(.SYS_HZ(SYS_HZ),.TOGGLE_INPUT(1)) counter(
  .sys_clk(sys_clk),.sys_rst_n(sys_rst_n),.display_clk(display_clk),
  .display_rst_n(display_rst_n),.display_vsync(frame_toggle),.fps(fps));
endmodule
