`timescale 1ns/1ps
// Assert immediately, release after two rising edges of the receiving clock.
// A stopped clock keeps its domain in reset until it actually runs.
module reset_release(input wire clk,async_rst_n,output wire rst_n);
 (* async_reg="true",syn_preserve=1 *) reg [1:0] release_pipe;
 always @(posedge clk or negedge async_rst_n)
  if(!async_rst_n) release_pipe<=0;
  else release_pipe<={release_pipe[0],1'b1};
 assign rst_n=release_pipe[1];
endmodule
