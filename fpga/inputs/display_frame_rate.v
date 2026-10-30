`timescale 1ns/1ps
// Count actual HDMI frame starts over one second of the existing system clock.
// DFPS describes display cadence, including repeated DDR frames, not sensor FPS.
module display_frame_rate #(parameter SYS_HZ=50000000,TOGGLE_INPUT=0)(
 input wire sys_clk,sys_rst_n,display_clk,display_rst_n,display_vsync,
 output wire [7:0] fps
);
 (* async_reg="true",syn_preserve=1 *) reg vs0,vs1;
 reg vs_previous;reg [31:0] ticks;reg [7:0] frames,publication;
 reg update;wire received;
 wire frame_edge=TOGGLE_INPUT?(vs1 ^ vs_previous):(vs1 && !vs_previous);
 always @(posedge sys_clk or negedge sys_rst_n)
  if(!sys_rst_n) begin
   vs0<=0;vs1<=0;vs_previous<=0;ticks<=0;frames<=0;publication<=0;update<=0;
  end else begin
   vs0<=display_vsync;vs1<=vs0;vs_previous<=vs1;update<=0;
   if(ticks==SYS_HZ-1) begin
    ticks<=0;publication<=frames+(frame_edge && frames!=255);frames<=0;update<=1;
   end else begin
    ticks<=ticks+1'b1;
    if(frame_edge && frames!=255) frames<=frames+1'b1;
   end
  end
 cdc_bus_toggle #(.WIDTH(8)) rate_cdc(
  .i_src_clk(sys_clk),.i_src_rst_n(sys_rst_n),.i_src_update(update),
  .i_src_data(publication),.o_src_busy(),.i_dst_clk(display_clk),
  .i_dst_rst_n(display_rst_n),.o_dst_update(received),.o_dst_data(fps));
endmodule
