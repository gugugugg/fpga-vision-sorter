`timescale 1ns/1ps
// A common microsecond Gray timebase crosses to the two event domains.
// Capture timestamp is latched at CSI frame start, then paired with the actual
// accepted demosaic frame ID. Display endpoint is first valid OSD pixel of
// that same DDR-selected frame. Quantization: 1us plus synchronizer latency.
module capture_display_latency #(parameter SYS_HZ=50000000)(
 input wire sys_clk,sys_rst_n,source_clk,source_rst_n,csi_start,source_sof,
 input wire [31:0] next_source_id,
 input wire display_clk,display_rst_n,display_vsync,display_de,
 input wire [31:0] display_id,
 output reg valid,output reg [15:0] milliseconds
);
 reg [15:0] prescale;reg [31:0] time_us,time_gray;
 always @(posedge sys_clk or negedge sys_rst_n)
  if(!sys_rst_n) begin prescale<=0;time_us<=0;time_gray<=0;end
  else if(prescale==SYS_HZ/1000000-1) begin
    prescale<=0;time_us<=time_us+1'b1;time_gray<=(time_us+1'b1)^((time_us+1'b1)>>1);
  end else prescale<=prescale+1'b1;
 (* async_reg="true" *) reg [31:0] sg0,sg1,dg0,dg1;
 function [31:0] binary;
 input [31:0] gray;reg [31:0] v;
 begin
  v=gray^(gray>>1);v=v^(v>>2);v=v^(v>>4);
  v=v^(v>>8);binary=v^(v>>16);
 end
 endfunction
 always @(posedge source_clk or negedge source_rst_n)
  if(!source_rst_n)begin sg0<=0;sg1<=0;end else begin sg0<=time_gray;sg1<=sg0;end
 always @(posedge display_clk or negedge display_rst_n)
  if(!display_rst_n)begin dg0<=0;dg1<=0;end else begin dg0<=time_gray;dg1<=dg0;end
 reg [31:0] captured;reg [63:0] transfer;reg publish;
 always @(posedge source_clk or negedge source_rst_n)
  if(!source_rst_n) begin captured<=0;transfer<=0;publish<=0;end
  else begin
    publish<=0;
    if(csi_start) captured<=binary(sg1);
    if(source_sof) begin transfer<={next_source_id,csi_start?binary(sg1):captured};publish<=1;end
  end
 wire update;wire [63:0] received;
 cdc_bus_toggle #(.WIDTH(64)) timestamp_cdc(
 .i_src_clk(source_clk),.i_src_rst_n(source_rst_n),.i_src_update(publish),.i_src_data(transfer),
 .o_src_busy(),.i_dst_clk(display_clk),.i_dst_rst_n(display_rst_n),.o_dst_update(update),.o_dst_data(received));
 reg [31:0] ids[0:7],stamps[0:7];reg [7:0] known;reg vs_d,armed;
 wire [31:0] elapsed=binary(dg1)-stamps[display_id[2:0]];
 reg [31:0] event_elapsed;reg event_known,divide_start;
 wire divide_done;wire [39:0] quotient;
 seq_div_u40_u32 ms_div(.i_clk(display_clk),.i_rst_n(display_rst_n),
 .i_start(divide_start),.i_numerator({8'd0,event_elapsed}),.i_denominator(32'd1000),
 .o_done(divide_done),.o_busy(),.o_quotient(quotient));
 always @(posedge display_clk or negedge display_rst_n)
  if(!display_rst_n) begin
    known<=0;vs_d<=0;armed<=0;valid<=0;milliseconds<=0;
    event_elapsed<=0;event_known<=0;divide_start<=0;
  end
  else begin
    vs_d<=display_vsync;divide_start<=0;
    if(update) begin ids[received[34:32]]<=received[63:32];stamps[received[34:32]]<=received[31:0];known[received[34:32]]<=1;end
    if(display_vsync && !vs_d) armed<=1;
    if(display_de && armed) begin
      armed<=0;valid<=0;
      event_known<=known[display_id[2:0]] && ids[display_id[2:0]]==display_id && display_id!=0;
      event_elapsed<=elapsed;divide_start<=1;
    end
    if(divide_done) begin
      valid<=event_known;
      milliseconds<=|quotient[39:16]?16'hffff:quotient[15:0];
    end
  end
endmodule
