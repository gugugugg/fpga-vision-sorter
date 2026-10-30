`timescale 1ns/1ps
// PH1P board serializer adapted from the BSP's lane_lvds_10_1. Preserve bit
// order and load cadence, but reset each clock domain independently.
module board_tmds_lane(
 input wire pixel_clk,serial_clk,pixel_rst_n,serial_rst_n,
 input wire [9:0] data,output wire serial_out
);
 reg odd_even;
 reg odd0,odd1,load,load0,load1;
 reg [9:0] shift;reg [1:0] oddr_data;
 always @(posedge pixel_clk or negedge pixel_rst_n)
  if(!pixel_rst_n) odd_even<=0;else odd_even<=~odd_even;
 always @(posedge serial_clk or negedge serial_rst_n)
  if(!serial_rst_n) begin odd0<=0;odd1<=0;load<=0;load0<=0;load1<=0;shift<=0;oddr_data<=0;end
  else begin
   odd0<=odd_even;odd1<=odd0;load<=odd0^odd1;load0<=load;load1<=load0;
   shift<=load1?data:(shift<<2);oddr_data<=shift[9:8];
  end
 PH1P_LOGIC_ODDR oddr(.clk(serial_clk),.rst(~serial_rst_n),
                    .d1(oddr_data[0]),.d0(oddr_data[1]),.q(serial_out));
endmodule

module board_tmds_phy #(parameter DEVICE="PH1P")(
 input wire I_pixel_clk,I_serial_clk,I_rst,I_serial_rst_n,
 input wire [9:0] I_tmds_channel_0,I_tmds_channel_1,I_tmds_channel_2,I_tmds_channel_clk,
 output wire O_tmds_ch0_p,O_tmds_ch1_p,O_tmds_ch2_p,O_tmds_clk_p
);
 function [9:0] reverse;
  input [9:0] x;integer i;begin for(i=0;i<10;i=i+1) reverse[i]=x[9-i];end
 endfunction
 board_tmds_lane ch0(I_pixel_clk,I_serial_clk,~I_rst,I_serial_rst_n,reverse(I_tmds_channel_0),O_tmds_ch0_p);
 board_tmds_lane ch1(I_pixel_clk,I_serial_clk,~I_rst,I_serial_rst_n,reverse(I_tmds_channel_1),O_tmds_ch1_p);
 board_tmds_lane ch2(I_pixel_clk,I_serial_clk,~I_rst,I_serial_rst_n,reverse(I_tmds_channel_2),O_tmds_ch2_p);
 board_tmds_lane chclk(I_pixel_clk,I_serial_clk,~I_rst,I_serial_rst_n,reverse(I_tmds_channel_clk),O_tmds_clk_p);
endmodule
