`timescale 1ns/1ps
// Fixed 16-byte configuration frame: D5, kind, address, 12 payload bytes,
// CRC-8/ATM (poly 07, init 00) over kind/address/payload, MSB first.
// Truncated frames expire; malformed frames never publish partial settings.
module arm_packet_decoder #(parameter TIMEOUT_CYCLES=250000)(
 input wire clk,rst_n,byte_valid,input wire [7:0] byte_data,
 output reg valid,output reg [7:0] kind,address,
 output reg [95:0] payload,output reg error,output wire active
);
 reg [3:0] count;reg [7:0] crc,k,a;reg [95:0] data;
 reg [31:0] timeout;
 assign active=count!=0;
 function [7:0] crc8;
  input [7:0] prev,b;reg [7:0] v;integer n;
  begin v=prev^b;for(n=0;n<8;n=n+1) v=v[7] ? (v<<1)^8'h07:v<<1;crc8=v;end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin count<=0;crc<=0;k<=0;a<=0;data<=0;timeout<=0;
   valid<=0;kind<=0;address<=0;payload<=0;error<=0;end
  else begin
   valid<=0;error<=0;
   if(count!=0 && timeout==TIMEOUT_CYCLES-1) begin count<=0;timeout<=0;error<=1;end
   else if(byte_valid) begin
    timeout<=0;
    if(count==0) begin if(byte_data==8'hd5) begin count<=1;crc<=0;end end
    else if(count==15) begin
     count<=0;
     if(byte_data==crc) begin kind<=k;address<=a;payload<=data;valid<=1;end
     else error<=1;
    end else begin
     crc<=crc8(crc,byte_data);count<=count+1'b1;
     if(count==1) k<=byte_data;
     else if(count==2) a<=byte_data;
     else data<={data[87:0],byte_data};
    end
   end else if(count!=0) timeout<=timeout+1'b1;
  end
 end
endmodule
