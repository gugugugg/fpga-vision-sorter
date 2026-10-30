`timescale 1ns/1ps
// Key edges come from the existing menu debouncer.
module template_capture_controls(
 input wire clk,rst_n,sw1,sw4,input wire [3:0] key_press,
 output wire capture_mode,command_update,output wire [15:0] command_code
);
 (* async_reg="true" *) reg capture_sync0,capture_sync1,sw4_sync0,sw4_sync1;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin capture_sync0<=0;capture_sync1<=0;sw4_sync0<=0;sw4_sync1<=0;end
  else begin capture_sync0<=sw1;capture_sync1<=capture_sync0;sw4_sync0<=sw4;sw4_sync1<=sw4_sync0;end
 assign capture_mode=capture_sync1;
 assign command_update=capture_mode && !sw4_sync1 && |key_press;
 assign command_code=key_press[3]?16'd4:key_press[0]?16'd1:key_press[1]?16'd2:16'd3;
endmodule
