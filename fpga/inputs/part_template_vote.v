`timescale 1ns/1ps
// Frame-aware classification vote, without any physical motion output.
// Duplicate source IDs do not accumulate votes. Unknown clears stability; skipped source IDs do not.
module part_template_vote #(
 parameter STABLE_RESULTS=3,CLK_HZ=50000000,TIMEOUT_MS=500
)(input wire clk,rst_n,clear,result_update,input wire [31:0] frame_id,
 input wire [1:0] class_id,output reg stable,output reg [1:0] stable_class);
 localparam TIMEOUT_CYCLES=CLK_HZ/1000*TIMEOUT_MS;
 reg seen;reg [31:0] last_id,age;reg [7:0] votes;reg [1:0] candidate;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin seen<=0;last_id<=0;age<=0;votes<=0;candidate<=0;stable<=0;stable_class<=0;end
  else begin
   if(age<TIMEOUT_CYCLES)age<=age+1'b1;
   else begin stable<=0;stable_class<=0;votes<=0;candidate<=0;seen<=0;end
   if(result_update && (!seen || frame_id!=last_id))begin
    seen<=1;last_id<=frame_id;age<=0;
    if(class_id==0 || class_id==3)begin votes<=0;candidate<=0;stable<=0;stable_class<=0;end
    else if(!seen || candidate!=class_id || votes==0)begin
     candidate<=class_id;votes<=1;stable<=STABLE_RESULTS<=1;stable_class<=STABLE_RESULTS<=1?class_id:0;
    end else if(votes<STABLE_RESULTS)begin
     votes<=votes+1'b1;
     if(votes+1>=STABLE_RESULTS)begin stable<=1;stable_class<=class_id;end
    end
   end
   if(clear)begin seen<=0;age<=0;votes<=0;candidate<=0;stable<=0;stable_class<=0;end
  end
endmodule
