`timescale 1ns/1ps
// One explicitly armed ID5 operation. Poll six FRESH servo replies per step.
// A plateau is a contact candidate, never an automatic lift/release trigger.
module arm_grip_feedback #(
 parameter CLK_HZ=50000000,REPLY_CYCLES=CLK_HZ/4,SAMPLE_CYCLES=CLK_HZ/10,
 parameter LOWER_P=1337,UPPER_P=1613,SPAN_P=3,ERROR_P=20,MIN_TRAVEL_P=20
)(input wire clk,rst_n,start,cancel,rx_error,
 input wire [15:0] limit_p,step_p,time_ms,
 input wire bus_ready,bus_done,bus_busy,position_valid,
 input wire [7:0] position_id,input wire [15:0] position,
 output wire active,command_valid,output wire [3:0] command_type,
 output wire [15:0] command_position,command_time,
 output reg stop_request,done,output reg [3:0] outcome,output reg [15:0] final_position);
 localparam IDLE=0,READ=1,READ_TX=2,REPLY=3,MOVE=4,MOVE_TX=5,WAIT_MOVE=6,GAP=7,STOP=8,STOP_WAIT=9;
 reg [3:0] state;reg next_move,initial_read,stop_busy;
 reg [15:0] cap,step,ms,start_p,before_p,target,min_p,max_p;
 reg [3:0] samples;reg [31:0] timer,wait_cycles;
 wire [15:0] low_next=samples==0 || position<min_p?position:min_p;
 wire [15:0] high_next=samples==0 || position>max_p?position:max_p;
 wire [16:0] proposed={1'b0,position}+{1'b0,step};
 assign active=state!=IDLE;
 assign command_valid=(state==READ || state==MOVE) && bus_ready && !cancel && !rx_error;
 assign command_type=state==MOVE?4'd0:4'd2;
 assign command_position=target;assign command_time=ms;
 task finish(input [3:0] why);begin outcome<=why;state<=STOP;timer<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;next_move<=0;initial_read<=0;stop_busy<=0;cap<=0;step<=0;ms<=0;
   start_p<=0;before_p<=0;target<=0;min_p<=0;max_p<=0;samples<=0;
   timer<=0;wait_cycles<=0;stop_request<=0;done<=0;outcome<=0;final_position<=0;
  end else begin
   stop_request<=0;done<=0;if(timer!=32'hffffffff)timer<=timer+1'b1;
   case(state)
    IDLE:if(start)begin
     outcome<=0;timer<=0;samples<=0;initial_read<=1;
     if(limit_p<=LOWER_P || limit_p>UPPER_P || step_p==0 || step_p>150 || time_ms<500 || time_ms>5000)begin
      outcome<=5;done<=1;
     end else begin cap<=limit_p;step<=step_p;ms<=time_ms;state<=READ;end
    end
    READ:if(command_valid)begin state<=READ_TX;timer<=0;end
    READ_TX:if(bus_done)begin state<=REPLY;timer<=0;end
     else if(timer>=REPLY_CYCLES)finish(6);
    REPLY:if(position_valid && position_id==5)begin
     timer<=0;final_position<=position;
     if(position<LOWER_P || position>cap)finish(5);
     else if(initial_read)begin
      initial_read<=0;start_p<=position;before_p<=position;
      if(position+ERROR_P>=cap)finish(2);
      else begin target<=proposed>cap?cap:proposed[15:0];state<=GAP;next_move<=1;end
     end else begin
      min_p<=low_next;max_p<=high_next;samples<=samples+1'b1;
      if(samples==5)begin
       if(high_next-low_next>SPAN_P)finish(4);
       else if(position+SPAN_P<before_p)finish(3);
       else if(position+ERROR_P>=cap)finish(2);
       else if(target>position+ERROR_P && position>=start_p+MIN_TRAVEL_P)finish(1);
       else if(position<=before_p+SPAN_P)finish(3);
       else begin
        before_p<=position;target<=proposed>cap?cap:proposed[15:0];state<=GAP;next_move<=1;
       end
      end else begin state<=GAP;next_move<=0;end
     end
    end else if(timer>=REPLY_CYCLES)finish(6);
    GAP:if(timer>=(next_move?CLK_HZ/1000:SAMPLE_CYCLES))begin state<=next_move?MOVE:READ;timer<=0;end
    MOVE:if(command_valid)begin state<=MOVE_TX;timer<=0;end
    MOVE_TX:if(bus_done)begin state<=WAIT_MOVE;timer<=0;wait_cycles<=ms*(CLK_HZ/1000)+SAMPLE_CYCLES;end
     else if(timer>=REPLY_CYCLES)finish(6);
    WAIT_MOVE:if(timer>=wait_cycles)begin samples<=0;state<=READ;timer<=0;end
    STOP:begin stop_request<=1;stop_busy<=0;timer<=0;state<=STOP_WAIT;end
    STOP_WAIT:begin
     if(bus_busy)stop_busy<=1;
     if(bus_done && stop_busy)begin state<=IDLE;done<=1;end
     else if(timer>=REPLY_CYCLES)begin outcome<=6;state<=IDLE;done<=1;end
    end
    default:state<=IDLE;
   endcase
   if(active && state!=STOP && state!=STOP_WAIT && rx_error)finish(5);
   // External STOP is already routed to the same bus emergency-stop input.
   // Do not enqueue a second stop or another motion after cancellation.
   if(cancel && active)begin state<=IDLE;outcome<=7;done<=1;stop_request<=0;end
  end
 end
endmodule
