`timescale 1ns/1ps
// One operator-started cycle; class 1 goes left and class 2 goes right.
module arm_sort_sequencer #(parameter CLK_HZ=50000000)(
 input wire clk,rst_n,start,cancel,recognition_stable,
 input wire [1:0] recognized_class,
 input wire bus_ready,bus_done,rx_error,position_valid,mode_valid,
 input wire [7:0] position_id,mode_id,
 input wire [15:0] position,input wire [3:0] servo_mode,
 output wire active,command_valid,output reg [3:0] command_type,
 output reg [7:0] command_id,output reg [15:0] command_position,
 output wire [13:0] command_time,output reg emergency_stop,
 output wire buzzer_n,output wire [15:0] status
);
 localparam [15:0] PICK_BASE_P=16'd1552;
 localparam IDLE=0,WAIT_CLASS=1,MODE_SEND=2,MODE_TX=3,MODE_REPLY=4,
  BEEP_FIRST=5,DELAY_FIRST=6,RESTORE_SEND=7,RESTORE_TX=8,
  FETCH=9,MOVE_SEND=10,MOVE_TX=11,MOVE_WAIT=12,
  READ_SEND=13,READ_TX=14,READ_REPLY=15,READ_GAP=16,
  STOP_GRIP=17,STOP_GRIP_TX=18,BEEP_LAST=19,DELAY_LAST=20,
  FINISHED=21,FAULT=22,MODE_GAP=23,RESTORE_GAP=24,
  BASE_SEND=25,BASE_TX=26,BASE_REPLY=27,
  BASE_DRIVE_SEND=28,BASE_DRIVE_TX=29,BASE_DRIVE_GAP=30;
 reg [4:0] state,pc;reg [2:0] axis,samples;
 reg [1:0] part;reg [25:0] timer;
 reg [15:0] min_p,max_p,home_command;
 reg base_verify;
 reg [15:0] base_before;
 wire base_at_pick=position>=PICK_BASE_P-16'd10 && position<=PICK_BASE_P+16'd10;
 wire base_stationary=position+16'd3>=base_before && position<=base_before+16'd3;
 wire [15:0] low_next=samples==0 || position<min_p?position:min_p;
 wire [15:0] high_next=samples==0 || position>max_p?position:max_p;
 wire [15:0] close_target=part==1?16'd1550:16'd1613;
 assign active=state!=IDLE && state!=FINISHED && state!=FAULT;
 assign status={3'd0,state,pc,part,active};
 assign buzzer_n=!(state==BEEP_FIRST || state==BEEP_LAST);
 assign command_time=14'd1000;
 assign command_valid=bus_ready && !cancel && !rx_error &&
  (state==MODE_SEND || state==RESTORE_SEND || state==MOVE_SEND ||
   state==READ_SEND || state==STOP_GRIP || state==BASE_SEND || state==BASE_DRIVE_SEND);
 always @* begin
  command_type=0;command_id=0;command_position=PICK_BASE_P;
  case(pc)
   0:begin command_id=2;command_position=1280;end
   1:begin command_id=3;command_position=1985;end
   2:begin command_id=1;command_position=1807;end
   3:begin command_id=0;command_position=home_command;end
   4:begin command_id=5;command_position=1337;end
   5:begin command_id=1;command_position=part==1?1906:1902;end
   6:begin command_id=5;command_position=close_target;end
   7:begin command_id=1;command_position=1802;end
   8:begin command_id=0;command_position=part==1?1653:1443;end
   9:begin command_id=1;command_position=1902;end
   10:begin command_type=3;command_id=5;end
   11:begin command_id=5;command_position=1337;end
   12:begin command_id=1;command_position=1807;end
   13:begin command_id=0;command_position=1437;end
   default:begin end
  endcase
  if(state==MODE_SEND || state==MODE_TX || state==MODE_REPLY || state==MODE_GAP)begin
   command_type=7;command_id={5'd0,axis};
  end else if(state==RESTORE_SEND || state==RESTORE_TX || state==RESTORE_GAP)begin
   command_type=3;command_id={5'd0,axis};
  end else if(state==READ_SEND || state==READ_TX || state==READ_REPLY || state==READ_GAP)begin
   command_type=2;command_id=5;
  end else if(state==STOP_GRIP || state==STOP_GRIP_TX)begin
   command_type=11;command_id=5;
  end else if(state==BASE_SEND || state==BASE_TX || state==BASE_REPLY)begin
   command_type=2;command_id=0;
  end else if(state==BASE_DRIVE_SEND || state==BASE_DRIVE_TX || state==BASE_DRIVE_GAP)begin
   command_type=3;command_id=0;
  end
 end
 task fail;begin state<=FAULT;emergency_stop<=1;timer<=0;end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;pc<=0;axis<=0;samples<=0;part<=0;timer<=0;
   min_p<=0;max_p<=0;home_command<=PICK_BASE_P;emergency_stop<=0;
   base_verify<=0;base_before<=0;
  end else begin
   emergency_stop<=0;
   if(timer<CLK_HZ+CLK_HZ/10)timer<=timer+1'b1;
   case(state)
    IDLE,FINISHED,FAULT:if(start)begin state<=WAIT_CLASS;part<=0;pc<=0;timer<=0;end
    WAIT_CLASS:if(recognition_stable && (recognized_class==1 || recognized_class==2))begin
     part<=recognized_class;axis<=0;state<=MODE_SEND;timer<=0;
    end
    MODE_SEND:if(command_valid)begin state<=MODE_TX;timer<=0;end
    MODE_TX:if(bus_done)begin state<=MODE_REPLY;timer<=0;end
    MODE_REPLY:if(mode_valid && mode_id==axis)begin
     if(servo_mode!=1)fail();else begin state<=MODE_GAP;timer<=0;end
    end else if(timer>=CLK_HZ/4)fail();
    MODE_GAP:if(timer>=CLK_HZ/1000)begin
     timer<=0;if(axis==5)state<=BEEP_FIRST;else begin axis<=axis+1'b1;state<=MODE_SEND;end
    end
    BEEP_FIRST:if(timer==CLK_HZ/10-1)begin state<=DELAY_FIRST;timer<=0;end
    DELAY_FIRST:if(timer==CLK_HZ-1)begin axis<=0;state<=RESTORE_SEND;timer<=0;end
    RESTORE_SEND:if(command_valid)begin state<=RESTORE_TX;timer<=0;end
    RESTORE_TX:if(bus_done)begin state<=RESTORE_GAP;timer<=0;end
    RESTORE_GAP:if(timer>=CLK_HZ/1000)begin
     timer<=0;if(axis==5)begin state<=FETCH;pc<=0;end
     else begin axis<=axis+1'b1;state<=RESTORE_SEND;end
    end
    FETCH:if(timer>=CLK_HZ/1000)begin
     state<=pc==3?BASE_SEND:MOVE_SEND;timer<=0;
     if(pc==3)base_verify<=0;
    end
    BASE_SEND:if(command_valid)begin state<=BASE_TX;timer<=0;end
    BASE_TX:if(bus_done)begin state<=BASE_REPLY;timer<=0;end
    BASE_REPLY:if(position_valid && position_id==0)begin
     timer<=0;
     if(base_at_pick)begin pc<=4;state<=FETCH;end
     else if(base_verify && base_stationary)fail();
     else begin
      base_before<=position;
      home_command<=position>PICK_BASE_P?PICK_BASE_P-16'd13:PICK_BASE_P+16'd13;
      state<=BASE_DRIVE_SEND;
     end
    end else if(timer>=CLK_HZ/4)fail();
    BASE_DRIVE_SEND:if(command_valid)begin state<=BASE_DRIVE_TX;timer<=0;end
    BASE_DRIVE_TX:if(bus_done)begin state<=BASE_DRIVE_GAP;timer<=0;end
    BASE_DRIVE_GAP:if(timer>=CLK_HZ/1000)begin state<=MOVE_SEND;timer<=0;end
    MOVE_SEND:if(command_valid)begin state<=MOVE_TX;timer<=0;end
    MOVE_TX:if(bus_done)begin
     timer<=0;
     if(pc==10)begin pc<=11;state<=FETCH;end
     else state<=MOVE_WAIT;
    end
    MOVE_WAIT:if(timer>=CLK_HZ+CLK_HZ/10-1)begin
     timer<=0;
     if(pc==3)begin base_verify<=1;state<=BASE_SEND;end
     else if(pc==6 || pc==11)begin samples<=0;state<=READ_SEND;end
     else if(pc==13)state<=FINISHED;
     else begin pc<=pc+1'b1;state<=FETCH;end
    end
    READ_SEND:if(command_valid)begin state<=READ_TX;timer<=0;end
    READ_TX:if(bus_done)begin state<=READ_REPLY;timer<=0;end
    READ_REPLY:if(position_valid && position_id==5)begin
     min_p<=low_next;max_p<=high_next;timer<=0;
     if(samples==5)begin
      if(high_next-low_next>3)fail();
      else if(pc==6)begin
       if(position+20>=close_target)fail();else state<=STOP_GRIP;
      end else if(position>=1400)fail();
      else state<=BEEP_LAST;
     end else begin samples<=samples+1'b1;state<=READ_GAP;end
    end else if(timer>=CLK_HZ/4)fail();
    READ_GAP:if(timer>=CLK_HZ/10)begin state<=READ_SEND;timer<=0;end
    STOP_GRIP:if(command_valid)begin state<=STOP_GRIP_TX;timer<=0;end
    STOP_GRIP_TX:if(bus_done)begin state<=FETCH;pc<=7;timer<=0;end
    BEEP_LAST:if(timer==CLK_HZ/10-1)begin state<=DELAY_LAST;timer<=0;end
    DELAY_LAST:if(timer==CLK_HZ-1)begin state<=FETCH;pc<=12;timer<=0;end
    default:state<=IDLE;
   endcase
   if(active && rx_error)fail();
   if(active && cancel)begin state<=IDLE;timer<=0;emergency_stop<=1;end
  end
 end
endmodule
