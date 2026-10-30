`timescale 1ns/1ps
// Visual teach/replay, calibrated tracking and six explicit scan poses.
// Startup is disarmed. Host configuration and an explicit ARM are mandatory.
// The DAT physical binding belongs to the board wrapper, never this module.
module arm_vision_controller #(
 parameter CLK_HZ=50000000,BAUD=115200,IMG_WIDTH=1024,IMG_HEIGHT=600,
 parameter LOST_MS=500,REPLY_MS=100,ARRIVAL_MS=3000
)(
 input wire clk,rst_n,
 input wire packet_valid,packet_error,input wire [7:0] kind,address,
 input wire [95:0] payload,
 input wire [5:0] calibrated_views,
 input wire target_update,target_valid,input wire [31:0] frame_id,
 input wire [10:0] target_x,input wire [9:0] target_y,
 input wire dat,output wire dat_drive_low,
 input wire [5:0] views_loaded,input wire hull_valid,qc_overrun,
 output reg view_valid,output reg [2:0] view_id,
 output wire [7:0] status,output reg [8:0] points,
 output reg [31:0] last_target_id
);
 localparam IDLE=0,MAP=1,SEND=2,TX=3,MOVE=4,READ_SEND=5,
  READ_TX=6,REPLY=7,CAPTURE=8,HULL=9,CLEAR_VIEWS=10;
 reg [3:0] state;reg armed,fault,recording,tracking,playing,scanning;
 reg [20:0] path[0:255];reg [7:0] read_index;
 reg [71:0] scan_pose[0:5];reg [5:0] scan_loaded;reg [2:0] scan_index,read_axis;
 reg [71:0] command_pose;reg [13:0] move_ms;reg [15:0] tolerance;
 reg [10:0] x0,x1,map_x,last_x;reg [9:0] y0,y1,map_y,last_y;
 reg last_seen;reg map_start,map_clear,map_write;reg stop;
 wire map_ready,map_done,map_rejected,calibrated;wire [71:0] mapped_pose;
 reg [5:0] map_write_addr;reg [71:0] map_write_pose;
 // Wire order: payload is six big-endian 16-bit words; axis0 is first.
 wire [71:0] packed_pose={payload[11:0],payload[27:16],payload[43:32],payload[59:48],payload[75:64],payload[91:80]};
 reg valid_pose;integer j;
 always @* begin
  valid_pose=1;
  for(j=0;j<6;j=j+1) if(((payload>>(j*16)) & 96'hffff)<500 ||
   ((payload>>(j*16)) & 96'hffff)>2500) valid_pose=0;
 end
 visual_joint_mapper #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_mapper(
  .clk(clk),.rst_n(rst_n),.clear(map_clear),.write(map_write),
  .write_addr(map_write_addr),.write_pose(map_write_pose),.start(map_start),.x(map_x),.y(map_y),
  .x0(x0),.x1(x1),.y0(y0),.y1(y1),.ready(map_ready),.done(map_done),
  .rejected(map_rejected),.pose(mapped_pose),.calibrated(calibrated));
 wire bus_ready,bus_done,bus_rejected,bus_clamped,pos_valid,rx_error;
 wire [7:0] pos_id;wire [15:0] position;
 wire send_now=armed && bus_ready && (state==SEND || state==READ_SEND);
 robot_arm_uart_ctrl #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_bus(
  .i_clk(clk),.i_rst_n(rst_n),.i_emergency_stop(stop),
  .i_cmd_valid(send_now),.i_cmd_type(state==READ_SEND ? 3'd2:3'd1),
  .i_servo_id({5'd0,read_axis}),
  .i_pwm0({4'd0,command_pose[11:0]}),.i_pwm1({4'd0,command_pose[23:12]}),
  .i_pwm2({4'd0,command_pose[35:24]}),.i_pwm3({4'd0,command_pose[47:36]}),
  .i_pwm4({4'd0,command_pose[59:48]}),.i_pwm5({4'd0,command_pose[71:60]}),
  .i_move_time_ms(move_ms),.i_group_number(14'd0),.o_cmd_ready(bus_ready),.o_busy(),
  .o_cmd_done(bus_done),.o_cmd_rejected(bus_rejected),.o_cmd_clamped(bus_clamped),
  .i_dat(dat),.o_dat_drive_low(dat_drive_low),.o_position_valid(pos_valid),
  .o_position_id(pos_id),.o_position(position),.o_id_valid(),.o_detected_id(),.o_rx_frame_error(rx_error));
 reg [31:0] ms_counter;reg [15:0] wait_ms,lost_ms,arrival_ms;
 wire ms_tick=ms_counter==CLK_HZ/1000-1;
 wire [11:0] requested_position=command_pose>>(read_axis*12);
 wire [15:0] difference=position>=requested_position ? position-requested_position:requested_position-position;
 assign status={fault,armed,recording,tracking,playing,scanning,dat_drive_low,calibrated};
 // Configuration RAMs are not reset; validity masks are authoritative.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;armed<=0;fault<=0;recording<=0;tracking<=0;playing<=0;scanning<=0;
   points<=0;read_index<=0;scan_loaded<=0;scan_index<=0;read_axis<=0;command_pose<=0;
   move_ms<=1000;tolerance<=15;x0<=0;x1<=IMG_WIDTH-1;y0<=0;y1<=IMG_HEIGHT-1;
   map_x<=0;map_y<=0;last_x<=0;last_y<=0;last_seen<=0;last_target_id<=0;
   map_start<=0;map_clear<=0;map_write<=0;map_write_addr<=0;map_write_pose<=0;stop<=0;view_valid<=0;view_id<=0;
   ms_counter<=0;wait_ms<=0;lost_ms<=0;arrival_ms<=0;
  end else begin
   map_start<=0;map_clear<=0;map_write<=0;stop<=0;view_valid<=0;
   if(ms_tick) begin
    ms_counter<=0;
    if(wait_ms!=65535) wait_ms<=wait_ms+1'b1;
    if(lost_ms!=65535) lost_ms<=lost_ms+1'b1;
    if(scanning && arrival_ms!=65535) arrival_ms<=arrival_ms+1'b1;
   end else ms_counter<=ms_counter+1'b1;
   // Record unique camera frames, independently of slow actuator commands.
   if(target_update && (!last_seen || frame_id!=last_target_id)) begin
    last_seen<=1;last_target_id<=frame_id;
    if(target_valid && target_x>=x0 && target_x<=x1 && target_y>=y0 && target_y<=y1) begin
     lost_ms<=0;
     if(recording && points<256) begin
      path[points[7:0]]<={target_x,target_y};points<=points+1'b1;
     end
     if(tracking && armed && state==IDLE && map_ready) begin
      map_x<=target_x;map_y<=target_y;map_start<=1;state<=MAP;
     end
     last_x<=target_x;last_y<=target_y;
    end
   end
   case(state)
    IDLE:if(playing && armed && map_ready) begin
     {map_x,map_y}<=path[read_index];map_start<=1;state<=MAP;
    end
    MAP:if(map_done) begin command_pose<=mapped_pose;state<=SEND;end
    SEND:if(send_now) state<=TX;
    TX:if(bus_done) begin wait_ms<=0;state<=MOVE;end
    MOVE:if(wait_ms>=move_ms) begin
     wait_ms<=0;
     if(scanning) begin read_axis<=0;state<=READ_SEND;end
     else begin
      state<=IDLE;
      if(playing) begin
       if({1'b0,read_index}+1>=points) playing<=0;else read_index<=read_index+1'b1;
      end
     end
    end
    READ_SEND:if(send_now) begin wait_ms<=0;state<=READ_TX;end
    READ_TX:if(bus_done) begin wait_ms<=0;state<=REPLY;end
    REPLY:if(pos_valid && pos_id=={5'd0,read_axis}) begin
     wait_ms<=0;
     if(difference>tolerance) state<=READ_SEND;
     else if(read_axis==5) begin view_valid<=1;view_id<=scan_index;state<=CAPTURE;end
     else begin read_axis<=read_axis+1'b1;state<=READ_SEND;end
    end
    CAPTURE:if(views_loaded[scan_index]) begin
     if(scan_index==5) state<=HULL;
     else begin scan_index<=scan_index+1'b1;command_pose<=scan_pose[scan_index+1'b1];
      arrival_ms<=0;state<=SEND;end
    end
    HULL:if(hull_valid) begin scanning<=0;state<=IDLE;end
    CLEAR_VIEWS:if(views_loaded==0 && !hull_valid) begin
     scan_index<=0;command_pose<=scan_pose[0];arrival_ms<=0;state<=SEND;
    end
    default:state<=IDLE;
   endcase
   // Uploads are accepted only while disarmed and idle, so a pose cannot be
   // changed underneath interpolation/replay. Validity survives disarming.
   if(packet_valid) begin
    if(kind==1 && !armed && state==IDLE && map_ready && address<64 && valid_pose) begin
     map_write<=1;map_write_addr<=address[5:0];map_write_pose<=packed_pose;
    end
    else if(kind==2 && !armed && state==IDLE && payload[95:80]<payload[79:64] &&
     payload[63:48]<payload[47:32] && payload[79:64]<IMG_WIDTH && payload[47:32]<IMG_HEIGHT &&
     payload[31:16]>=20 && payload[31:16]<=9999 && payload[15:0]<=200) begin
     x0<=payload[90:80];x1<=payload[74:64];y0<=payload[57:48];y1<=payload[41:32];
     move_ms<=payload[29:16];tolerance<=payload[15:0];
    end
    else if(kind==4 && !armed && state==IDLE && address<6 && valid_pose) begin
     scan_pose[address]<=packed_pose;scan_loaded[address]<=1;
    end
    else if(kind==3) case(address)
     0,6:begin armed<=0;tracking<=0;playing<=0;scanning<=0;recording<=0;state<=IDLE;stop<=1;end
     1:if(state==IDLE && !recording && (calibrated || ((&scan_loaded) && (&calibrated_views)))) begin armed<=1;fault<=0;lost_ms<=0;end else fault<=1;
     2:if(state==IDLE && !playing && !scanning) begin recording<=1;tracking<=0;points<=0;last_seen<=0;end else fault<=1;
     3:recording<=0;
     4:if(armed && calibrated && points!=0 && state==IDLE && !scanning) begin
      recording<=0;tracking<=0;playing<=1;read_index<=0;end else fault<=1;
     5:if(armed && calibrated && state==IDLE && !scanning) begin
      recording<=0;playing<=0;tracking<=1;lost_ms<=0;end else fault<=1;
     7:if(!armed && state==IDLE) begin map_clear<=1;scan_loaded<=0;fault<=0;end else fault<=1;
     8:if(armed && (&scan_loaded) && (&calibrated_views) && state==IDLE && !playing) begin
      scanning<=1;recording<=0;tracking<=0;view_valid<=1;view_id<=6;
      state<=CLEAR_VIEWS;wait_ms<=0;arrival_ms<=0;end else fault<=1;
     default:fault<=1;
    endcase
    else fault<=1;
   end
   // A failed mapping, corrupt packet, absent feedback or lost live target
   // disarms and schedules the existing complete-frame software stop.
   if(packet_error || map_rejected || bus_rejected || bus_clamped || rx_error ||
    (tracking && lost_ms>=LOST_MS) ||
    (scanning && ((state==REPLY && wait_ms>=REPLY_MS) || arrival_ms>=ARRIVAL_MS || qc_overrun))) begin
    fault<=1;armed<=0;tracking<=0;playing<=0;scanning<=0;state<=IDLE;stop<=1;
   end
  end
 end
endmodule
