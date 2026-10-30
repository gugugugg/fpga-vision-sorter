`timescale 1ns/1ps
// Independent board bring-up: USB packet console -> FPGA protocol engine ->
// translated two-wire ZLink TTL UART. Boot/reset never issue a servo command.
module robot_arm_zlink_console #(
    parameter CLK_HZ=50000000, BAUD=115200,
    parameter REPLY_TIMEOUT_CYCLES=CLK_HZ/4,
    parameter SETTLE_CYCLES=CLK_HZ/10,
    parameter ALLOW_MULTI_MANUAL=0,
    parameter [15:0] SIGNATURE=16'h3913
)(
    input wire I_sys_clk,I_rst_n,I_usb_uart_rx,I_arm_uart_rx,
    input wire I_key1_n,I_key2_n,I_key4_n,I_sw4,
    input wire I_sort_enable,I_sort_start_n,I_template_key_n,
    output wire O_buzzer_n,output wire [15:0] O_sort_status,
    output wire O_usb_uart_tx,O_arm_uart_tx,
    output reg O_vision_update,output reg [15:0] O_vision_code,
    input wire [239:0] I_vision_status,input wire I_vision_busy,
    output wire [5:0] O_manual_mask
);
    reg [239:0] vision_snapshot;
    wire clk,rst_n;
    assign clk=I_sys_clk;
    reset_release reset(.clk(clk),.async_rst_n(I_rst_n),.rst_n(rst_n));
    wire host_byte_valid,host_frame_error;wire[7:0]host_byte;
    uart_rx_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) host_rx(
        .i_clk(clk),.i_rst_n(rst_n),.i_rx(I_usb_uart_rx),.o_valid(host_byte_valid),
        .o_data(host_byte),.o_frame_error(host_frame_error));
    wire host_packet_valid,packet_error,packet_active;wire[7:0]host_kind,host_address;wire[95:0]host_payload;
    wire key_read,key_jog,key_stop;
    zlink_key_pulse #(.CYCLES(CLK_HZ/50)) k1(.clk(clk),.rst_n(rst_n),.key_n(I_key1_n),.press(key_read));
    zlink_key_pulse #(.CYCLES(CLK_HZ/50)) k2(.clk(clk),.rst_n(rst_n),.key_n(I_key2_n),.press(key_jog));
    zlink_key_pulse #(.CYCLES(CLK_HZ/50)) k4(.clk(clk),.rst_n(rst_n),.key_n(I_key4_n),.press(key_stop));
    (* async_reg="true",syn_preserve=1 *)reg[1:0]sw_sync;
    reg previous_key_allow;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin sw_sync<=2'b11;previous_key_allow<=0;end
        else begin sw_sync<={sw_sync[0],I_sw4};previous_key_allow<=!sw_sync[1];end
    wire key_allow=!sw_sync[1]; // ON switch shorts to ground, as in the tested board router.
    wire key_request=key_stop || key_read || key_jog;
    wire packet_valid=host_packet_valid || key_request;
    wire[7:0]kind=key_stop?8'h12:key_read?8'h10:key_jog?8'h11:host_kind;
    wire[7:0]address=key_request?8'd5:host_address;
    wire[95:0]payload=key_jog && !key_stop && !key_read?{16'd20,16'd2000,64'd0}:key_request?96'd0:host_payload;
    arm_packet_decoder #(.TIMEOUT_CYCLES(CLK_HZ/200)) decoder(
        .clk(clk),.rst_n(rst_n),.byte_valid(host_byte_valid),.byte_data(host_byte),
        .valid(host_packet_valid),.kind(host_kind),.address(host_address),.payload(host_payload),
        .error(packet_error),.active(packet_active));
    wire arm_byte_valid,arm_frame_error;wire[7:0]arm_byte;
    uart_rx_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) arm_monitor(
        .i_clk(clk),.i_rst_n(rst_n),.i_rx(I_arm_uart_rx),.o_valid(arm_byte_valid),
        .o_data(arm_byte),.o_frame_error(arm_frame_error));
    reg[15:0]rx_bytes,tx_commands,error_count;
    reg[95:0]mode_capture;reg[3:0]mode_capture_count;
    always @(posedge clk or negedge rst_n)
        if(!rst_n)begin rx_bytes<=0;error_count<=0;end
        else begin
            if(arm_byte_valid)rx_bytes<=rx_bytes+1'b1;
            if(arm_frame_error || host_frame_error || packet_error)error_count<=error_count+1'b1;
        end
    reg sort_start;
    wire sort_key,template_key,sort_active,sort_valid,sort_stop;
    wire [3:0] sort_type;wire [7:0] sort_id;wire [15:0] sort_position;
    wire [13:0] sort_time;
    zlink_key_pulse #(.CYCLES(CLK_HZ/50)) k_sort(.clk(clk),.rst_n(rst_n),.key_n(I_sort_start_n),.press(sort_key));
    zlink_key_pulse #(.CYCLES(CLK_HZ/50)) k_teach(.clk(clk),.rst_n(rst_n),.key_n(I_template_key_n),.press(template_key));
    reg grip_start;reg[15:0]grip_cap,grip_step,grip_ms;
    wire grip_active,grip_valid,grip_stop,grip_done;wire[3:0]grip_type,grip_outcome;
    wire[15:0]grip_position,grip_time,grip_final;
    reg cmd_valid,stop_request;reg[3:0]cmd_type;reg[7:0]cmd_id;
    reg align_restore;
    reg[5:0] manual_release_mask;
    wire manual_release_active=|manual_release_mask;
    assign O_manual_mask=manual_release_mask;
    reg[15:0]cmd_position;reg[13:0]cmd_time;
    wire cmd_ready,cmd_busy,cmd_done,position_valid,drive_low,id_valid,mode_valid;wire[7:0]position_id,detected_id,mode_id;
    wire[3:0]servo_mode;
    reg[3:0]cached_mode[0:5];integer mode_index;
    wire[15:0]position;
    arm_sort_sequencer #(.CLK_HZ(CLK_HZ)) u_sort(
      .clk(clk),.rst_n(rst_n),.start(sort_start),.cancel(stop_request || !I_sort_enable),
      .recognition_stable(I_vision_status[156]),.recognized_class(I_vision_status[155:154]),
      .bus_ready(cmd_ready),.bus_done(cmd_done),.rx_error(arm_frame_error),
      .position_valid(position_valid),.position_id(position_id),.position(position),
      .mode_valid(mode_valid),.mode_id(mode_id),.servo_mode(servo_mode),
      .active(sort_active),.command_valid(sort_valid),.command_type(sort_type),.command_id(sort_id),
      .command_position(sort_position),.command_time(sort_time),.emergency_stop(sort_stop),
      .buzzer_n(O_buzzer_n),.status(O_sort_status));
    arm_grip_feedback #(.CLK_HZ(CLK_HZ),.REPLY_CYCLES(REPLY_TIMEOUT_CYCLES)) u_grip_feedback(
      .clk(clk),.rst_n(rst_n),.start(grip_start),.cancel(stop_request),.rx_error(arm_frame_error),
      .limit_p(grip_cap),.step_p(grip_step),.time_ms(grip_ms),
      .bus_ready(cmd_ready),.bus_done(cmd_done),.bus_busy(cmd_busy),
      .position_valid(position_valid),.position_id(position_id),.position(position),
      .active(grip_active),.command_valid(grip_valid),.command_type(grip_type),
      .command_position(grip_position),.command_time(grip_time),
      .stop_request(grip_stop),.done(grip_done),.outcome(grip_outcome),.final_position(grip_final));
    wire [3:0]bus_cmd_type=sort_active?sort_type:grip_active?grip_type:cmd_type;
    robot_arm_uart_ctrl #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.APPEND_CRLF(0),.APPEND_MOVE_CRLF(1),.RESTORE_SINGLE(1),.RELEASE_SINGLE(1)) controller(
        .i_clk(clk),.i_rst_n(rst_n),.i_emergency_stop(stop_request || grip_stop || sort_stop),
        .i_cmd_valid(sort_active?sort_valid:grip_active?grip_valid:cmd_valid),.i_cmd_type(bus_cmd_type),.i_servo_id(sort_active?sort_id:grip_active?8'd5:cmd_id),
        .i_pwm0(sort_active?sort_position:grip_active?grip_position:cmd_position),.i_pwm1(16'd1500),.i_pwm2(16'd1500),
        .i_pwm3(16'd1500),.i_pwm4(16'd1500),.i_pwm5(16'd1500),
        .i_move_time_ms(sort_active?sort_time:grip_active?grip_time:cmd_time),.i_group_number(14'd0),
        .o_cmd_ready(cmd_ready),.o_busy(cmd_busy),.o_cmd_done(cmd_done),
        .o_cmd_rejected(),.o_cmd_clamped(),.i_dat(I_arm_uart_rx),.o_dat_drive_low(drive_low),
        .o_position_valid(position_valid),.o_position_id(position_id),.o_position(position),
        .o_id_valid(id_valid),.o_detected_id(detected_id),
        .o_mode_valid(mode_valid),.o_mode_id(mode_id),.o_mode(servo_mode),.o_rx_frame_error());
    // The protocol engine exposes the inverse TX bit as drive_low. The physical
    // ZLink input is now push-pull 3.3V TX, not the old single-wire DAT pad.
    assign O_arm_uart_tx=~drive_low;
    // Observe the actual TX net before the output pad, independently of the
    // command scheduler. External RX edge counting diagnoses header continuity.
    wire local_tx_valid,local_tx_error;wire[7:0]local_tx_byte;
    uart_rx_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) local_tx_monitor(
        .i_clk(clk),.i_rst_n(rst_n),.i_rx(O_arm_uart_tx),.o_valid(local_tx_valid),
        .o_data(local_tx_byte),.o_frame_error(local_tx_error));
    reg[159:0]move_capture,restore_capture;
    reg[4:0]move_capture_count,restore_capture_count;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin move_capture<=0;restore_capture<=0;move_capture_count<=0;restore_capture_count<=0;end
        else if(local_tx_valid)begin
            if(bus_cmd_type==0)begin
                if(local_tx_byte=="#")begin move_capture<={local_tx_byte,152'd0};move_capture_count<=1;end
                else if(move_capture_count<20)begin
                    move_capture[159-move_capture_count*8-:8]<=local_tx_byte;move_capture_count<=move_capture_count+1'b1;
                end
            end else if(bus_cmd_type==3)begin
                if(local_tx_byte=="#")begin restore_capture<={local_tx_byte,152'd0};restore_capture_count<=1;end
                else if(restore_capture_count<20)begin
                    restore_capture[159-restore_capture_count*8-:8]<=local_tx_byte;restore_capture_count<=restore_capture_count+1'b1;
                end
            end
        end
    end
    (* async_reg="true",syn_preserve=1 *)reg[1:0]diag_rx_sync;
    reg diag_rx_previous;reg[15:0]local_tx_bytes,rx_edges;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin diag_rx_sync<=2'b11;diag_rx_previous<=1;local_tx_bytes<=0;rx_edges<=0;end
        else begin
            diag_rx_sync<={diag_rx_sync[0],I_arm_uart_rx};diag_rx_previous<=diag_rx_sync[1];
            if(local_tx_valid)local_tx_bytes<=local_tx_bytes+1'b1;
            if(diag_rx_sync[1]!=diag_rx_previous)rx_edges<=rx_edges+1'b1;
        end
    end
    localparam IDLE=0,WAIT_READ=1,WAIT_MOVE_TX=2,WAIT_MOVE=3,VERIFY=4,WAIT_STOP=5,WAIT_ID=6,WAIT_TURNAROUND=7,WAIT_MODE=8,WAIT_WIRE_REPLY=9,WIRE_FINAL=10;
    reg[3:0]state;
    reg[191:0]wire_capture;reg[4:0]wire_capture_count;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin wire_capture<=0;wire_capture_count<=0;end
        else if(packet_valid && (kind==8'h1b || kind==8'h1c || kind==8'h1d) && state==IDLE)begin
            wire_capture<=0;wire_capture_count<=0;
        end else if(state==WAIT_WIRE_REPLY && arm_byte_valid && wire_capture_count<24)begin
            wire_capture[191-wire_capture_count*8-:8]<=arm_byte;wire_capture_count<=wire_capture_count+1'b1;
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin mode_capture<=0;mode_capture_count<=0;end
        else if(packet_valid && kind==8'h18 && state==IDLE)begin mode_capture<=0;mode_capture_count<=0;end
        else if(state==WAIT_MODE && arm_byte_valid && mode_capture_count<12)begin
            mode_capture[95-mode_capture_count*8-:8]<=arm_byte;
            mode_capture_count<=mode_capture_count+1'b1;
        end
    end
    reg armed,key_origin,stop_seen_busy;reg[7:0]request_kind,request_id;
    reg signed[15:0]jog_delta;reg[15:0]move_ms,target;
    reg[31:0]elapsed,wait_cycles;
    wire signed[17:0]relative_target=$signed({2'd0,position})+$signed({{2{jog_delta[15]}},jog_delta});
    wire[15:0]target_error=position>target?position-target:target-position;
    reg event0,event1;reg[127:0]message0,message1;
    function[7:0]crc8;
        input[7:0]prev,b;reg[7:0]v;integer n;
        begin v=prev^b;for(n=0;n<8;n=n+1)v=v[7]?(v<<1)^8'h07:v<<1;crc8=v;end
    endfunction
    function[127:0]packet;
        input[7:0]k,a;input[95:0]p;reg[7:0]c;integer n;
        begin c=crc8(crc8(0,k),a);for(n=0;n<12;n=n+1)c=crc8(c,p[95-n*8-:8]);packet={8'hd5,k,a,p,c};end
    endfunction
    function[127:0]ack;
        input[7:0]status,k,id;input[15:0]value;input[7:0]flags;
        begin ack=packet(8'h80,status,{k,id,value,rx_bytes,tx_commands,error_count,{7'd0,armed},flags});end
    endfunction
    function[127:0]position_reply;
        input[7:0]k,id;input[15:0]value;input[7:0]flags;
        begin position_reply=packet(8'h81,id,{k,id,value,rx_bytes,tx_commands,error_count,{7'd0,armed},flags});end
    endfunction
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            vision_snapshot<=0;O_vision_update<=0;O_vision_code<=0;
            sort_start<=0;grip_start<=0;grip_cap<=1613;grip_step<=80;grip_ms<=1000;
            state<=IDLE;armed<=0;key_origin<=0;stop_seen_busy<=0;cmd_valid<=0;stop_request<=0;cmd_type<=2;cmd_id<=0;
            manual_release_mask<=0;align_restore<=0;
            cmd_position<=1500;cmd_time<=1500;request_kind<=0;request_id<=0;
            jog_delta<=0;move_ms<=1500;target<=0;elapsed<=0;wait_cycles<=0;
            tx_commands<=0;event0<=0;event1<=0;message0<=0;message1<=0;
            for(mode_index=0;mode_index<6;mode_index=mode_index+1)cached_mode[mode_index]<=0;
        end else begin
            sort_start<=0;grip_start<=0;O_vision_update<=0;cmd_valid<=0;stop_request<=0;event0<=0;event1<=0;
            if((cmd_valid || grip_valid || sort_valid) && cmd_ready)tx_commands<=tx_commands+1'b1;
            if(grip_done && request_kind==8'h34)begin event1<=1;message1<=position_reply(8'h34,8'd5,grip_final,{4'd0,grip_outcome});end
            if(state!=IDLE)elapsed<=elapsed+1'b1;
            if(sort_key && I_sort_enable && !sort_active && !manual_release_active && !armed && state==IDLE && !cmd_busy)
                sort_start<=1;
            if(template_key && I_sort_enable && !sort_active && !manual_release_active && !armed && !I_vision_busy && I_vision_status[153:152]==0)begin
                O_vision_update<=1;
                O_vision_code<=!I_vision_status[159]?1:!I_vision_status[157]?2:!I_vision_status[158]?3:1;
            end
            if(packet_error)begin event0<=1;message0<=ack(1,0,0,0,0);end
            if(packet_valid)begin
                if(kind==8'h12 || kind==8'h14)begin
                    // Disable cancels any pending movement, even during a reply wait.
                    armed<=0;
                    if(kind==8'h12 || state!=IDLE || cmd_busy)begin
                        stop_request<=1;stop_seen_busy<=0;request_kind<=kind;request_id<=0;state<=WAIT_STOP;elapsed<=0;
                    end else begin event0<=1;message0<=ack(0,kind,0,0,0);end
                end else if(kind==8'h36)begin event0<=1;message0<=ack(0,kind,address,O_sort_status,0);end
                else if(kind==8'h15 && sort_active)begin event0<=1;message0<=ack(0,kind,address,SIGNATURE,0);end
                else if(kind==8'h1a)begin
                        if(address!=0 || payload[87:0]!=0 || payload[95:88]>1)begin event0<=1;message0<=ack(5,kind,address,0,0);end
                        else if(payload[95:88]==0)begin
                            event0<=1;event1<=1;
                            message0<=packet(8'h85,move_capture_count>12?8'd12:{3'd0,move_capture_count},move_capture[159:64]);
                            message1<=packet(8'h86,move_capture_count>12?{3'd0,move_capture_count}-8'd12:8'd0,{move_capture[63:0],32'd0});
                        end else begin
                            event0<=1;event1<=1;
                            message0<=packet(8'h85,restore_capture_count>12?8'd12:{3'd0,restore_capture_count},restore_capture[159:64]);
                            message1<=packet(8'h86,restore_capture_count>12?{3'd0,restore_capture_count}-8'd12:8'd0,{restore_capture[63:0],32'd0});
                        end
                    end
                else if(kind==8'h16)begin
                        event0<=1;
                        message0<=packet(8'h80,8'd0,{kind,address,local_tx_bytes,rx_bytes,rx_edges,error_count,
                            {5'd0,local_tx_error,O_arm_uart_tx,diag_rx_sync[1]},{4'd0,state}});
                    end
                else if(sort_active || sort_start || grip_active || grip_start || state!=IDLE || cmd_busy || cmd_valid)begin event0<=1;message0<=ack(3,kind,address,0,0);end
                else case(kind)
                    8'h35:begin
                        event0<=1;
                        if(address!=0 || payload!=0 || !I_sort_enable || manual_release_active || armed)
                            message0<=ack(5,kind,address,0,0);
                        else begin sort_start<=1;message0<=ack(0,kind,address,0,0);end
                    end
                    8'h37:begin event0<=1;message0<=ack(0,kind,address,I_vision_status[239:224],0);end
                    8'h30:begin
                        event0<=1;
                        if(address<1 || address>3 || payload!=0)message0<=ack(5,kind,address,0,0);
                        else if(armed || manual_release_active || I_vision_busy || I_vision_status[153:152]!=0)
                            message0<=ack(3,kind,address,0,0);
                        else begin O_vision_update<=1;O_vision_code<=address[1:0];message0<=ack(0,kind,address,0,0);end
                    end
                    8'h31:begin
                        event0<=1;
                        if(address>9 || payload!=0)message0<=ack(5,kind,address,0,0);
                        else if(address==0)begin vision_snapshot<=I_vision_status;message0<=ack(0,kind,address,I_vision_status[159:144],0);end
                        else message0<=ack(0,kind,address,vision_snapshot[159-address*16-:16],0);
                    end
                    8'h32:begin
                        event0<=1;
                        if(address>3 || payload!=0)message0<=ack(5,kind,address,0,0);
                        else if(address==0)begin vision_snapshot<=I_vision_status;message0<=ack(0,kind,address,I_vision_status[223:208],0);end
                        else message0<=ack(0,kind,address,vision_snapshot[223-address*16-:16],0);
                    end
                    8'h33:begin
                        event0<=1;
                        if(address!=0 || payload[79:0]!=0 || payload[95:88]==0 || payload[95:88]>31 || payload[87:80]==0 || payload[87:80]>63)
                            message0<=ack(5,kind,address,0,0);
                        else if(armed || manual_release_active || I_vision_busy || I_vision_status[153:152]!=0)
                            message0<=ack(3,kind,address,0,0);
                        else begin O_vision_update<=1;O_vision_code<={3'b100,payload[85:80],payload[92:88],2'b00};message0<=ack(0,kind,address,0,0);end
                    end
                    8'h34:begin
                        event0<=1;
                        if(address!=5 || payload[47:0]!=0 || payload[95:80]<=1337 || payload[95:80]>1613 ||
                           payload[79:64]==0 || payload[79:64]>150 || payload[63:48]<500 || payload[63:48]>5000)
                            message0<=ack(5,kind,address,0,0);
                        else if(!armed || manual_release_active || cached_mode[5]!=1)message0<=ack(4,kind,address,0,0);
                        else begin
                            grip_cap<=payload[95:80];grip_step<=payload[79:64];grip_ms<=payload[63:48];grip_start<=1;
                            request_kind<=kind;request_id<=5;key_origin<=0;message0<=ack(0,kind,address,0,0);
                        end
                    end
                    8'h15:begin event0<=1;message0<=ack(0,kind,address,SIGNATURE,0);end
                    8'h1f:begin
                        event0<=1;
                        message0<=ack(address==0 && payload==0?0:5,kind,address,{10'd0,manual_release_mask},0);
                    end
                    8'h1b,8'h1c,8'h1d:begin
                        if(address>5 || payload!=0)begin event0<=1;message0<=ack(5,kind,address,0,0);end
                        else begin
                            key_origin<=0;request_kind<=kind;request_id<=address;
                            cmd_valid<=1;cmd_type<=kind==8'h1b ? 4'd8 : kind==8'h1c ? 4'd9 : 4'd10;
                            cmd_id<=address;state<=WAIT_WIRE_REPLY;elapsed<=0;
                        end
                    end
                    8'h17:begin
                        if(address>5 || payload!=0)begin event0<=1;message0<=ack(5,kind,address,0,0);end
                        else begin
                            key_origin<=0;request_kind<=kind;request_id<=address;
                            cmd_valid<=1;cmd_type<=6;cmd_id<=address;state<=WAIT_ID;elapsed<=0;
                        end
                    end
                    8'h18:begin
                        if(address>5 || payload!=0)begin event0<=1;message0<=ack(5,kind,address,0,0);end
                        else begin
                            cached_mode[address]<=0;
                            key_origin<=0;request_kind<=kind;request_id<=address;
                            cmd_valid<=1;cmd_type<=7;cmd_id<=address;state<=WAIT_MODE;elapsed<=0;
                        end
                    end
                    8'h13:begin
                        event0<=1;
                        if(ALLOW_MULTI_MANUAL && manual_release_active)message0<=ack(10,kind,address,0,0);
                        else if(payload=={32'h41524d21,64'd0})begin armed<=1;message0<=ack(0,kind,address,0,0);end
                        else message0<=ack(5,kind,address,0,0);
                    end
                    // Deliberate single-axis release; boot/queries never unload.
                    // Reserve manual mode before sending so an interrupted
                    // release cannot accidentally permit a subsequent jog.
                    8'h1e:begin
                        if(address>5 || payload!={32'h46524545,64'd0})begin event0<=1;message0<=ack(5,kind,address,0,0);end
                        else if(armed || (manual_release_active && (!ALLOW_MULTI_MANUAL || manual_release_mask[address])))begin event0<=1;message0<=ack(10,kind,address,0,0);end
                        else if(cached_mode[address]<1 || cached_mode[address]>4)begin event0<=1;message0<=ack(9,kind,address,0,0);end
                        else begin
                            manual_release_mask[address]<=1;align_restore<=0;
                            key_origin<=0;request_kind<=kind;request_id<=address;
                            cmd_valid<=1;cmd_type<=2;cmd_id<=address;state<=WAIT_READ;elapsed<=0;
                        end
                    end
                    8'h10,8'h11,8'h19:begin
                        if(address>5 || ((kind==8'h10 || kind==8'h19) && payload!=0))begin event0<=1;message0<=ack(5,kind,address,0,0);end
                        else if(manual_release_active && (kind==8'h11 || (kind==8'h19 && !manual_release_mask[address])))begin event0<=1;message0<=ack(10,kind,address,0,0);end
                        else if((kind==8'h11 || kind==8'h19) && !(armed || (key_request && key_allow) || (kind==8'h19 && manual_release_mask[address])))begin event0<=1;message0<=ack(4,kind,address,0,0);end
                        else if((kind==8'h11 || kind==8'h19) && (cached_mode[address]<1 || cached_mode[address]>4))begin event0<=1;message0<=ack(9,kind,address,0,0);end
                        else if(kind==8'h11 && (payload[63:0]!=0 || $signed(payload[95:80])==0 ||
                            $signed(payload[95:80])>150 || $signed(payload[95:80])< -150 ||
                            payload[79:64]<500 || payload[79:64]>5000))begin event0<=1;message0<=ack(5,kind,address,0,0);end
                        else begin
                            key_origin<=key_request;request_kind<=kind;request_id<=address;jog_delta<=payload[95:80];move_ms<=payload[79:64];
                            cmd_valid<=1;cmd_type<=2;cmd_id<=address;state<=WAIT_READ;elapsed<=0;
                        end
                    end
                    default:begin event0<=1;message0<=ack(2,kind,address,0,0);end
                endcase
            end
            // Host abort requests take priority over an RX reply or timeout.
            if(!(packet_valid && (kind==8'h12 || kind==8'h14)) && !(key_origin && state!=IDLE && previous_key_allow && !key_allow))case(state)
                WAIT_WIRE_REPLY:if(elapsed>=REPLY_TIMEOUT_CYCLES)begin
                    event0<=1;event1<=1;
                    message0<=packet(8'h84,wire_capture_count>12?8'd12:{3'd0,wire_capture_count},wire_capture[191:96]);
                    message1<=packet(8'h87,wire_capture_count>12?{3'd0,wire_capture_count}-8'd12:8'd0,wire_capture[95:0]);
                    state<=WIRE_FINAL;
                end
                WIRE_FINAL:begin
                    event0<=1;message0<=ack(wire_capture_count==0?8'd6:8'd0,request_kind,request_id,{11'd0,wire_capture_count},wire_capture_count==24?8'd1:8'd0);state<=IDLE;
                end
                WAIT_MODE:begin
                    if(mode_valid && mode_id==request_id)begin
                        event0<=1;message0<=packet(8'h84,{4'd0,mode_capture_count},mode_capture);
                        cached_mode[request_id]<=servo_mode;event1<=1;
                        message1<=packet(8'h83,mode_id,{request_kind,mode_id,{12'd0,servo_mode},rx_bytes,tx_commands,error_count,{7'd0,armed},8'd0});
                        state<=IDLE;
                    end else if(elapsed>=REPLY_TIMEOUT_CYCLES)begin
                        event0<=1;message0<=packet(8'h84,{4'd0,mode_capture_count},mode_capture);
                        event1<=1;message1<=ack(6,request_kind,request_id,0,0);state<=IDLE;
                    end
                end
                WAIT_ID:begin
                    if(id_valid && detected_id==request_id)begin
                        event1<=1;
                        message1<=packet(8'h82,detected_id,{request_kind,detected_id,{8'd0,detected_id},rx_bytes,tx_commands,error_count,{7'd0,armed},8'd0});
                        state<=IDLE;
                    end else if(elapsed>=REPLY_TIMEOUT_CYCLES)begin
                        event1<=1;message1<=ack(6,request_kind,request_id,0,0);state<=IDLE;
                    end
                end
                WAIT_READ:begin
                    if(position_valid && position_id==request_id)begin
                        if(position<500 || position>2500)begin event1<=1;message1<=ack(8,request_kind,request_id,position,0);state<=IDLE;end
                        else if(request_kind==8'h10)begin event1<=1;message1<=position_reply(request_kind,request_id,position,0);state<=IDLE;end
                        else if(request_kind==8'h1e)begin
                            target<=position;cmd_type<=4;cmd_id<=request_id;state<=WAIT_TURNAROUND;elapsed<=0;
                        end
                        else if(request_kind==8'h19 && manual_release_active)begin
                            // Send the freshly measured position as the new
                            // target BEFORE restoring torque, avoiding an old
                            // target after a manual move. Keep manual mode if
                            // the alignment feedback fails or is interrupted.
                            target<=position;cmd_position<=position;cmd_type<=0;cmd_time<=2000;move_ms<=2000;
                            align_restore<=1;cmd_id<=request_id;state<=WAIT_TURNAROUND;elapsed<=0;
                        end
                        else if(request_kind==8'h19)begin
                            target<=position;cmd_type<=3;cmd_id<=request_id;state<=WAIT_TURNAROUND;elapsed<=0;
                        end
                        else if(relative_target<500 || relative_target>2500)begin event1<=1;message1<=ack(5,request_kind,request_id,position,0);state<=IDLE;end
                        else begin
                            target<=relative_target[15:0];cmd_position<=relative_target[15:0];cmd_type<=0;cmd_time<=move_ms[13:0];
                            cmd_id<=request_id;state<=WAIT_TURNAROUND;elapsed<=0;
                        end
                    end else if(elapsed>=REPLY_TIMEOUT_CYCLES)begin event1<=1;message1<=ack(6,request_kind,request_id,0,0);state<=IDLE;end
                end
                // RX publishes data halfway through the stop bit. A bus peer
                // may still have its driver enabled; allow a quiet 1ms before
                // driving the next start bit, including after trailing bytes.
                WAIT_TURNAROUND:begin
                    if(arm_byte_valid)elapsed<=0;
                    else if(elapsed>=CLK_HZ/1000 && cmd_ready)begin cmd_valid<=1;state<=WAIT_MOVE_TX;elapsed<=0;end
                end
                WAIT_MOVE_TX:if(cmd_done)begin
                    if(request_kind==8'h1e)begin
                        armed<=0;event1<=1;message1<=ack(0,request_kind,request_id,target,0);state<=IDLE;
                    end else if(request_kind==8'h19 && !align_restore)begin
                        if(manual_release_mask[request_id])manual_release_mask[request_id]<=0;
                        event1<=1;message1<=ack(0,request_kind,request_id,target,0);state<=IDLE;
                    end else begin state<=WAIT_MOVE;elapsed<=0;wait_cycles<=move_ms*(CLK_HZ/1000)+SETTLE_CYCLES;end
                end
                WAIT_MOVE:if(elapsed>=wait_cycles && cmd_ready)begin cmd_valid<=1;cmd_type<=2;cmd_id<=request_id;state<=VERIFY;elapsed<=0;end
                VERIFY:begin
                    if(position_valid && position_id==request_id)begin
                        if(request_kind==8'h19 && align_restore && position>=500 && position<=2500 && target_error<=15)begin
                            align_restore<=0;cmd_type<=3;cmd_id<=request_id;state<=WAIT_TURNAROUND;elapsed<=0;
                        end else begin
                            event1<=1;
                            if(position<500 || position>2500)message1<=ack(8,request_kind,request_id,position,0);
                            else if(request_kind==8'h19 && align_restore)message1<=ack(11,request_kind,request_id,position,0);
                            else message1<=position_reply(request_kind,request_id,position,{7'd0,target_error<=15});
                            state<=IDLE;
                        end
                    end else if(elapsed>=REPLY_TIMEOUT_CYCLES)begin event1<=1;message1<=ack(6,request_kind,request_id,target,0);state<=IDLE;end
                end
                WAIT_STOP:begin
                    if(cmd_busy)stop_seen_busy<=1;
                    if(cmd_done && stop_seen_busy)begin event1<=1;message1<=ack(0,request_kind,0,0,0);state<=IDLE;key_origin<=0;end
                end
                default:;
            endcase
            if(key_origin && state!=IDLE && state!=WAIT_STOP && previous_key_allow && !key_allow)begin
                armed<=0;stop_request<=1;stop_seen_busy<=0;request_kind<=8'h12;request_id<=0;state<=WAIT_STOP;elapsed<=0;
            end
        end
    end
    // Two independent producer events can coincide (busy NACK + operation reply).
    // Tiny FF queue; explicit modulo occupancy handles pointer wrap correctly.
    reg[127:0]queue[0:3];reg[2:0]wp,rp;reg[15:0]reply_overflow;
    reg sending;reg[127:0]word;reg[3:0]byte_index;
    wire[2:0]level=wp-rp;
    wire pop=!sending && level!=0;
    wire[3:0]room=4'd4-{1'b0,level}+pop;
    wire push0=event0 && room!=0;
    wire push1=event1 && room>{3'd0,push0};
    wire tx_ready,tx_busy;
    uart_tx_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) host_tx(
        .i_clk(clk),.i_rst_n(rst_n),.i_valid(sending),.i_data(word[127:120]),
        .o_ready(tx_ready),.o_busy(tx_busy),.o_tx(O_usb_uart_tx));
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin wp<=0;rp<=0;reply_overflow<=0;sending<=0;word<=0;byte_index<=0;end
        else begin
            if(push0)queue[wp[1:0]]<=message0;
            if(push1)queue[(wp[1:0]+push0)&2'd3]<=message1;
            wp<=wp+push0+push1;
            if((event0 && !push0) || (event1 && !push1))reply_overflow<=reply_overflow+1'b1;
            if(pop)begin word<=queue[rp[1:0]];rp<=rp+1'b1;sending<=1;byte_index<=0;end
            else if(sending && tx_ready)begin
                word<=word<<8;
                if(byte_index==15)sending<=0;else byte_index<=byte_index+1'b1;
            end
        end
    end
endmodule

// A held key at reset never creates a movement request. Require stable release
// before a debounced falling edge; a held press emits exactly one pulse.
module zlink_key_pulse #(parameter CYCLES=1000000)(
    input wire clk,rst_n,key_n,output reg press
);
    (* async_reg="true",syn_preserve=1 *)reg[1:0]sync;
    reg stable,ready;reg[31:0]count,release_count;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin sync<=3;stable<=1;ready<=0;count<=0;release_count<=0;press<=0;end
        else begin
            sync<={sync[0],key_n};press<=0;
            if(sync[1]!=stable)begin
                if(count==CYCLES-1)begin
                    stable<=sync[1];count<=0;release_count<=0;
                    if(!sync[1] && ready)begin press<=1;ready<=0;end
                end else count<=count+1'b1;
            end else begin
                count<=0;
                if(stable)begin
                    if(release_count<CYCLES)release_count<=release_count+1'b1;
                    else ready<=1;
                end else release_count<=0;
            end
        end
    end
endmodule
