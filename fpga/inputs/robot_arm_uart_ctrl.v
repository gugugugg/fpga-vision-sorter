`timescale 1ns/1ps

// Zhongling single-wire bus command engine.
//
// Command types on i_cmd_type:
//   0 single move      #iiiPppppTtttt!\r\n
//   1 six-axis group   {Ggggg#000P....! ... #005P....!}\r\n
//   2 read position    #iiiPRAD!\r\n
//   3 restore all      #255PULR!\r\n
//   4 release all      #255PULK!\r\n
//   5 stop all         #255PDST!\r\n
//   6 query ID         #iiiPID!\r\n
//   7 read mode        #iiiPMOD! (never changes mode)
//
// o_dat_drive_low is deliberately open-drain style: the FPGA may only pull
// DAT low.  Board integration must use:
//   assign IO_ARM_DAT = arm_dat_drive_low ? 1'b0 : 1'bz;
// and an external 3.3 V/5 V bidirectional level shifter with pull-ups.
module robot_arm_uart_ctrl #(
    parameter integer CLK_HZ  = 50000000,
    parameter integer BAUD    = 115200,
    parameter integer PWM_MIN = 500,
    parameter integer PWM_MAX = 2500,
    // Query replies may arrive immediately after '!'. Vendor005 motion
    // examples use println; they can append CR/LF without delaying queries.
    parameter integer APPEND_CRLF = 1,
    parameter integer APPEND_MOVE_CRLF = 0,
    parameter integer RESTORE_SINGLE = 0,
    parameter integer RELEASE_SINGLE = 0
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    // Rising edge requests a software stop.  If a frame is already on the
    // wire, the request is latched and sent immediately after that complete
    // frame so a truncated ASCII command can never reach the servos.
    input  wire        i_emergency_stop,

    input  wire        i_cmd_valid,
    input  wire [3:0]  i_cmd_type,
    input  wire [7:0]  i_servo_id,
    input  wire [15:0] i_pwm0,
    input  wire [15:0] i_pwm1,
    input  wire [15:0] i_pwm2,
    input  wire [15:0] i_pwm3,
    input  wire [15:0] i_pwm4,
    input  wire [15:0] i_pwm5,
    input  wire [13:0] i_move_time_ms,
    input  wire [13:0] i_group_number,

    output wire        o_cmd_ready,
    output wire        o_busy,
    output reg         o_cmd_done,
    output reg         o_cmd_rejected,
    output reg         o_cmd_clamped,

    input  wire        i_dat,
    output wire        o_dat_drive_low,

    output reg         o_position_valid,
    output reg [7:0]   o_position_id,
    output reg [15:0]  o_position,
    output reg         o_id_valid,
    output reg [7:0]   o_detected_id,
    output reg         o_mode_valid,
    output reg [7:0]   o_mode_id,
    output reg [3:0]   o_mode,
    output reg         o_rx_frame_error
);
    localparam [2:0] CMD_SINGLE_MOVE = 3'd0;
    localparam [2:0] CMD_GROUP_MOVE  = 3'd1;
    localparam [2:0] CMD_READ_POS    = 3'd2;
    localparam [2:0] CMD_RESTORE_ALL = 3'd3;
    localparam [2:0] CMD_RELEASE_ALL = 3'd4;
    localparam [2:0] CMD_STOP_ALL    = 3'd5;
    localparam [2:0] CMD_QUERY_ID    = 3'd6;
    localparam [2:0] CMD_READ_MODE   = 3'd7;
    localparam [3:0] CMD_READ_ENV_LEGACY=8, CMD_READ_ENV=9, CMD_READ_VERSION=10, CMD_STOP_SINGLE=11;

    localparam [1:0] TX_IDLE = 2'd0;
    localparam [1:0] TX_SEND = 2'd1;
    localparam [1:0] TX_WAIT = 2'd2;

    reg [1:0] tx_state;
    reg [3:0] cmd_type_reg;
    reg [7:0] servo_id_reg;
    reg [15:0] pwm0_reg, pwm1_reg, pwm2_reg;
    reg [15:0] pwm3_reg, pwm4_reg, pwm5_reg;
    reg [13:0] time_reg, group_reg;
    reg [6:0] byte_index;
    reg [6:0] command_length;
    reg emergency_stop_d;
    reg stop_pending;

    wire emergency_stop_rise = i_emergency_stop & ~emergency_stop_d;

    wire tx_ready, tx_busy, tx_line;
    wire tx_valid = (tx_state == TX_SEND);
    wire [7:0] tx_data;

    wire rx_valid;
    wire [7:0] rx_data;
    wire rx_frame_error;

    assign o_cmd_ready = (tx_state == TX_IDLE);
    assign o_busy = (tx_state != TX_IDLE);
    assign o_dat_drive_low = ~tx_line;

    function [15:0] clamp_pwm;
        input [15:0] value;
        begin
            if(value < PWM_MIN)
                clamp_pwm = PWM_MIN;
            else if(value > PWM_MAX)
                clamp_pwm = PWM_MAX;
            else
                clamp_pwm = value;
        end
    endfunction

    function [13:0] clamp_9999;
        input [13:0] value;
        begin
            if(value > 14'd9999)
                clamp_9999 = 14'd9999;
            else
                clamp_9999 = value;
        end
    endfunction

    function [7:0] ascii3;
        input [7:0] value;
        input [1:0] digit;
        begin
            case(digit)
                2'd0: ascii3 = 8'h30 + ((value/100)%10);
                2'd1: ascii3 = 8'h30 + ((value/10)%10);
                default: ascii3 = 8'h30 + (value%10);
            endcase
        end
    endfunction

    function [7:0] ascii4;
        input [15:0] value;
        input [1:0] digit;
        begin
            case(digit)
                2'd0: ascii4 = 8'h30 + ((value/1000)%10);
                2'd1: ascii4 = 8'h30 + ((value/100)%10);
                2'd2: ascii4 = 8'h30 + ((value/10)%10);
                default: ascii4 = 8'h30 + (value%10);
            endcase
        end
    endfunction

    function [7:0] group_fragment_byte;
        input [2:0] slot;
        input [3:0] offset;
        reg [15:0] pwm_value;
        begin
            case(slot)
                3'd0: pwm_value = pwm0_reg;
                3'd1: pwm_value = pwm1_reg;
                3'd2: pwm_value = pwm2_reg;
                3'd3: pwm_value = pwm3_reg;
                3'd4: pwm_value = pwm4_reg;
                default: pwm_value = pwm5_reg;
            endcase
            case(offset)
                4'd0: group_fragment_byte = 8'h23; // #
                4'd1: group_fragment_byte = 8'h30;
                4'd2: group_fragment_byte = 8'h30;
                4'd3: group_fragment_byte = 8'h30 + slot;
                4'd4: group_fragment_byte = 8'h50; // P
                4'd5: group_fragment_byte = ascii4(pwm_value,2'd0);
                4'd6: group_fragment_byte = ascii4(pwm_value,2'd1);
                4'd7: group_fragment_byte = ascii4(pwm_value,2'd2);
                4'd8: group_fragment_byte = ascii4(pwm_value,2'd3);
                4'd9: group_fragment_byte = 8'h54; // T
                4'd10: group_fragment_byte = ascii4({2'b00,time_reg},2'd0);
                4'd11: group_fragment_byte = ascii4({2'b00,time_reg},2'd1);
                4'd12: group_fragment_byte = ascii4({2'b00,time_reg},2'd2);
                4'd13: group_fragment_byte = ascii4({2'b00,time_reg},2'd3);
                default: group_fragment_byte = 8'h21; // !
            endcase
        end
    endfunction

    function [7:0] command_byte;
        input [6:0] index;
        integer rel;
        integer slot_i;
        integer offset_i;
        begin
            command_byte = 8'h00;
            case(cmd_type_reg)
                CMD_SINGLE_MOVE: begin
                    case(index)
                        7'd0: command_byte = 8'h23;
                        7'd1: command_byte = ascii3(servo_id_reg,2'd0);
                        7'd2: command_byte = ascii3(servo_id_reg,2'd1);
                        7'd3: command_byte = ascii3(servo_id_reg,2'd2);
                        7'd4: command_byte = 8'h50;
                        7'd5: command_byte = ascii4(pwm0_reg,2'd0);
                        7'd6: command_byte = ascii4(pwm0_reg,2'd1);
                        7'd7: command_byte = ascii4(pwm0_reg,2'd2);
                        7'd8: command_byte = ascii4(pwm0_reg,2'd3);
                        7'd9: command_byte = 8'h54;
                        7'd10: command_byte = ascii4({2'b00,time_reg},2'd0);
                        7'd11: command_byte = ascii4({2'b00,time_reg},2'd1);
                        7'd12: command_byte = ascii4({2'b00,time_reg},2'd2);
                        7'd13: command_byte = ascii4({2'b00,time_reg},2'd3);
                        7'd14: command_byte = 8'h21;
                        7'd15: command_byte = 8'h0d;
                        default: command_byte = 8'h0a;
                    endcase
                end
                CMD_GROUP_MOVE: begin
                    if(index == 0)
                        command_byte = 8'h7b; // {
                    else if(index == 1)
                        command_byte = 8'h47; // G
                    else if(index < 6)
                        command_byte = ascii4({2'b00,group_reg},index-2);
                    else if(index < 96) begin
                        rel = index-6;
                        slot_i = rel/15;
                        offset_i = rel%15;
                        command_byte = group_fragment_byte(slot_i,offset_i);
                    end else if(index == 96)
                        command_byte = 8'h7d; // }
                    else if(index == 97)
                        command_byte = 8'h0d;
                    else
                        command_byte = 8'h0a;
                end
                CMD_READ_POS, CMD_READ_MODE, CMD_READ_ENV_LEGACY, CMD_READ_ENV, CMD_READ_VERSION: begin
                    case(index)
                        7'd0: command_byte = 8'h23;
                        7'd1: command_byte = ascii3(servo_id_reg,2'd0);
                        7'd2: command_byte = ascii3(servo_id_reg,2'd1);
                        7'd3: command_byte = ascii3(servo_id_reg,2'd2);
                        7'd4: command_byte = 8'h50;
                        7'd5: command_byte = cmd_type_reg==CMD_READ_MODE?"M":cmd_type_reg==CMD_READ_VERSION?"V":"R";
                        7'd6: command_byte = cmd_type_reg==CMD_READ_MODE?"O":cmd_type_reg==CMD_READ_VERSION?"E":
                                            (cmd_type_reg==CMD_READ_ENV_LEGACY || cmd_type_reg==CMD_READ_ENV)?"T":"A";
                        7'd7: command_byte = cmd_type_reg==CMD_READ_VERSION?"R":cmd_type_reg==CMD_READ_ENV_LEGACY?"E":
                                            cmd_type_reg==CMD_READ_ENV?"V":"D";
                        7'd8: command_byte = 8'h21;
                        7'd9: command_byte = 8'h0d;
                        default: command_byte = 8'h0a;
                    endcase
                end
                CMD_QUERY_ID: begin
                    case(index)
                        7'd0: command_byte = 8'h23;
                        7'd1: command_byte = ascii3(servo_id_reg,2'd0);
                        7'd2: command_byte = ascii3(servo_id_reg,2'd1);
                        7'd3: command_byte = ascii3(servo_id_reg,2'd2);
                        7'd4: command_byte = 8'h50;
                        7'd5: command_byte = 8'h49; // I
                        7'd6: command_byte = 8'h44; // D
                        7'd7: command_byte = 8'h21;
                        7'd8: command_byte = 8'h0d;
                        default: command_byte = 8'h0a;
                    endcase
                end
                default: begin
                    case(index)
                        7'd0: command_byte = 8'h23;
                        7'd1: command_byte = ((cmd_type_reg==CMD_RESTORE_ALL && RESTORE_SINGLE)||(cmd_type_reg==CMD_RELEASE_ALL && RELEASE_SINGLE) || cmd_type_reg==CMD_STOP_SINGLE)?ascii3(servo_id_reg,0):8'h32;
                        7'd2: command_byte = ((cmd_type_reg==CMD_RESTORE_ALL && RESTORE_SINGLE)||(cmd_type_reg==CMD_RELEASE_ALL && RELEASE_SINGLE) || cmd_type_reg==CMD_STOP_SINGLE)?ascii3(servo_id_reg,1):8'h35;
                        7'd3: command_byte = ((cmd_type_reg==CMD_RESTORE_ALL && RESTORE_SINGLE)||(cmd_type_reg==CMD_RELEASE_ALL && RELEASE_SINGLE) || cmd_type_reg==CMD_STOP_SINGLE)?ascii3(servo_id_reg,2):8'h35;
                        7'd4: command_byte = 8'h50;
                        7'd5: begin
                            if(cmd_type_reg == CMD_STOP_ALL || cmd_type_reg == CMD_STOP_SINGLE)
                                command_byte = 8'h44; // D
                            else
                                command_byte = 8'h55; // U
                        end
                        7'd6: begin
                            if(cmd_type_reg == CMD_STOP_ALL || cmd_type_reg == CMD_STOP_SINGLE)
                                command_byte = 8'h53; // S
                            else
                                command_byte = 8'h4c; // L
                        end
                        7'd7: begin
                            if(cmd_type_reg == CMD_RESTORE_ALL)
                                command_byte = 8'h52; // R
                            else if(cmd_type_reg == CMD_RELEASE_ALL)
                                command_byte = 8'h4b; // K
                            else
                                command_byte = 8'h54; // T
                        end
                        7'd8: command_byte = 8'h21;
                        7'd9: command_byte = 8'h0d;
                        default: command_byte = 8'h0a;
                    endcase
                end
            endcase
        end
    endfunction

    assign tx_data = command_byte(byte_index);

    uart_tx_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_tx (
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_valid(tx_valid),.i_data(tx_data),
        .o_ready(tx_ready),.o_busy(tx_busy),.o_tx(tx_line)
    );

    uart_rx_8n1 #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_rx (
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_rx(i_dat),
        .o_valid(rx_valid),.o_data(rx_data),.o_frame_error(rx_frame_error)
    );

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            tx_state <= TX_IDLE;
            cmd_type_reg <= CMD_SINGLE_MOVE;
            servo_id_reg <= 8'd0;
            pwm0_reg <= 16'd1500;
            pwm1_reg <= 16'd1500;
            pwm2_reg <= 16'd1500;
            pwm3_reg <= 16'd1500;
            pwm4_reg <= 16'd1500;
            pwm5_reg <= 16'd1500;
            time_reg <= 14'd1000;
            group_reg <= 14'd0;
            byte_index <= 7'd0;
            command_length <= 7'd0;
            emergency_stop_d <= 1'b0;
            stop_pending <= 1'b0;
            o_cmd_done <= 1'b0;
            o_cmd_rejected <= 1'b0;
            o_cmd_clamped <= 1'b0;
        end else begin
            o_cmd_done <= 1'b0;
            o_cmd_rejected <= 1'b0;
            emergency_stop_d <= i_emergency_stop;

            if(emergency_stop_rise && tx_state != TX_IDLE)
                stop_pending <= 1'b1;

            if(i_cmd_valid && tx_state != TX_IDLE)
                o_cmd_rejected <= 1'b1;

            case(tx_state)
                TX_IDLE: begin
                    if(emergency_stop_rise || stop_pending) begin
                        cmd_type_reg <= CMD_STOP_ALL;
                        byte_index <= 7'd0;
                        command_length <= APPEND_CRLF?7'd11:7'd9;
                        stop_pending <= 1'b0;
                        o_cmd_clamped <= 1'b0;
                        tx_state <= TX_SEND;
                    end else if(i_cmd_valid) begin
                        if(i_cmd_type>CMD_STOP_SINGLE)begin o_cmd_rejected<=1;end
                        else begin
                            cmd_type_reg <= i_cmd_type;
                            servo_id_reg <= (i_servo_id > 8'd254) ? 8'd254 : i_servo_id;
                            pwm0_reg <= clamp_pwm(i_pwm0);
                            pwm1_reg <= clamp_pwm(i_pwm1);
                            pwm2_reg <= clamp_pwm(i_pwm2);
                            pwm3_reg <= clamp_pwm(i_pwm3);
                            pwm4_reg <= clamp_pwm(i_pwm4);
                            pwm5_reg <= clamp_pwm(i_pwm5);
                            time_reg <= clamp_9999(i_move_time_ms);
                            group_reg <= clamp_9999(i_group_number);
                            byte_index <= 7'd0;
                            case(i_cmd_type)
                                CMD_SINGLE_MOVE: command_length <= (APPEND_CRLF || APPEND_MOVE_CRLF)?7'd17:7'd15;
                                CMD_GROUP_MOVE:  command_length <= (APPEND_CRLF || APPEND_MOVE_CRLF)?7'd99:7'd97;
                                CMD_READ_POS, CMD_READ_MODE, CMD_READ_ENV_LEGACY, CMD_READ_ENV, CMD_READ_VERSION:
                                                 command_length <= APPEND_CRLF?7'd11:7'd9;
                                CMD_QUERY_ID:    command_length <= APPEND_CRLF?7'd10:7'd8;
                                default:         command_length <= APPEND_CRLF?7'd11:7'd9;
                            endcase
                            o_cmd_clamped <=
                                ((i_cmd_type == CMD_SINGLE_MOVE || i_cmd_type == CMD_GROUP_MOVE) &&
                                 ((i_pwm0 < PWM_MIN) || (i_pwm0 > PWM_MAX) ||
                                  (i_move_time_ms > 14'd9999))) ||
                                ((i_cmd_type == CMD_GROUP_MOVE) &&
                                 ((i_pwm1 < PWM_MIN) || (i_pwm1 > PWM_MAX) ||
                                  (i_pwm2 < PWM_MIN) || (i_pwm2 > PWM_MAX) ||
                                  (i_pwm3 < PWM_MIN) || (i_pwm3 > PWM_MAX) ||
                                  (i_pwm4 < PWM_MIN) || (i_pwm4 > PWM_MAX) ||
                                  (i_pwm5 < PWM_MIN) || (i_pwm5 > PWM_MAX) ||
                                  (i_group_number > 14'd9999))) ||
                                ((i_cmd_type == CMD_SINGLE_MOVE || i_cmd_type == CMD_READ_POS ||
                                  i_cmd_type == CMD_QUERY_ID || i_cmd_type == CMD_READ_MODE) && (i_servo_id > 8'd254));
                            tx_state <= TX_SEND;
                        end
                    end
                end
                TX_SEND: begin
                    if(tx_ready) begin
                        if(byte_index == command_length-1'b1)
                            tx_state <= TX_WAIT;
                        else
                            byte_index <= byte_index + 1'b1;
                    end
                end
                default: begin // TX_WAIT
                    if(!tx_busy) begin
                        if(stop_pending || emergency_stop_rise) begin
                            cmd_type_reg <= CMD_STOP_ALL;
                            byte_index <= 7'd0;
                            command_length <= APPEND_CRLF?7'd11:7'd9;
                            stop_pending <= 1'b0;
                            o_cmd_clamped <= 1'b0;
                            tx_state <= TX_SEND;
                        end else begin
                            tx_state <= TX_IDLE;
                            o_cmd_done <= 1'b1;
                        end
                    end
                end
            endcase
        end
    end

    // Parse position replies (#003P1725!) and ID replies (#003P!).  Traffic
    // echoed while transmitting is harmless: a move contains 'T' after the
    // four position digits and is therefore discarded rather than published.
    localparam [2:0] RP_WAIT = 3'd0;
    localparam [2:0] RP_ID0  = 3'd1;
    localparam [2:0] RP_ID1  = 3'd2;
    localparam [2:0] RP_ID2  = 3'd3;
    localparam [2:0] RP_P    = 3'd4;
    localparam [2:0] RP_VAL  = 3'd5;
    localparam [2:0] RP_SKIP = 3'd6;
    localparam [3:0] RP_MODE_O=7, RP_MODE_D=8, RP_MODE_NUM=9, RP_MODE_END=10;
    reg [3:0] reply_state;
    reg [3:0] reply_mode;
    reg [7:0] reply_id;
    reg [15:0] reply_value;
    reg [2:0] reply_digits;

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            reply_state <= RP_WAIT;
            reply_id <= 8'd0;
            reply_value <= 16'd0;
            reply_digits <= 3'd0;
            o_position_valid <= 1'b0;
            o_position_id <= 8'd0;
            o_position <= 16'd0;
            o_id_valid <= 1'b0;
            o_detected_id <= 8'd0;
            o_mode_valid <= 0; o_mode_id <= 0; o_mode <= 0; reply_mode <= 0;
            o_rx_frame_error <= 1'b0;
        end else begin
            o_position_valid <= 1'b0;
            o_id_valid <= 1'b0;
            o_mode_valid <= 1'b0;
            o_rx_frame_error <= rx_frame_error;
            if(rx_valid) begin
                case(reply_state)
                    RP_WAIT: begin
                        if(rx_data == 8'h23) begin
                            reply_id <= 8'd0;
                            reply_state <= RP_ID0;
                        end
                    end
                    RP_ID0, RP_ID1, RP_ID2: begin
                        if(rx_data >= 8'h30 && rx_data <= 8'h39) begin
                            reply_id <= reply_id*10 + (rx_data-8'h30);
                            if(reply_state == RP_ID2)
                                reply_state <= RP_P;
                            else
                                reply_state <= reply_state + 1'b1;
                        end else begin
                            reply_state <= RP_WAIT;
                        end
                    end
                    RP_P: begin
                        if(rx_data == 8'h50) begin
                            reply_value <= 16'd0;
                            reply_digits <= 3'd0;
                            reply_state <= RP_VAL;
                        end else if(rx_data=="M")begin
                            // Actual ZP firmware replies #iiiMODn!, while the
                            // vendor manual shows #iiiPMODn!. Accept both.
                            reply_state<=RP_MODE_O;
                        end else begin
                            reply_state <= RP_SKIP;
                        end
                    end
                    RP_VAL: begin
                        if(rx_data=="M" && reply_digits==0)begin
                            reply_state<=RP_MODE_O;
                        end else if(rx_data >= 8'h30 && rx_data <= 8'h39 && reply_digits < 4) begin
                            reply_value <= reply_value*10 + (rx_data-8'h30);
                            reply_digits <= reply_digits + 1'b1;
                        end else if(rx_data == 8'h21) begin
                            if(reply_digits == 4) begin
                                o_position_id <= reply_id;
                                o_position <= reply_value;
                                o_position_valid <= 1'b1;
                            end else if(reply_digits == 0) begin
                                o_detected_id <= reply_id;
                                o_id_valid <= 1'b1;
                            end
                            reply_state <= RP_WAIT;
                        end else begin
                            reply_state <= RP_SKIP;
                        end
                    end
                    RP_MODE_O: reply_state<=rx_data=="O"?RP_MODE_D:RP_SKIP;
                    RP_MODE_D: reply_state<=rx_data=="D"?RP_MODE_NUM:RP_SKIP;
                    RP_MODE_NUM: begin
                        if(rx_data>="1" && rx_data<="8")begin
                            reply_mode<=rx_data-8'h30;reply_state<=RP_MODE_END;
                        end else reply_state<=RP_SKIP;
                    end
                    RP_MODE_END: begin
                        if(rx_data=="!")begin
                            o_mode_valid<=1;o_mode_id<=reply_id;o_mode<=reply_mode;
                        end
                        reply_state<=RP_WAIT;
                    end
                    default: begin // RP_SKIP
                        if(rx_data == 8'h23) begin
                            reply_id <= 8'd0;
                            reply_state <= RP_ID0;
                        end else if(rx_data == 8'h21) begin
                            reply_state <= RP_WAIT;
                        end
                    end
                endcase
            end
        end
    end
endmodule
