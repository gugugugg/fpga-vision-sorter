`timescale 1ns/1ps
// VD v1 diagnostic endpoint. Requests are 12 header bytes plus CRC32;
// replies are 24 header bytes, actual telemetry or frozen samples, and CRC32.
module network_diagnostic_endpoint #(
    parameter [31:0] PEER_IP=32'ha9fe125a,
    parameter SAMPLE_HZ=112500000
)(
    input wire i_clk,i_rst_n,
    input wire i_udp_valid,i_udp_last,input wire [7:0] i_udp_data,
    input wire [15:0] i_udp_length,i_udp_src_port,
    input wire [47:0] i_peer_mac,input wire [31:0] i_peer_ip,
    output wire o_udp_ready,
    output wire o_request,o_start,output reg [47:0] o_dest_mac,
    output reg [15:0] o_dest_port,output wire [10:0] o_length,
    input wire i_ready,i_payload_ready,i_done,i_error,
    output wire o_payload_valid,output wire [7:0] o_payload_data,
    output wire o_abort,
    input wire [255:0] i_telemetry,
    output reg o_arm,input wire i_arm_ready,i_trace_valid,i_trace_busy,
    output reg o_trace_read,output reg [10:0] o_trace_addr,input wire [31:0] i_trace_data
);
    `include "network_crc.vh"
    localparam IDLE=0,RX=1,CHECK=2,BUILD=3,RWAIT1=4,RWAIT2=5,
        CRC=6,START=7,LOAD=8,SEND=9,DONE=10;
    reg [3:0] state;
    reg [127:0] request;
    reg [4:0] rx_count;
    reg [31:0] rx_crc,rx_tail;
    reg rx_bad;
    reg [31:0] session,trace_session,trace_id;
    reg [15:0] request_seq,offset,reply_status,count,trace_port,arm_sequence;
    reg [7:0] kind;
    reg [255:0] telemetry;
    reg [31:0] trace_word,tx_crc,watchdog;
    reg [10:0] pos,body_end,packet_length;
    reg [1:0] crc_pos;
    reg [7:0] tx_byte;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [7:0] buffer[0:1055];
    function [7:0] h(input integer index);
        h=request[(15-index)*8+:8];
    endfunction
    wire [191:0] header={16'h5644,8'd1,(kind|8'h80),session,request_seq,offset,
        reply_status,count,32'(SAMPLE_HZ),trace_id};
    reg [7:0] build_byte;
    always @* begin
        build_byte=0;
        if(pos<24) build_byte=header[(23-pos)*8+:8];
        else if(kind==3) build_byte=trace_word[(3-pos[1:0])*8+:8];
        else build_byte=telemetry[(55-pos)*8+:8];
    end
    assign o_udp_ready=state==IDLE;
    assign o_request=state==START;
    assign o_start=o_request && i_ready;
    assign o_length=packet_length;
    assign o_payload_valid=state==SEND;
    assign o_payload_data=tx_byte;
    assign o_abort=state>=START && watchdog>=500000;
    // A single synchronous RAM write port and unreset synchronous read port.
    // Control registers reset independently; output data is valid only in SEND.
    wire build_write=state==BUILD && !(kind==3 && pos>=24 && pos<body_end && pos[1:0]==0 && !o_trace_read);
    wire buffer_write=build_write || state==RWAIT2 || state==CRC;
    wire [7:0] buffer_byte=state==CRC?~tx_crc[(3-crc_pos)*8+:8]:
        state==RWAIT2?i_trace_data[31:24]:build_byte;
    always @(posedge i_clk) begin
        if(i_rst_n && buffer_write) buffer[pos]<=buffer_byte;
        if(state==LOAD) tx_byte<=buffer[pos];
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            state<=IDLE;request<=0;rx_count<=0;rx_crc<=32'hffffffff;rx_tail<=0;rx_bad<=0;
            session<=0;request_seq<=0;offset<=0;reply_status<=0;count<=0;kind<=0;
            trace_session<=0;trace_port<=0;trace_id<=0;arm_sequence<=0;
            telemetry<=0;trace_word<=0;tx_crc<=32'hffffffff;watchdog<=0;
            pos<=0;body_end<=0;packet_length<=0;crc_pos<=0;
            o_dest_mac<=0;o_dest_port<=0;o_arm<=0;o_trace_read<=0;o_trace_addr<=0;
        end else begin
            o_arm<=0;o_trace_read<=0;
            if(state>=START) watchdog<=watchdog+1'b1;else watchdog<=0;
            if(o_abort || i_error) state<=IDLE;
            else case(state)
                IDLE:if(i_udp_valid) begin
                    request<={i_udp_data,120'd0};rx_count<=1;
                    rx_crc<=crc32_byte(32'hffffffff,i_udp_data);rx_tail<=0;
                    rx_bad<=i_udp_length!=16 || i_peer_ip!=PEER_IP || i_peer_mac==0 || i_udp_last;
                    o_dest_mac<=i_peer_mac;o_dest_port<=i_udp_src_port;state<=RX;
                end
                RX:if(i_udp_valid) begin
                    if(rx_count<16) request[(15-rx_count)*8+:8]<=i_udp_data;
                    if(rx_count<12) rx_crc<=crc32_byte(rx_crc,i_udp_data);
                    else rx_tail<={rx_tail[23:0],i_udp_data};
                    rx_count<=rx_count+1'b1;
                    if(i_udp_last) begin state<=CHECK;if(rx_count!=15) rx_bad<=1;end
                    else if(rx_count>=15) begin rx_bad<=1;state<=CHECK;end
                end
                CHECK:begin
                    if(rx_bad || h(0)!="V" || h(1)!="D" || h(2)!=1 || rx_tail!=~rx_crc)
                        state<=IDLE;
                    else begin
                        session<={h(4),h(5),h(6),h(7)};request_seq<={h(8),h(9)};
                        offset<={h(10),h(11)};kind<=h(3);count<=0;
                        telemetry<=i_telemetry;body_end<=56;packet_length<=60;
                        reply_status<=i_trace_busy?16'd1:(i_trace_valid?16'd0:16'd2);
                        if(h(3)==2) begin
                            if(trace_session=={h(4),h(5),h(6),h(7)} && trace_port==o_dest_port && arm_sequence=={h(8),h(9)} && trace_id!=0) begin end
                            else if(i_arm_ready) begin
                                o_arm<=1;trace_id<=trace_id+1'b1;reply_status<=1;
                                trace_session<={h(4),h(5),h(6),h(7)};
                                trace_port<=o_dest_port;arm_sequence<={h(8),h(9)};
                            end else reply_status<=1;
                        end else if(h(3)==3) begin
                            body_end<=24;packet_length<=28;
                            if(i_trace_valid && !i_trace_busy && trace_session=={h(4),h(5),h(6),h(7)} &&
                                trace_port==o_dest_port && {h(10),h(11)}<2048 && h(11)==0) begin
                                count<=256;reply_status<=0;body_end<=1048;packet_length<=1052;
                            end else reply_status<=3;
                        end else if(h(3)!=1) reply_status<=3;
                        pos<=0;tx_crc<=32'hffffffff;state<=BUILD;
                    end
                end
                BUILD:begin
                    if(kind==3 && pos>=24 && pos<body_end && pos[1:0]==0 && !o_trace_read) begin
                        o_trace_read<=1;o_trace_addr<=offset+((pos-24)>>2);state<=RWAIT1;
                    end else begin
                        tx_crc<=crc32_byte(tx_crc,build_byte);
                        if(pos==body_end-1) begin crc_pos<=0;pos<=body_end;state<=CRC;end
                        else pos<=pos+1'b1;
                    end
                end
                RWAIT1:state<=RWAIT2;
                RWAIT2:begin
                    // Write the first byte after the synchronous RAM read;
                    // subsequent three bytes use the stable captured word.
                    trace_word<=i_trace_data;
                    tx_crc<=crc32_byte(tx_crc,i_trace_data[31:24]);pos<=pos+1'b1;state<=BUILD;
                end
                CRC:begin
                    pos<=pos+1'b1;
                    if(crc_pos==3) begin pos<=0;state<=START;end
                    else crc_pos<=crc_pos+1'b1;
                end
                START:if(i_ready) state<=LOAD;
                LOAD:state<=SEND;
                SEND:if(i_payload_ready) begin
                    if(pos==packet_length-1) state<=DONE;
                    else begin pos<=pos+1'b1;state<=LOAD;end
                end
                DONE:if(i_done) state<=IDLE;
            endcase
        end
    end
endmodule
