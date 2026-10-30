`timescale 1ns/1ps
// All ports are in i_clk (125 MHz), except reset. See PORTS.md.
module network_debug_app #(
    parameter [63:0] MODE_BITMAP=64'h1,
    parameter [31:0] PEER_IP=32'h0a4d0001,
    parameter BASIC_CATALOG=0,
    parameter CLK_HZ=125000000,
    parameter FRAME_WIDTH=1024,FRAME_HEIGHT=600,
    parameter TIMEOUT_CYCLES=125000000,
    parameter CONTROL_ENABLE=1
)(
    input wire i_clk,i_rst_n,
    output wire o_udp_ready,
    input wire i_udp_valid,i_udp_last,input wire [7:0] i_udp_data,
    input wire [15:0] i_udp_length,i_udp_src_port,i_udp_dest_port,
    input wire [47:0] i_peer_mac,input wire [31:0] i_peer_ip,
    output wire o_start,output wire [47:0] o_dest_mac,
    output wire [31:0] o_dest_ip,
    output wire [15:0] o_src_port,o_dest_port,output wire [10:0] o_length,
    input wire i_ready,i_payload_ready,
    output wire o_payload_valid,output wire [7:0] o_payload_data,
    output wire o_abort,input wire i_done,i_error,
    input wire i_source_ready,i_capture_ready,
    output reg o_capture,output wire o_original,
    input wire i_frame_valid,input wire [31:0] i_frame_id,i_frame_crc,
    input wire [15:0] i_frame_width,i_frame_height,
    input wire [31:0] i_frame_bytes,input wire [5:0] i_frame_mode,
    input wire i_frame_original,
    output reg o_frame_release,
    input wire i_video_valid,input wire [7:0] i_video_data,
    output wire o_video_ready,input wire i_video_error,
    output wire o_control_valid,output reg o_control_threshold,
    output reg [15:0] o_control_value,
    input wire i_control_ready,i_control_done,i_control_error,
    input wire [231:0] i_status,
    output reg o_streaming,output reg [31:0] o_rejected,o_bad_packets,o_frames
);
    `include "network_crc.vh"
    // String lengths use $bits, avoiding implicit zero padding.
    localparam BASIC_PREFIX="{\"device\":\"HX1P35 network\",\"profiles\":[\"preview\",\"original\"],\"sources\":[1],\"mode_catalog\":\"basic19-v1\",\"fps\":[5],\"debug_port\":6012,\"modes\":[";
    localparam OTHER_PREFIX="{\"device\":\"HX1P35 network\",\"profiles\":[\"preview\",\"original\"],\"sources\":[1],\"mode_catalog\":\"mode30-v1\",\"fps\":[5],\"modes\":[";
    localparam PREFIX=BASIC_CATALOG?BASIC_PREFIX:OTHER_PREFIX;
    localparam SUFFIX=CONTROL_ENABLE?
        "],\"controls\":[\"SET_MODE\",\"SET_THRESHOLD\",\"GET_STATUS\"]}":
        "],\"controls\":[]}";
    localparam PL=BASIC_CATALOG?$bits(BASIC_PREFIX)/8:$bits(OTHER_PREFIX)/8,SL=$bits(SUFFIX)/8;
    function integer cap_size;
        integer k,count,total;
        begin
            count=0;total=PL+SL;
            for(k=0;k<64;k=k+1) if(MODE_BITMAP[k]) begin
                total=total+(count!=0)+(k>=10?2:1);count=count+1;
            end
            cap_size=total;
        end
    endfunction
    function [8191:0] cap_text;
        integer k,j,p,count;
        begin
            cap_text=0;p=0;count=0;
            for(j=0;j<PL;j=j+1) begin cap_text[p*8+:8]=PREFIX[(PL-1-j)*8+:8];p=p+1;end
            for(k=0;k<64;k=k+1) if(MODE_BITMAP[k]) begin
                if(count!=0) begin cap_text[p*8+:8]=",";p=p+1;end
                if(k>=10) begin cap_text[p*8+:8]="0"+k/10;p=p+1;end
                cap_text[p*8+:8]="0"+k%10;p=p+1;count=count+1;
            end
            for(j=0;j<SL;j=j+1) begin cap_text[p*8+:8]=SUFFIX[(SL-1-j)*8+:8];p=p+1;end
        end
    endfunction
    localparam CAP_SIZE=cap_size();
    localparam [8191:0] CAP_TEXT=cap_text();
    // Constant words are stored with the first wire byte at [31:24]. The
    // runtime page mux and the two-bit byte mux are separated by a register.
    function [8191:0] cap_words;
        integer j;
        begin
            cap_words=0;
            for(j=0;j<CAP_SIZE;j=j+1)
                cap_words[(j/4)*32+(3-j%4)*8+:8]=CAP_TEXT[j*8+:8];
        end
    endfunction
    localparam [((CAP_SIZE+3)/4)*32-1:0] CAP_WORDS=cap_words();
    localparam ACK_OK="{\"ok\":true}",ACK_NO="{\"ok\":false}";
    localparam RX_IDLE=0,RX_LOAD=1,CHECK=2,JSON_START=3,JSON_WAIT=4,
               CTRL_LOAD=5,CTRL_WAIT=6,RESP_WAIT=7;
    reg [3:0] rs;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [7:0] rx[0:1099];
    reg [143:0] rx_header;
    function [7:0] h;
        input [4:0] p;
        begin h=rx_header[(17-p)*8+:8];end
    endfunction
    reg [10:0] rx_count;reg [15:0] rx_length,rx_port,rx_dest;
    reg [47:0] rx_mac;reg [31:0] rx_ip;
    reg [31:0] rx_crc,rx_crc_tail;reg [15:0] rx_crc16,rx_tail16;
    reg rx_bad;reg [31:0] rx_watch;
    reg [63:0] request_session;reg [31:0] request_id;
    reg [7:0] query_kind;reg [15:0] vx_sequence;
    reg [10:0] json_size;
    wire [10:0] json_addr;wire json_done,json_ok,json_original;
    reg [7:0] json_byte;reg json_read_valid;
    // Single synchronous RAM read port. A byte is consumed every other cycle.
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin json_byte<=0;json_read_valid<=0;end
        else begin
            json_read_valid<=0;
            if(rs==JSON_WAIT && !json_read_valid) begin
                json_byte<=rx[json_addr];json_read_valid<=1;
            end
        end
    end
    network_vq_json parser(.i_clk(i_clk),.i_rst_n(i_rst_n),.i_start(rs==JSON_START),
        .i_stream(query_kind==3),.i_size(json_size),.o_addr(json_addr),
        .i_byte(json_byte),.i_byte_valid(json_read_valid),
        .o_done(json_done),.o_ok(json_ok),.o_original(json_original));
    reg session_valid;reg [63:0] session;
    reg [47:0] stream_mac;reg [31:0] stream_ip;reg [15:0] stream_port;
    reg original;reg [31:0] cadence;
    reg capture_pending;
    reg reply_pending;reg [2:0] reply_type; // 0=cap,1=ok,2=no,3=status,4=VX error
    assign o_original=original;
    assign o_udp_ready=rs==RX_IDLE && !reply_pending;
    assign o_control_valid=rs==CTRL_LOAD;
    reg [231:0] status_snapshot;
    wire [10:0] reply_body_len=reply_type==0?CAP_SIZE:
        reply_type==1?11:reply_type==2?12:reply_type==3?29:1;
    wire reply_vx=reply_type>=3;
    wire [10:0] reply_len=reply_body_len+(reply_vx?10:22);
    localparam TX_IDLE=0,TX_START=1,TX_DATA=2,TX_DONE=3;
    reg [1:0] ts;reg tx_video;reg [10:0] tx_pos,tx_len;
    reg [31:0] tx_crc;reg [15:0] tx_crc16;
    // One byte in flight, four serial edges: decode -> word -> byte -> accept.
    // No stage starts the next position before N accepts the current byte.
    localparam SEL_VF=0,SEL_PIXEL=1,SEL_VQ=2,SEL_CAP=3,SEL_OK=4,SEL_NO=5,
               SEL_VX=6,SEL_STATUS=7,SEL_ERROR=8,SEL_CRC32=9,SEL_CRC16=10;
    reg tx_sel_valid,tx_sel_body,tx_sel_last;
    reg [3:0] tx_sel_kind;
    reg [9:0] tx_sel_index;
    reg tx_word_valid,tx_word_body,tx_word_last;
    reg [31:0] tx_word_data;
    reg [1:0] tx_word_index;
    reg tx_byte_valid,tx_byte_body,tx_byte_last;
    reg [7:0] tx_byte_data;
    reg [31:0] tx_watch;
    reg [47:0] dest_mac;reg [31:0] dest_ip;reg [15:0] dest_port,src_port;
    reg frame_active;reg [63:0] frame_session;
    reg [31:0] frame_id,frame_crc,frame_bytes,frame_offset;
    reg [15:0] frame_width,frame_height,chunk_index,chunk_count;
    reg [5:0] frame_mode;reg frame_original;reg [10:0] chunk_size;
    function [31:0] vf_word;
        input [3:0] p;
        begin
            vf_word=0;
            case(p)
                0:vf_word={8'h56,8'h46,8'd1,(frame_original?8'd1:8'd2)};
                1:vf_word={(frame_original?8'd1:8'd0),8'd1,2'd0,frame_mode,8'd0};
                2:vf_word=frame_session[63:32];3:vf_word=frame_session[31:0];
                4:vf_word=frame_id;
                5:vf_word={frame_width,frame_height};
                6:vf_word={chunk_index,chunk_count};
                7:vf_word=frame_bytes;8:vf_word=frame_crc;
            endcase
        end
    endfunction
    function [31:0] vq_word;
        input [2:0] p;
        begin
            vq_word=0;
            case(p)
                0:vq_word={8'h56,8'h51,8'd1,(reply_type==0?8'd2:8'd4)};
                1:vq_word=request_session[63:32];2:vq_word=request_session[31:0];
                3:vq_word=request_id;
                4:vq_word={5'd0,reply_body_len,16'd0};
            endcase
        end
    endfunction
    function [31:0] status_word;
        input [2:0] p;
        begin
            case(p)
                0:status_word=status_snapshot[231:200];1:status_word=status_snapshot[199:168];
                2:status_word=status_snapshot[167:136];3:status_word=status_snapshot[135:104];
                4:status_word=status_snapshot[103:72];5:status_word=status_snapshot[71:40];
                6:status_word=status_snapshot[39:8];7:status_word={status_snapshot[7:0],24'd0};
            endcase
        end
    endfunction
    function [31:0] ack_word;
        input no;input [1:0] p;
        begin
            ack_word=0;
            case(p)
                0:ack_word=32'h7b226f6b; // {"ok
                1:ack_word=no?32'h223a6661:32'h223a7472; // ":fa / ":tr
                2:ack_word=no?32'h6c73657d:32'h75657d00; // lse} / ue}
            endcase
        end
    endfunction
    reg [31:0] selected_word;
    always @* begin
        selected_word=0;
        case(tx_sel_kind)
            SEL_VF:selected_word=vf_word(tx_sel_index[5:2]);
            SEL_PIXEL:selected_word={i_video_data,24'd0};
            SEL_VQ:selected_word=vq_word(tx_sel_index[4:2]);
            SEL_CAP:selected_word=CAP_WORDS[{tx_sel_index[9:2],5'd0}+:32];
            SEL_OK:selected_word=ack_word(1'b0,tx_sel_index[3:2]);
            SEL_NO:selected_word=ack_word(1'b1,tx_sel_index[3:2]);
            SEL_VX:selected_word=tx_sel_index[2]?
                {vx_sequence,5'd0,reply_body_len}:
                {8'h56,8'h58,8'd1,(reply_type==3?8'h81:8'hff)};
            SEL_STATUS:selected_word=status_word(tx_sel_index[4:2]);
            SEL_ERROR:selected_word=32'h01000000;
            SEL_CRC32:selected_word=~tx_crc;
            SEL_CRC16:selected_word={tx_crc16,16'd0};
        endcase
    end
    wire [10:0] tail_start=tx_len-(tx_video || !reply_vx?4:2);
    wire video_pixel=tx_video && tx_pos>=36 && tx_pos<tail_start;
    assign o_start=ts==TX_START && i_ready;
    assign o_dest_mac=dest_mac;assign o_dest_ip=dest_ip;
    assign o_dest_port=dest_port;assign o_src_port=src_port;assign o_length=tx_len;
    assign o_payload_valid=ts==TX_DATA && tx_byte_valid && !o_abort && !i_error;
    // Consumes a source pixel INTO the slot, independently of N's load stall.
    // At most one consumed pixel can be ahead of N; cancellation discards the
    // complete frame and releases the descriptor, including that prefetched byte.
    assign o_video_ready=ts==TX_DATA && tx_sel_valid && tx_sel_kind==SEL_PIXEL &&
                         !tx_word_valid && !tx_byte_valid && !o_abort && !i_error;
    assign o_payload_data=tx_byte_data;
    wire frame_fault=frame_active && (!i_source_ready || i_video_error || !o_streaming);
    wire tx_timeout=ts!=TX_IDLE && tx_watch==TIMEOUT_CYCLES-1;
    assign o_abort=ts!=TX_IDLE && (tx_timeout || (tx_video && frame_fault));
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            rs<=RX_IDLE;rx_header<=0;rx_count<=0;rx_length<=0;rx_port<=0;rx_dest<=0;rx_mac<=0;rx_ip<=0;
            rx_crc<=32'hffffffff;rx_crc_tail<=0;rx_crc16<=16'hffff;rx_tail16<=0;
            rx_bad<=0;rx_watch<=0;request_session<=0;request_id<=0;query_kind<=0;
            vx_sequence<=0;json_size<=0;session_valid<=0;session<=0;stream_mac<=0;
            stream_ip<=0;stream_port<=0;original<=0;cadence<=0;capture_pending<=0;
            o_capture<=0;o_frame_release<=0;o_streaming<=0;o_rejected<=0;o_bad_packets<=0;
            o_frames<=0;o_control_threshold<=0;o_control_value<=0;
            reply_pending<=0;reply_type<=0;status_snapshot<=0;
            ts<=TX_IDLE;tx_video<=0;tx_pos<=0;tx_len<=0;tx_crc<=32'hffffffff;
            tx_byte_valid<=0;tx_byte_data<=0;tx_byte_body<=0;tx_byte_last<=0;
            tx_sel_valid<=0;tx_sel_body<=0;tx_sel_last<=0;tx_sel_kind<=0;tx_sel_index<=0;
            tx_word_valid<=0;tx_word_body<=0;tx_word_last<=0;tx_word_data<=0;tx_word_index<=0;
            tx_crc16<=16'hffff;tx_watch<=0;dest_mac<=0;dest_ip<=0;dest_port<=0;src_port<=0;
            frame_active<=0;frame_session<=0;frame_id<=0;frame_crc<=0;frame_bytes<=0;
            frame_offset<=0;frame_width<=0;frame_height<=0;chunk_index<=0;chunk_count<=0;
            frame_mode<=0;frame_original<=0;chunk_size<=0;
        end else begin
            o_capture<=0;o_frame_release<=0;
            if(cadence!=0) cadence<=cadence-1'b1;
            if(o_streaming && i_source_ready && i_capture_ready && !frame_active && !capture_pending &&
               !i_frame_valid && cadence==0) begin
                o_capture<=1;capture_pending<=1;cadence<=CLK_HZ/5-1;
            end
            if(!o_streaming || !i_source_ready || i_video_error) capture_pending<=0;
            if(i_frame_valid && !frame_active && !o_frame_release) begin
                capture_pending<=0;
                if(o_streaming && i_source_ready && i_frame_original==original &&
                   i_frame_width==(original?FRAME_WIDTH:FRAME_WIDTH/2) &&
                   i_frame_height==(original?FRAME_HEIGHT:FRAME_HEIGHT/2) &&
                   i_frame_bytes==(original?FRAME_WIDTH*FRAME_HEIGHT*3:(FRAME_WIDTH/2)*(FRAME_HEIGHT/2)*2) &&
                   MODE_BITMAP[i_frame_mode]) begin
                    frame_active<=1;frame_session<=session;frame_id<=i_frame_id;
                    frame_crc<=i_frame_crc;frame_bytes<=i_frame_bytes;
                    frame_width<=i_frame_width;frame_height<=i_frame_height;
                    frame_mode<=i_frame_mode;frame_original<=i_frame_original;
                    frame_offset<=0;chunk_index<=0;
                    chunk_count<=original?(FRAME_WIDTH*FRAME_HEIGHT*3+1199)/1200:
                        ((FRAME_WIDTH/2)*(FRAME_HEIGHT/2)*2+1199)/1200;
                end else o_frame_release<=1;
            end
            rx_watch<=rs==RX_IDLE || i_udp_valid || json_done?0:rx_watch+1'b1;
            if(rs!=RX_IDLE && rs!=RESP_WAIT && rx_watch==TIMEOUT_CYCLES-1) begin
                rs<=RX_IDLE;o_bad_packets<=o_bad_packets+1'b1;
            end else case(rs)
                RX_IDLE:if(i_udp_valid && o_udp_ready) begin
                    rx_header[143:136]<=i_udp_data;rx_count<=1;rx_length<=i_udp_length;
                    rx_port<=i_udp_src_port;rx_dest<=i_udp_dest_port;rx_mac<=i_peer_mac;rx_ip<=i_peer_ip;
                    rx_crc<=crc32_byte(32'hffffffff,i_udp_data);rx_crc16<=crc16_byte(16'hffff,i_udp_data);
                    rx_crc_tail<=0;rx_tail16<=0;
                    rx_bad<=i_udp_length<10 || i_udp_length>1472 || i_udp_last ||
                        i_peer_ip!=PEER_IP || (i_udp_dest_port!=6011 && i_udp_dest_port!=6002);
                    rs<=i_udp_last?CHECK:RX_LOAD;
                end
                RX_LOAD:if(i_udp_valid) begin
                    if(rx_count<18) rx_header[(17-rx_count)*8+:8]<=i_udp_data;
                    else if(rx_count<1118) rx[rx_count-18]<=i_udp_data;
                    if(rx_count<rx_length-4) rx_crc<=crc32_byte(rx_crc,i_udp_data);
                    else rx_crc_tail<={rx_crc_tail[23:0],i_udp_data};
                    if(rx_count<rx_length-2) rx_crc16<=crc16_byte(rx_crc16,i_udp_data);
                    else rx_tail16<={rx_tail16[7:0],i_udp_data};
                    rx_count<=rx_count+1'b1;
                    if(i_udp_last) begin rs<=CHECK;if(rx_count!=rx_length-1) rx_bad<=1;end
                    else if(rx_count>=rx_length-1) begin rx_bad<=1;rs<=CHECK;end
                end
                CHECK:begin
                    if(rx_bad || h(0)!="V" || h(2)!=1) begin
                        rs<=RX_IDLE;o_bad_packets<=o_bad_packets+1'b1;
                    end else if(rx_dest==6011 && h(1)=="Q" && rx_length>=22 && rx_length<=1122 &&
                                {h(16),h(17)}==rx_length-22 && rx_crc_tail==~rx_crc) begin
                        request_session<={h(4),h(5),h(6),h(7),h(8),h(9),h(10),h(11)};
                        request_id<={h(12),h(13),h(14),h(15)};query_kind<=h(3);
                        json_size<={h(16),h(17)};rs<=JSON_START;
                    end else if(rx_dest==6002 && h(1)=="X" &&
                                {h(6),h(7)}==rx_length-10 && rx_tail16==rx_crc16) begin
                        vx_sequence<={h(4),h(5)};reply_type<=4;
                        if(CONTROL_ENABLE && h(3)==8'h30 && rx_length==10) begin
                            reply_type<=3;status_snapshot<=i_status;reply_pending<=1;rs<=RESP_WAIT;
                        end else if(CONTROL_ENABLE &&
                            ((h(3)==8'h10 && rx_length==11 && h(8)<64 && MODE_BITMAP[h(8)]) ||
                             (h(3)==8'h11 && rx_length==12 && h(8)==0))) begin
                            o_control_threshold<=h(3)==8'h11;
                            o_control_value<=h(3)==8'h11 ? {h(8),h(9)}:{8'd0,h(8)};rs<=CTRL_LOAD;
                        end else begin reply_pending<=1;rs<=RESP_WAIT;o_rejected<=o_rejected+1'b1;end
                    end else begin rs<=RX_IDLE;o_bad_packets<=o_bad_packets+1'b1;end
                end
                JSON_START:rs<=JSON_WAIT;
                JSON_WAIT:if(json_done) begin
                    if(json_ok && query_kind==1 && !frame_active && !capture_pending) begin
                        session_valid<=1;session<=request_session;stream_mac<=rx_mac;
                        stream_ip<=rx_ip;stream_port<=rx_port;o_streaming<=0;
                        reply_type<=0;reply_pending<=1;rs<=RESP_WAIT;
                    end else if(json_ok && query_kind==6 && session_valid && request_session==session &&
                                rx_ip==stream_ip && rx_mac==stream_mac && rx_port==stream_port) begin
                        reply_type<=0;reply_pending<=1;rs<=RESP_WAIT;
                    end else if(query_kind==3 || query_kind==5) begin
                        reply_type<=2;reply_pending<=1;rs<=RESP_WAIT;
                        if(json_ok && session_valid && request_session==session && rx_ip==stream_ip &&
                           rx_mac==stream_mac && rx_port==stream_port &&
                           (query_kind==5 || (i_source_ready && !frame_active && !capture_pending))) begin
                            reply_type<=1;o_streaming<=query_kind==3;
                            if(query_kind==3) begin original<=json_original;cadence<=0;end
                        end else o_rejected<=o_rejected+1'b1;
                    end else begin rs<=RX_IDLE;o_rejected<=o_rejected+1'b1;end
                end
                CTRL_LOAD:if(i_control_ready) rs<=CTRL_WAIT;
                CTRL_WAIT:if(i_control_done) begin
                    status_snapshot<=i_status;reply_type<=i_control_error?4:3;
                    reply_pending<=1;rs<=RESP_WAIT;
                end
                RESP_WAIT:if(!reply_pending && ts==TX_IDLE) rs<=RX_IDLE;
            endcase
            tx_watch<=ts==TX_IDLE || (o_payload_valid && i_payload_ready) || i_done?0:tx_watch+1'b1;
            if(frame_fault) begin frame_active<=0;o_frame_release<=1;end
            if(o_abort || i_error) begin
                ts<=TX_IDLE;tx_byte_valid<=0;tx_byte_data<=0;tx_byte_body<=0;tx_byte_last<=0;
                tx_sel_valid<=0;tx_sel_body<=0;tx_sel_last<=0;tx_sel_kind<=0;tx_sel_index<=0;
                tx_word_valid<=0;tx_word_body<=0;tx_word_last<=0;tx_word_data<=0;tx_word_index<=0;
                if(tx_video) begin frame_active<=0;o_frame_release<=1;end
                else reply_pending<=0;
            end else case(ts)
                TX_IDLE:begin
                    tx_byte_valid<=0;
                    tx_sel_valid<=0;tx_word_valid<=0;
                    tx_pos<=0;tx_crc<=32'hffffffff;tx_crc16<=16'hffff;
                    if(reply_pending) begin
                        tx_video<=0;tx_len<=reply_len;dest_mac<=rx_mac;dest_ip<=rx_ip;
                        dest_port<=rx_port;src_port<=reply_vx?6002:6011;ts<=TX_START;
                    end else if(frame_active && !frame_fault) begin
                        tx_video<=1;chunk_size<=frame_bytes-frame_offset>1200?1200:frame_bytes-frame_offset;
                        tx_len<=40+(frame_bytes-frame_offset>1200?1200:frame_bytes-frame_offset);
                        dest_mac<=stream_mac;dest_ip<=stream_ip;dest_port<=stream_port;src_port<=6011;
                        ts<=TX_START;
                    end
                end
                TX_START:if(i_ready) begin
                    tx_byte_valid<=0;tx_sel_valid<=0;tx_word_valid<=0;ts<=TX_DATA;
                end
                TX_DATA:begin
                    if(!tx_sel_valid && !tx_word_valid && !tx_byte_valid) begin
                        tx_sel_valid<=1;tx_sel_body<=tx_pos<tail_start;tx_sel_last<=tx_pos==tx_len-1;
                        tx_sel_index<=tx_pos[9:0];
                        if(tx_pos>=tail_start) begin
                            tx_sel_kind<=tx_video || !reply_vx?SEL_CRC32:SEL_CRC16;
                            tx_sel_index<={8'd0,tx_pos[1:0]-tail_start[1:0]};
                        end else if(tx_video) begin
                            tx_sel_kind<=video_pixel?SEL_PIXEL:SEL_VF;
                            if(video_pixel) tx_sel_index<=0;
                        end else if(reply_vx) begin
                            if(tx_pos<8) tx_sel_kind<=SEL_VX;
                            else begin
                                tx_sel_kind<=reply_type==3?SEL_STATUS:SEL_ERROR;
                                tx_sel_index<={5'd0,tx_pos[4:0]-5'd8};
                            end
                        end else if(tx_pos<18) tx_sel_kind<=SEL_VQ;
                        else begin
                            tx_sel_kind<=reply_type==0?SEL_CAP:reply_type==1?SEL_OK:SEL_NO;
                            tx_sel_index<=tx_pos[9:0]-10'd18;
                        end
                    end
                    if(tx_sel_valid && (tx_sel_kind!=SEL_PIXEL || i_video_valid)) begin
                        tx_sel_valid<=0;tx_word_valid<=1;tx_word_data<=selected_word;
                        tx_word_index<=tx_sel_index[1:0];
                        tx_word_body<=tx_sel_body;tx_word_last<=tx_sel_last;
                    end
                    if(tx_word_valid) begin
                        tx_word_valid<=0;tx_byte_valid<=1;
                        case(tx_word_index)
                            0:tx_byte_data<=tx_word_data[31:24];1:tx_byte_data<=tx_word_data[23:16];
                            2:tx_byte_data<=tx_word_data[15:8];3:tx_byte_data<=tx_word_data[7:0];
                        endcase
                        tx_byte_body<=tx_word_body;tx_byte_last<=tx_word_last;
                    end
                    if(o_payload_valid && i_payload_ready) begin
                        tx_byte_valid<=0;
                        if(tx_byte_body) begin
                            tx_crc<=crc32_byte(tx_crc,tx_byte_data);
                            tx_crc16<=crc16_byte(tx_crc16,tx_byte_data);
                        end
                        tx_pos<=tx_pos+1'b1;
                        if(tx_byte_last) ts<=TX_DONE;
                    end
                end
                TX_DONE:if(i_done) begin
                    ts<=TX_IDLE;
                    if(tx_video) begin
                        if(chunk_index==chunk_count-1) begin
                            frame_active<=0;o_frame_release<=1;o_frames<=o_frames+1'b1;
                        end else begin chunk_index<=chunk_index+1'b1;frame_offset<=frame_offset+chunk_size;end
                    end else reply_pending<=0;
                end
            endcase
        end
    end
endmodule
