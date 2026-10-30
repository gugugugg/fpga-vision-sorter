`timescale 1ns/1ps
// K4: retain ten subsequent complete RGB24 frames in a disjoint DDR region.
// Live writes are redirected, not duplicated. During export the HDMI reader
// is drained and paused; live camera writes continue in the original ring.
// Packet: 16-byte HXCP header, RGB bytes, little-endian CRC32 (IEEE/zlib).
module board_capture_export #(
    parameter WIDTH=1024,HEIGHT=600,FRAMES=10,
    parameter [24:0] SAVE_BASE=25'd22000000,
    parameter DRAIN_CYCLES=1024,
    parameter TIMEOUT_CYCLES=100000000
)(
    input wire i_clk,i_rst_n,i_capture,i_resend,i_init_done,i_fault,
    input wire i_wr,i_sof,i_wr_busy,input wire [24:0] i_wr_addr,
    input wire [127:0] i_wr_data,
    input wire i_rd,i_rd_busy,input wire [24:0] i_rd_addr,
    input wire i_ready,i_rd_valid,input wire [127:0] i_rd_data,
    output wire o_live_ready,o_live_rd_valid,o_block_reads,
    output wire o_wr,o_rd,output wire [24:0] o_addr,
    output wire [127:0] o_wr_data,
    output wire o_byte_valid,output reg [7:0] o_byte,
    input wire i_byte_ready,
    output wire o_busy,output reg o_error,output reg [3:0] o_saved,
    output reg [3:0] o_sent,output reg [2:0] o_batch
);
    localparam BYTES=WIDTH*HEIGHT*3;
    localparam BEATS=BYTES/16;
    localparam IDLE=0,ARM=1,CAP=2,DRAIN=3,HEADER=4,
               READ=5,WAIT_READ=6,DATA=7,CRC=8,FINISH=9,VERIFY_END=10;
    reg [3:0] state;
    reg in_frame,retained,export_lock;
    reg [16:0] write_count,read_count;
    reg [24:0] save_addr,export_addr;
    reg [10:0] outstanding;
    reg [15:0] drain_count;
    reg [4:0] byte_index;
    reg [127:0] word_data;
    reg [31:0] crc;
    reg [2:0] next_batch;
    reg [31:0] watchdog;
    wire redirect=(state==ARM && i_sof) ||
                  (state==CAP && (in_frame || i_sof)) ||
                  (state==VERIFY_END && i_sof && o_saved!=FRAMES-1);
    wire export_read=state==READ && !i_wr_busy && !i_wr && i_ready;
    assign o_live_ready=i_ready && !export_lock;
    assign o_block_reads=state>=DRAIN && state!=VERIFY_END;
    assign o_live_rd_valid=i_rd_valid && !export_lock;
    assign o_wr=i_wr;
    assign o_rd=export_lock?export_read:i_rd;
    assign o_addr=i_wr?(redirect?save_addr:i_wr_addr):
                  export_lock?export_addr:i_rd_addr;
    assign o_wr_data=i_wr_data;
    assign o_busy=state!=IDLE;
    assign o_byte_valid=state==HEADER || state==DATA || state==CRC;
    function [31:0] crc_byte;
        input [31:0] old_crc;input [7:0] b;
        reg [31:0] v;integer k;
        begin
            v=old_crc^b;
            for(k=0;k<8;k=k+1) v=v[0]?(v>>1)^32'hedb88320:v>>1;
            crc_byte=v;
        end
    endfunction
    localparam [15:0] W16=WIDTH,H16=HEIGHT;
    localparam [31:0] N32=BYTES;
    always @* begin
        o_byte=0;
        if(state==HEADER) case(byte_index)
            0:o_byte="H";1:o_byte="X";2:o_byte="C";3:o_byte="P";
            4:o_byte=2;5:o_byte={4'd0,o_sent};
            6:o_byte={5'd0,o_batch};7:o_byte=1;
            8:o_byte=W16[7:0];9:o_byte=W16[15:8];
            10:o_byte=H16[7:0];11:o_byte=H16[15:8];
            12:o_byte=N32[7:0];13:o_byte=N32[15:8];
            14:o_byte=N32[23:16];15:o_byte=N32[31:24];
        endcase
        else if(state==DATA) o_byte=word_data[127:120];
        else if(state==CRC) case(byte_index)
            0:o_byte=~crc[7:0];1:o_byte=~crc[15:8];
            2:o_byte=~crc[23:16];3:o_byte=~crc[31:24];
        endcase
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            state<=IDLE;in_frame<=0;retained<=0;export_lock<=0;
            write_count<=0;read_count<=0;save_addr<=SAVE_BASE;
            export_addr<=SAVE_BASE;outstanding<=0;drain_count<=0;
            byte_index<=0;word_data<=0;crc<=32'hffffffff;
            o_error<=0;o_saved<=0;o_sent<=0;o_batch<=0;next_batch<=0;watchdog<=0;
        end else begin
            // Counts accepted *HDMI* requests, including the variable DDR
            // latency. Export never begins while an old response is pending.
            if(!export_lock) case({i_rd,i_rd_valid})
                2'b10:outstanding<=outstanding+1'b1;
                2'b01:if(outstanding!=0) outstanding<=outstanding-1'b1;
            endcase
            // Bound waits for missing SOF, DDR responses and UART progress.
            if(state==IDLE ||
               ((state==ARM || state==CAP || state==VERIFY_END) && i_wr) ||
               (state==WAIT_READ && i_rd_valid) || (o_byte_valid && i_byte_ready))
                watchdog<=0;
            else watchdog<=watchdog+1'b1;
            if(state!=IDLE && (i_fault || watchdog==TIMEOUT_CYCLES-1)) begin
                state<=IDLE;o_error<=1;retained<=0;export_lock<=0;
            end else case(state)
                IDLE:begin
                    if(i_capture && i_init_done && !i_fault) begin
                        state<=ARM;o_error<=0;o_saved<=0;retained<=0;
                        save_addr<=SAVE_BASE;write_count<=0;in_frame<=0;
                        o_batch<=next_batch;
                    end else if(i_resend && retained) begin
                        state<=DRAIN;drain_count<=0;o_sent<=0;
                    end
                end
                ARM,CAP:begin
                    if(i_fault) begin state<=IDLE;o_error<=1;retained<=0;end
                    else if(i_wr) begin
                        if(redirect) begin
                            if((i_sof && in_frame) || (!i_sof && !in_frame)) begin
                                state<=IDLE;o_error<=1;retained<=0;
                            end else begin
                                state<=CAP;save_addr<=save_addr+25'd8;
                                write_count<=i_sof?17'd1:write_count+1'b1;
                                in_frame<=1;
                                if((i_sof && BEATS==1) || (!i_sof && write_count==BEATS-1))
                                    state<=VERIFY_END;
                            end
                        end else if(state==CAP) begin
                            // Too many words is an invalid frame too.
                            state<=IDLE;o_error<=1;retained<=0;
                        end
                    end
                end
                VERIFY_END:if(i_wr) begin
                    // Exact length is provisional. Only the next SOF proves
                    // that no extra word belongs to this frame, including #10.
                    if(!i_sof) begin state<=IDLE;o_error<=1;retained<=0;end
                    else begin
                        o_saved<=o_saved+1'b1;write_count<=0;in_frame<=0;
                        if(o_saved==FRAMES-1) begin
                            state<=DRAIN;retained<=1;drain_count<=0;o_sent<=0;
                            next_batch<=(o_batch==4)?3'd0:o_batch+1'b1;
                        end else begin
                            // First beat of the next saved frame is handled
                            // on this same edge; do not drop it at the boundary.
                            state<=BEATS==1?VERIFY_END:CAP;
                            save_addr<=save_addr+25'd8;write_count<=1;in_frame<=1;
                        end
                    end
                end
                DRAIN:begin
                    if(!i_wr_busy && !i_wr && !i_rd_busy && outstanding==0 && !i_rd) begin
                        // Also gives the controller time to commit the final
                        // writes before the first retained-frame read.
                        if(drain_count==DRAIN_CYCLES-1) begin
                            export_lock<=1;state<=HEADER;byte_index<=0;
                            export_addr<=SAVE_BASE;read_count<=0;crc<=32'hffffffff;
                        end else drain_count<=drain_count+1'b1;
                    end else drain_count<=0;
                end
                HEADER:if(o_byte_valid && i_byte_ready) begin
                    crc<=crc_byte(crc,o_byte);
                    if(byte_index==15) begin byte_index<=0;state<=READ;end
                    else byte_index<=byte_index+1'b1;
                end
                READ:if(export_read) state<=WAIT_READ;
                WAIT_READ:if(i_rd_valid) begin
                    word_data<=i_rd_data;byte_index<=0;state<=DATA;
                end
                DATA:if(o_byte_valid && i_byte_ready) begin
                    crc<=crc_byte(crc,o_byte);
                    word_data<={word_data[119:0],8'd0};
                    if(byte_index==15) begin
                        byte_index<=0;export_addr<=export_addr+25'd8;
                        if(read_count==BEATS-1) state<=CRC;
                        else begin read_count<=read_count+1'b1;state<=READ;end
                    end else byte_index<=byte_index+1'b1;
                end
                CRC:if(o_byte_valid && i_byte_ready) begin
                    if(byte_index==3) begin
                        o_sent<=o_sent+1'b1;byte_index<=0;
                        if(o_sent==FRAMES-1) state<=FINISH;
                        else begin state<=HEADER;read_count<=0;crc<=32'hffffffff;end
                    end else byte_index<=byte_index+1'b1;
                end
                FINISH:if(i_byte_ready) begin export_lock<=0;state<=IDLE;end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
