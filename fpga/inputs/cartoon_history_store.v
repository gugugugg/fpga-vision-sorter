`timescale 1ns/1ps
// Independent common DDR client. Four FINAL-centre history states per beat.
// Request strobes are accepted transactions (ready && intent), no tail words.
// A frame is committed only after ALL state words have entered the ordered DDR
// port. Duplicate source frames reuse the original prior bank, never recurse.
module cartoon_history_store #(
    parameter PIXELS=614400, FIFO_BITS=9,
    parameter [24:0] BASE0=25'd1000000, BASE1=25'd2300000
)(
    input wire rst_n,pixel_clk,ddr_clk,
    input wire start,input wire [31:0] frame_id,input wire [15:0] configuration,
    input wire read_pixel,write_pixel,input wire [31:0] new_state,
    output wire old_valid,output wire [31:0] old_state,
    output reg [7:0] status,output wire [127:0] diagnostics,
    output wire wr,rd,output wire [24:0] addr,output wire [127:0] data,
    input wire ready,rd_valid,i_fault,input wire [127:0] rd_data
);
    localparam BEATS=PIXELS/4, DEPTH=1<<FIFO_BITS;
    localparam IDLE=0,CLEAR=1,RUN=2,DRAIN=3;
    wire pixel_reset_n,ddr_reset_n;
    reset_release u_pixel_control_reset(pixel_clk,rst_n,pixel_reset_n);
    reset_release u_ddr_control_reset(ddr_clk,rst_n,ddr_reset_n);
    reg [1:0] state;
    reg [4:0] reset_count;
    reg job_generation,compare_frame,write_frame,read_bank,write_bank;
    reg last_good,last_bank,epoch_good,epoch_compare,epoch_read_bank;
    reg [31:0] last_id,epoch_id;reg [15:0] last_config,epoch_config;
    reg [31:0] active_id;reg [15:0] active_config;
    reg [17:0] read_count,write_count;
    reg [FIFO_BITS:0] outstanding;
    reg end_seen,bad_frame,fair;
    reg [127:0] held_write;reg held_valid,write_pending;
    reg [22:0] watchdog;
    reg ddr_fault_seen;
    reg [15:0] committed_frames,aborted_frames,read_bad_frames,write_bad_frames;
    reg [7:0] last_cause,frame_cause;
    reg read_bad_this_frame,write_bad_this_frame;
    reg [47:0] abort_snapshot;
    wire [8:0] diagnostic_outstanding=outstanding;
    wire [95:0] ddr_diagnostics;
    (* async_reg="true",syn_preserve=1 *) reg fault_sync0,fault_sync1;
    reg generation;
    wire job_update;wire [48:0] job;
    cdc_bus_toggle #(.WIDTH(49)) u_job(
        .i_src_clk(pixel_clk),.i_src_rst_n(pixel_reset_n),.i_src_update(start && frame_id!=0),
        .i_src_data({~generation,frame_id,configuration}),.o_src_busy(),
        .i_dst_clk(ddr_clk),.i_dst_rst_n(ddr_reset_n),.o_dst_update(job_update),.o_dst_data(job));
    reg announce;reg [3:0] announcement;
    wire [3:0] pixel_announcement;wire announce_update;
    cdc_bus_toggle #(.WIDTH(4)) u_ready(
        .i_src_clk(ddr_clk),.i_src_rst_n(ddr_reset_n),.i_src_update(announce),
        .i_src_data(announcement),.o_src_busy(),
        .i_dst_clk(pixel_clk),.i_dst_rst_n(pixel_reset_n),.o_dst_update(announce_update),.o_dst_data(pixel_announcement));
    reg end_send;reg [1:0] end_data;
    wire end_update;wire [1:0] end_message;
    cdc_bus_toggle #(.WIDTH(2)) u_end(
        .i_src_clk(pixel_clk),.i_src_rst_n(pixel_reset_n),.i_src_update(end_send),
        .i_src_data(end_data),.o_src_busy(),
        .i_dst_clk(ddr_clk),.i_dst_rst_n(ddr_reset_n),.o_dst_update(end_update),.o_dst_data(end_message));
    // Publish the previous transaction at each new job. Atomic counters let
    // the host distinguish a boot-only fault from continuing frame aborts.
    cdc_bus_toggle #(.WIDTH(96)) u_diagnostics(
        .i_src_clk(ddr_clk),.i_src_rst_n(ddr_reset_n),.i_src_update(job_update),
        .i_src_data({abort_snapshot,committed_frames,aborted_frames,8'd2,last_cause}),.o_src_busy(),
        .i_dst_clk(pixel_clk),.i_dst_rst_n(pixel_reset_n),.o_dst_update(),.o_dst_data(ddr_diagnostics));
    assign diagnostics={ddr_diagnostics[95:48],ddr_diagnostics[47:16],read_bad_frames,write_bad_frames,ddr_diagnostics[15:0]};

    reg reset_fifos;
    wire fifo_rst_n=rst_n && !reset_fifos;
    wire pixel_fifo_rst_n;
    reset_release u_pixel_reset(pixel_clk,fifo_rst_n,pixel_fifo_rst_n);
    wire read_full,read_overflow,read_word_valid;wire [127:0] read_word;
    wire [FIFO_BITS:0] read_used,read_wr_used;
    wire pop_read;
    capture_stream_fifo #(.WIDTH(128),.ADDR_BITS(FIFO_BITS)) u_read_fifo(
        .i_rst_n(fifo_rst_n),.i_wr_clk(ddr_clk),.i_rd_clk(pixel_clk),
        .i_we(rd_valid && state==RUN && compare_frame),.i_data(rd_data),
        .o_full(read_full),.o_overflow(read_overflow),.o_wr_used(read_wr_used),
        .i_re(pop_read),.o_data(read_word),.o_valid(read_word_valid),.o_used(read_used));
    reg [95:0] pack;
    reg [19:0] input_count,output_count;
    reg pixel_bad,pixel_started,session_seen,have_job;
    wire session_ok=session_seen && pixel_announcement[3]==generation && pixel_fifo_rst_n;
    wire comparing=session_ok && pixel_announcement[2];
    wire writing=session_ok && pixel_announcement[1];
    reg [127:0] head,tail;reg head_valid,tail_valid,read_pending;
    wire consume_word=read_pixel && !pixel_bad && comparing && input_count[1:0]==3 && head_valid;
    assign pop_read=session_ok && comparing && !pixel_bad && !read_pending && !tail_valid && read_used!=0;
    assign old_valid=read_pixel && comparing && !pixel_bad && head_valid && input_count<PIXELS;
    assign old_state=head >> (input_count[1:0]*32);
    always @(posedge pixel_clk or negedge pixel_fifo_rst_n)begin
        if(!pixel_fifo_rst_n)begin head<=0;tail<=0;head_valid<=0;tail_valid<=0;read_pending<=0;end
        else begin
            if(pop_read)read_pending<=1;
            if(consume_word)begin head<=tail;head_valid<=tail_valid;tail_valid<=0;end
            if(read_word_valid)begin
                read_pending<=0;
                if((!head_valid) || (consume_word && !tail_valid))begin head<=read_word;head_valid<=1;end
                else begin tail<=read_word;tail_valid<=1;end
            end
        end
    end
    wire write_full,write_overflow,write_word_valid;wire [127:0] write_word;
    wire [FIFO_BITS:0] write_used;
    wire push_write=write_pixel && writing && !pixel_bad && output_count<PIXELS && output_count[1:0]==3;
    wire pop_write;
    capture_stream_fifo #(.WIDTH(128),.ADDR_BITS(FIFO_BITS)) u_write_fifo(
        .i_rst_n(fifo_rst_n),.i_wr_clk(pixel_clk),.i_rd_clk(ddr_clk),
        .i_we(push_write),.i_data({new_state,pack}),.o_full(write_full),.o_overflow(write_overflow),.o_wr_used(),
        .i_re(pop_write),.o_data(write_word),.o_valid(write_word_valid),.o_used(write_used));
    always @(posedge pixel_clk or negedge pixel_reset_n)begin
        if(!pixel_reset_n)begin
            generation<=0;input_count<=0;output_count<=0;pixel_bad<=1;pixel_started<=0;
            session_seen<=0;have_job<=0;pack<=0;end_send<=0;end_data<=0;status<=0;fault_sync0<=0;fault_sync1<=0;
            read_bad_frames<=0;write_bad_frames<=0;read_bad_this_frame<=0;write_bad_this_frame<=0;
        end else begin
            fault_sync0<=ddr_fault_seen;fault_sync1<=fault_sync0;status[5]<=fault_sync1;
            end_send<=0;
            if(announce_update && pixel_announcement[3]==generation)session_seen<=1;
            if(start)begin
                have_job<=frame_id!=0;
                if(frame_id!=0)generation<=~generation;
                input_count<=0;output_count<=0;pixel_bad<=frame_id==0 || read_pixel || write_pixel;
                pixel_started<=0;session_seen<=0;status<={status[7:4],3'd0,frame_id!=0};
                read_bad_this_frame<=0;write_bad_this_frame<=0;
            end else begin
                if(read_pixel && have_job)begin
                    pixel_started<=1;
                    if(input_count<PIXELS)input_count<=input_count+1'b1;
                    if(!session_ok || (comparing && !head_valid) || input_count>=PIXELS)begin
                        pixel_bad<=1;status[7]<=1;
                        if(!read_bad_this_frame)read_bad_frames<=read_bad_frames+1'b1;
                        read_bad_this_frame<=1;
                    end
                    if(!pixel_started)status[3:1]<={writing,comparing,session_ok};
                end
                if(write_pixel && have_job)begin
                    if(output_count<PIXELS)output_count<=output_count+1'b1;
                    case(output_count[1:0])
                        0:pack[31:0]<=new_state;1:pack[63:32]<=new_state;2:pack[95:64]<=new_state;
                    endcase
                    if(!session_ok || (push_write && write_full))begin
                        pixel_bad<=1;status[6]<=1;
                        if(!write_bad_this_frame)write_bad_frames<=write_bad_frames+1'b1;
                        write_bad_this_frame<=1;
                    end
                    if(output_count==PIXELS-1)begin
                        end_send<=1;end_data<={generation,pixel_bad || !session_ok ||
                            (push_write && write_full) || (input_count+read_pixel)!=PIXELS};
                    end
                end
            end
        end
    end

    // Break the Gray-decode/subtract/credit/arbiter path before the DDR port.
    // A one-cycle stale credit can admit at most one extra beat: seven of the
    // eight guard slots remain. Returning a read moves, rather than adds, credit.
    reg read_room,write_pressure;
    reg [24:0] read_addr,write_addr;
    wire want_read=state==RUN && compare_frame && read_count<BEATS && read_room && !bad_frame;
    wire want_write=state==RUN && write_frame && held_valid && !bad_frame;
    wire choose_write=want_write && (!want_read || fair || write_pressure);
    assign wr=ready && choose_write;
    assign rd=ready && want_read && !choose_write;
    assign addr=choose_write?write_addr:read_addr;
    assign data=held_write;
    assign pop_write=state==RUN && write_frame && !held_valid && !write_pending && write_used!=0 && !bad_frame;
    always @(posedge ddr_clk or negedge ddr_reset_n)begin
        if(!ddr_reset_n)begin
            state<=IDLE;reset_fifos<=1;reset_count<=0;job_generation<=0;
            compare_frame<=0;write_frame<=0;read_bank<=0;write_bank<=1;
            last_good<=0;last_bank<=0;last_id<=0;last_config<=0;epoch_id<=0;epoch_config<=0;
            epoch_good<=0;epoch_compare<=0;epoch_read_bank<=0;active_id<=0;active_config<=0;
            read_count<=0;write_count<=0;outstanding<=0;end_seen<=0;bad_frame<=0;fair<=0;
            read_room<=0;write_pressure<=0;read_addr<=BASE0;write_addr<=BASE1;
            held_write<=0;held_valid<=0;write_pending<=0;announce<=0;announcement<=0;watchdog<=0;ddr_fault_seen<=0;
            committed_frames<=0;aborted_frames<=0;last_cause<=0;frame_cause<=0;abort_snapshot<=0;
        end else begin
            announce<=0;
            if(state==RUN && !bad_frame && !job_update)begin
                read_room<=({1'b0,read_wr_used}+{1'b0,outstanding})<DEPTH-8;
                write_pressure<=write_used>DEPTH/2;
            end else begin read_room<=0;write_pressure<=0;end
            case({rd,rd_valid})
                2'b10:outstanding<=outstanding+1'b1;
                2'b01:if(outstanding!=0)outstanding<=outstanding-1'b1;else begin bad_frame<=1;frame_cause[0]<=1;end
            endcase
            if(rd)begin read_count<=read_count+1'b1;read_addr<=read_addr+25'd8;fair<=1;end
            if(wr)begin write_count<=write_count+1'b1;write_addr<=write_addr+25'd8;held_valid<=0;fair<=0;end
            if(pop_write)write_pending<=1;
            if(write_word_valid)begin held_write<=write_word;held_valid<=1;write_pending<=0;end
            if(end_update && end_message[1]==job_generation && state==RUN)begin
                end_seen<=1;if(end_message[0])begin bad_frame<=1;frame_cause[1]<=1;end
            end
            if(state==RUN)begin
                watchdog<=watchdog+1'b1;
                if(i_fault || read_overflow || watchdog==23'h7ffffe)bad_frame<=1;
                if(i_fault)frame_cause[2]<=1;
                if(read_overflow)frame_cause[3]<=1;
                if(watchdog==23'h7ffffe)frame_cause[4]<=1;
                if(bad_frame)begin
                    state<=DRAIN;last_good<=0;epoch_good<=0;ddr_fault_seen<=1;
                    aborted_frames<=aborted_frames+1'b1;last_cause<=frame_cause;
                    abort_snapshot<={read_count,write_count,end_seen,compare_frame,write_frame,diagnostic_outstanding};
                end
                else if(end_seen && (!write_frame || write_count==BEATS) &&
                        (!compare_frame || read_count==BEATS) && outstanding==0)begin
                    state<=IDLE;epoch_good<=1;
                    committed_frames<=committed_frames+1'b1;
                    if(write_frame)begin last_good<=1;last_bank<=write_bank;last_id<=active_id;last_config<=active_config;end
                end
            end
            if(state==DRAIN && outstanding==0)begin state<=IDLE;reset_fifos<=1;end
            if(state==CLEAR)begin
                read_addr<=read_bank?BASE1:BASE0;
                write_addr<=write_bank?BASE1:BASE0;
                reset_count<=reset_count+1'b1;
                if(reset_count==15)reset_fifos<=0;
                if(reset_count==31)begin
                    state<=RUN;announce<=1;announcement<={job_generation,compare_frame,write_frame,1'b1};
                end
            end
            if(job_update)begin
                if(state==IDLE && outstanding==0)begin
                    state<=CLEAR;reset_fifos<=1;reset_count<=0;job_generation<=job[48];
                    active_id<=job[47:16];active_config<=job[15:0];
                    read_count<=0;write_count<=0;end_seen<=0;bad_frame<=0;watchdog<=0;
                    frame_cause<=0;
                    held_valid<=0;write_pending<=0;fair<=0;
                    if(epoch_good && job[47:16]==epoch_id && job[15:0]==epoch_config)begin
                        compare_frame<=epoch_compare;read_bank<=epoch_read_bank;write_frame<=0;
                    end else begin
                        compare_frame<=last_good && job[47:16]==last_id+1'b1 && job[15:0]==last_config && job[47:16]!=0;
                        read_bank<=last_bank;write_bank<=~last_bank;write_frame<=job[47:16]!=0;
                        epoch_compare<=last_good && job[47:16]==last_id+1'b1 && job[15:0]==last_config && job[47:16]!=0;
                        epoch_read_bank<=last_bank;
                    end
                    epoch_id<=job[47:16];epoch_config<=job[15:0];epoch_good<=0;
                end else begin
                    state<=DRAIN;bad_frame<=1;last_good<=0;epoch_good<=0;ddr_fault_seen<=1;
                    aborted_frames<=aborted_frames+1'b1;last_cause<=frame_cause | 8'h20;
                    abort_snapshot<={read_count,write_count,end_seen,compare_frame,write_frame,diagnostic_outstanding};
                end
            end
        end
    end
endmodule
