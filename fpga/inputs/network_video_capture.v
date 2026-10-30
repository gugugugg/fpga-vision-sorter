`timescale 1ns/1ps
// Real HDMI pixel input -> CDC packed FIFO -> frozen DDR -> verified byte stream.
// DDR is one ordered request/response interface; reads have ONE outstanding.
// Address unit is 2 bytes. No live/K4 memory is touched. See PORTS.md.
module network_video_capture #(
    parameter WIDTH=1024,HEIGHT=600,FIFO_BITS=10,
    parameter [24:0] BASE=25'd32000000,
    parameter TIMEOUT_CYCLES=125000000
)(
    input wire i_rst_n,i_net_clk,i_pixel_clk,i_ddr_clk,
    input wire i_capture,i_original,i_net_enable,output wire o_capture_ready,
    input wire i_sof,i_frame_end,i_pixel_valid,
    input wire [23:0] i_rgb,input wire [5:0] i_display_mode,
    input wire i_source_ok,
    output wire o_frame_valid,output wire o_frame_original,
    output wire [5:0] o_frame_mode,
    output wire [31:0] o_frame_id,o_frame_crc,o_frame_bytes,
    output wire [15:0] o_frame_width,o_frame_height,
    input wire i_release,output wire o_video_valid,
    output wire [7:0] o_video_data,input wire i_video_ready,
    output reg o_video_error,
    output wire o_wr,o_rd,output wire [24:0] o_addr,
    output wire [127:0] o_wr_data,input wire i_ddr_ready,i_rd_valid,
    input wire [127:0] i_rd_data,
    output wire o_busy,output reg [31:0] o_failed
);
    wire net_rst_n,pixel_rst_n,ddr_rst_n;
    network_reset_sync nr(i_net_clk,i_rst_n,net_rst_n);
    network_reset_sync pr(i_pixel_clk,i_rst_n,pixel_rst_n);
    network_reset_sync dr(i_ddr_clk,i_rst_n,ddr_rst_n);
    `include "network_crc.vh"
    localparam [15:0] FULL_W=WIDTH,FULL_H=HEIGHT,PRE_W=WIDTH/2,PRE_H=HEIGHT/2;
    wire job_valid,job_original,job_ready;
    reg job_active;
    wire job_src_ready;
    reg net_busy;
    (* async_reg="true" *) reg idle_net0,idle_net1,enable_ddr0,enable_ddr1,enable_pixel0,enable_pixel1;
    wire ddr_idle;
    assign o_capture_ready=net_rst_n && i_net_enable && job_src_ready && !net_busy && idle_net1;
    network_mailbox #(.WIDTH(1)) job(
        .i_rst_n(i_rst_n),.i_src_clk(i_net_clk),.i_src_valid(i_capture && o_capture_ready),
        .i_src_data(i_original),.o_src_ready(job_src_ready),.i_dst_clk(i_ddr_clk),
        .o_dst_valid(job_valid),.o_dst_data(job_original),.i_dst_ready(job_ready || !enable_ddr1));
    wire arm_ready,arm_valid,arm_original;
    reg arm_send,profile;
    localparam P_IDLE=0,P_ARM=1,P_PIXELS=2,P_FINAL=3,P_DONE=4;
    reg [2:0] pstate;reg poriginal,pbad;
    network_mailbox #(.WIDTH(1)) arm(
        .i_rst_n(i_rst_n),.i_src_clk(i_ddr_clk),.i_src_valid(arm_send),
        .i_src_data(profile),.o_src_ready(arm_ready),.i_dst_clk(i_pixel_clk),
        .o_dst_valid(arm_valid),.o_dst_data(arm_original),.i_dst_ready(pstate==P_IDLE));
    reg [31:0] pixel_count,pbytes,pwords,pcrc,pwatch;
    reg [15:0] x,y;reg [5:0] pmode;
    // High-aligned uncommitted bytes; unused byte lanes are always zero.
    // The old accumulator's low 'used' bytes are the only live suffix.
    reg [119:0] partial;reg [4:0] used;
    wire selected=poriginal || (!x[0] && !y[0]);
    wire [15:0] rgb565={i_rgb[23:19],i_rgb[15:10],i_rgb[7:3]};
    wire [2:0] bpp=poriginal?3:2;
    // Both profiles share the first two CRC bytes. P_IDLE seeds pcrc before
    // P_ARM, so the first pixel uses the same chain as subsequent pixels.
    wire [7:0] crc_first=poriginal?i_rgb[23:16]:rgb565[15:8];
    wire [7:0] crc_second=poriginal?i_rgb[15:8]:rgb565[7:0];
    wire [31:0] crc_two=crc32_byte(crc32_byte(pcrc,crc_first),crc_second);
    wire [31:0] pixel_crc_next=poriginal?crc32_byte(crc_two,i_rgb[7:0]):crc_two;
    // The old append OR converts Z bits to X; retain that four-state behavior.
    wire [119:0] partial_bits=partial | 120'b0;
    wire [23:0] rgb_bits=i_rgb | 24'b0;
    wire [15:0] rgb565_bits=rgb565 | 16'b0;
    reg [119:0] partial_next;
    reg [127:0] pixel_word,tail_word;
    always @* begin
        // Unknown phase/profile remains unknown in four-state simulation.
        // Non-emitting pixel_word is not sampled by the FIFO.
        partial_next=120'bx;pixel_word=128'bx;
        case(poriginal)
            1'b1:case(used)
                5'd0:partial_next={rgb_bits,96'b0};
                5'd1:partial_next={partial_bits[119:112],rgb_bits,88'b0};
                5'd2:partial_next={partial_bits[119:104],rgb_bits,80'b0};
                5'd3:partial_next={partial_bits[119:96],rgb_bits,72'b0};
                5'd4:partial_next={partial_bits[119:88],rgb_bits,64'b0};
                5'd5:partial_next={partial_bits[119:80],rgb_bits,56'b0};
                5'd6:partial_next={partial_bits[119:72],rgb_bits,48'b0};
                5'd7:partial_next={partial_bits[119:64],rgb_bits,40'b0};
                5'd8:partial_next={partial_bits[119:56],rgb_bits,32'b0};
                5'd9:partial_next={partial_bits[119:48],rgb_bits,24'b0};
                5'd10:partial_next={partial_bits[119:40],rgb_bits,16'b0};
                5'd11:partial_next={partial_bits[119:32],rgb_bits,8'b0};
                5'd12:partial_next={partial_bits[119:24],rgb_bits};
                5'd13:begin pixel_word={partial_bits[119:16],rgb_bits};partial_next=0;end
                5'd14:begin pixel_word={partial_bits[119:8],rgb_bits[23:8]};partial_next={rgb_bits[7:0],112'b0};end
                5'd15:begin pixel_word={partial_bits,rgb_bits[23:16]};partial_next={rgb_bits[15:0],104'b0};end
                default:begin end
            endcase
            1'b0:case(used)
                5'd0:partial_next={rgb565_bits,104'b0};
                5'd2:partial_next={partial_bits[119:104],rgb565_bits,88'b0};
                5'd4:partial_next={partial_bits[119:88],rgb565_bits,72'b0};
                5'd6:partial_next={partial_bits[119:72],rgb565_bits,56'b0};
                5'd8:partial_next={partial_bits[119:56],rgb565_bits,40'b0};
                5'd10:partial_next={partial_bits[119:40],rgb565_bits,24'b0};
                5'd12:partial_next={partial_bits[119:24],rgb565_bits,8'b0};
                5'd14:begin pixel_word={partial_bits[119:8],rgb565_bits};partial_next=0;end
                default:begin end
            endcase
            default:begin end
        endcase
        // A variable shift with an unknown occupancy produced all X before.
        // Do not let zero padding silently turn those bits into known values.
        tail_word=128'bx;
        case(used)
            5'd1,5'd2,5'd3,5'd4,5'd5,5'd6,5'd7,5'd8,
            5'd9,5'd10,5'd11,5'd12,5'd13,5'd14,5'd15:tail_word={partial,8'b0};
            default:begin end
        endcase
    end
    wire [5:0] appended_used=used+bpp;
    wire emit_pixel=pstate==P_PIXELS && i_pixel_valid && selected && !pbad && appended_used>=16;
    wire emit_final=pstate==P_FINAL && used!=0 && !pbad;
    wire fifo_we=pixel_rst_n && (emit_pixel || emit_final);
    wire [127:0] fifo_in=emit_final?tail_word:pixel_word;
    wire fifo_full,fifo_overflow,fifo_valid;
    wire [127:0] fifo_data;wire [FIFO_BITS:0] fifo_used;
    wire fifo_re;
    capture_stream_fifo #(.WIDTH(128),.ADDR_BITS(FIFO_BITS)) pixels(
        .i_rst_n(i_rst_n),.i_wr_clk(i_pixel_clk),.i_rd_clk(i_ddr_clk),
        .i_we(fifo_we),.i_data(fifo_in),.o_full(fifo_full),.o_overflow(fifo_overflow),
        .i_re(fifo_re),.o_data(fifo_data),.o_valid(fifo_valid),.o_used(fifo_used));
    reg completion_send;wire completion_ready,completion_valid;
    reg [103:0] completion_data;wire [103:0] completion;
    network_mailbox #(.WIDTH(104)) completed(
        .i_rst_n(i_rst_n),.i_src_clk(i_pixel_clk),.i_src_valid(completion_send),
        .i_src_data(completion_data),.o_src_ready(completion_ready),.i_dst_clk(i_ddr_clk),
        .o_dst_valid(completion_valid),.o_dst_data(completion),.i_dst_ready(1'b1));
    always @(posedge i_pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin enable_pixel0<=0;enable_pixel1<=0;end
        else begin enable_pixel0<=i_net_enable;enable_pixel1<=enable_pixel0;end
    end
    always @(posedge i_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin enable_ddr0<=0;enable_ddr1<=0;end
        else begin enable_ddr0<=i_net_enable;enable_ddr1<=enable_ddr0;end
    end
    always @(posedge i_pixel_clk or negedge pixel_rst_n) begin
        if(!pixel_rst_n) begin
            pstate<=P_IDLE;poriginal<=0;pbad<=0;pixel_count<=0;pbytes<=0;pwords<=0;
            pcrc<=32'hffffffff;pwatch<=0;x<=0;y<=0;pmode<=0;partial<=0;used<=0;
            completion_send<=0;completion_data<=0;
        end else begin
            if(completion_send && completion_ready) completion_send<=0;
            if(pstate==P_IDLE || (pstate==P_PIXELS && i_pixel_valid)) pwatch<=0;
            else pwatch<=pwatch+1'b1;
            if((pstate==P_ARM || pstate==P_PIXELS) &&
               (!enable_pixel1 || pwatch==TIMEOUT_CYCLES-1)) begin
                pbad<=1;pstate<=P_FINAL;
            end else case(pstate)
                P_IDLE:if(arm_valid) begin
                    poriginal<=arm_original;pstate<=P_ARM;pbad<=0;pixel_count<=0;
                    pbytes<=0;pwords<=0;pcrc<=32'hffffffff;used<=0;partial<=0;x<=0;y<=0;
                end
                P_ARM:if(i_sof) begin
                    // First pixel is consumed on this edge, never discarded.
                    pmode<=i_display_mode;pstate<=P_PIXELS;
                    if(!i_pixel_valid || !i_source_ok) pbad<=1;
                    else begin
                        pixel_count<=1;x<=WIDTH==1?0:1;y<=WIDTH==1?1:0;
                        pbytes<=poriginal?3:2;used<=poriginal?3:2;
                        partial<=poriginal?{i_rgb,96'b0}:{rgb565,104'b0};
                        pcrc<=pixel_crc_next;
                    end
                end
                P_PIXELS:begin
                    // B may update the following frame's mode during blanking.
                    // Every active input pixel is checked, including preview's
                    // unselected coordinates; blanking carries no pixel identity.
                    if(!i_source_ok || (i_pixel_valid && i_display_mode!=pmode) || i_sof) pbad<=1;
                    if(i_pixel_valid) begin
                        pixel_count<=pixel_count+1'b1;
                        if(pixel_count>=WIDTH*HEIGHT) pbad<=1;
                        if(x==WIDTH-1) begin x<=0;y<=y+1'b1;end else x<=x+1'b1;
                        if(selected && !pbad) begin
                            partial<=partial_next;pbytes<=pbytes+bpp;
                            pcrc<=pixel_crc_next;
                            if(appended_used>=16) begin
                                used<=appended_used-16;
                                if(fifo_full) pbad<=1;else pwords<=pwords+1'b1;
                            end else used<=appended_used;
                        end
                    end
                    if(i_frame_end) begin
                        pstate<=P_FINAL;
                        if(i_pixel_valid || pixel_count!=WIDTH*HEIGHT) pbad<=1;
                    end
                end
                P_FINAL:begin
                    if(emit_final) begin if(fifo_full) pbad<=1;else pwords<=pwords+1'b1;end
                    used<=0;pstate<=P_DONE;
                end
                P_DONE:if(completion_ready && !completion_send) begin
                    completion_data<={poriginal,!pbad,pmode,~pcrc,pbytes,pwords};
                    completion_send<=1;pstate<=P_IDLE;
                end
            endcase
        end
    end
    localparam D_IDLE=0,D_CAPTURE=1,D_VERIFY_REQ=2,D_VERIFY_WAIT=3,D_VERIFY_BYTES=4,
               D_PUBLISH=5,D_STREAM_REQ=6,D_STREAM_WAIT=7,D_STREAM_SEND=8,
               D_RELEASE=9,D_FAIL=10,D_QUARANTINE=11;
    reg [3:0] ds;reg [31:0] writes,read_bytes,read_words,verify_crc,expected_crc,total_bytes;
    reg [31:0] expected_words,frame_id,watchdog;
    reg watchdog_terminal;
    reg read_pending;
    wire cancel_transfer=ds!=D_IDLE && ds!=D_RELEASE && ds!=D_FAIL && ds!=D_QUARANTINE &&
        (!enable_ddr1 || watchdog_terminal);
    wire read_return=read_pending && i_rd_valid;
    reg [5:0] mode;reg got_completion,completion_ok;
    reg pending_fifo,held_valid;reg [127:0] held,read_word;
    reg [4:0] byte_index;reg [24:0] addr;
    wire desc_ready;reg desc_send;reg [174:0] descriptor;wire [174:0] net_descriptor;
    wire raw_frame_valid;
    reg [31:0] net_offset;reg net_discard;
    // original(1),mode(6),id/crc/bytes(96),width/height(32), reserved(40).
    network_mailbox #(.WIDTH(175)) desc(
        .i_rst_n(i_rst_n),.i_src_clk(i_ddr_clk),.i_src_valid(desc_send),
        .i_src_data(descriptor),.o_src_ready(desc_ready),.i_dst_clk(i_net_clk),
        .o_dst_valid(raw_frame_valid),.o_dst_data(net_descriptor),
        .i_dst_ready(i_release || !i_net_enable || net_discard));
    assign {o_frame_original,o_frame_mode,o_frame_id,o_frame_crc,o_frame_bytes,
            o_frame_width,o_frame_height}=net_descriptor[174:40];
    wire word_ready,word_valid;reg word_send;wire [127:0] net_word;
    reg [4:0] net_byte;
    assign o_frame_valid=net_rst_n && raw_frame_valid && !net_discard && i_net_enable;
    wire consume_word=(o_video_valid && i_video_ready &&
        (net_byte==15 || net_offset==o_frame_bytes-1)) || (word_valid && net_discard);
    network_mailbox #(.WIDTH(128)) stream_word(
        .i_rst_n(i_rst_n),.i_src_clk(i_ddr_clk),.i_src_valid(word_send),
        .i_src_data(read_word),.o_src_ready(word_ready),.i_dst_clk(i_net_clk),
        .o_dst_valid(word_valid),.o_dst_data(net_word),.i_dst_ready(consume_word));
    assign o_video_valid=word_valid && o_frame_valid && !net_discard;
    assign o_video_data=net_word[(15-net_byte)*8+:8];
    // Error completion is also delivered reliably; source retains it until ACK.
    reg error_send;wire error_ready,error_valid;
    network_mailbox #(.WIDTH(1)) fault(
        .i_rst_n(i_rst_n),.i_src_clk(i_ddr_clk),.i_src_valid(error_send),
        .i_src_data(1'b1),.o_src_ready(error_ready),.i_dst_clk(i_net_clk),
        .o_dst_valid(error_valid),.o_dst_data(),.i_dst_ready(1'b1));
    always @(posedge i_net_clk or negedge net_rst_n) begin
        if(!net_rst_n) begin
            net_busy<=0;net_byte<=0;net_offset<=0;net_discard<=1;o_video_error<=0;idle_net0<=0;idle_net1<=0;
        end
        else begin
            idle_net0<=ddr_idle;idle_net1<=idle_net0;
            o_video_error<=error_valid;
            if(i_capture && o_capture_ready) begin net_busy<=1;net_byte<=0;net_offset<=0;net_discard<=0;end
            if(consume_word) net_byte<=0;
            else if(o_video_valid && i_video_ready) net_byte<=net_byte+1'b1;
            if(o_video_valid && i_video_ready) net_offset<=net_offset+1'b1;
            if(i_release || error_valid || !i_net_enable) begin net_discard<=1;end
            // Wait for the descriptor/stream/error mailboxes to drain before rearming.
            if(net_discard && !raw_frame_valid && !word_valid && job_src_ready && idle_net1) net_busy<=0;
        end
    end
    assign ddr_idle=ds==D_IDLE && !read_pending && !error_send && error_ready && desc_ready && word_ready;
    assign job_ready=ddr_rst_n && ddr_idle && enable_ddr1 && fifo_used==0 && !pending_fifo && !held_valid && arm_ready;
    assign o_busy=ds!=D_IDLE;
    assign fifo_re=ddr_rst_n && (ds==D_CAPTURE || ds==D_FAIL) && !pending_fifo && !held_valid && fifo_used!=0;
    assign o_wr=ddr_rst_n && ds==D_CAPTURE && held_valid && i_ddr_ready;
    // The request strobe is the accepted transaction, not merely an FSM intent.
    // Cancel/deadline and descriptor release win BEFORE any new read is issued.
    assign o_rd=ddr_rst_n && (ds==D_VERIFY_REQ || (ds==D_STREAM_REQ && !desc_ready)) &&
        i_ddr_ready && !read_pending && !cancel_transfer;
    // Address is registered before READY; only an accepted write advances it.
    // The read passes reuse this same register with their existing sequencing.
    assign o_addr=addr;
    assign o_wr_data=held;
    // Match the counter's actual 32-bit next value before comparing with the
    // full parameter expression. This preserves wrap and wide/signed limits;
    // the terminal decode ends at this register, not at the DDR request FIFO.
    wire [31:0] watchdog_incremented=watchdog+32'd1;
    // Idle count is not published or consumed. Clear throughout D_IDLE,
    // rather than putting the cross-domain mailbox/availability decode on
    // the counter's clock-enable path when a job is accepted. The first
    // D_CAPTURE cycle still starts at zero; accepted writes increment once.
    always @(posedge i_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) writes<=0;
        else if(ds==D_IDLE) writes<=0;
        else if(o_wr) writes<=writes+1'b1;
    end
    always @(posedge i_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin
            ds<=D_IDLE;job_active<=0;arm_send<=0;profile<=0;read_bytes<=0;read_words<=0;
            verify_crc<=32'hffffffff;expected_crc<=0;total_bytes<=0;expected_words<=0;
            frame_id<=0;watchdog<=0;read_pending<=0;mode<=0;got_completion<=0;completion_ok<=0;
            watchdog_terminal<=(32'd0==TIMEOUT_CYCLES-1);
            pending_fifo<=0;held_valid<=0;held<=0;read_word<=0;byte_index<=0;addr<=BASE;
            desc_send<=0;descriptor<=0;word_send<=0;error_send<=0;o_failed<=0;
        end else begin
            // This ledger also runs on cancel/timeout edges. In particular an
            // owned return arriving on that edge is consumed exactly once.
            case({o_rd,read_return})
                2'b10:read_pending<=1;
                2'b01:read_pending<=0;
            endcase
            if(arm_send && arm_ready) arm_send<=0;
            if(desc_send && desc_ready) desc_send<=0;
            if(word_send && word_ready) word_send<=0;
            if(error_send && error_ready) error_send<=0;
            if(fifo_re) pending_fifo<=1;
            if(fifo_valid) begin held<=fifo_data;held_valid<=1;pending_fifo<=0;end
            if(o_wr) begin held_valid<=0;addr<=addr+25'd8;end
            if(ds==D_FAIL) held_valid<=0;
            if(completion_valid) begin
                got_completion<=1;completion_ok<=completion[102];mode<=completion[101:96];
                expected_crc<=completion[95:64];total_bytes<=completion[63:32];expected_words<=completion[31:0];
            end
            if(ds==D_IDLE || o_wr || i_rd_valid || (ds==D_VERIFY_BYTES) ||
               (ds==D_STREAM_SEND && word_ready) || completion_valid) begin
                watchdog<=0;watchdog_terminal<=(32'd0==TIMEOUT_CYCLES-1);
            end else begin
                watchdog<=watchdog+1'b1;
                watchdog_terminal<=(watchdog_incremented==TIMEOUT_CYCLES-1);
            end
            if(cancel_transfer) begin
                o_failed<=o_failed+1'b1;error_send<=1;
                // Retain only a genuinely accepted, still-unreturned request.
                // A same-edge return is discarded as old data and permits drain;
                // it must not leave a quarantine waiting for a second response.
                if((read_pending || o_rd) && !read_return) ds<=D_QUARANTINE;
                else ds<=D_FAIL;
            end else case(ds)
                D_IDLE:if(job_valid && job_ready) begin
                    ds<=D_CAPTURE;job_active<=1;profile<=job_original;arm_send<=1;
                    addr<=BASE;got_completion<=0;completion_ok<=0;
                end
                D_CAPTURE:if(got_completion && fifo_used==0 && !pending_fifo && !held_valid && !fifo_valid) begin
                    if(completion_ok && writes==expected_words && total_bytes==
                       (profile?WIDTH*HEIGHT*3:(WIDTH/2)*(HEIGHT/2)*2) && writes==(total_bytes+15)/16) begin
                        addr<=BASE;read_bytes<=0;read_words<=0;verify_crc<=32'hffffffff;ds<=D_VERIFY_REQ;
                    end else begin ds<=D_FAIL;o_failed<=o_failed+1'b1;error_send<=1;end
                end
                D_VERIFY_REQ:if(o_rd) ds<=D_VERIFY_WAIT;
                D_VERIFY_WAIT:if(read_return) begin read_word<=i_rd_data;byte_index<=0;ds<=D_VERIFY_BYTES;end
                D_VERIFY_BYTES:begin
                    verify_crc<=crc32_byte(verify_crc,read_word[127:120]);
                    read_word<={read_word[119:0],8'd0};read_bytes<=read_bytes+1'b1;
                    if(read_bytes==total_bytes-1) begin
                        if(~crc32_byte(verify_crc,read_word[127:120])==expected_crc) begin
                            frame_id<=frame_id+1'b1;
                            descriptor<={profile,mode,frame_id+1'b1,expected_crc,total_bytes,
                                (profile?FULL_W:PRE_W),(profile?FULL_H:PRE_H),40'd0};
                            desc_send<=1;ds<=D_PUBLISH;addr<=BASE;read_words<=0;
                        end else begin ds<=D_FAIL;o_failed<=o_failed+1'b1;error_send<=1;end
                    end else if(byte_index==15) begin addr<=addr+8;read_words<=read_words+1'b1;ds<=D_VERIFY_REQ;end
                    else byte_index<=byte_index+1'b1;
                end
                D_PUBLISH:if(!desc_send && !desc_ready) ds<=D_STREAM_REQ;
                D_STREAM_REQ:if(desc_ready) ds<=D_RELEASE;else if(o_rd) ds<=D_STREAM_WAIT;
                D_STREAM_WAIT:if(read_return) begin read_word<=i_rd_data;word_send<=1;ds<=D_STREAM_SEND;end
                D_STREAM_SEND:if(!word_send && word_ready) begin
                    read_words<=read_words+1'b1;addr<=addr+8;
                    if(desc_ready || read_words==expected_words-1) ds<=D_RELEASE;else ds<=D_STREAM_REQ;
                end
                D_RELEASE:if(desc_ready && word_ready && !word_send) begin ds<=D_IDLE;job_active<=0;end
                D_FAIL:if(got_completion && fifo_used==0 && !held_valid && !pending_fifo && !fifo_valid &&
                          !read_pending && desc_ready && word_ready && !error_send) begin ds<=D_IDLE;job_active<=0;end
                // Recover ONLY after late response has been consumed. If never
                // returned, common reset is required, preventing owner aliasing.
                D_QUARANTINE:if(read_return) ds<=D_FAIL;
            endcase
        end
    end
    initial begin
        if(WIDTH%2!=0 || HEIGHT%2!=0 || WIDTH<2 || HEIGHT<2 ||
           BASE<32000000 || BASE+(WIDTH*HEIGHT*3+15)/16*8>33554432)
            $error("invalid network frozen region/dimensions");
    end
endmodule
