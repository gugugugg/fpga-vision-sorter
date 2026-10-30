`timescale 1ns/1ps
// DDR-domain identity cache for the four physical live frame buffers.
// A late detection result is accepted ONLY if its frame_id still owns a slot.
// Saved capture writes do not change the identity of live buffers.
module frame_metadata_binder #(
    parameter FRAME_BEATS=115200,PIPELINE_INPUT=0
)(
    input wire i_clk,i_rst_n,i_fault,
    input wire i_write,i_sof,
    input wire [24:0] i_addr,
    input wire [37:0] i_stream_meta,
    input wire i_meta_update,
    input wire [123:0] i_meta,
    input wire i_display_start,
    input wire [1:0] i_display_slot,
    output reg o_update,
    output reg [31:0] o_frame_id,
    output reg [91:0] o_meta
);
    // One coherent input stage removes the FIFO SOF/address/decode path from
    // the 133 MHz cache write enables. Display publication is one DDR tick later.
    reg [193:0] input_d;
    wire q_fault,q_write,q_sof,q_meta_update,q_display_start;
    wire [24:0] q_addr;wire [37:0] q_stream_meta;
    wire [123:0] q_meta;wire [1:0] q_display_slot;
    wire [193:0] input_bus={i_fault,i_write,i_sof,i_addr,i_stream_meta,
                           i_meta_update,i_meta,i_display_start,i_display_slot};
    always @(posedge i_clk or negedge i_rst_n)
      if(!i_rst_n) input_d<=0;else input_d<=input_bus;
    assign {q_fault,q_write,q_sof,q_addr,q_stream_meta,q_meta_update,q_meta,
            q_display_start,q_display_slot}=PIPELINE_INPUT?input_d:input_bus;
    reg [31:0] id[0:3];
    reg [91:0] meta[0:3];
    reg [3:0] complete,result_valid;
    reg [1:0] write_slot;
    reg [16:0] count;
    wire live=q_addr<22000000;
    wire [1:0] slot=q_addr>=21000000?3:q_addr>=14000000?2:q_addr>=7000000?1:0;
    integer k;
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            complete<=0;result_valid<=0;write_slot<=0;count<=0;o_update<=0;o_meta<=0;o_frame_id<=0;
            for(k=0;k<4;k=k+1) begin id[k]<=0;meta[k]<=0;end
        end else begin
            o_update<=0;
            if(q_write && live) begin
                if(q_sof) begin
                    id[slot]<=q_stream_meta[37:6];
                    meta[slot]<={q_stream_meta[5:0],86'd0};
                    complete[slot]<=FRAME_BEATS==1;result_valid[slot]<=0;
                    write_slot<=slot;count<=1;
                end else if(count<FRAME_BEATS) begin
                    count<=count+1'b1;
                    if(count==FRAME_BEATS-1) complete[write_slot]<=1;
                end else complete[write_slot]<=0;
            end
            if(q_meta_update) for(k=0;k<4;k=k+1) begin
                if(id[k]==q_meta[123:92] && id[k]!=0 &&
                   !(q_write && live && q_sof && slot==k)) begin
                    meta[k]<=q_meta[91:0];result_valid[k]<=1;
                end
            end
            if(q_fault) begin complete<=0;result_valid<=0;end
            if(q_display_start) begin
                o_update<=1;
                o_frame_id<=complete[q_display_slot] && !q_fault?id[q_display_slot]:0;
                // Keep the recorded mode but hide stale AE/AWB/target state
                // when a matching result or a complete frame is absent.
                o_meta<=complete[q_display_slot] && result_valid[q_display_slot] && !q_fault &&
                       !(q_write && live && q_sof && slot==q_display_slot)
                       ?meta[q_display_slot]:{meta[q_display_slot][91:86],86'd0};
            end
        end
    end
endmodule
