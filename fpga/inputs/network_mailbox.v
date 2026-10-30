`timescale 1ns/1ps
// Lossless one-entry bundled-data CDC. Common reset MUST reset both domains.
// Source holds payload until destination consumption has been acknowledged.
module network_mailbox #(parameter WIDTH=1)(
    input wire i_rst_n,i_src_clk,i_src_valid,
    input wire [WIDTH-1:0] i_src_data,output wire o_src_ready,
    input wire i_dst_clk,output reg o_dst_valid,
    output reg [WIDTH-1:0] o_dst_data,input wire i_dst_ready
);
    reg request,ack;reg [WIDTH-1:0] hold_data;
    wire src_rst_n,dst_rst_n;
    network_reset_sync sr(i_src_clk,i_rst_n,src_rst_n);
    network_reset_sync dr(i_dst_clk,i_rst_n,dst_rst_n);
    (* async_reg="true" *) reg ack0,ack1,req0,req1;
    assign o_src_ready=src_rst_n && request==ack1;
    always @(posedge i_src_clk or negedge src_rst_n) begin
        if(!src_rst_n) begin request<=0;hold_data<=0;ack0<=0;ack1<=0;end
        else begin
            ack0<=ack;ack1<=ack0;
            if(i_src_valid && o_src_ready) begin hold_data<=i_src_data;request<=~request;end
        end
    end
    always @(posedge i_dst_clk or negedge dst_rst_n) begin
        if(!dst_rst_n) begin ack<=0;req0<=0;req1<=0;o_dst_data<=0;o_dst_valid<=0;end
        else begin
            req0<=request;req1<=req0;
            if(!o_dst_valid && req1!=ack) begin
                // hold_data has been stable through two request synchronizers.
                o_dst_data<=hold_data;o_dst_valid<=1;
            end
            if(o_dst_valid && i_dst_ready) begin o_dst_valid<=0;ack<=req1;end
        end
    end
endmodule
