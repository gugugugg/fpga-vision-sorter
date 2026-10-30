`timescale 1ns/1ps
// 2048 consecutive CSI byte-clock samples, triggered by the next HS burst.
// RAM is immutable after completion until a subsequent accepted arm command.
module network_csi_trace #(
    parameter WAIT_CYCLES=11250000
)(
    input wire i_rst_n,i_net_clk,i_csi_clk,i_arm,
    output wire o_arm_ready,output reg o_valid,output reg o_busy,
    input wire [31:0] i_sample,
    input wire i_read,input wire [10:0] i_addr,output reg [31:0] o_data
);
    wire cmd_ready,cmd_valid;wire unused_cmd;
    reg pending,done_send,done_good;
    reg [1:0] state;
    wire done_ready,done_valid,done_result;
    assign o_arm_ready=cmd_ready && !pending;
    network_mailbox #(.WIDTH(1)) arm(
        .i_rst_n(i_rst_n),.i_src_clk(i_net_clk),.i_src_valid(i_arm && o_arm_ready),
        .i_src_data(1'b1),.o_src_ready(cmd_ready),.i_dst_clk(i_csi_clk),
        .o_dst_valid(cmd_valid),.o_dst_data(unused_cmd),.i_dst_ready(state==0));
    network_mailbox #(.WIDTH(1)) finish(
        .i_rst_n(i_rst_n),.i_src_clk(i_csi_clk),.i_src_valid(done_send),
        .i_src_data(done_good),.o_src_ready(done_ready),.i_dst_clk(i_net_clk),
        .o_dst_valid(done_valid),.o_dst_data(done_result),.i_dst_ready(1'b1));
    wire nr,cr;
    network_reset_sync rn(i_net_clk,i_rst_n,nr);
    network_reset_sync rc(i_csi_clk,i_rst_n,cr);
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [31:0] memory[0:2047];
    reg [10:0] address;reg [23:0] age;reg hs_d;
    // One write port, with no asynchronous reset on the RAM process.
    // Keeping RAM writes inside the resettable FSM prevents native ERAM inference.
    always @(posedge i_csi_clk)
        if(cr && (state==2 || (state==1 && i_sample[16] && !hs_d)))
            memory[address]<=i_sample;
    always @(posedge i_net_clk) if(i_read) o_data<=memory[i_addr];
    always @(posedge i_net_clk or negedge nr) begin
        if(!nr) begin pending<=0;o_valid<=0;o_busy<=0;end
        else begin
            if(i_arm && o_arm_ready) begin pending<=1;o_busy<=1;o_valid<=0;end
            if(done_valid) begin pending<=0;o_busy<=0;o_valid<=done_result;end
        end
    end
    always @(posedge i_csi_clk or negedge cr) begin
        if(!cr) begin state<=0;address<=0;age<=0;hs_d<=0;done_send<=0;done_good<=0;end
        else begin
            hs_d<=i_sample[16];
            if(done_send && done_ready) done_send<=0;
            case(state)
                0:if(cmd_valid) begin state<=1;age<=0;address<=0;end
                1:begin
                    if(i_sample[16] && !hs_d) begin
                        address<=1;state<=2;
                    end else if(age==WAIT_CYCLES-1) begin
                        done_send<=1;done_good<=0;state<=3;
                    end else age<=age+1'b1;
                end
                2:begin
                    if(address==2047) begin done_send<=1;done_good<=1;state<=3;end
                    else address<=address+1'b1;
                end
                3:if(!done_send && done_ready) state<=0;
            endcase
        end
    end
endmodule
