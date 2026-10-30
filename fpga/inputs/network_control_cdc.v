`timescale 1ns/1ps
// Bridge proposal for B: pulse SYS inputs only at an explicit frame boundary.
// Actual SYS state is sampled on the FOLLOWING edge; never request echo.
module network_control_cdc(
    input wire i_rst_n,i_net_clk,i_sys_clk,
    input wire i_valid,i_threshold,i_net_enable,input wire [15:0] i_value,
    output wire o_ready,output wire o_done,o_error,
    output wire [231:0] o_status,
    input wire i_sys_frame_boundary,input wire [231:0] i_sys_status,
    input wire i_remote_rejected,
    output wire o_remote_mode_valid,output wire [5:0] o_remote_mode,
    output wire o_remote_threshold_valid,output wire [9:0] o_remote_threshold
);
    wire command_valid,command_threshold;wire [16:0] command;
    wire net_rst_n,sys_rst_n;
    network_reset_sync nr(i_net_clk,i_rst_n,net_rst_n);
    network_reset_sync sr(i_sys_clk,i_rst_n,sys_rst_n);
    reg [1:0] state;
    wire response_ready;
    wire cmd_ready;reg net_pending;
    (* async_reg="true" *) reg enable_sys0,enable_sys1,idle_net0,idle_net1;
    assign o_ready=net_rst_n && i_net_enable && cmd_ready && !net_pending && idle_net1;
    wire accept=sys_rst_n && state==0 && command_valid && (i_sys_frame_boundary || !enable_sys1) && response_ready;
    assign command_threshold=command[16];
    assign o_remote_mode_valid=accept && enable_sys1 && !command_threshold;
    assign o_remote_threshold_valid=accept && enable_sys1 && command_threshold;
    assign o_remote_mode=command[5:0];assign o_remote_threshold=command[9:0];
    network_mailbox #(.WIDTH(17)) cmd(
        .i_rst_n(i_rst_n),.i_src_clk(i_net_clk),.i_src_valid(i_valid && o_ready),
        .i_src_data({i_threshold,i_value}),.o_src_ready(cmd_ready),
        .i_dst_clk(i_sys_clk),.o_dst_valid(command_valid),.o_dst_data(command),.i_dst_ready(accept));
    // Share one status mailbox between periodic snapshots and command results.
    // Stop snapshots as soon as a command arrives. Accept only after the last
    // snapshot has been consumed, so it cannot overwrite a later result.
    wire result_valid;wire [233:0] result;
    wire status_send=state==1 || (state==0 && !command_valid);
    wire [233:0] status_payload={state==1,
        state==1 && (i_remote_rejected || !enable_sys1),i_sys_status};
    network_mailbox #(.WIDTH(234)) resp(
        .i_rst_n(i_rst_n),.i_src_clk(i_sys_clk),.i_src_valid(status_send),
        .i_src_data(status_payload),.o_src_ready(response_ready),
        .i_dst_clk(i_net_clk),.o_dst_valid(result_valid),.o_dst_data(result),.i_dst_ready(1'b1));
    wire command_result=result_valid && result[233];
    assign o_done=net_rst_n && command_result && i_net_enable;assign o_error=result[232];
    // The destination mailbox keeps the last coherent word after valid drops.
    assign o_status=result[231:0];
    always @(posedge i_net_clk or negedge net_rst_n) begin
        if(!net_rst_n) begin net_pending<=0;idle_net0<=0;idle_net1<=0;end
        else begin
            idle_net0<=state==0;idle_net1<=idle_net0;
            if(i_valid && o_ready) net_pending<=1;
            if(command_result) net_pending<=0;
        end
    end
    always @(posedge i_sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            state<=0;enable_sys0<=0;enable_sys1<=0;
        end
        else begin
            enable_sys0<=i_net_enable;enable_sys1<=enable_sys0;
            case(state)
                0:if(accept) state<=1;
                1:begin
                    // B controller consumed the pulse at the previous edge.
                    if(response_ready) state<=2;
                end
                2:if(response_ready) state<=0;
            endcase
        end
    end
endmodule
