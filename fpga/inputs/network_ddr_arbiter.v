`timescale 1ns/1ps
// Insert AFTER board_capture_export, BEFORE mc_to_user_interface. Legacy reads
// include HDMI and K4 export; their order and return destination are preserved.
// i_legacy_reserved covers live write bursts and K4 export. Network requests
// are held until granted; a legacy burst's ready remains uninterrupted.
module network_ddr_arbiter #(
    parameter TAG_BITS=9,
    parameter [24:0] NET_BASE=25'd32000000,
    parameter [24:0] NET_END=25'd32921600,
    // Only producers whose strobes already include o_net_ready may enable this.
    // Raw held requests require the default grant gate below.
    parameter NET_STROBES_ACCEPTED=0
)(
    input wire i_clk,i_rst_n,
    input wire i_legacy_wr,i_legacy_rd,i_legacy_reserved,
    input wire [24:0] i_legacy_addr,input wire [127:0] i_legacy_data,
    output wire o_legacy_ready,o_legacy_rd_valid,
    output wire [127:0] o_legacy_rd_data,
    input wire i_net_wr,i_net_rd,input wire [24:0] i_net_addr,
    input wire [127:0] i_net_data,output wire o_net_ready,o_net_rd_valid,
    output wire [127:0] o_net_rd_data,
    output wire o_wr,o_rd,output wire [24:0] o_addr,
    output wire [127:0] o_data,input wire i_ready,i_rd_valid,
    input wire [127:0] i_rd_data,
    output reg o_fault,output wire [TAG_BITS:0] o_outstanding
);
    localparam DEPTH=1<<TAG_BITS;
    reg tags[0:DEPTH-1];reg [TAG_BITS-1:0] wp,rp;
    reg [TAG_BITS:0] count;
    wire room=count<DEPTH;
    wire legacy=i_legacy_wr || i_legacy_rd || i_legacy_reserved;
    wire net_addr_ok=i_net_addr>=NET_BASE && i_net_addr<NET_END && i_net_addr[2:0]==0;
    wire choose_net=!legacy && net_addr_ok && !o_fault;
    assign o_legacy_ready=i_ready && room && !o_fault;
    assign o_net_ready=i_ready && room && choose_net;
    // board_frame_writer emits registered FIFO tail words after ready falls.
    // mc_to_user_interface reserves eight slots for those already issued words.
    // Forward them regardless of current ready; never silently drop camera data.
    wire legacy_take=i_legacy_wr || (i_legacy_rd && room && !o_fault);
    wire net_take=(i_net_wr || i_net_rd) && o_net_ready;
    generate if(NET_STROBES_ACCEPTED) begin: accepted_strobes
        // Capture emits logical-AND accepted strobes (0/1/X, never Z).
        // With N=ready&&intent, net_take?N:0 equals N even for X inputs.
        // Keep the legacy ternary and the original owner/fault ledger intact.
        assign o_wr=legacy_take?i_legacy_wr:i_net_wr;
        assign o_rd=legacy_take?i_legacy_rd:i_net_rd;
    end else begin: raw_strobes
        assign o_wr=legacy_take?i_legacy_wr:net_take?i_net_wr:1'b0;
        assign o_rd=legacy_take?i_legacy_rd:net_take?i_net_rd:1'b0;
    end endgenerate
    assign o_addr=legacy_take?i_legacy_addr:i_net_addr;
    assign o_data=legacy_take?i_legacy_data:i_net_data;
    wire return_ok=i_rd_valid && count!=0;
    assign o_legacy_rd_valid=return_ok && !tags[rp];
    assign o_net_rd_valid=return_ok && tags[rp];
    assign o_legacy_rd_data=i_rd_data;assign o_net_rd_data=i_rd_data;
    assign o_outstanding=count;
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin wp<=0;rp<=0;count<=0;o_fault<=0;end
        else begin
            if((i_rd_valid && count==0) ||
               (legacy_take && i_legacy_rd && i_legacy_wr) ||
               (net_take && i_net_rd && i_net_wr)) o_fault<=1;
            if(o_rd) begin tags[wp]<=net_take;wp<=wp+1'b1;end
            if(return_ok) rp<=rp+1'b1;
            case({o_rd,return_ok})
                2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;
            endcase
        end
    end
endmodule
