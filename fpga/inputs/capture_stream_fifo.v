`timescale 1ns/1ps
// Frame markers travel with their pixels; never reset the FIFO at SOF.
// Common asynchronous reset, synchronous RAM read, Gray-pointer CDC.
module capture_stream_fifo #(
    parameter WIDTH=129, parameter ADDR_BITS=10
)(
    input wire i_rst_n, i_wr_clk, i_rd_clk,
    input wire i_we, input wire [WIDTH-1:0] i_data,
    output wire o_full, output reg o_overflow,
    input wire i_re, output reg [WIDTH-1:0] o_data,
    output reg o_valid, output wire [ADDR_BITS:0] o_used,
    // Conservative write-domain occupancy, for reserving outstanding responses.
    output wire [ADDR_BITS:0] o_wr_used
);
    localparam DEPTH=1<<ADDR_BITS;
    wire wr_rst_n,rd_rst_n;
    reset_release wr_reset(i_wr_clk,i_rst_n,wr_rst_n);
    reset_release rd_reset(i_rd_clk,i_rst_n,rd_rst_n);
    (* ram_style="block", syn_ramstyle="block_ram" *)
    reg [WIDTH-1:0] mem[0:DEPTH-1];
    reg [ADDR_BITS:0] wp, rp, wg, rg;
    (* async_reg="true",syn_preserve=1 *) reg [ADDR_BITS:0] rg0,rg1,wg0,wg1;
    wire [ADDR_BITS:0] wp_next=wp+1'b1;
    wire [ADDR_BITS:0] wg_next=(wp_next>>1)^wp_next;
    // Current pointer comparison permits all DEPTH slots to be used.
    assign o_full=wg=={~rg1[ADDR_BITS:ADDR_BITS-1],rg1[ADDR_BITS-2:0]};
    function [ADDR_BITS:0] ungray;
        input [ADDR_BITS:0] g;
        integer n;
        begin
            ungray[ADDR_BITS]=g[ADDR_BITS];
            for(n=ADDR_BITS-1;n>=0;n=n-1)
                ungray[n]=ungray[n+1]^g[n];
        end
    endfunction
    assign o_used=ungray(wg1)-rp;
    assign o_wr_used=wp-ungray(rg1);
    always @(posedge i_wr_clk or negedge wr_rst_n) begin
        if(!wr_rst_n) begin
            wp<=0;wg<=0;rg0<=0;rg1<=0;o_overflow<=0;
        end else begin
            rg0<=rg;rg1<=rg0;
            if(i_we && o_full) o_overflow<=1;
            if(i_we && !o_full) begin wp<=wp_next;wg<=wg_next;end
        end
    end
    always @(posedge i_wr_clk)
        if(wr_rst_n && i_we && !o_full) mem[wp[ADDR_BITS-1:0]]<=i_data;
    wire take=rd_rst_n && i_re && wg1!=rg;
    always @(posedge i_rd_clk or negedge rd_rst_n) begin
        if(!rd_rst_n) begin rp<=0;rg<=0;wg0<=0;wg1<=0;o_valid<=0;end
        else begin
            wg0<=wg;wg1<=wg0;o_valid<=take;
            if(take) begin rp<=rp+1'b1;rg<=((rp+1'b1)>>1)^(rp+1'b1);end
        end
    end
    always @(posedge i_rd_clk)
        if(take) o_data<=mem[rp[ADDR_BITS-1:0]];
endmodule
