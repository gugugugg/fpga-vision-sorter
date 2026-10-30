`timescale 1ns/1ps
// Self-developed complete-frame DDR writer. Unlike the demo writer, SOF
// remains attached to the first beat and a short tail is drained as well.
module board_frame_writer #(
    parameter BURST=240, parameter FIFO_BITS=10, parameter META_BITS=38
)(
    input wire i_rst_n, i_camera_clk, i_camera_sof, i_camera_valid,
    input wire [127:0] i_camera_data,
    input wire [META_BITS-1:0] i_camera_meta,
    input wire i_ddr_clk, i_ready, i_read_busy, i_enable,
    output wire o_busy, o_write, o_sof,
    output wire [127:0] o_data, output wire [24:0] o_addr,
    output reg [1:0] o_display_rp, output wire o_overflow,
    output wire [META_BITS-1:0] o_meta
);
    wire [128+META_BITS:0] q;
    wire camera_rst_n,ddr_rst_n;
    reset_release camera_reset(i_camera_clk,i_rst_n,camera_rst_n);
    reset_release ddr_reset(i_ddr_clk,i_rst_n,ddr_rst_n);
    wire q_valid,full,overflow;
    wire [FIFO_BITS:0] used;
    reg active;
    reg [8:0] remaining;
    reg [1:0] wp;
    reg [24:0] next_addr;
    (* async_reg="true",syn_preserve=1 *) reg overflow0,overflow1;
    // DDR calibration can finish after the camera starts. Drop startup
    // partial frames instead of permanently overflowing the stream FIFO.
    (* async_reg="true",syn_preserve=1 *) reg enable0,enable1;
    reg stream_started;
    wire camera_accept=i_camera_valid && enable1 &&
                       (stream_started || i_camera_sof);
    always @(posedge i_camera_clk or negedge camera_rst_n) begin
        if(!camera_rst_n) begin enable0<=0;enable1<=0;stream_started<=0;end
        else begin
            enable0<=i_enable;enable1<=enable0;
            if(!enable1) stream_started<=0;
            else if(i_camera_valid && i_camera_sof) stream_started<=1;
        end
    end
    function [24:0] base;
        input [1:0] p;
        begin case(p)
            0:base=0;1:base=7000000;2:base=14000000;3:base=21000000;
        endcase end
    endfunction
    wire pop=active && i_ready;
    capture_stream_fifo #(.WIDTH(129+META_BITS),.ADDR_BITS(FIFO_BITS)) u_fifo(
        .i_rst_n(i_rst_n),.i_wr_clk(i_camera_clk),.i_rd_clk(i_ddr_clk),
        .i_we(camera_accept),.i_data({i_camera_meta,i_camera_sof,i_camera_data}),
        .o_full(full),.o_overflow(overflow),.i_re(pop),
        .o_data(q),.o_valid(q_valid),.o_used(used));
    assign o_write=q_valid;
    assign o_sof=q_valid && q[128];
    assign o_data=q[127:0];
    assign o_meta=q[128+META_BITS:129];
    assign o_addr=q[128]?base(wp):next_addr;
    assign o_busy=active | q_valid;
    assign o_overflow=overflow1;
    always @(posedge i_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin
            active<=0;remaining<=0;wp<=0;next_addr<=0;
            o_display_rp<=0;overflow0<=0;overflow1<=0;
        end else begin
            overflow0<=overflow;overflow1<=overflow0;
            if(!active && !q_valid && !i_read_busy && used!=0) begin
                active<=1;
                remaining<=(used>=BURST)?BURST:used;
            end else if(pop) begin
                remaining<=remaining-1'b1;
                if(remaining==1) active<=0;
            end
            if(q_valid) begin
                next_addr<=o_addr+25'd8;
                if(q[128]) begin
                    wp<=wp+1'b1;
                    o_display_rp<=wp+2'd2;
                end
            end
        end
    end
endmodule
