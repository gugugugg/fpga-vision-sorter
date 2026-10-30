`timescale 1ns/1ps
// Project-owned GMII stack. No RGMII/PHY or vendor primitives are used here.
// One immutable RX mailbox and one TX payload buffer. Application domain i_clk.
module network_udp_stack #(
    parameter [47:0] LOCAL_MAC=48'h027700000002,
    parameter [31:0] LOCAL_IP=32'h0a4d0002,
    parameter [31:0] PEER_IP=32'h0a4d0001,
    parameter [15:0] CONTROL_PORT=6002, VIDEO_PORT=6011, DEBUG_PORT=6012,
    parameter integer LOAD_TIMEOUT=500000
)(
    input wire i_clk, i_rx_clk, i_rst_n,
    input wire i_rx_valid, i_rx_error,
    input wire [7:0] i_rx_data,
    output wire o_tx_valid,
    output wire [7:0] o_tx_data,
    input wire i_udp_ready,
    output wire o_udp_valid, o_udp_last,
    output wire [7:0] o_udp_data,
    output wire [15:0] o_udp_length, o_udp_src_port, o_udp_dest_port,
    output wire [47:0] o_peer_mac,
    output wire [31:0] o_peer_ip,
    input wire i_start,
    input wire [47:0] i_dest_mac,
    input wire [31:0] i_dest_ip,
    input wire [15:0] i_src_port, i_dest_port,
    input wire [10:0] i_length,
    output wire o_ready, o_payload_ready,
    input wire i_payload_valid,
    input wire [7:0] i_payload_data,
    input wire i_abort,
    output wire o_done, o_error,
    output wire [31:0] o_bad_frames, o_busy_drops, o_rejected, o_accepted
);
    // Asynchronous assertion, independent synchronous reset removal per domain.
    // Both domains must run before traffic starts after a common reset.
    (* async_reg="true" *) reg [1:0] app_reset, rx_reset;
    always @(posedge i_clk or negedge i_rst_n)
        if(!i_rst_n) app_reset<=0; else app_reset<={app_reset[0],1'b1};
    always @(posedge i_rx_clk or negedge i_rst_n)
        if(!i_rst_n) rx_reset<=0; else rx_reset<={rx_reset[0],1'b1};

    wire frame_ready, read_enable, read_valid, release_frame;
    wire [10:0] frame_length, read_addr;
    wire [7:0] read_data;
    wire arp_valid, tx_ready;
    // ARP peer fields are held by the parser until the ARP start handshake.
    wire arp_start=arp_valid && tx_ready;
    assign o_ready=tx_ready && !arp_valid && app_reset[1];
    wire udp_start=i_start && o_ready;
    // Distinguish application completion from internal ARP completion.
    reg udp_transaction;
    wire tx_done, tx_error;
    assign o_done=tx_done && udp_transaction;
    assign o_error=tx_error && udp_transaction;
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) udp_transaction<=0;
        else if(!app_reset[1]) udp_transaction<=0;
        else if(udp_start) udp_transaction<=1;
        else if(arp_start) udp_transaction<=0;
        // Keep ownership through the one-cycle registered done/error pulse.
    end

    ethernet_frame_rx rx(
        .i_rst_n(rx_reset[1]), .i_app_rst_n(app_reset[1]), .i_rx_clk(i_rx_clk),
        .i_rx_valid(i_rx_valid), .i_rx_error(i_rx_error), .i_rx_data(i_rx_data),
        .i_app_clk(i_clk), .i_release(release_frame),
        .o_frame_ready(frame_ready), .o_frame_length(frame_length),
        .i_read(read_enable), .i_read_addr(read_addr),
        .o_read_valid(read_valid), .o_read_data(read_data),
        .o_bad_frames(o_bad_frames), .o_busy_drops(o_busy_drops)
    );
    ethernet_ipv4_udp_rx #(
        .LOCAL_MAC(LOCAL_MAC), .LOCAL_IP(LOCAL_IP), .PEER_IP(PEER_IP),
        .CONTROL_PORT(CONTROL_PORT), .VIDEO_PORT(VIDEO_PORT),.DEBUG_PORT(DEBUG_PORT)
    ) parser(
        .i_clk(i_clk), .i_rst_n(app_reset[1]),
        .i_frame_ready(frame_ready), .i_frame_length(frame_length),
        .o_release(release_frame), .o_read(read_enable), .o_read_addr(read_addr),
        .i_read_valid(read_valid), .i_read_data(read_data),
        .o_arp_valid(arp_valid), .i_arp_ready(arp_start),
        .o_peer_mac(o_peer_mac), .o_peer_ip(o_peer_ip),
        .i_udp_ready(i_udp_ready), .o_udp_valid(o_udp_valid),
        .o_udp_data(o_udp_data), .o_udp_last(o_udp_last),
        .o_udp_length(o_udp_length), .o_udp_src_port(o_udp_src_port),
        .o_udp_dest_port(o_udp_dest_port), .o_rejected(o_rejected), .o_accepted(o_accepted)
    );
    ethernet_frame_tx #(
        .LOCAL_MAC(LOCAL_MAC), .LOCAL_IP(LOCAL_IP), .LOAD_TIMEOUT(LOAD_TIMEOUT)
    ) tx(
        .i_clk(i_clk), .i_rst_n(app_reset[1]),
        .i_start(arp_start || udp_start), .i_arp(arp_start),
        .i_dest_mac(arp_start ? o_peer_mac : i_dest_mac),
        .i_dest_ip(arp_start ? o_peer_ip : i_dest_ip),
        .i_src_port(i_src_port), .i_dest_port(i_dest_port), .i_length(i_length),
        .o_ready(tx_ready), .o_payload_ready(o_payload_ready),
        .i_payload_valid(i_payload_valid), .i_payload_data(i_payload_data),
        .i_abort(i_abort), .o_tx_valid(o_tx_valid), .o_tx_data(o_tx_data),
        .o_done(tx_done), .o_error(tx_error)
    );
endmodule
