`timescale 1ns/1ps
// Parses only packets already validated by ethernet_frame_rx. Bounded IHL=5
// IPv4, no VLAN/jumbo/IP fragments. Validates IP/UDP lengths and checksums.
// IPv4 UDP checksum zero is legal; nonzero is fully checked including odd tails.
// Whole-frame validation precedes ALL application payload output/ARP replies.
module ethernet_ipv4_udp_rx #(
    parameter [47:0] LOCAL_MAC=48'h027700000002,
    parameter [31:0] LOCAL_IP=32'h0a4d0002,
    parameter [31:0] PEER_IP=32'h0a4d0001,
    parameter [15:0] CONTROL_PORT=6002,VIDEO_PORT=6011,DEBUG_PORT=6012
)(
    input wire i_clk,input wire i_rst_n,
    input wire i_frame_ready,input wire [10:0] i_frame_length,
    output reg o_release,output wire o_read,output wire [10:0] o_read_addr,
    input wire i_read_valid,input wire [7:0] i_read_data,
    output reg o_arp_valid,input wire i_arp_ready,
    output reg [47:0] o_peer_mac,output reg [31:0] o_peer_ip,
    input wire i_udp_ready,output reg o_udp_valid,output reg [7:0] o_udp_data,
    output reg o_udp_last,output reg [15:0] o_udp_length,
    output reg [15:0] o_udp_src_port,output reg [15:0] o_udp_dest_port,
    output reg [31:0] o_rejected,output reg [31:0] o_accepted
);
    localparam IDLE=0,PRIME=1,SCAN=2,CHECK=3,ARP=4,WAIT_APP=5,
               PAYLOAD=6,RELEASE=7,DRAIN=8,SUM=9,FOLD=10,VALIDATE=11;
    reg [3:0] state;
    reg [7:0] header [0:41];
    reg [10:0] frame_length,issue_index,consume_index;
    reg [31:0] ip_sum,udp_sum;
    reg [31:0] pseudo_a,pseudo_b,pseudo_total;
    reg [15:0] ip_fold,udp_fold;
    reg [7:0] high_byte;
    wire [47:0] eth_dst={header[0],header[1],header[2],header[3],header[4],header[5]};
    wire [47:0] eth_src={header[6],header[7],header[8],header[9],header[10],header[11]};
    wire [15:0] eth_type={header[12],header[13]};
    wire [15:0] ip_len={header[16],header[17]};
    wire [31:0] ip_src={header[26],header[27],header[28],header[29]};
    wire [31:0] ip_dst={header[30],header[31],header[32],header[33]};
    wire [15:0] udp_len={header[38],header[39]};
    wire [15:0] udp_cksum={header[40],header[41]};
    function [15:0] fold;
        input [31:0] s_in;reg [31:0] s;
        begin s=s_in[15:0]+s_in[31:16];s=s[15:0]+s[31:16];fold=s[15:0];end
    endfunction
    wire ipv4_ok=frame_length>=42 && eth_dst==LOCAL_MAC && eth_type==16'h0800 &&
        header[14]==8'h45 && header[23]==17 && header[22]!=0 &&
        (header[20]&8'hbf)==0 && header[21]==0 && // reject reserved/MF/offset; DF allowed
        ip_len>=28 && ip_len<=1500 && ip_len+14<=frame_length &&
        ip_src==PEER_IP && ip_dst==LOCAL_IP && ip_fold==16'hffff &&
        udp_len>=8 && udp_len==ip_len-20 &&
        ({header[36],header[37]}==CONTROL_PORT || {header[36],header[37]}==VIDEO_PORT || {header[36],header[37]}==DEBUG_PORT) &&
        (udp_cksum==0 || udp_fold==16'hffff) &&
        !eth_src[40] && eth_src!=0;
    wire arp_ok=frame_length>=42 && (eth_dst==LOCAL_MAC || eth_dst==48'hffffffffffff) &&
        eth_type==16'h0806 && {header[14],header[15]}==1 &&
        {header[16],header[17]}==16'h0800 && header[18]==6 && header[19]==4 &&
        {header[20],header[21]}==1 &&
        {header[22],header[23],header[24],header[25],header[26],header[27]}==eth_src &&
        {header[28],header[29],header[30],header[31]}==PEER_IP &&
        {header[38],header[39],header[40],header[41]}==LOCAL_IP &&
        !eth_src[40] && eth_src!=0;
    assign o_read=(state==SCAN && issue_index<frame_length) ||
                  (state==PAYLOAD && issue_index<14+ip_len);
    assign o_read_addr=issue_index;
    integer j;
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            state<=IDLE;o_release<=0;frame_length<=0;issue_index<=0;consume_index<=0;
            ip_sum<=0;udp_sum<=0;high_byte<=0;o_arp_valid<=0;o_peer_mac<=0;o_peer_ip<=0;
            pseudo_a<=0;pseudo_b<=0;pseudo_total<=0;ip_fold<=0;udp_fold<=0;
            o_udp_valid<=0;o_udp_data<=0;o_udp_last<=0;o_udp_length<=0;
            o_udp_src_port<=0;o_udp_dest_port<=0;o_rejected<=0;o_accepted<=0;
            for(j=0;j<42;j=j+1) header[j]<=0;
        end else begin
            o_release<=0;o_udp_valid<=0;o_udp_last<=0;
            case(state)
                IDLE:if(i_frame_ready) state<=PRIME;
                PRIME:begin
                    frame_length<=i_frame_length;issue_index<=0;consume_index<=0;
                    ip_sum<=0;udp_sum<=0;state<=SCAN;
                end
                SCAN:begin
                    if(o_read) issue_index<=issue_index+1'b1;
                    if(i_read_valid) begin
                        if(consume_index<42) header[consume_index]<=i_read_data;
                        if(!consume_index[0]) high_byte<=i_read_data;
                        if(consume_index>=14 && consume_index<34 && consume_index[0])
                            ip_sum<=ip_sum+{high_byte,i_read_data};
                        if(consume_index>=34 && consume_index<14+ip_len) begin
                            if(consume_index[0]) udp_sum<=udp_sum+{high_byte,i_read_data};
                            else if(consume_index==14+ip_len-1) udp_sum<=udp_sum+{i_read_data,8'd0};
                        end
                        consume_index<=consume_index+1'b1;
                        if(consume_index==frame_length-1) state<=SUM;
                    end
                end
                // Bounded store-and-forward validation allows checksum work
                // to be split over clocks rather than seven chained adders
                // on the packet-accept path at 125 MHz.
                SUM:begin
                    pseudo_a<=udp_sum+ip_src[31:16]+ip_src[15:0];
                    pseudo_b<=32'd17+ip_dst[31:16]+ip_dst[15:0]+udp_len;
                    state<=FOLD;
                end
                FOLD:begin
                    pseudo_total<=pseudo_a+pseudo_b;ip_fold<=fold(ip_sum);
                    state<=VALIDATE;
                end
                VALIDATE:begin udp_fold<=fold(pseudo_total);state<=CHECK;end
                CHECK:begin
                    if(arp_ok) begin
                        o_peer_mac<=eth_src;o_peer_ip<=PEER_IP;o_arp_valid<=1;state<=ARP;
                        o_accepted<=o_accepted+1'b1;
                    end else if(ipv4_ok) begin
                        o_peer_mac<=eth_src;o_peer_ip<=ip_src;
                        o_udp_length<=udp_len-8;
                        o_udp_src_port<={header[34],header[35]};
                        o_udp_dest_port<={header[36],header[37]};
                        o_accepted<=o_accepted+1'b1;
                        state<=udp_len==8 ? RELEASE : WAIT_APP;
                    end else begin o_rejected<=o_rejected+1'b1;state<=RELEASE;end
                end
                ARP:if(i_arp_ready) begin o_arp_valid<=0;state<=RELEASE;end
                WAIT_APP:if(i_udp_ready) begin
                    issue_index<=42;consume_index<=42;state<=PAYLOAD;
                end
                PAYLOAD:begin
                    if(o_read) issue_index<=issue_index+1'b1;
                    if(i_read_valid) begin
                        o_udp_valid<=1;o_udp_data<=i_read_data;
                        consume_index<=consume_index+1'b1;
                        if(consume_index==14+ip_len-1) begin
                            o_udp_last<=1;state<=RELEASE;
                        end
                    end
                end
                RELEASE:begin o_release<=1;state<=DRAIN;end
                DRAIN:if(!i_frame_ready) state<=IDLE;
                default:state<=IDLE;
            endcase
        end
    end
endmodule
