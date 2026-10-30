`timescale 1ns/1ps
// Project-owned single-packet GMII transmitter, all signals in i_clk domain.
// Buffers the entire UDP payload first: an underrun can never truncate a wire
// frame. Generates IPv4 header checksum, legal zero IPv4 UDP checksum, Ethernet
// padding, reflected FCS (least significant byte first), and >=12-byte IFG.
module ethernet_frame_tx #(
    parameter [47:0] LOCAL_MAC=48'h027700000002,
    parameter [31:0] LOCAL_IP=32'h0a4d0002,
    parameter integer MAX_PAYLOAD=1472,
    parameter integer LOAD_TIMEOUT=500000
)(
    input wire i_clk,input wire i_rst_n,
    input wire i_start,input wire i_arp,
    input wire [47:0] i_dest_mac,input wire [31:0] i_dest_ip,
    input wire [15:0] i_src_port,input wire [15:0] i_dest_port,
    input wire [10:0] i_length,
    output wire o_ready,output wire o_payload_ready,
    input wire i_payload_valid,input wire [7:0] i_payload_data,
    input wire i_abort,
    output reg o_tx_valid,output reg [7:0] o_tx_data,
    output reg o_done,output reg o_error
);
    localparam IDLE=0,LOAD=1,PREAMBLE=2,BODY=3,FCS=4,GAP=5;
    reg [2:0] state;
    reg [7:0] payload [0:2047];
    reg [7:0] payload_q;
    reg [10:0] length_reg,load_index;
    reg [11:0] position,body_length;
    reg [3:0] small_count;
    reg [31:0] crc,fcs,load_wait;
    reg [31:0] header_partial,header_total;
    reg arp_reg;
    reg [47:0] dest_mac;
    reg [31:0] dest_ip;
    reg [15:0] src_port,dest_port,ip_length,ip_checksum;
    assign o_ready=state==IDLE;
    assign o_payload_ready=state==LOAD;
    function [31:0] crc_byte;
        input [31:0] old_crc;input [7:0] value;
        reg [31:0] c;integer k;
        begin
            c=old_crc ^ value;
            for(k=0;k<8;k=k+1) c=c[0] ? (c>>1)^32'hedb88320 : c>>1;
            crc_byte=c;
        end
    endfunction
    function [15:0] header_checksum;
        input [31:0] value;
        reg [31:0] s;
        begin
            s=value;
            s=s[15:0]+s[31:16];s=s[15:0]+s[31:16];
            header_checksum=~s[15:0];
        end
    endfunction
    function [7:0] header_byte;
        input [11:0] p;
        // Explicit arguments make the first byte update when IDLE latches a
        // new destination while position remains zero throughout PREAMBLE.
        input [47:0] target_mac;
        input [31:0] target_ip;
        input [15:0] from_port,to_port,total,check;
        input [10:0] payload_length;
        input is_arp;
        begin
            header_byte=0;
            // Byte cases avoid variable subtract/shift barrel networks on
            // the GMII data path; all byte positions are fixed by the wire.
            if(p<12) case(p)
                0:header_byte=target_mac[47:40];1:header_byte=target_mac[39:32];
                2:header_byte=target_mac[31:24];3:header_byte=target_mac[23:16];
                4:header_byte=target_mac[15:8];5:header_byte=target_mac[7:0];
                6:header_byte=LOCAL_MAC[47:40];7:header_byte=LOCAL_MAC[39:32];
                8:header_byte=LOCAL_MAC[31:24];9:header_byte=LOCAL_MAC[23:16];
                10:header_byte=LOCAL_MAC[15:8];11:header_byte=LOCAL_MAC[7:0];
                default:header_byte=0;
            endcase
            else if(is_arp) case(p)
                12:header_byte=8'h08;13:header_byte=8'h06;
                15:header_byte=1;16:header_byte=8'h08;
                18:header_byte=6;19:header_byte=4;21:header_byte=2;
                22:header_byte=LOCAL_MAC[47:40];23:header_byte=LOCAL_MAC[39:32];
                24:header_byte=LOCAL_MAC[31:24];25:header_byte=LOCAL_MAC[23:16];
                26:header_byte=LOCAL_MAC[15:8];27:header_byte=LOCAL_MAC[7:0];
                28:header_byte=LOCAL_IP[31:24];29:header_byte=LOCAL_IP[23:16];
                30:header_byte=LOCAL_IP[15:8];31:header_byte=LOCAL_IP[7:0];
                32:header_byte=target_mac[47:40];33:header_byte=target_mac[39:32];
                34:header_byte=target_mac[31:24];35:header_byte=target_mac[23:16];
                36:header_byte=target_mac[15:8];37:header_byte=target_mac[7:0];
                38:header_byte=target_ip[31:24];39:header_byte=target_ip[23:16];
                40:header_byte=target_ip[15:8];41:header_byte=target_ip[7:0];
                default:header_byte=0;
            endcase
            else case(p)
                12:header_byte=8'h08;14:header_byte=8'h45;
                16:header_byte=total[15:8];17:header_byte=total[7:0];
                20:header_byte=8'h40;22:header_byte=8'h40;23:header_byte=8'h11;
                24:header_byte=check[15:8];25:header_byte=check[7:0];
                26:header_byte=LOCAL_IP[31:24];27:header_byte=LOCAL_IP[23:16];
                28:header_byte=LOCAL_IP[15:8];29:header_byte=LOCAL_IP[7:0];
                30:header_byte=target_ip[31:24];31:header_byte=target_ip[23:16];
                32:header_byte=target_ip[15:8];33:header_byte=target_ip[7:0];
                34:header_byte=from_port[15:8];35:header_byte=from_port[7:0];
                36:header_byte=to_port[15:8];37:header_byte=to_port[7:0];
                38:header_byte=(payload_length+16'd8)>>8;
                39:header_byte=(payload_length+16'd8);
                default:header_byte=0;
            endcase
        end
    endfunction
    wire [7:0] body_byte=(!arp_reg && position>=42 && position<42+length_reg)
                         ? payload_q : header_byte(position,dest_mac,dest_ip,
                             src_port,dest_port,ip_length,ip_checksum,length_reg,arp_reg);
    wire [10:0] read_addr=(position>=42 && position<42+length_reg-1)
                         ? position-41 : 11'd0;
    always @(posedge i_clk) payload_q<=payload[read_addr];
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            state<=IDLE;length_reg<=0;load_index<=0;position<=0;small_count<=0;
            body_length<=0;crc<=32'hffffffff;fcs<=0;load_wait<=0;
            header_partial<=0;header_total<=0;
            arp_reg<=0;dest_mac<=0;dest_ip<=0;src_port<=0;dest_port<=0;
            ip_length<=0;ip_checksum<=0;o_tx_valid<=0;o_tx_data<=0;
            o_done<=0;o_error<=0;
        end else begin
            o_tx_valid<=0;o_done<=0;o_error<=0;
            case(state)
                IDLE:if(i_start) begin
                    if(!i_arp && (i_length==0 || i_length>MAX_PAYLOAD)) o_error<=1;
                    else begin
                        arp_reg<=i_arp;dest_mac<=i_dest_mac;dest_ip<=i_dest_ip;
                        src_port<=i_src_port;dest_port<=i_dest_port;
                        length_reg<=i_length;load_index<=0;load_wait<=0;
                        ip_length<=28+i_length;
                        body_length<=(i_arp || i_length<18) ? 60 : 42+i_length;
                        small_count<=0;position<=0;crc<=32'hffffffff;
                        state<=(i_arp || i_length==0) ? PREAMBLE : LOAD;
                    end
                end
                LOAD:if(i_abort || load_wait>=LOAD_TIMEOUT-1) begin
                    state<=IDLE;o_error<=1;
                end else if(i_payload_valid) begin
                    payload[load_index]<=i_payload_data;load_wait<=0;
                    load_index<=load_index+1'b1;
                    if(load_index==length_reg-1) state<=PREAMBLE;
                end else load_wait<=load_wait+1'b1;
                PREAMBLE:begin
                    // Use preamble cycles to compute the IPv4 checksum from
                    // latched fields, avoiding the caller-length -> sum/fold
                    // chain on the start handshake.
                    if(small_count==0)
                        header_partial<=32'h4500+32'h4000+32'h4011+
                            LOCAL_IP[31:16]+LOCAL_IP[15:0]+dest_ip[31:16]+dest_ip[15:0];
                    if(small_count==1) header_total<=header_partial+ip_length;
                    if(small_count==2) ip_checksum<=header_checksum(header_total);
                    o_tx_valid<=1;o_tx_data<=small_count==7 ? 8'hd5 : 8'h55;
                    small_count<=small_count+1'b1;
                    if(small_count==7) begin position<=0;state<=BODY;end
                end
                BODY:begin
                    o_tx_valid<=1;o_tx_data<=body_byte;
                    // CRC trails the registered wire byte by one cycle. This
                    // separates header/payload selection from the CRC XOR
                    // network, keeping one output byte on every GMII clock.
                    if(position!=0) crc<=crc_byte(crc,o_tx_data);
                    position<=position+1'b1;
                    if(position==body_length-1) begin
                        small_count<=0;state<=FCS;
                    end
                end
                FCS:begin
                    o_tx_valid<=1;
                    if(small_count==0) begin
                        // Incorporate the final body byte on the first FCS
                        // cycle; later FCS bytes use this captured complement.
                        fcs<=~crc_byte(crc,o_tx_data);
                        o_tx_data<=~crc_byte(crc,o_tx_data);
                    end else o_tx_data<=fcs >> (small_count*8);
                    small_count<=small_count+1'b1;
                    if(small_count==3) begin small_count<=0;state<=GAP;end
                end
                GAP:begin
                    small_count<=small_count+1'b1;
                    if(small_count==11) begin state<=IDLE;o_done<=1;end
                end
                default:state<=IDLE;
            endcase
        end
    end
endmodule
