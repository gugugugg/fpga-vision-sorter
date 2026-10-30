`timescale 1ns/1ps
// Monitor the real 16-bit DPHY packet boundaries BEFORE uial2axis invents
// raster coordinates. RAW10 valid contains packing bubbles, so falling RAW
// valid is not a line boundary. Matches the BSP's byte/lane ordering.
// Sticky fail-stop until reset: do not accept CRC-correct but shifted crops.
// This checks length/count only, NOT CSI header ECC or payload CRC16.
// DPHY HS-valid includes receiver tail samples beyond the CSI packet. The
// BSP decoder stops at header WC; require the full packet, not exact HS length.
module csi_packet_guard #(
    parameter WIDTH=1280, HEIGHT=720
)(
    input wire i_clk,i_rst_n,i_hs_valid,
    input wire [15:0] i_hs_data,
    input wire i_lane_fault,
    output reg o_fault,
    output reg o_diag_update,
    // First fault: reason, data type, HS words, completed rows, header WC.
    output reg [63:0] o_diag
);
    localparam LINE_BYTES=WIDTH*10/8;
    localparam LINE_WORDS=2+LINE_BYTES/2+1;
    reg hs_d,in_frame;
    reg [7:0] dt,wc_low,wc_high;
    reg [15:0] words,lines;
    reg [7:0] reason;
    wire [15:0] header_wc = (i_hs_valid && hs_d && words==1) ?
                                 {i_hs_data[15:8],wc_low} : {wc_high,wc_low};
    // Priority selects the diagnosis
    // when more than one condition occurs on the first failing cycle.
    always @(*) begin
        reason=0;
        if(i_lane_fault && in_frame) reason=1;
        else if(i_hs_valid && !hs_d) begin
            if(i_hs_data[15:8]==0 && in_frame) reason=2;
            else if(i_hs_data[15:8]==1 && !in_frame) reason=3;
            else if(i_hs_data[15:8]==1 && lines!=HEIGHT) reason=4;
        end else if(i_hs_valid) begin
            if(words==16'hffff) reason=5;
            else if(words==1 && dt==8'h2b && header_wc!=LINE_BYTES) reason=6;
        end else if(hs_d && dt==8'h2b) begin
            if(!in_frame) reason=7;
            else if(words<LINE_WORDS) reason=8;
            else if(lines>=HEIGHT) reason=9;
        end
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            hs_d<=0;in_frame<=0;dt<=0;wc_low<=0;words<=0;lines<=0;o_fault<=0;
            wc_high<=0;o_diag<=0;o_diag_update<=0;
        end else begin
            hs_d<=i_hs_valid;
            o_diag_update<=0;
            if(reason!=0) begin
                o_fault<=1;
                if(!o_fault) begin
                    o_diag<={reason,(i_hs_valid && !hs_d ? i_hs_data[15:8] : dt),
                             words,lines,header_wc};
                    o_diag_update<=1;
                end
            end
            if(i_hs_valid && !hs_d) begin
                dt<=i_hs_data[15:8];wc_low<=i_hs_data[7:0];wc_high<=0;words<=1;
                if(i_hs_data[15:8]==0) begin
                    in_frame<=1;lines<=0;
                end else if(i_hs_data[15:8]==1) begin
                    in_frame<=0;
                end
            end else if(i_hs_valid) begin
                if(words!=16'hffff) words<=words+1'b1;
                if(words==1) wc_high<=i_hs_data[15:8];
            end else if(hs_d && dt==8'h2b) begin
                if(in_frame && words>=LINE_WORDS && lines<HEIGHT) lines<=lines+1'b1;
            end
        end
    end
endmodule
