`timescale 1ns/1ps
// Camera colour-response calibration before demosaic. Subtract the measured
// sensor pedestal on all sites, then preserve green signal;
// only the two same-colour R/B sites in each BGGR 4-PPC beat are multiplied.
// 16x16 spatial tiles, Q10 gains, three registered stages including ROM read.
module raw_color_shading_4ppc #(
    parameter WIDTH=1024, HEIGHT=600,
    parameter TABLE_FILE="camera_color_shading.hex",
    parameter [9:0] BLACK_LEVEL=10'd0
)(
    input wire i_clk,i_rst_n,
    input wire [39:0] i_raw,
    input wire i_valid,i_sof,i_last,
    output reg [39:0] o_raw,
    output reg o_valid,o_sof,o_last
);
    localparam COLS=(WIDTH+15)/16, ROWS=(HEIGHT+15)/16;
    (* ram_style="block" *) reg [23:0] gains[0:COLS*ROWS-1];
    initial $readmemh(TABLE_FILE,gains);
    reg [9:0] x,y;
    wire [9:0] sx=i_sof ? 10'd0:x, sy=i_sof ? 10'd0:y;
    wire [11:0] address=(sy>>4)*COLS+(sx>>4);
    reg [23:0] gain;
    reg [39:0] p_raw,q_raw;
    reg p_valid,p_sof,p_last,p_red,q_valid,q_sof,q_last,q_red;
    reg [21:0] product0,product1;
    wire [11:0] selected_gain=p_red ? gain[23:12]:gain[11:0];
    function [9:0] scaled;
        input [21:0] product;
        reg [22:0] rounded;
        begin
            rounded={1'b0,product}+23'd512;
            scaled=(rounded[22:10]>13'd1023) ? 10'd1023:rounded[19:10];
        end
    endfunction
    function [9:0] remove_black;
        input [9:0] value;
        begin remove_black=value>BLACK_LEVEL?value-BLACK_LEVEL:10'd0;end
    endfunction
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            x<=0;y<=0;p_valid<=0;q_valid<=0;o_valid<=0;
            p_sof<=0;q_sof<=0;o_sof<=0;p_last<=0;q_last<=0;o_last<=0;
            p_red<=0;q_red<=0;p_raw<=0;q_raw<=0;o_raw<=0;
            product0<=0;product1<=0;
        end else begin
            p_valid<=i_valid;p_sof<=i_valid && i_sof;p_last<=i_valid && i_last;
            if(i_valid) begin
                p_raw<={remove_black(i_raw[39:30]),remove_black(i_raw[29:20]),
                        remove_black(i_raw[19:10]),remove_black(i_raw[9:0])};
                p_red<=sy[0];
                if(i_last) begin x<=0;y<=(sy==HEIGHT-1) ? 10'd0:sy+10'd1;end
                else begin x<=sx+10'd4;y<=sy;end
            end
            q_valid<=p_valid;q_sof<=p_sof;q_last<=p_last;q_red<=p_red;q_raw<=p_raw;
            product0<=(p_red ? p_raw[19:10]:p_raw[9:0])*selected_gain;
            product1<=(p_red ? p_raw[39:30]:p_raw[29:20])*selected_gain;
            o_valid<=q_valid;o_sof<=q_sof;o_last<=q_last;
            o_raw<=q_red ? {scaled(product1),q_raw[29:20],scaled(product0),q_raw[9:0]}:
                                {q_raw[39:30],scaled(product1),q_raw[19:10],scaled(product0)};
        end
    end
    // Keep the ROM read synchronous and without reset for ERAM inference.
    always @(posedge i_clk) if(i_valid) gain<=gains[address];
endmodule
