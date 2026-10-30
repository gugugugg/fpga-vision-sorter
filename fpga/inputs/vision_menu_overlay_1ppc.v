`timescale 1ns/1ps
// Seven visible rows, atomic frame-boundary menu updates, synchronous text
// bitmap ROM. Two registered pixel stages; VS/HS/DE/RGB travel together.
// Closed menu passes every RGB pixel exactly, with the same two-cycle delay.
module vision_menu_overlay_1ppc #(
    parameter [63:0] MODE_BITMAP=64'h30fff,
    parameter TEXT_FILE="inputs/hdmi_menu_text.hex"
)(
    input wire i_clk,i_rst_n,i_vsync,i_hsync,i_de,
    input wire [23:0] i_rgb,
    input wire i_menu_open,
    input wire [5:0] i_cursor,i_first,i_applied_mode,
    output reg o_vsync,o_hsync,o_de,
    output reg [23:0] o_rgb
);
    function integer count_modes;
        input [63:0] bitmap;
        integer j;
        begin count_modes=0;for(j=0;j<24;j=j+1) count_modes=count_modes+bitmap[j];end
    endfunction
    localparam COUNT=count_modes(MODE_BITMAP);
    function [4:0] mode_at;
        input [5:0] index;
        integer j,n;
        begin n=0;mode_at=0;for(j=0;j<24;j=j+1) if(MODE_BITMAP[j]) begin
            if(n==index) mode_at=j;n=n+1;end end
    endfunction
    reg [7:0] text_rom [0:16383];
    initial $readmemh(TEXT_FILE,text_rom);
    reg [10:0] x;
    reg [9:0] y;
    reg de_d,vs_d,opened;
    reg [5:0] cursor,first,applied;
    wire frame_start=i_vsync && !vs_d;
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin x<=0;y<=0;de_d<=0;vs_d<=0;
            opened<=0;cursor<=0;first<=0;applied<=0;end
        else begin
            vs_d<=i_vsync;de_d<=i_de;
            if(frame_start) begin
                x<=0;y<=0;opened<=i_menu_open;
                cursor<=i_cursor;first<=i_first;applied<=i_applied_mode;
            end else if(i_de) x<=x+1'b1;
            else if(de_d) begin x<=0;y<=y+1'b1;end
        end
    end
    integer r;
    reg in_row,highlight,current_row;
    reg [5:0] row_index;
    reg [9:0] text_y;
    reg [4:0] slot;
    reg text_window;
    wire [10:0] text_x=(x-11'd56)>>1;
    wire [13:0] text_addr={slot,text_y[4:1],text_x[7:3]};
    wire panel=i_de && opened && x>=24 && x<600 && y>=24 && y<576;
    localparam THUMB_STEP=336/COUNT;
    wire [9:0] thumb_top=128+first*THUMB_STEP;
    localparam THUMB_HEIGHT=COUNT>7?7*THUMB_STEP:336;
    reg [23:0] background,ink;
    always @(*) begin
        in_row=0;highlight=0;current_row=0;row_index=0;
        text_y=0;slot=0;text_window=0;
        for(r=0;r<7;r=r+1) begin
            if(y>=128+r*48 && y<176+r*48 && first+r<COUNT) begin
                in_row=1;row_index=first+r;
                highlight=cursor==first+r;
                slot=mode_at(first+r);current_row=applied==slot;
                text_y=y-(136+r*48);
                text_window=y>=136+r*48 && y<168+r*48;
            end
        end
        if(y>=48 && y<80) begin slot=24;text_y=y-48;text_window=1;end
        if(y>=88 && y<120) begin slot=25;text_y=y-88;text_window=1;end
        if(y>=492 && y<524) begin slot=26;text_y=y-492;text_window=1;end
        if(y>=532 && y<564) begin slot=27;text_y=y-532;text_window=1;end
        text_window=text_window && x>=56 && x<568;
        background={{2'b0,i_rgb[23:18]}+8'd8,
                    {2'b0,i_rgb[15:10]}+8'd12,{2'b0,i_rgb[7:2]}+8'd18};
        ink=24'he6edf5;
        if(slot>=25) ink=24'ha6b8ca;
        if(in_row && highlight && x>=32 && x<576) begin
            background=24'h244e70;ink=24'hffffff;
        end
        if(x>=32 && x<36 && in_row && highlight) background=24'h75d6cd;
        if(current_row && x>=548 && x<556 && text_window) background=24'h75d6cd;
        if(y==124 || y==480) background=24'h314052;
        if(COUNT>7 && x>=584 && x<588 && y>=128 && y<464)
            background=(y>=thumb_top && y<thumb_top+THUMB_HEIGHT)?24'h75d6cd:24'h314052;
    end
    reg [7:0] bitmap;
    reg [2:0] bit_index;
    reg panel_d,text_d,vs1,hs1,de1;
    reg [23:0] rgb1,bg1,ink1;
    // No reset on ROM read register: preserve synchronous ERAM inference.
    always @(posedge i_clk) bitmap<=text_rom[text_addr];
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin panel_d<=0;text_d<=0;bit_index<=0;
            vs1<=0;hs1<=0;de1<=0;rgb1<=0;bg1<=0;ink1<=0;
            o_vsync<=0;o_hsync<=0;o_de<=0;o_rgb<=0;end
        else begin
            panel_d<=panel;text_d<=text_window;bit_index<=7-text_x[2:0];
            vs1<=i_vsync;hs1<=i_hsync;de1<=i_de;rgb1<=i_rgb;
            bg1<=background;ink1<=ink;
            o_vsync<=vs1;o_hsync<=hs1;o_de<=de1;
            o_rgb<=panel_d?(text_d && bitmap[bit_index]?ink1:bg1):rgb1;
        end
    end
endmodule
