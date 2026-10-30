`timescale 1ns/1ps
// Two synchronous ROM reads. Transparent index 0; all video signals travel
// through the identical two-clock pipeline. Animation commits only in vblank.
module mascot_overlay #(
    parameter IMG_WIDTH=1024, IMG_HEIGHT=600,
    parameter X0=880, Y0=16, FRAME_COUNT=27, FRAME_TICKS=6,
    parameter TILE_COUNT=1075
)(
    input wire clk,rst_n,vsync,hsync,de,
    input wire [23:0] rgb,
    output wire out_vsync,out_hsync,out_de,
    output wire [23:0] out_rgb
);
    reg [10:0] x;
    reg [9:0] y;
    reg de_d,vs_d;
    reg [4:0] frame;
    reg [7:0] ticks;
    wire [10:0] dx=x-X0;
    wire [9:0] dy=y-Y0;
    wire window=de && x>=X0 && x<X0+128 && y>=Y0 && y<Y0+128;
    // 64x64 stored pixels enlarged by exactly 2 with no interpolator/DSP.
    wire [10:0] map_addr=window ? {frame,dy[6:4],dx[6:4]} : 11'd0;
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [10:0] tile_map[0:1727];
    (* ram_style="block",syn_ramstyle="block_ram" *) reg [3:0] atlas[0:TILE_COUNT*64-1];
    reg [23:0] palette[0:15];
    reg [10:0] tile_q;
    reg [5:0] offset_q;
    reg [3:0] index_q;
    reg win1,win2,vs1,vs2,hs1,hs2,de1,de2;
    reg [23:0] rgb1,rgb2;
    initial begin
        $readmemh("mascot_map.hex",tile_map);
        $readmemh("mascot_atlas.hex",atlas);
        $readmemh("mascot_palette.hex",palette);
    end
    // Do not reset the ROM output registers: retain native block RAM inference.
    always @(posedge clk) begin
        tile_q<=tile_map[map_addr];
        offset_q<={dy[3:1],dx[3:1]};
        index_q<=atlas[{tile_q,offset_q}];
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;de_d<=0;vs_d<=0;frame<=0;ticks<=0;
            win1<=0;win2<=0;vs1<=0;vs2<=0;hs1<=0;hs2<=0;
            de1<=0;de2<=0;rgb1<=0;rgb2<=0;
        end else begin
            de_d<=de;vs_d<=vsync;
            vs1<=vsync;vs2<=vs1;hs1<=hsync;hs2<=hs1;
            de1<=de;de2<=de1;rgb1<=rgb;rgb2<=rgb1;
            win1<=window;win2<=win1;
            if(vsync) begin x<=0;y<=0;end
            else if(de) begin if(x==IMG_WIDTH-1)x<=0;else x<=x+1'b1;end
            else if(de_d) begin x<=0;if(y==IMG_HEIGHT-1)y<=0;else y<=y+1'b1;end
            if(vsync && !vs_d) begin
                if(ticks==FRAME_TICKS-1) begin
                    ticks<=0;
                    if(frame==FRAME_COUNT-1)frame<=0;else frame<=frame+1'b1;
                end else ticks<=ticks+1'b1;
            end
        end
    end
    assign out_vsync=vs2;
    assign out_hsync=hs2;
    assign out_de=de2;
    assign out_rgb=!de2 ? 24'd0 : (win2 && index_q!=0 ? palette[index_q] : rgb2);
endmodule
