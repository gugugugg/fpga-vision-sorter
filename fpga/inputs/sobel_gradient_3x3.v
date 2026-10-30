`timescale 1ns/1ps
// Shared combinational Sobel L1 core; opposite differences and balanced sums
// retain the exact arithmetic used by the existing SOBL mode.
module sobel_gradient_3x3(
    input wire [7:0] a,b,c,d,f,g,h,i,
    output wire signed [10:0] gx,gy,
    output wire [10:0] magnitude
);
    wire signed [8:0] ca=$signed({1'b0,c})-$signed({1'b0,a});
    wire signed [8:0] fd=$signed({1'b0,f})-$signed({1'b0,d});
    wire signed [8:0] ig=$signed({1'b0,i})-$signed({1'b0,g});
    wire signed [8:0] ga=$signed({1'b0,g})-$signed({1'b0,a});
    wire signed [8:0] hb=$signed({1'b0,h})-$signed({1'b0,b});
    wire signed [8:0] ic=$signed({1'b0,i})-$signed({1'b0,c});
    wire signed [9:0] ox=$signed(ca)+$signed(ig),oy=$signed(ga)+$signed(ic);
    assign gx=$signed(ox)+($signed({fd[8],fd})<<<1);
    assign gy=$signed(oy)+($signed({hb[8],hb})<<<1);
    wire [10:0] ax=gx<0?-gx:gx,ay=gy<0?-gy:gy;
    assign magnitude=ax+ay;
endmodule
