`timescale 1ns/1ps
// Use osd_de/osd_rgb/o_display_mode AFTER the actual OSD. i_vsync is active high.
// Frame-end is the next VSYNC rising edge, so extra/missing active pixels are
// detected before publishing a frame. SOF coincides with its first DE pixel.
module network_hdmi_tap(
    input wire i_clk,i_rst_n,i_vsync,i_de,
    input wire [23:0] i_rgb,input wire [5:0] i_display_mode,
    output wire o_sof,o_frame_end,o_valid,
    output wire [23:0] o_rgb,output wire [5:0] o_mode
);
    reg vs_d,first;
    assign o_frame_end=i_vsync && !vs_d;
    assign o_sof=i_de && first;
    assign o_valid=i_de;assign o_rgb=i_rgb;assign o_mode=i_display_mode;
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin vs_d<=0;first<=1;end
        else begin vs_d<=i_vsync;if(o_frame_end) first<=1;else if(i_de) first<=0;end
    end
endmodule
