`timescale 1ns/1ps
// The board_frame_reader supplies RGB two clocks after its read request.
// Register that RGB once, alongside the third control clock. Delaying RGB
// another two clocks (as in the reference mixer) shifts every displayed row.
module board_hdmi_mixer(
 input wire clk,rst_n,vsync,hsync,de,input wire [23:0] reader_rgb,
 output wire read_enable,
 output reg out_vsync,out_hsync,out_de,output reg [23:0] out_rgb
);
 reg [1:0] vs_pipe,hs_pipe,de_pipe;
 assign read_enable=de;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin
   vs_pipe<=0;hs_pipe<=0;de_pipe<=0;
   out_vsync<=0;out_hsync<=0;out_de<=0;out_rgb<=0;
  end else begin
   vs_pipe<={vs_pipe[0],vsync};hs_pipe<={hs_pipe[0],hsync};de_pipe<={de_pipe[0],de};
   out_vsync<=vs_pipe[1];out_hsync<=hs_pipe[1];out_de<=de_pipe[1];
   out_rgb<=de_pipe[1]?reader_rgb:24'd0;
  end
endmodule
