`timescale 1ns/1ps
// Lossless 580-color image. The index ROM is an explicit device BRAM so the
// synthesis mapper cannot absorb its read register into the palette lookup.
module anlogic_logo_bram #(parameter INDEX_FILE="logo_indices.dath",PALETTE_FILE="logo_palette.hex")(
 input wire I_clk,input wire [13:0] I_addr,output reg [24:0] O_pixel
);
 wire [9:0] index_q;
`ifdef LOGO_SIM
 reg [9:0] indices[0:16383];reg [9:0] index_sim;
 initial $readmemh(INDEX_FILE,indices);
 always @(posedge I_clk)index_sim<=indices[I_addr];
 assign index_q=index_sim;
`else
 PH1P_LOGIC_ERAM #(.DATA_WIDTH_A(10),.DATA_WIDTH_B(10),.ADDR_WIDTH_A(14),.ADDR_WIDTH_B(14),
  .DATA_DEPTH_A(16384),.DATA_DEPTH_B(16384),.MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
  .INIT_FILE(INDEX_FILE),.FILL_ALL("0")) u_indices(
  .doa(index_q),.dob(),.dia(10'd0),.dib(10'd0),.addra(I_addr),.addrb(14'd0),
  .cea(1'b1),.ocea(1'b1),.clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b1),
  .ceb(1'b0),.oceb(1'b0),.clkb(1'b0),.web(1'b0),.rstb(1'b0),.beb(1'b0),
  .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr());
`endif
 (* ram_style="bram" *) reg [24:0] palette[0:1023];
 initial $readmemh(PALETTE_FILE,palette);
 always @(posedge I_clk)O_pixel<=palette[index_q];
endmodule
