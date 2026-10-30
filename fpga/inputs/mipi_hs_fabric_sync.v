// LP-domain HS enable crosses into the sensor's independent byte clock.
// Delay the PHY words by the same extra cycle as the second synchronizer.
// Like the BSP data registers, these flops have no reset. LP detection holds
// HS low during reset; the existing offset counter clears before a packet.
module mipi_hs_fabric_sync(
 input wire i_clk,input wire i_hs_async,input wire [127:0] i_words,
 output wire o_hs,output reg [127:0] o_words
);
 (* async_reg="true",syn_preserve=1,keep=1 *) reg hs_meta,hs_sync;
 reg [127:0] words_first;
 always @(posedge i_clk) begin
  hs_meta<=i_hs_async;
  hs_sync<=hs_meta;
  words_first<=i_words;
  o_words<=words_first;
 end
 assign o_hs=hs_sync;
endmodule
