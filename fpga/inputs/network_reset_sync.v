`timescale 1ns/1ps
// Asynchronous assertion; release after two destination clock edges.
module network_reset_sync(input i_clk, input i_async_rst_n, output o_rst_n);
    (* ASYNC_REG = "TRUE" *) reg [1:0] release_pipe;
    always @(posedge i_clk or negedge i_async_rst_n)
        if (!i_async_rst_n) release_pipe <= 2'b00;
        else release_pipe <= {release_pipe[0], 1'b1};
    assign o_rst_n = release_pipe[1];
endmodule
