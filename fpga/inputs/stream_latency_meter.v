`timescale 1ns/1ps

module stream_latency_meter (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_input_sof,
    input  wire        i_output_sof,
    output reg  [15:0] o_pipeline_cycles
);
    reg [31:0] cycle_counter;
    reg [31:0] input_stamp;
    wire [31:0] latency_diff = cycle_counter - input_stamp;
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            cycle_counter <= 0;
            input_stamp <= 0;
            o_pipeline_cycles <= 0;
        end else begin
            cycle_counter <= cycle_counter + 1'b1;
            if (i_input_sof)
                input_stamp <= cycle_counter;
            if (i_output_sof)
                o_pipeline_cycles <= latency_diff[15:0];
        end
    end
endmodule
