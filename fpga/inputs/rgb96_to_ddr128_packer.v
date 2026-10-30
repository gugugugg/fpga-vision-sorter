`timescale 1ns/1ps

// Lossless 96-to-128 stream gearbox. Four 4PPC words (384 bits) become three
// DDR words (384 bits). State is explicitly reset and frame_start is aligned
// with the first valid output word.
module rgb96_to_ddr128_packer (
    input  wire         i_clk,
    input  wire         i_rst_n,
    input  wire         i_frame_start,
    input  wire         i_valid,
    input  wire [95:0]  i_data,
    output reg          o_frame_start,
    output reg          o_valid,
    output reg  [127:0] o_data
);
    reg [1:0] phase;
    reg [95:0] previous;
    reg frame_pending;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            phase <= 2'd0;
            previous <= 96'd0;
            frame_pending <= 1'b0;
            o_frame_start <= 1'b0;
            o_valid <= 1'b0;
            o_data <= 128'd0;
        end else begin
            o_frame_start <= 1'b0;
            o_valid <= 1'b0;

            if (i_valid) begin
                if (i_frame_start) begin
                    previous <= i_data;
                    phase <= 2'd1;
                    frame_pending <= 1'b1;
                end else begin
                    case (phase)
                        2'd0: begin
                            previous <= i_data;
                            phase <= 2'd1;
                        end
                        2'd1: begin
                            o_data <= {previous,i_data[95:64]};
                            o_valid <= 1'b1;
                            o_frame_start <= frame_pending;
                            frame_pending <= 1'b0;
                            previous <= i_data;
                            phase <= 2'd2;
                        end
                        2'd2: begin
                            o_data <= {previous[63:0],i_data[95:32]};
                            o_valid <= 1'b1;
                            previous <= i_data;
                            phase <= 2'd3;
                        end
                        default: begin
                            o_data <= {previous[31:0],i_data};
                            o_valid <= 1'b1;
                            previous <= i_data;
                            phase <= 2'd0;
                        end
                    endcase
                end
            end
        end
    end
endmodule
