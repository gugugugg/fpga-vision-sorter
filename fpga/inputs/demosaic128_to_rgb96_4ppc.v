`timescale 1ns/1ps

// Convert the vendor demosaic output (four 32-bit lanes, RGB in each lane's
// low 24 bits) to the contest core's four packed RGB888 pixels.
module demosaic128_to_rgb96_4ppc (
    input  wire [127:0] i_data,
    output wire [95:0]  o_rgb
);
    assign o_rgb = {i_data[23:0], i_data[55:32],
                    i_data[87:64], i_data[119:96]};
endmodule
