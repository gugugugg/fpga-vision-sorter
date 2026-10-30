`timescale 1ns/1ps
// Board strap address is 4, not address-0/broadcast alias. Fail closed on
// missing turnaround, unexpected PHY ID, or configuration readback mismatch.
module network_phy_init #(
    parameter [4:0] PHY_ADDR = 5'd4,
    parameter integer RESET_CYCLES = 1000000, // 20 ms; datasheet >= 10 ms
    parameter integer SETTLE_CYCLES = 1000000, // conservative post-release wait
    parameter integer POLL_CYCLES = 500000,
    parameter integer MDIO_HALF_PERIOD = 25
)(
    input i_clk, i_rst_n,
    output reg o_phy_rst_n,
    output o_mdc, inout io_mdio,
    output reg o_init_done, o_link_gigabit, o_fault
);
    reg [5:0] step;
    integer timer;
    reg waiting, start;
    reg [15:0] expected, phy_id1, phy_id2, phy_status;
    reg cmd_read;
    reg [4:0] cmd_reg;
    reg [15:0] cmd_data;
    wire busy, done, error;
    wire [15:0] read_data;
    network_mdio_master #(.HALF_PERIOD(MDIO_HALF_PERIOD)) u_mdio(
        .i_clk(i_clk), .i_rst_n(i_rst_n), .i_start(start), .i_read(cmd_read),
        .i_phy_addr(PHY_ADDR), .i_reg_addr(cmd_reg), .i_write_data(cmd_data),
        .o_busy(busy), .o_done(done), .o_error(error), .o_read_data(read_data),
        .o_mdc(o_mdc), .io_mdio(io_mdio));
    always @* begin
        cmd_read = 1; cmd_reg = 5'h1f; cmd_data = expected;
        case (step)
            0: cmd_reg = 2;
            1: cmd_reg = 3;
            2: begin cmd_read = 0; cmd_reg = 5'h1e; cmd_data = 16'h000c; end
            4,8,12: cmd_read = 0;
            6: begin cmd_read = 0; cmd_reg = 5'h1e; cmd_data = 16'h001e; end
            10: begin cmd_read = 0; cmd_reg = 5'h1e; cmd_data = 16'h0027; end
            14,16: cmd_reg = 4;
            15: begin cmd_read = 0; cmd_reg = 4; end
            17,19: cmd_reg = 9;
            18: begin cmd_read = 0; cmd_reg = 9; end
            20: begin cmd_read = 0; cmd_reg = 0; cmd_data = 16'h1200; end
            21: cmd_reg = 0;
            22: begin cmd_read = 0; cmd_reg = 5'h1e; cmd_data = 0; end
            23: cmd_reg = 5'h11;
            default: begin end
        endcase
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_phy_rst_n <= 0; o_init_done <= 0; o_link_gigabit <= 0; o_fault <= 0;
            timer <= 0; step <= 0; waiting <= 0; start <= 0;
            expected <= 0; phy_id1 <= 0; phy_id2 <= 0; phy_status <= 0;
        end else begin
            start <= 0;
            if (!o_phy_rst_n) begin
                if (timer == RESET_CYCLES-1) begin o_phy_rst_n <= 1; timer <= 0; end
                else timer <= timer+1;
            end else if (!o_fault) begin
                if (!waiting && !start && !busy) begin
                    if (timer == ((o_init_done ? POLL_CYCLES : SETTLE_CYCLES)-1)) begin
                        timer <= 0; start <= 1; waiting <= 1;
                    end else timer <= timer+1;
                end else if (done) begin
                    waiting <= 0;
                    // Only the first transaction needs the post-reset settle wait.
                    timer <= SETTLE_CYCLES-1;
                    if (error) begin o_fault <= 1; o_link_gigabit <= 0; o_init_done <= 0; end
                    else begin
                        case (step)
                            0: begin phy_id1 <= read_data; if (read_data != 0) o_fault <= 1; end
                            1: begin phy_id2 <= read_data; if (read_data != 16'h010a) o_fault <= 1; end
                            3: expected <= (read_data & ~16'h10f1) | 16'h0021;
                            5: if ((read_data & 16'h10f1) != (expected & 16'h10f1)) o_fault <= 1;
                            // YT8511C V1.11 p57: EXT 1E[15:10]=000001
                            // adds 200 ps RX clock delay, preserving ADC timer.
                            7: expected <= (read_data & 16'h03ff) | 16'h0400;
                            9: if ((read_data & 16'hfc00) != 16'h0400) o_fault <= 1;
                            11: expected <= read_data & 16'h7fff; // sleep off; RO PLL bit preserved
                            13: if (read_data[15]) o_fault <= 1;
                            14: expected <= (read_data & ~16'h0dff) | 16'h0001;
                            16: if ((read_data & 16'h0dff) != (expected & 16'h0dff)) o_fault <= 1;
                            17: expected <= (read_data & ~16'h0300) | 16'h0200;
                            19: if ((read_data & 16'h0300) != 16'h0200) o_fault <= 1;
                            21: if (!read_data[12]) o_fault <= 1;
                            22: begin o_init_done <= 1; timer <= 0; end
                            23: begin
                                phy_status <= read_data;
                                // resolved + link + 1000 Mbps + full duplex
                                o_link_gigabit <= ((read_data & 16'hec00) == 16'hac00);
                                timer <= 0;
                            end
                        endcase
                        if (step != 23) step <= step+1'b1;
                    end
                end
            end
        end
    end
endmodule
