`timescale 1ns/1ps

// Atomic multi-bit clock-domain crossing using a bundled-data toggle
// handshake.  The source mailbox keeps the newest pending value while a
// transfer is in flight.  The destination data changes only with
// o_dst_update and remains stable between updates.
//
// The ready handshake deliberately suppresses traffic while either domain is
// starting.  If the destination is reset independently, the last transmitted
// value is queued again after it becomes ready, rather than interpreting the
// reset toggle value as a new request.
module cdc_bus_toggle #(
    parameter WIDTH = 1,
    // The request may reach its first synchronizer before the routed data.
    // One additional destination edge permits that legal one-edge skew.
    parameter SETTLE_DATA = 1
)(
    input  wire             i_src_clk,
    input  wire             i_src_rst_n,
    input  wire             i_src_update,
    input  wire [WIDTH-1:0] i_src_data,
    output wire             o_src_busy,

    input  wire             i_dst_clk,
    input  wire             i_dst_rst_n,
    output reg              o_dst_update,
    output reg  [WIDTH-1:0] o_dst_data
);
    reg req_toggle;
    reg ack_toggle;
    reg [WIDTH-1:0] tx_hold;
    reg [WIDTH-1:0] pending_data;
    reg pending_valid;
    reg have_value;

    (* async_reg="true", syn_preserve=1 *) reg ack_sync0, ack_sync1;
    (* async_reg="true", syn_preserve=1 *) reg ready_sync0, ready_sync1;
    (* async_reg="true", syn_preserve=1 *) reg req_sync0, req_sync1;
    (* async_reg="true", syn_preserve=1 *) reg [WIDTH-1:0] data_sync0, data_sync1;
    (* async_reg="true", syn_preserve=1 *) reg src_reset_sync0, src_reset_sync1;
    reg [1:0] dst_startup;
    reg dst_ready_state;
    (* async_reg="true",syn_preserve=1 *) reg src_channel_reset_sync0,src_channel_reset_sync1;
    wire channel_rst_n = i_src_rst_n && i_dst_rst_n;
    // These reset notifications asynchronously remember even a pulse while
    // the receiving clock is stopped, then release in that clock domain.
    wire dst_ready = dst_ready_state && src_reset_sync1;

    reg capture_pending;

    wire in_flight = (req_toggle != ack_sync1);
    assign o_src_busy = in_flight | pending_valid | ~ready_sync1;

    always @(posedge i_src_clk or negedge channel_rst_n) begin
        if (!channel_rst_n) begin
            src_channel_reset_sync0 <= 1'b0;
            src_channel_reset_sync1 <= 1'b0;
        end else begin
            src_channel_reset_sync0 <= 1'b1;
            src_channel_reset_sync1 <= src_channel_reset_sync0;
        end
    end

    always @(posedge i_src_clk or negedge src_channel_reset_sync1) begin
        if (!src_channel_reset_sync1) begin
            req_toggle <= 1'b0;
            ack_sync0 <= 1'b0;
            ack_sync1 <= 1'b0;
            ready_sync0 <= 1'b0;
            ready_sync1 <= 1'b0;
        end else begin
            ack_sync0 <= ack_toggle;
            ack_sync1 <= ack_sync0;
            ready_sync0 <= dst_ready;
            ready_sync1 <= ready_sync0;
            if (!ready_sync1)
                req_toggle <= ack_sync1;
            else if (!in_flight && (i_src_update || pending_valid))
                req_toggle <= ~req_toggle;
        end
    end

    always @(posedge i_src_clk or negedge i_src_rst_n) begin
        if (!i_src_rst_n) begin
            tx_hold     <= {WIDTH{1'b0}};
            pending_data<= {WIDTH{1'b0}};
            pending_valid <= 1'b0;
            have_value   <= 1'b0;
        end else begin

            if (!ready_sync1 || !src_channel_reset_sync1) begin
                // Cancel any old phase relationship while the destination is
                // down and make sure an interrupted value is retransmitted.
                if (i_src_update) begin
                    pending_data  <= i_src_data;
                    pending_valid <= 1'b1;
                    have_value    <= 1'b1;
                end else if (have_value && !pending_valid) begin
                    pending_data  <= tx_hold;
                    pending_valid <= 1'b1;
                end
            end else if (i_src_update) begin
                if (!in_flight) begin
                    // A new value supersedes any older queued value.
                    tx_hold       <= i_src_data;
                    pending_valid <= 1'b0;
                    have_value    <= 1'b1;
                end else begin
                    pending_data  <= i_src_data;
                    pending_valid <= 1'b1;
                    have_value    <= 1'b1;
                end
            end else if (!in_flight && pending_valid) begin
                tx_hold       <= pending_data;
                pending_valid <= 1'b0;
            end
        end
    end

    always @(posedge i_dst_clk or negedge channel_rst_n) begin
        if (!channel_rst_n) begin
            src_reset_sync0 <= 1'b0;
            src_reset_sync1 <= 1'b0;
        end else begin
            src_reset_sync0 <= 1'b1;
            src_reset_sync1 <= src_reset_sync0;
        end
    end

    always @(posedge i_dst_clk or negedge src_reset_sync1) begin
        if (!src_reset_sync1) begin
            req_sync0 <= 1'b0;
            req_sync1 <= 1'b0;
            ack_toggle <= 1'b0;
        end else begin
            req_sync0 <= req_toggle;
            req_sync1 <= req_sync0;
            if (!dst_ready)
                ack_toggle <= req_sync1;
            else if ((SETTLE_DATA && capture_pending)
                || (!SETTLE_DATA && req_sync1 != ack_toggle))
                ack_toggle <= req_sync1;
        end
    end

    always @(posedge i_dst_clk or negedge i_dst_rst_n) begin
        if (!i_dst_rst_n) begin
            data_sync0     <= {WIDTH{1'b0}};
            data_sync1     <= {WIDTH{1'b0}};
            dst_startup    <= 2'd0;
            dst_ready_state <= 1'b0;
            capture_pending<= 1'b0;
            o_dst_update   <= 1'b0;
            o_dst_data     <= {WIDTH{1'b0}};
        end else begin
            data_sync0      <= tx_hold;
            data_sync1      <= data_sync0;
            o_dst_update    <= 1'b0;

            if (!src_reset_sync1) begin
                dst_startup <= 2'd0;
                dst_ready_state <= 1'b0;
                capture_pending <= 1'b0;
            end else if (!dst_ready) begin
                capture_pending <= 1'b0;
                // Allow the request and bundled data synchronizers to settle,
                // then establish a no-request baseline.
                if (dst_startup == 2'd3) begin
                    dst_ready_state <= 1'b1;
                end else begin
                    dst_startup <= dst_startup + 1'b1;
                end
            end else if (SETTLE_DATA && capture_pending) begin
                o_dst_data   <= data_sync1;
                o_dst_update <= 1'b1;
                capture_pending <= 1'b0;
            end else if (req_sync1 != ack_toggle) begin
                if (SETTLE_DATA) capture_pending <= 1'b1;
                else begin
                    o_dst_data   <= data_sync1;
                    o_dst_update <= 1'b1;
                end
            end
        end
    end
endmodule
