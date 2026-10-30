`timescale 1ns/1ps

// Toggle-handshake CDC from the CSI/ISP clock to the 24 MHz camera I2C clock.
// The multi-bit values remain stable until the destination acknowledges them.
module ae_i2c_cdc_bridge (
    input  wire        i_src_clk,
    input  wire        i_src_rst_n,
    input  wire        i_src_update,
    input  wire [15:0] i_src_exposure,
    input  wire [15:0] i_src_gain,
    output wire        o_src_busy,

    input  wire        i_dst_clk,
    input  wire        i_dst_rst_n,
    input  wire        i_cam_cfg_done,
    input  wire        i_ae_cfg_done,
    output reg         o_ae_req,
    output reg  [15:0] o_ae,
    output reg  [15:0] o_ag
);
    reg req_toggle;
    reg [15:0] src_exposure_hold;
    reg [15:0] src_gain_hold;
    reg [15:0] pending_exposure;
    reg [15:0] pending_gain;
    reg pending_valid;
    reg have_value;
    reg ack_toggle;
    (* async_reg="true",syn_preserve=1 *) reg ack_sync0, ack_sync1;
    (* async_reg="true",syn_preserve=1 *) reg ready_sync0, ready_sync1;
    (* async_reg="true",syn_preserve=1 *) reg req_sync0, req_sync1;
    (* async_reg="true",syn_preserve=1 *) reg [15:0] exp_sync0, exp_sync1;
    (* async_reg="true",syn_preserve=1 *) reg [15:0] gain_sync0, gain_sync1;
    reg capture_pending;
    reg wait_complete;
    reg seen_busy;
    (* async_reg="true",syn_preserve=1 *) reg src_reset_sync0,src_reset_sync1;
    reg [1:0] dst_startup;
    reg dst_ready_state;
    (* async_reg="true",syn_preserve=1 *) reg src_channel_reset_sync0,src_channel_reset_sync1;
    wire channel_rst_n = i_src_rst_n && i_dst_rst_n;
    // These reset notifications asynchronously remember even a pulse while
    // the receiving clock is stopped, then release in that clock domain.
    wire dst_ready = dst_ready_state && src_reset_sync1;


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
            src_exposure_hold <= 16'd1500;
            src_gain_hold <= 16'd16;
            pending_exposure <= 16'd1500;
            pending_gain <= 16'd16;
            pending_valid <= 1'b0;
            have_value <= 1'b0;
        end else begin

            if (!ready_sync1 || !src_channel_reset_sync1) begin
                if (i_src_update) begin
                    pending_exposure <= i_src_exposure;
                    pending_gain <= i_src_gain;
                    pending_valid <= 1'b1;
                    have_value <= 1'b1;
                end else if (have_value && !pending_valid) begin
                    pending_exposure <= src_exposure_hold;
                    pending_gain <= src_gain_hold;
                    pending_valid <= 1'b1;
                end
            end else if (i_src_update) begin
                if (!in_flight) begin
                    // The newest update wins if an older queued value has not
                    // yet started.
                    src_exposure_hold <= i_src_exposure;
                    src_gain_hold <= i_src_gain;
                    pending_valid <= 1'b0;
                    have_value <= 1'b1;
                end else begin
                    pending_exposure <= i_src_exposure;
                    pending_gain <= i_src_gain;
                    pending_valid <= 1'b1;
                    have_value <= 1'b1;
                end
            end else if (!in_flight && pending_valid) begin
                src_exposure_hold <= pending_exposure;
                src_gain_hold <= pending_gain;
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
            else if (wait_complete && seen_busy && i_ae_cfg_done)
                ack_toggle <= req_sync1;
        end
    end

    always @(posedge i_dst_clk or negedge i_dst_rst_n) begin
        if (!i_dst_rst_n) begin
            exp_sync0 <= 16'd1500;
            exp_sync1 <= 16'd1500;
            gain_sync0 <= 16'd16;
            gain_sync1 <= 16'd16;
            o_ae_req <= 1'b0;
            o_ae <= 16'd1500;
            o_ag <= 16'd16;
            capture_pending <= 1'b0;
            wait_complete <= 1'b0;
            seen_busy <= 1'b0;
            dst_startup <= 2'd0;
            dst_ready_state <= 1'b0;
        end else begin
            exp_sync0 <= src_exposure_hold;
            exp_sync1 <= exp_sync0;
            gain_sync0 <= src_gain_hold;
            gain_sync1 <= gain_sync0;
            o_ae_req <= 1'b0;

            if (!src_reset_sync1) begin
                dst_startup <= 2'd0;
                dst_ready_state <= 1'b0;
                capture_pending <= 1'b0;
                wait_complete <= 1'b0;
                seen_busy <= 1'b0;
            end else if (!dst_ready) begin
                capture_pending <= 1'b0;
                if (dst_startup == 2'd3)
                    dst_ready_state <= 1'b1;
                else
                    dst_startup <= dst_startup + 1'b1;
            end else if (capture_pending) begin
                // Launch only after bundled data has had one extra full edge.
                // If the camera stops being idle, keep the request pending.
                if (i_cam_cfg_done && i_ae_cfg_done) begin
                    o_ae <= exp_sync1;
                    o_ag <= gain_sync1;
                    o_ae_req <= 1'b1;
                    wait_complete <= 1'b1;
                    capture_pending <= 1'b0;
                    seen_busy <= 1'b0;
                end
            end else if (!wait_complete && (req_sync1 != ack_toggle)
                && i_cam_cfg_done && i_ae_cfg_done) begin
                capture_pending <= 1'b1;
            end else if (wait_complete) begin
                if (!i_ae_cfg_done)
                    seen_busy <= 1'b1;
                if (seen_busy && i_ae_cfg_done) begin
                    wait_complete <= 1'b0;
                    seen_busy <= 1'b0;
                end
            end
        end
    end
endmodule
