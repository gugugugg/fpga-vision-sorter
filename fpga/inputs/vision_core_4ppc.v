`timescale 1ns/1ps

// Board-side vision core implementing contest base, expansion 1/2 and
// advanced 1/3/5 without any mechanical-arm dependency.
module vision_core_4ppc #(
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600,
    parameter AE_EXP_MAX = 2246,
    parameter AE_EXP_INIT = 1500,
    parameter AE_GAIN_MAX = 89,
    parameter ENABLE_DETECTION = 1,
    parameter ENABLE_LEGACY_STYLES = 1,
    parameter PREFER_NEUTRAL_AWB = 1
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_sof,
    input  wire        i_eof,
    input  wire        i_valid,
    input  wire [95:0] i_rgb,
    input  wire [2:0]  i_mode,
    input  wire [9:0]  i_edge_threshold,

    output wire        o_sof,
    output reg         o_eof,
    output wire        o_valid,
    output reg  [95:0] o_rgb,

    output wire [15:0] o_exposure,
    output wire [15:0] o_gain_index,
    output wire        o_ae_update,
    output wire        o_ae_stable,
    output wire [9:0]  o_awb_r_gain_q8,
    output wire [9:0]  o_awb_b_gain_q8,
    output reg         o_awb_stable,
    output wire [15:0] o_pipeline_cycles,

    output wire        o_target_update,
    output wire        o_target_valid,
    output wire [3:0]  o_target_class,
    output wire [10:0] o_center_x,
    output wire [9:0]  o_center_y,
    output wire [10:0] o_bbox_x0,
    output wire [10:0] o_bbox_x1,
    output wire [9:0]  o_bbox_y0,
    output wire [9:0]  o_bbox_y1,
    output wire [19:0] o_target_area,
    output wire [7:0]  o_target_quality
);
    wire stats_valid;
    wire [31:0] luma_sum, r_sum, g_sum, b_sum;
    wire [19:0] luma_count, awb_count;
    wire awb_gain_valid;
    reg [10:0] eof_pipe;
    reg [2:0] mode_active;
    reg [9:0] threshold_active;
    reg [9:0] awb_prev_r_gain;
    reg [9:0] awb_prev_b_gain;
    reg [1:0] awb_stable_count;
    wire reference_frame=mode_active==3'd7;
    reg stats_reference;
    always @(posedge i_clk or negedge i_rst_n)
        if(!i_rst_n)stats_reference<=0;
        else if(i_sof && i_valid)stats_reference<=reference_frame;

    wire [9:0] awb_r_step = (o_awb_r_gain_q8 >= awb_prev_r_gain)
                             ? (o_awb_r_gain_q8-awb_prev_r_gain)
                             : (awb_prev_r_gain-o_awb_r_gain_q8);
    wire [9:0] awb_b_step = (o_awb_b_gain_q8 >= awb_prev_b_gain)
                             ? (o_awb_b_gain_q8-awb_prev_b_gain)
                             : (awb_prev_b_gain-o_awb_b_gain_q8);

    awb_controller u_awb_ctrl(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_stats_valid(stats_valid && !reference_frame && !stats_reference),
        .i_r_sum(r_sum),.i_g_sum(g_sum),.i_b_sum(b_sum),.i_pixel_count(awb_count),
        .o_r_gain_q8(o_awb_r_gain_q8),.o_b_gain_q8(o_awb_b_gain_q8),
        .o_gain_valid(awb_gain_valid)
    );

    ae_controller #(.EXP_INIT(AE_EXP_INIT),.EXP_MAX(AE_EXP_MAX),.GAIN_MAX(AE_GAIN_MAX)) u_ae_ctrl(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_enable(!reference_frame),
        .i_stats_valid(stats_valid && !stats_reference),.i_luma_sum(luma_sum),.i_luma_count(luma_count),
        .o_exposure(o_exposure),.o_gain_index(o_gain_index),
        .o_update(o_ae_update),.o_stable(o_ae_stable)
    );

    wire awb_sof, awb_valid;
    wire [95:0] awb_rgb;
    // Pixel-exact pre-AWB branch for mode 0. These two enabled registers
    // mirror the latency/valid behaviour of awb_apply plus the raw register
    // in gradient_stylize, so all modes retain one common alignment pipeline.
    reg [95:0] input_rgb_d0;
    reg [95:0] input_rgb_d1;
    awb_apply_4ppc u_awb_apply(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_sof(i_sof),.i_valid(i_valid),.i_rgb(i_rgb),
        .i_r_gain_q8(i_mode==3'd7 ? 10'd256 : o_awb_r_gain_q8),
        .i_b_gain_q8(i_mode==3'd7 ? 10'd256 : o_awb_b_gain_q8),
        .o_sof(awb_sof),.o_valid(awb_valid),.o_rgb(awb_rgb)
    );

    // Reuse balanced pixels for selection, aligned pre-AWB pixels for gain
    // targets and exposure statistics. No duplicate gain multipliers.
    vision_stats_4ppc #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),
        .PREFER_NEUTRAL_AWB(PREFER_NEUTRAL_AWB)) u_stats(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_sof(awb_sof),.i_valid(awb_valid),
        .i_rgb(input_rgb_d0),.i_neutral_rgb(awb_rgb),
        .o_stats_valid(stats_valid),.o_luma_sum(luma_sum),.o_luma_count(luma_count),
        .o_r_sum(r_sum),.o_g_sum(g_sum),.o_b_sum(b_sum),.o_awb_count(awb_count)
    );

    wire stylize_sof, stylize_valid;
    wire [95:0] raw_rgb, gray_rgb, binary_rgb, edge_rgb, ukiyo_v1_rgb;
    wire v2_sof, v2_valid;
    wire [95:0] ukiyo_v2_rgb;
    wire style_ink_select = i_sof ? (i_mode == 3'd6) : (mode_active == 3'd6);
    generate if(ENABLE_LEGACY_STYLES) begin: g_legacy_styles
    gradient_stylize_4ppc #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_stylize(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_sof(awb_sof),.i_valid(awb_valid),.i_rgb(awb_rgb),
        .i_edge_threshold(threshold_active),
        .o_sof(stylize_sof),.o_valid(stylize_valid),.o_raw(raw_rgb),.o_gray(gray_rgb),
        .o_binary(binary_rgb),.o_edge(edge_rgb),.o_ukiyo(ukiyo_v1_rgb)
    );

    ukiyo_stylize_4ppc_v2 #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_ukiyo_v2(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_sof(awb_sof),.i_valid(awb_valid),
        .i_rgb(awb_rgb),.i_edge_threshold(threshold_active),.i_ink_style(style_ink_select),
        .o_sof(v2_sof),.o_valid(v2_valid),.o_rgb(ukiyo_v2_rgb)
    );
    end else begin: g_no_legacy_styles
      // Preserve the old 10-register control latency after AWB, without
      // instantiating either old edge/palette engine or its line memories.
      reg [9:0] sof_delay,valid_delay;reg [95:0] raw_delay;
      always @(posedge i_clk or negedge i_rst_n)
        if(!i_rst_n) begin sof_delay<=0;valid_delay<=0;raw_delay<=0;end
        else begin
          sof_delay<={sof_delay[8:0],awb_sof};
          valid_delay<={valid_delay[8:0],awb_valid};
          if(awb_valid) raw_delay<=awb_rgb;
        end
      assign v2_sof=sof_delay[9];assign v2_valid=valid_delay[9];
      assign raw_rgb=raw_delay;assign gray_rgb=0;assign binary_rgb=0;
      assign edge_rgb=0;assign ukiyo_v1_rgb=0;assign ukiyo_v2_rgb=0;
      assign stylize_sof=0;assign stylize_valid=0;
    end endgenerate

    // V2 has nine more registered stages than the original stylize block.
    // The mode is frame-latched, so select one non-V2 stream first and delay
    // only that 96-bit bus.  Keeping four parallel nine-stage delay lines
    // wastes 2592 flip-flops without adding any observable capability.
    reg [95:0] base_rgb_selected;
    reg [95:0] base_delay0,base_delay1,base_delay2,base_delay3,base_delay4;
    reg [95:0] base_delay5,base_delay6,base_delay7,base_delay8;
    assign o_sof = v2_sof;
    assign o_valid = v2_valid;

    // The detector receives the colour-corrected stream and exports metadata.
    generate if(ENABLE_DETECTION) begin: g_detection
    color_shape_detector_4ppc #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_target(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_sof(awb_sof),.i_valid(awb_valid),.i_rgb(awb_rgb),
        .o_target_update(o_target_update),
        .o_target_valid(o_target_valid),.o_target_class(o_target_class),
        .o_center_x(o_center_x),.o_center_y(o_center_y),
        .o_bbox_x0(o_bbox_x0),.o_bbox_x1(o_bbox_x1),
        .o_bbox_y0(o_bbox_y0),.o_bbox_y1(o_bbox_y1),
        .o_area(o_target_area),.o_quality(o_target_quality)
    );

    end else begin: g_no_detection
        // Metadata publication must survive detector removal: DDR binding and
        // the network camera heartbeat depend on one event per completed frame.
        assign o_target_update=stats_valid;
        assign o_target_valid=0;assign o_target_class=0;
        assign o_center_x=0;assign o_center_y=0;
        assign o_bbox_x0=0;assign o_bbox_x1=0;
        assign o_bbox_y0=0;assign o_bbox_y1=0;
        assign o_target_area=0;assign o_target_quality=0;
    end endgenerate

    always @(*) begin
        case (mode_active)
            3'd0: base_rgb_selected = input_rgb_d1;
            3'd1: base_rgb_selected = gray_rgb;
            3'd2: base_rgb_selected = binary_rgb;
            3'd3: base_rgb_selected = edge_rgb;
            3'd5: base_rgb_selected = raw_rgb;
            default: base_rgb_selected = raw_rgb;
        endcase
        o_rgb = ((mode_active == 3'd4)||(mode_active == 3'd6)) ? ukiyo_v2_rgb : base_delay8;
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            eof_pipe     <= 0;
            o_eof        <= 1'b0;
            o_awb_stable <= 1'b0;
            awb_prev_r_gain <= 10'd256;
            awb_prev_b_gain <= 10'd256;
            awb_stable_count <= 0;
            input_rgb_d0 <= 0;
            input_rgb_d1 <= 0;
            mode_active  <= 3'd0;
            threshold_active <= 10'd72;
            base_delay0<=0;base_delay1<=0;base_delay2<=0;base_delay3<=0;base_delay4<=0;
            base_delay5<=0;base_delay6<=0;base_delay7<=0;base_delay8<=0;
        end else begin
            eof_pipe[0] <= i_eof;
            eof_pipe[1] <= eof_pipe[0];
            eof_pipe[2] <= eof_pipe[1];
            eof_pipe[3] <= eof_pipe[2];
            eof_pipe[4] <= eof_pipe[3];
            eof_pipe[5] <= eof_pipe[4];
            eof_pipe[6] <= eof_pipe[5];
            eof_pipe[7] <= eof_pipe[6];
            eof_pipe[8] <= eof_pipe[7];
            eof_pipe[9] <= eof_pipe[8];
            eof_pipe[10] <= eof_pipe[9];
            // Pixel data is physically ten registers after the input; the
            // latency meter reports eleven because it observes registered SOF
            // on the following edge. Tap pipe[9] so EOL stays with its word.
            o_eof <= eof_pipe[9];
            base_delay0<=base_rgb_selected;base_delay1<=base_delay0;base_delay2<=base_delay1;
            base_delay3<=base_delay2;base_delay4<=base_delay3;base_delay5<=base_delay4;
            base_delay6<=base_delay5;base_delay7<=base_delay6;base_delay8<=base_delay7;
            if (i_valid)
                input_rgb_d0 <= i_rgb;
            if (awb_valid)
                input_rgb_d1 <= input_rgb_d0;
            if(stats_valid && (awb_count<4096 || r_sum==0 || b_sum==0 || g_sum==0)) begin
                o_awb_stable<=0;awb_stable_count<=0;
            end else if (awb_gain_valid) begin
                awb_prev_r_gain <= o_awb_r_gain_q8;
                awb_prev_b_gain <= o_awb_b_gain_q8;
                if ((awb_r_step <= 10'd2) && (awb_b_step <= 10'd2)) begin
                    if (awb_stable_count == 2) begin
                        o_awb_stable <= 1'b1;
                    end else begin
                        awb_stable_count <= awb_stable_count + 1'b1;
                        o_awb_stable <= 1'b0;
                    end
                end else begin
                    awb_stable_count <= 0;
                    o_awb_stable <= 1'b0;
                end
            end
            if (i_sof) begin
                mode_active <= i_mode;
                threshold_active <= i_edge_threshold;
            end
        end
    end

    stream_latency_meter u_latency(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_input_sof(i_sof),.i_output_sof(o_sof),
        .o_pipeline_cycles(o_pipeline_cycles)
    );

endmodule
