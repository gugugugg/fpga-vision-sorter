`timescale 1ns/1ps

// Low-cost HDMI-side algorithm gallery.  The contest perception pipeline stays
// 4PPC; these presentation modes run after DDR at 1PPC and share one pixel
// transform datapath.  Modes are frame-latched so changing buttons cannot tear
// one frame into two colour styles.
module display_filter_1ppc #(
    // Set to zero when i_mode is already committed only at frame boundaries.
    parameter LATCH_MODE = 1,
    parameter IMG_WIDTH = 1024,
    parameter IMG_HEIGHT = 600,
    parameter ENABLE_HEAVY_FILTERS = 1,
    parameter ENABLE_EVA = 0,
    parameter ENABLE_CARTOON = 0,
    parameter ENABLE_CARTOON_TEMPORAL = 0,
    parameter ENABLE_BINARY_DENOISE = 0,
    parameter ENABLE_ADPT_STABLE = 0,
    parameter ADPT_HYSTERESIS = 6,
    parameter USE_CONFIGURED_THRESHOLD = 0,
    parameter [63:0] MODE_BITMAP = 64'hffffffffffffffff
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_vsync,
    input  wire        i_hsync,
    input  wire        i_de,
    input  wire [23:0] i_rgb,
    input  wire [7:0]  i_mode,
    output wire        o_vsync,
    output wire        o_hsync,
    output wire        o_de,
    output wire [23:0] o_rgb,
    input wire [9:0] i_threshold,
    input wire i_history_valid,input wire [31:0] i_history,
    output wire o_history_read,o_history_write,output wire [31:0] o_history
);
    reg legacy_vsync,legacy_hsync,legacy_de;
    reg [23:0] legacy_rgb;
    reg vsync_d;
    reg [7:0] mode_active;
    reg vsync_stage1,hsync_stage1,de_stage1;
    reg [23:0] rgb_stage1;
    reg [7:0] mode_stage1;
    reg vsync_stage2,hsync_stage2,de_stage2;
    reg [23:0] rgb_stage2;reg [7:0] mode_stage2;
    wire frame_start = i_vsync & ~vsync_d;
    wire [7:0] selected_mode = LATCH_MODE ? mode_active : i_mode;
    reg [9:0] threshold_active,threshold_stage1,threshold_stage2;
    wire [9:0] configured_threshold=USE_CONFIGURED_THRESHOLD ? i_threshold:10'd128;
    wire [9:0] selected_threshold=LATCH_MODE ? threshold_active:configured_threshold;

    wire [23:0] eva_rgb,eva_clean_rgb,neighbourhood_rgb,adaptive_rgb;
    wire [30:0] adaptive_info;
    reg pre_vsync_d;
    wire pre_frame_start=vsync_stage2 && !pre_vsync_d;
    always @(posedge i_clk or negedge i_rst_n)
        if(!i_rst_n)pre_vsync_d<=0;else pre_vsync_d<=vsync_stage2;
    generate if(ENABLE_EVA) begin: g_eva
      eva_denoise_1ppc u_denoise(.clk(i_clk),.rst_n(i_rst_n),
        .frame_start(frame_start),.de(i_de),.rgb(i_rgb),.clean_rgb(eva_clean_rgb));
      eva_color_map_1ppc u_eva_map(.clk(i_clk),.rst_n(i_rst_n),
        .rgb(eva_clean_rgb),.mapped_rgb(eva_rgb));
    end else begin: g_no_eva
      assign eva_rgb=24'd0;
      assign eva_clean_rgb=i_rgb;
    end endgenerate

    function [7:0] soft_cel;
      input [7:0] v;reg [8:0] sum;
      begin sum={1'b0,v}+{1'b0,v[7:5],5'd16};soft_cel=sum[8:1];end
    endfunction
    function [7:0] half_ink;
      input [7:0] v;input [7:0] ink;reg [8:0] sum;
      begin sum={1'b0,v}+{1'b0,ink};half_ink=sum[8:1];end
    endfunction
    // Continuous soft levels are fused into the colour ROM vertices. A
    // second channel quantizer here would recreate 17-code noise jumps.
    wire [23:0] cel_rgb=eva_rgb;
    wire [23:0] eva_finished=neighbourhood_rgb[7:0]>=72?24'h181022:
      neighbourhood_rgb[7:0]>=40?
       {half_ink(cel_rgb[23:16],8'h18),half_ink(cel_rgb[15:8],8'h10),half_ink(cel_rgb[7:0],8'h22)}:cel_rgb;

    function [7:0] sat_add;
        input [7:0] value;
        input [7:0] amount;
        reg [8:0] sum;
        begin
            sum = {1'b0,value}+{1'b0,amount};
            sat_add = sum[8] ? 8'hff : sum[7:0];
        end
    endfunction

    function [7:0] sat_sub;
        input [7:0] value;
        input [7:0] amount;
        begin
            sat_sub = (value < amount) ? 8'd0 : value-amount;
        end
    endfunction

    function [7:0] luma8;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        reg [17:0] re,ge,be,sum;
        begin
            // 77=64+8+4+1, 150=128+16+4+2, 29=16+8+4+1.
            // Explicit extensions and shifts prevent TD from spending three
            // DSPs on a presentation-only colour transform.
            re={10'd0,r}; ge={10'd0,g}; be={10'd0,b};
            sum = (re<<6)+(re<<3)+(re<<2)+re
                + (ge<<7)+(ge<<4)+(ge<<2)+(ge<<1)
                + (be<<4)+(be<<3)+(be<<2)+be;
            luma8 = sum[15:8];
        end
    endfunction

    // Continuous S-curve: 0->0, 64->48, 192->208, 255->255.
    // The old 2x stretch discarded every channel <=64 and >=192.
    // Shoulder slopes preserve shadow/highlight steps without DSPs.
    function [7:0] contrast_curve;
        input [7:0] value;
        reg [9:0] delta;
        begin
            if(value < 8'd64) begin
                delta={2'd0,value};
                contrast_curve=((delta<<1)+delta)>>2;
            end else if(value < 8'd192) begin
                delta={2'd0,value}-10'd64;
                contrast_curve=10'd48+(((delta<<2)+delta)>>2);
            end else begin
                delta={2'd0,value}-10'd192;
                contrast_curve=10'd208+(((delta<<1)+delta)>>2);
            end
        end
    endfunction

    function [7:0] max3;
        input [7:0] a,b,c;
        reg [7:0] ab;
        begin ab=(a>b)?a:b; max3=(ab>c)?ab:c; end
    endfunction

    function [7:0] min3;
        input [7:0] a,b,c;
        reg [7:0] ab;
        begin ab=(a<b)?a:b; min3=(ab<c)?ab:c; end
    endfunction

    function [7:0] saturation_boost;
        input [7:0] value;
        input [7:0] grey;
        begin
            saturation_boost = (value>=grey)
                               ? sat_add(value,value-grey)
                               : sat_sub(value,grey-value);
        end
    endfunction

    function [7:0] avg2;
        input [7:0] a,b;
        reg [8:0] sum;
        begin sum={1'b0,a}+{1'b0,b}; avg2=sum[8:1]; end
    endfunction

    function [7:0] positive_excess;
        input [7:0] primary;
        input [7:0] other0;
        input [7:0] other1;
        reg signed [10:0] value;
        begin
            value=({3'd0,primary}<<<1)-{3'd0,other0}-{3'd0,other1};
            if(value<=0) positive_excess=8'd0;
            else if(value>=255) positive_excess=8'hff;
            else positive_excess=value[7:0];
        end
    endfunction

    function [23:0] pseudo_colour;
        input [7:0] y;
        reg [7:0] r,g,b;
        begin
            if(y < 8'd64) begin
                r=0; g={y[5:0],2'b00}; b=8'hff;
            end else if(y < 8'd128) begin
                r=0; g=8'hff; b=8'hff-{y[5:0],2'b00};
            end else if(y < 8'd192) begin
                r={y[5:0],2'b00}; g=8'hff; b=0;
            end else begin
                r=8'hff; g=8'hff-{y[5:0],2'b00}; b=0;
            end
            pseudo_colour={r,g,b};
        end
    endfunction

    function [23:0] transform;
        input [23:0] rgb;
        input [7:0] mode;
        reg [7:0] r,g,b,y,chroma,excess;
        begin
            r=rgb[23:16]; g=rgb[15:8]; b=rgb[7:0]; y=luma8(r,g,b);
            case(mode)
                5'd7:  transform={~r,~g,~b};                         // negative
                5'd8:  transform={sat_add(y,8'd40),sat_add(y,8'd15),sat_sub(y,8'd25)}; // sepia
                5'd9:  transform={{r[7:5],5'b10000},{g[7:5],5'b10000},{b[7:5],5'b10000}}; // posterise
                5'd10: transform={(r[7]?~r:r),(g[7]?~g:g),(b[7]?~b:b)}; // solarise
                5'd11: transform=pseudo_colour(y);
                5'd12: transform={sat_add(r,8'd24),sat_add(g,8'd6),sat_sub(b,8'd20)}; // warm
                5'd13: transform={sat_sub(r,8'd20),g,sat_add(b,8'd24)}; // cool
                5'd14: transform={contrast_curve(r),contrast_curve(g),contrast_curve(b)};
                5'd15: transform={sat_add(r,(8'hff-r)>>2),sat_add(g,(8'hff-g)>>2),
                                  sat_add(b,(8'hff-b)>>2)};            // gamma lift
                5'd16: transform={r,8'd0,8'd0};
                5'd17: transform={8'd0,g,8'd0};
                5'd18: transform={8'd0,8'd0,b};
                5'd19: transform={b,g,r};                             // swap R/B
                5'd20: transform=(y>=8'd128)?24'hffffff:24'h000000;
                5'd21: transform={{r[7:3],r[7:5]},
                                  {g[7:2],g[7:6]},
                                  {b[7:3],b[7:5]}};                  // RGB565 preview
                5'd22: transform={~y,~y,~y};                         // inverse luma
                5'd23: transform={saturation_boost(r,y),
                                  saturation_boost(g,y),
                                  saturation_boost(b,y)};            // saturation boost
                5'd24: transform={avg2(r,y),avg2(g,y),avg2(b,y)};    // desaturate
                5'd25: transform={sat_add(r,8'd32),sat_add(g,8'd32),sat_add(b,8'd32)};
                5'd26: transform={sat_sub(r,8'd32),sat_sub(g,8'd32),sat_sub(b,8'd32)};
                5'd27: begin excess=positive_excess(g,r,b); transform={8'd0,excess,8'd0}; end
                5'd28: begin excess=positive_excess(r,g,b); transform={excess,8'd0,8'd0}; end
                5'd29: begin chroma=max3(r,g,b)-min3(r,g,b); transform={chroma,chroma,chroma}; end
                5'd30: transform=(max3(r,g,b)>=8'd235)?24'hff2020:
                                  (max3(r,g,b)<=8'd20)?24'h2040ff:{y,y,y}; // channel clipping map
                5'd31: transform=({1'b0,r}>{1'b0,g}+9'd16 && {1'b0,r}>{1'b0,b}+9'd16)?24'hff0000:
                                  ({1'b0,g}>{1'b0,r}+9'd16 && {1'b0,g}>{1'b0,b}+9'd16)?24'h00ff00:
                                  ({1'b0,b}>{1'b0,r}+9'd16 && {1'b0,b}>{1'b0,g}+9'd16)?24'h0000ff:{y,y,y};
                default: transform=rgb;
            endcase
        end
    endfunction

    wire [7:0] input_y=luma8(i_rgb[23:16],i_rgb[15:8],i_rgb[7:0]);
    wire [7:0] equalized; reg [7:0] equalized_d,equalized_d2;
    generate if(ENABLE_HEAVY_FILTERS && MODE_BITMAP[3]) begin: g_equalizer
    histogram_equalizer_1ppc #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT)) u_equalizer(
      .clk(i_clk),.rst_n(i_rst_n),.frame_start(frame_start),.de(i_de),
      .y(input_y),.equalized(equalized),.lut_valid());
    end else begin: g_no_equalizer
        assign equalized=input_y;
    end endgenerate
    reg [4:0] operator_id;
    always @* case(selected_mode)
      5:operator_id=1;6:operator_id=31;7:operator_id=4;
      8:operator_id=ENABLE_BINARY_DENOISE?1:16;
      9:operator_id=ENABLE_ADPT_STABLE?1:16;
      10:operator_id=9;11:operator_id=10;
      13:operator_id=11;19:operator_id=30;default:operator_id=16;
    endcase
    localparam NEED_NEIGHBOURHOOD=MODE_BITMAP[5] || (ENABLE_BINARY_DENOISE && MODE_BITMAP[8]) || (ENABLE_ADPT_STABLE && MODE_BITMAP[9]) || MODE_BITMAP[6] || MODE_BITMAP[7] ||
      MODE_BITMAP[10] || MODE_BITMAP[11] || MODE_BITMAP[13] ||
      (ENABLE_EVA && MODE_BITMAP[19]);
    localparam [31:0] NEIGHBOUR_OPERATOR_MASK=32'h00010000 |
      ((MODE_BITMAP[5] || (ENABLE_BINARY_DENOISE && MODE_BITMAP[8]) || (ENABLE_ADPT_STABLE && MODE_BITMAP[9]))?32'h00000002:32'd0) |
      (MODE_BITMAP[6]?32'h80000000:32'd0) |
      (MODE_BITMAP[7]?32'h00000010:32'd0) |
      (MODE_BITMAP[10]?32'h00000200:32'd0) |
      (MODE_BITMAP[11]?32'h00000400:32'd0) |
      (MODE_BITMAP[13]?32'h00000800:32'd0) |
      ((ENABLE_EVA && MODE_BITMAP[19])?32'h40000000:32'd0);
    generate if(ENABLE_HEAVY_FILTERS && NEED_NEIGHBOURHOOD) begin: g_neighbourhood
    neighborhood_filter_1ppc #(.IMG_WIDTH(IMG_WIDTH),.PIPELINED_GALLERY(1),
        .OPERATOR_MASK(NEIGHBOUR_OPERATOR_MASK)) u_neighbourhood(
        .i_clk(i_clk),.i_rst_n(i_rst_n),.i_frame_start(frame_start),
        .i_de(i_de),.i_rgb(selected_mode==19 && ENABLE_EVA?eva_clean_rgb:i_rgb),.i_operator(operator_id),
        .i_level(8'd12),.o_rgb(neighbourhood_rgb)
    );

    end else begin: g_no_neighbourhood
        assign neighbourhood_rgb=rgb_stage2;
    end endgenerate

    generate if(ENABLE_HEAVY_FILTERS && MODE_BITMAP[9]) begin: g_adaptive
        adaptive_threshold_1ppc #(.IMG_WIDTH(IMG_WIDTH),
            .WINDOW_RADIUS(ENABLE_ADPT_STABLE?6:7),
            .ENABLE_LOW_CONTRAST_MEAN(ENABLE_BINARY_DENOISE)) u_adaptive(
            .i_clk(i_clk),.i_rst_n(i_rst_n),
            .i_frame_start(ENABLE_ADPT_STABLE?pre_frame_start:frame_start),
            .i_de(ENABLE_ADPT_STABLE?de_stage2:i_de),
            .i_y(ENABLE_ADPT_STABLE?neighbourhood_rgb[7:0]:input_y),
            .i_global_threshold(selected_threshold),.o_rgb(adaptive_rgb),.o_decision_info(adaptive_info));
    end else begin: g_no_adaptive
        assign adaptive_rgb=24'd0;
        assign adaptive_info=31'd0;
    end endgenerate

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            vsync_d<=1'b0; mode_active<=8'd0;
            threshold_active<=128;threshold_stage1<=128;threshold_stage2<=128;
            vsync_stage1<=0; hsync_stage1<=0; de_stage1<=0;
            rgb_stage1<=0; mode_stage1<=0;equalized_d<=0;
            vsync_stage2<=0;hsync_stage2<=0;de_stage2<=0;rgb_stage2<=0;mode_stage2<=0;equalized_d2<=0;
            legacy_vsync<=1'b0; legacy_hsync<=1'b0; legacy_de<=1'b0; legacy_rgb<=24'd0;
        end else begin
            vsync_d<=i_vsync;
            if(frame_start) begin
                mode_active<=i_mode;threshold_active<=configured_threshold;
            end
            // Match the registered neighbourhood statistics, including blanking.
            vsync_stage1<=i_vsync; hsync_stage1<=i_hsync; de_stage1<=i_de;
            rgb_stage1<=i_rgb; mode_stage1<=selected_mode;equalized_d<=equalized;
            threshold_stage1<=selected_threshold;threshold_stage2<=threshold_stage1;
            vsync_stage2<=vsync_stage1;hsync_stage2<=hsync_stage1;de_stage2<=de_stage1;
            rgb_stage2<=rgb_stage1;mode_stage2<=mode_stage1;equalized_d2<=equalized_d;
            legacy_vsync<=vsync_stage2;
            legacy_hsync<=hsync_stage2;
            legacy_de<=de_stage2;
            if(!de_stage2) legacy_rgb<=0;
            else case(mode_stage2)
              3:legacy_rgb<={equalized_d2,equalized_d2,equalized_d2};
              4:legacy_rgb<=transform(rgb_stage2,14);
              5,6,7,10,11,13:legacy_rgb<=neighbourhood_rgb;
              9:legacy_rgb<=adaptive_rgb;
              8:legacy_rgb<=ENABLE_BINARY_DENOISE ?
                  {3{({2'd0,neighbourhood_rgb[7:0]}>=threshold_stage2)?8'hff:8'h00}} :
                  {3{({2'd0,luma8(rgb_stage2[23:16],rgb_stage2[15:8],rgb_stage2[7:0])}>=threshold_stage2)?8'hff:8'h00}};
              16:legacy_rgb<=transform(rgb_stage2,11);
              17:legacy_rgb<=transform(rgb_stage2,23);
              19:legacy_rgb<=ENABLE_EVA?eva_finished:rgb_stage2;
              default:legacy_rgb<=rgb_stage2;
            endcase
        end
    end
    generate if(ENABLE_CARTOON)begin:g_cartoon
        wire cartoon_vs,cartoon_hs,cartoon_de;wire [23:0] cartoon_rgb;
        wire cartoon_read,cartoon_write;wire [31:0] cartoon_history;
        // The merged basic gallery retains its original unblurred Sobel.
        // The standalone cartoon profile previews its shared preblurred Sobel.
        localparam SHARE_SOBEL=!ENABLE_HEAVY_FILTERS;
        localparam HAS_STYLE_DIAGNOSTICS=MODE_BITMAP[21] || MODE_BITMAP[22] || MODE_BITMAP[23];
        wire [2:0] cartoon_style=SHARE_SOBEL && selected_mode==7?3'd4:
            HAS_STYLE_DIAGNOSTICS?{1'b0,selected_mode[1:0]}:3'd0;
        // Production menu uses the accepted CEL preset. Its basic threshold
        // stays independent; standalone diagnostic controls remain tunable.
        wire [9:0] cartoon_threshold=HAS_STYLE_DIAGNOSTICS?selected_threshold:10'd96;
        cartoon_filter_1ppc #(.IMG_WIDTH(IMG_WIDTH),.ENABLE_TEMPORAL(ENABLE_CARTOON_TEMPORAL)) u_cartoon(
            .clk(i_clk),.rst_n(i_rst_n),.vsync(i_vsync),.hsync(i_hsync),.de(i_de),.rgb(i_rgb),
            .style(cartoon_style),.edge_threshold(cartoon_threshold),
            .i_history_valid(i_history_valid),.i_history(i_history),
            .o_history_read(cartoon_read),.o_history_write(cartoon_write),.o_history(cartoon_history),
            .o_vsync(cartoon_vs),.o_hsync(cartoon_hs),.o_de(cartoon_de),.o_rgb(cartoon_rgb));
        reg [26:0] old_pipe[0:5];reg [7:0] mode_pipe[0:8];integer n;
        always @(posedge i_clk or negedge i_rst_n)begin
            if(!i_rst_n)begin
                for(n=0;n<6;n=n+1)old_pipe[n]<=0;
                for(n=0;n<9;n=n+1)mode_pipe[n]<=0;
            end else begin
                old_pipe[0]<={legacy_vsync,legacy_hsync,legacy_de,legacy_rgb};
                for(n=1;n<6;n=n+1)old_pipe[n]<=old_pipe[n-1];
                // Gaussian two stages + 13x13 two stages arrive here at
                // stage five. Six-pixel radius + Gaussian radius one keeps
                // the old raw-source centre (x-7,y-7) and nine-cycle output.
                if(ENABLE_ADPT_STABLE && mode_pipe[3]==9)
                    old_pipe[1]<={old_pipe[0][26:24],adaptive_rgb};
                mode_pipe[0]<=selected_mode;for(n=1;n<9;n=n+1)mode_pipe[n]<=mode_pipe[n-1];
            end
        end
        assign {o_vsync,o_hsync,o_de}=old_pipe[5][26:24];
        if(ENABLE_ADPT_STABLE)begin:g_adpt_stable
            reg [30:0] info5,info6;
            reg valid7,white7,white8;reg [7:0] sample7,cut7;
            reg [31:0] history8;reg [23:0] rgb9;
            wire signed [13:0] score=$signed(info6[13:0]);
            localparam signed [13:0] MARGIN=ADPT_HYSTERESIS*18;
            wire [7:0] sample=info6[29:22],cut=info6[21:14];
            wire [7:0] previous_sample=i_history[23:16],previous_cut=i_history[15:8];
            wire [7:0] sample_delta=sample>previous_sample?sample-previous_sample:previous_sample-sample;
            wire [7:0] cut_delta=cut>previous_cut?cut-previous_cut:previous_cut-cut;
            // The outer store admits only complete, configuration-matched,
            // consecutive source frames. The tag rejects CEL-format payloads.
            wire history_ok=ENABLE_CARTOON_TEMPORAL && i_history_valid && i_history[31]
                && i_history[30:24]==7'h2d && sample_delta<=12 && cut_delta<=12;
            wire decided=score>=MARGIN?1'b1:score<=-MARGIN?1'b0:
                history_ok?i_history[0]:!score[13];
            always @(posedge i_clk or negedge i_rst_n)begin
                if(!i_rst_n)begin
                    info5<=0;info6<=0;valid7<=0;white7<=0;white8<=0;
                    sample7<=0;cut7<=0;history8<=0;rgb9<=0;
                end else begin
                    info5<=adaptive_info;info6<=info5;
                    valid7<=info6[30] && old_pipe[2][24];
                    white7<=info6[30] && old_pipe[2][24] && decided;
                    sample7<=sample;cut7<=cut;
                    white8<=white7;
                    history8<=valid7?{1'b1,7'h2d,sample7,cut7,7'd0,white7}:32'd0;
                    rgb9<={24{white8}};
                end
            end
            assign o_rgb=mode_pipe[8]==9?rgb9:
                (SHARE_SOBEL && mode_pipe[8]==7) || (mode_pipe[8]>=20 && mode_pipe[8]<=23)?cartoon_rgb:old_pipe[5][23:0];
            assign o_history=mode_pipe[7]==9?history8:cartoon_history;
        end else begin:g_no_adpt_stable
            assign o_rgb=(SHARE_SOBEL && mode_pipe[8]==7) || (mode_pipe[8]>=20 && mode_pipe[8]<=23)?cartoon_rgb:old_pipe[5][23:0];
            assign o_history=cartoon_history;
        end
        assign o_history_read=cartoon_read;assign o_history_write=cartoon_write;
    end else begin:g_no_cartoon
        assign o_history_read=0;assign o_history_write=0;assign o_history=0;
        assign {o_vsync,o_hsync,o_de,o_rgb}={legacy_vsync,legacy_hsync,legacy_de,legacy_rgb};
    end endgenerate
endmodule
