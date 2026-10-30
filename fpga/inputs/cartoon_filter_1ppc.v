`timescale 1ns/1ps
// Two smoothing passes BEFORE shared Sobel; colour-only third pass reuses
// the same final window. Weak edges require consecutive source-frame support.
// Three causal windows. Optional external history is indexed at the FINAL
// centre raster, not at the unshifted input. Nine registered pipeline stages.
module cartoon_filter_1ppc #(
    parameter IMG_WIDTH=1024, parameter ENABLE_TEMPORAL=0
)(
    input wire clk,rst_n,vsync,hsync,de,
    input wire [23:0] rgb,
    input wire [2:0] style, // 0 composite, 1 smooth, 2 flat, 3 ink, 4 Sobel
    input wire [9:0] edge_threshold,
    output wire o_vsync,o_hsync,o_de,
    output reg [23:0] o_rgb,
    input wire i_history_valid, input wire [31:0] i_history,
    output wire o_history_read, o_history_write, output wire [31:0] o_history
);
    reg vs_d,clean_vs_d,packet_vs_d;
    reg [8:0] vs_pipe,hs_pipe,de_pipe;
    reg [2:0] style_pipe[0:8];
    reg [9:0] threshold_pipe[0:4];
    wire frame_start=vsync && !vs_d;
    wire clean_start=vs_pipe[2] && !clean_vs_d;
    wire packet_start=vs_pipe[4] && !packet_vs_d;
    wire [215:0] window0;
    wire [287:0] window1;
    wire [287:0] window2;
    wire valid0,valid1,valid2;
    wire [23:0] centre0=window0[119:96],centre1=window1[151:128];
    reg [23:0] clean_rgb;
    reg [31:0] contour_packet;
    cartoon_window_1ppc #(.WIDTH(IMG_WIDTH)) u_smooth_window(
        .clk(clk),.rst_n(rst_n),.sof(frame_start),.de(de),.rgb(rgb),
        .window(window0),.valid(valid0),.x());
    function [7:0] luma8;
        input [23:0] p;
        reg [17:0] r,g,b,sum;
        begin
            r={10'd0,p[23:16]};g={10'd0,p[15:8]};b={10'd0,p[7:0]};
            sum=(r<<6)+(r<<3)+(r<<2)+r+(g<<7)+(g<<4)+(g<<2)+(g<<1)
                +(b<<4)+(b<<3)+(b<<2)+b;
            luma8=sum[15:8];
        end
    endfunction
    cartoon_window_1ppc #(.WIDTH(IMG_WIDTH),.BITS(32)) u_ink_window(
        .clk(clk),.rst_n(rst_n),.sof(clean_start),.de(de_pipe[2]),.rgb({luma8(clean_rgb),clean_rgb}),
        .window(window1),.valid(valid1),.x());
    cartoon_window_1ppc #(.WIDTH(IMG_WIDTH),.BITS(32)) u_contour_window(
        .clk(clk),.rst_n(rst_n),.sof(packet_start),.de(de_pipe[4]),.rgb(contour_packet),
        .window(window2),.valid(valid2),.x());

    function [7:0] difference;
        input [7:0] a,b;
        begin difference=a>b?a-b:b-a;end
    endfunction
    function [7:0] distance_rgb;
        input [23:0] a,b;
        reg [7:0] r,g,v;
        begin
            r=difference(a[23:16],b[23:16]);g=difference(a[15:8],b[15:8]);
            v=difference(a[7:0],b[7:0]);distance_rgb=r>g?(r>v?r:v):(g>v?g:v);
        end
    endfunction
    function near_channel;
        input [7:0] a,b;
        reg signed [8:0] delta;
        begin
            delta=$signed({1'b0,a})-$signed({1'b0,b});
            near_channel=delta>=-9'sd64 && delta<=9'sd64;
        end
    endfunction
    function near_rgb;
        input [23:0] a,b;
        begin near_rgb=near_channel(a[23:16],b[23:16]) &&
            near_channel(a[15:8],b[15:8]) && near_channel(a[7:0],b[7:0]);end
    endfunction
    integer k;
    reg [23:0] neighbour;
    reg [12:0] sr,sg,sb;
    always @* begin
        sr={2'd0,centre0[23:16],3'd0};
        sg={2'd0,centre0[15:8],3'd0};sb={2'd0,centre0[7:0],3'd0};
        neighbour=centre0;
        for(k=0;k<9;k=k+1) if(k!=4) begin
            neighbour=window0[215-k*24-:24];
            // Fixed Gaussian weights. Range-rejected samples become centre;
            // the sum stays exactly 32, removing normalization LUT and DSPs.
            if(!near_rgb(neighbour,centre0))neighbour=centre0;
            if(k==1 || k==3 || k==5 || k==7)begin
                sr=sr+{3'd0,neighbour[23:16],2'b0};
                sg=sg+{3'd0,neighbour[15:8],2'b0};sb=sb+{3'd0,neighbour[7:0],2'b0};
            end else begin
                sr=sr+{4'd0,neighbour[23:16],1'b0};
                sg=sg+{4'd0,neighbour[15:8],1'b0};sb=sb+{4'd0,neighbour[7:0],1'b0};
            end
        end
    end
    reg [12:0] sum_r,sum_g,sum_b,sum_r_delay,sum_g_delay,sum_b_delay;
    wire [12:0] rounded_r=sum_r_delay+13'd16;
    wire [12:0] rounded_g=sum_g_delay+13'd16;
    wire [12:0] rounded_b=sum_b_delay+13'd16;

    function [2:0] tone_index;
        input [7:0] y;
        begin tone_index=y<16?0:y<56?1:y<108?2:y<166?3:y<226?4:5;end
    endfunction
    function [7:0] tone_value;
        input [2:0] q;
        begin case(q)
            0:tone_value=0;1:tone_value=32;2:tone_value=80;
            3:tone_value=136;4:tone_value=196;default:tone_value=255;
        endcase end
    endfunction
    // Soft posterization: six plateaus, 16-code transitions, max step four.
    function [7:0] soft_tone;
        input [7:0] y;
        begin case(y)
            8'd0,8'd1,8'd2,8'd3,8'd4,8'd5,8'd6,8'd7,8'd8:soft_tone=8'd0;
            8'd9:soft_tone=8'd2;
            8'd10:soft_tone=8'd4;
            8'd11:soft_tone=8'd6;
            8'd12:soft_tone=8'd8;
            8'd13:soft_tone=8'd10;
            8'd14:soft_tone=8'd12;
            8'd15:soft_tone=8'd14;
            8'd16:soft_tone=8'd16;
            8'd17:soft_tone=8'd18;
            8'd18:soft_tone=8'd20;
            8'd19:soft_tone=8'd22;
            8'd20:soft_tone=8'd24;
            8'd21:soft_tone=8'd26;
            8'd22:soft_tone=8'd28;
            8'd23:soft_tone=8'd30;
            8'd24,8'd25,8'd26,8'd27,8'd28,8'd29,8'd30,8'd31,8'd32,8'd33,8'd34,8'd35,8'd36,8'd37,8'd38,8'd39,8'd40,8'd41,8'd42,8'd43,8'd44,8'd45,8'd46,8'd47,8'd48:soft_tone=8'd32;
            8'd49:soft_tone=8'd35;
            8'd50:soft_tone=8'd38;
            8'd51:soft_tone=8'd41;
            8'd52:soft_tone=8'd44;
            8'd53:soft_tone=8'd47;
            8'd54:soft_tone=8'd50;
            8'd55:soft_tone=8'd53;
            8'd56:soft_tone=8'd56;
            8'd57:soft_tone=8'd59;
            8'd58:soft_tone=8'd62;
            8'd59:soft_tone=8'd65;
            8'd60:soft_tone=8'd68;
            8'd61:soft_tone=8'd71;
            8'd62:soft_tone=8'd74;
            8'd63:soft_tone=8'd77;
            8'd64,8'd65,8'd66,8'd67,8'd68,8'd69,8'd70,8'd71,8'd72,8'd73,8'd74,8'd75,8'd76,8'd77,8'd78,8'd79,8'd80,8'd81,8'd82,8'd83,8'd84,8'd85,8'd86,8'd87,8'd88,8'd89,8'd90,8'd91,8'd92,8'd93,8'd94,8'd95,8'd96,8'd97,8'd98,8'd99,8'd100:soft_tone=8'd80;
            8'd101:soft_tone=8'd84;
            8'd102:soft_tone=8'd87;
            8'd103:soft_tone=8'd91;
            8'd104:soft_tone=8'd94;
            8'd105:soft_tone=8'd98;
            8'd106:soft_tone=8'd101;
            8'd107:soft_tone=8'd105;
            8'd108:soft_tone=8'd108;
            8'd109:soft_tone=8'd112;
            8'd110:soft_tone=8'd115;
            8'd111:soft_tone=8'd119;
            8'd112:soft_tone=8'd122;
            8'd113:soft_tone=8'd126;
            8'd114:soft_tone=8'd129;
            8'd115:soft_tone=8'd133;
            8'd116,8'd117,8'd118,8'd119,8'd120,8'd121,8'd122,8'd123,8'd124,8'd125,8'd126,8'd127,8'd128,8'd129,8'd130,8'd131,8'd132,8'd133,8'd134,8'd135,8'd136,8'd137,8'd138,8'd139,8'd140,8'd141,8'd142,8'd143,8'd144,8'd145,8'd146,8'd147,8'd148,8'd149,8'd150,8'd151,8'd152,8'd153,8'd154,8'd155,8'd156,8'd157,8'd158:soft_tone=8'd136;
            8'd159:soft_tone=8'd140;
            8'd160:soft_tone=8'd144;
            8'd161:soft_tone=8'd147;
            8'd162:soft_tone=8'd151;
            8'd163:soft_tone=8'd155;
            8'd164:soft_tone=8'd159;
            8'd165:soft_tone=8'd162;
            8'd166:soft_tone=8'd166;
            8'd167:soft_tone=8'd170;
            8'd168:soft_tone=8'd174;
            8'd169:soft_tone=8'd177;
            8'd170:soft_tone=8'd181;
            8'd171:soft_tone=8'd185;
            8'd172:soft_tone=8'd189;
            8'd173:soft_tone=8'd192;
            8'd174,8'd175,8'd176,8'd177,8'd178,8'd179,8'd180,8'd181,8'd182,8'd183,8'd184,8'd185,8'd186,8'd187,8'd188,8'd189,8'd190,8'd191,8'd192,8'd193,8'd194,8'd195,8'd196,8'd197,8'd198,8'd199,8'd200,8'd201,8'd202,8'd203,8'd204,8'd205,8'd206,8'd207,8'd208,8'd209,8'd210,8'd211,8'd212,8'd213,8'd214,8'd215,8'd216,8'd217,8'd218:soft_tone=8'd196;
            8'd219:soft_tone=8'd200;
            8'd220:soft_tone=8'd203;
            8'd221:soft_tone=8'd207;
            8'd222:soft_tone=8'd211;
            8'd223:soft_tone=8'd214;
            8'd224:soft_tone=8'd218;
            8'd225:soft_tone=8'd222;
            8'd226:soft_tone=8'd226;
            8'd227:soft_tone=8'd229;
            8'd228:soft_tone=8'd233;
            8'd229:soft_tone=8'd237;
            8'd230:soft_tone=8'd240;
            8'd231:soft_tone=8'd244;
            8'd232:soft_tone=8'd248;
            8'd233:soft_tone=8'd251;
            8'd234,8'd235,8'd236,8'd237,8'd238,8'd239,8'd240,8'd241,8'd242,8'd243,8'd244,8'd245,8'd246,8'd247,8'd248,8'd249,8'd250,8'd251,8'd252,8'd253,8'd254,8'd255:soft_tone=8'd255;
            default:soft_tone=0;
        endcase end
    endfunction

    function [7:0] boundary;
        input [2:0] q;
        begin case(q)
            0:boundary=0;1:boundary=16;2:boundary=56;3:boundary=108;
            4:boundary=166;5:boundary=226;default:boundary=255;
        endcase end
    endfunction
    function signed [10:0] colour_band;
        input signed [10:0] v;
        reg signed [10:0] base;
        reg [4:0] ramp;
        begin
            base=(v>>>4)<<<4;
            ramp=v[3:0]<=4?5'd0:v[3:0]>=12?5'd16:({1'b0,v[3:0]}-5'd4)<<1;
            colour_band=base+$signed({6'd0,ramp});
        end
    endfunction
    function [7:0] blend;
        input [7:0] current,previous; input [1:0] strength;
        reg [9:0] value;
        begin
            case(strength)
                2:begin value={2'd0,current}+{2'd0,previous}+{1'd0,previous,1'b0}+10'd2;blend=value[9:2];end
                1:begin value={2'd0,current}+{2'd0,previous}+10'd1;blend=value[8:1];end
                default:blend=current;
            endcase
        end
    endfunction
    function [7:0] clip;
        input signed [10:0] v;
        begin clip=v<0?8'd0:v>255?8'hff:v[7:0];end
    endfunction
    function [26:0] flatten;
        input [23:0] p; input hold_label; input [2:0] previous_label;
        reg [2:0] label; reg [8:0] yy,lo,hi;
        reg signed [10:0] r,g,b,co,cg,t,y,qco,qcg,qy,rr,gg,bb;
        begin
            r=$signed({3'd0,p[23:16]});g=$signed({3'd0,p[15:8]});b=$signed({3'd0,p[7:0]});
            co=r-b;t=b+(co>>>1);cg=g-t;y=t+(cg>>>1);
            label=tone_index(y[7:0]);yy={1'b0,y[7:0]};
            lo={1'b0,boundary(previous_label)};hi={1'b0,boundary(previous_label+1'b1)};
            if(hold_label && previous_label<6 &&
                (previous_label==0 || yy+9'd6>=lo) &&
                (previous_label==5 || yy<hi+9'd6)) label=previous_label;
            qco=colour_band(co);qcg=colour_band(cg);qy=$signed({3'd0,soft_tone(y[7:0])});
            t=qy-(qcg>>>1);gg=qcg+t;bb=t-(qco>>>1);rr=bb+qco;
            flatten={label,clip(rr),clip(gg),clip(bb)};
        end
    endfunction
    wire [31:0] taps[0:8];wire [31:0] contour[0:8];
    genvar tap;
    generate for(tap=0;tap<9;tap=tap+1)begin:g_taps
        assign taps[tap]=window1[287-tap*32-:32];
        assign contour[tap]=window2[287-tap*32-:32];
    end endgenerate
    wire signed [10:0] sobel_x,sobel_y;
    wire [10:0] sobel_magnitude;
    sobel_gradient_3x3 u_sobel(
        .a(contour[0][31:24]),.b(contour[1][31:24]),.c(contour[2][31:24]),.d(contour[3][31:24]),
        .f(contour[5][31:24]),.g(contour[6][31:24]),.h(contour[7][31:24]),.i(contour[8][31:24]),
        .gx(sobel_x),.gy(sobel_y),.magnitude(sobel_magnitude));
    function [7:0] gaussian9;
        input [7:0] a,b,c,d,e,f,g,h,i;
        reg [11:0] sum;
        begin
            sum={4'd0,a}+({4'd0,b}<<1)+{4'd0,c}+({4'd0,d}<<1)+({4'd0,e}<<2)
                +({4'd0,f}<<1)+{4'd0,g}+({4'd0,h}<<1)+{4'd0,i}+12'd8;
            gaussian9=sum[11:4];
        end
    endfunction
    wire [23:0] gaussian_rgb={
        gaussian9(taps[0][23:16],taps[1][23:16],taps[2][23:16],taps[3][23:16],taps[4][23:16],taps[5][23:16],taps[6][23:16],taps[7][23:16],taps[8][23:16]),
        gaussian9(taps[0][15:8],taps[1][15:8],taps[2][15:8],taps[3][15:8],taps[4][15:8],taps[5][15:8],taps[6][15:8],taps[7][15:8],taps[8][15:8]),
        gaussian9(taps[0][7:0],taps[1][7:0],taps[2][7:0],taps[3][7:0],taps[4][7:0],taps[5][7:0],taps[6][7:0],taps[7][7:0],taps[8][7:0])};
    wire [10:0] ax=sobel_x<0?-sobel_x:sobel_x,ay=sobel_y<0?-sobel_y:sobel_y;
    wire [1:0] direction={1'b0,ax}>{ay,1'b0}?2'd0:
        {1'b0,ay}>{ax,1'b0}?2'd1:(sobel_x[10]==sobel_y[10]?2'd2:2'd3);
    reg [23:0] filtered_stage4;
    wire [10:0] strong_limit=threshold_pipe[4]<64?11'd64:{1'b0,threshold_pipe[4]};
    wire [10:0] weak_limit=threshold_pipe[4]<64?11'd32:{2'd0,threshold_pipe[4][9:1]};
    wire edge_strong=valid2 && sobel_magnitude>=strong_limit;
    wire edge_weak=valid2 && sobel_magnitude>=weak_limit;
    wire [23:0] final_gaussian={
        gaussian9(contour[0][23:16],contour[1][23:16],contour[2][23:16],contour[3][23:16],contour[4][23:16],contour[5][23:16],contour[6][23:16],contour[7][23:16],contour[8][23:16]),
        gaussian9(contour[0][15:8],contour[1][15:8],contour[2][15:8],contour[3][15:8],contour[4][15:8],contour[5][15:8],contour[6][15:8],contour[7][15:8],contour[8][15:8]),
        gaussian9(contour[0][7:0],contour[1][7:0],contour[2][7:0],contour[3][7:0],contour[4][7:0],contour[5][7:0],contour[6][7:0],contour[7][7:0],contour[8][7:0])};
    wire [23:0] final_colour=sobel_magnitude<192?final_gaussian:contour[4][23:0];
    reg ink_stage6,ink_stage8;
    reg [1:0] confidence_stage7,direction_stage7;
    reg hold_stage7;reg [2:0] label_stage7;
    reg [31:0] history_stage8;
    reg [23:0] smooth_stage8;reg [7:0] sobel_stage8;
    reg [38:0] centre_stage6;
    reg [23:0] smooth_stage7,flat_stage7;
    reg [7:0] sobel_stage7;
    wire [7:0] temporal_delta=distance_rgb(centre_stage6[23:0],i_history[23:0]);
    wire use_history=ENABLE_TEMPORAL && i_history_valid && i_history[31] && temporal_delta<=24;
    wire [1:0] blend_strength=!use_history?2'd0:temporal_delta<=12?2'd2:2'd1;
    wire weak_confirmed=use_history && i_history[30:29]==centre_stage6[38:37]
        && i_history[28:27]!=0;
    wire [1:0] next_confidence=centre_stage6[36]?2'd3:
        ink_stage6?(weak_confirmed?(i_history[28:27]==3?2'd3:i_history[28:27]+1'b1):2'd1):
        (use_history && i_history[28:27]==3)?2'd2:2'd0;
    wire [26:0] flattened=flatten(smooth_stage7,hold_stage7,label_stage7);
    assign o_history_read=de_pipe[5];assign o_history_write=de_pipe[7];assign o_history=history_stage8;
    integer j;
    assign o_vsync=vs_pipe[8];assign o_hsync=hs_pipe[8];assign o_de=de_pipe[8];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            vs_d<=0;clean_vs_d<=0;packet_vs_d<=0;vs_pipe<=0;hs_pipe<=0;de_pipe<=0;
            for(j=0;j<9;j=j+1)style_pipe[j]<=0;
            for(j=0;j<5;j=j+1)threshold_pipe[j]<=72;
            sum_r<=0;sum_g<=0;sum_b<=0;
            sum_r_delay<=0;sum_g_delay<=0;sum_b_delay<=0;clean_rgb<=0;
            filtered_stage4<=0;contour_packet<=0;
            ink_stage6<=0;ink_stage8<=0;confidence_stage7<=0;direction_stage7<=0;hold_stage7<=0;label_stage7<=0;history_stage8<=0;smooth_stage8<=0;sobel_stage8<=0;centre_stage6<=0;
            smooth_stage7<=0;flat_stage7<=0;sobel_stage7<=0;o_rgb<=0;
        end else begin
            vs_d<=vsync;clean_vs_d<=vs_pipe[2];packet_vs_d<=vs_pipe[4];
            vs_pipe<={vs_pipe[7:0],vsync};hs_pipe<={hs_pipe[7:0],hsync};de_pipe<={de_pipe[7:0],de};
            style_pipe[0]<=style;for(j=1;j<9;j=j+1)style_pipe[j]<=style_pipe[j-1];
            threshold_pipe[0]<=edge_threshold<32?10'd32:edge_threshold;
            for(j=1;j<5;j=j+1)threshold_pipe[j]<=threshold_pipe[j-1];
            sum_r<=sr;sum_g<=sg;sum_b<=sb;
            sum_r_delay<=sum_r;sum_g_delay<=sum_g;sum_b_delay<=sum_b;
            clean_rgb<={rounded_r[12:5],rounded_g[12:5],rounded_b[12:5]};
            filtered_stage4<=gaussian_rgb;
            contour_packet<={luma8(filtered_stage4),filtered_stage4};
            centre_stage6<={direction,edge_strong,edge_weak,sobel_magnitude,final_colour};
            ink_stage6<=edge_weak;
            smooth_stage7<={blend(centre_stage6[23:16],i_history[23:16],blend_strength),
                blend(centre_stage6[15:8],i_history[15:8],blend_strength),blend(centre_stage6[7:0],i_history[7:0],blend_strength)};
            confidence_stage7<=next_confidence;
            // A missing/weak gradient has no reliable direction. Keep the old
            // direction while its finite confidence decays, never refresh it.
            direction_stage7<=ink_stage6 || !use_history?centre_stage6[38:37]:i_history[30:29];
            hold_stage7<=use_history;label_stage7<=i_history[26:24];
            sobel_stage7<=centre_stage6[34:24]>255?8'hff:centre_stage6[31:24];
            flat_stage7<=flattened[23:0];smooth_stage8<=smooth_stage7;sobel_stage8<=sobel_stage7;
            ink_stage8<=confidence_stage7>=2;
            history_stage8<={1'b1,direction_stage7,confidence_stage7,flattened[26:24],smooth_stage7};
            if(!de_pipe[7])o_rgb<=0;
            else case(style_pipe[7])
                1:o_rgb<=smooth_stage8;
                2:o_rgb<=flat_stage7;
                3:o_rgb<=ink_stage8?24'h000000:24'hffffff;
                4:o_rgb<={3{sobel_stage8}};
                default:o_rgb<=ink_stage8?24'h000000:flat_stage7;
            endcase
        end
    end
endmodule

// Synchronous read-first RGB line memories with next-column prefetch. Valid
// masks hide uninitialised memory; resets never clear or expand the RAM arrays.
module cartoon_window_1ppc #(parameter WIDTH=1024,BITS=24)(
    input wire clk,rst_n,sof,de,input wire [BITS-1:0] rgb,
    output wire [BITS*9-1:0] window,output wire valid,output reg [10:0] x
);
    localparam AB=WIDTH<=2?1:$clog2(WIDTH);
    (* ram_style="bram",syn_ramstyle="block_ram" *)reg [BITS-1:0] line1[0:WIDTH-1];
    (* ram_style="bram",syn_ramstyle="block_ram" *)reg [BITS-1:0] line2[0:WIDTH-1];
    reg [BITS-1:0] p1,p2,a0,b0,a1,b1,a2,b2;
    reg [10:0] y;reg de_d;
    wire [10:0] read_x=(!rst_n || sof || (!de && de_d))?11'd0:de?x+1'b1:x;
    always @(posedge clk)begin
        p1<=line1[read_x[AB-1:0]];p2<=line2[read_x[AB-1:0]];
        if(rst_n && !sof && de && x<WIDTH)begin line1[x[AB-1:0]]<=rgb;line2[x[AB-1:0]]<=p1;end
    end
    assign valid=x>=2 && y>=2 && de;
    assign window=valid?{a2,b2,p2,a1,b1,p1,a0,b0,rgb}:{9{rgb}};
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin x<=0;y<=0;de_d<=0;a0<=0;b0<=0;a1<=0;b1<=0;a2<=0;b2<=0;end
        else begin
            de_d<=de;
            if(sof)begin x<=0;y<=0;a0<=0;b0<=0;a1<=0;b1<=0;a2<=0;b2<=0;end
            else if(de)begin x<=x+1'b1;a0<=b0;b0<=rgb;a1<=b1;b1<=p1;a2<=b2;b2<=p2;end
            else if(de_d)begin x<=0;y<=y+1'b1;a0<=0;b0<=0;a1<=0;b1<=0;a2<=0;b2<=0;end
        end
    end
endmodule
