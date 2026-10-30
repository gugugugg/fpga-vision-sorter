`timescale 1ns/1ps

// Shared 3x3 grayscale neighbourhood datapath for HDMI-side analysis modes.
// One pair of line buffers serves sixteen operators, so adding menu entries
// does not replicate the expensive image storage. The window is causal and
// sustains one pixel per clock.
module neighborhood_filter_1ppc #(
    parameter integer IMG_WIDTH=1024,
    // Gallery inserts a statistics/sort midpoint. Legacy clients retain the
    // original one-clock contract; both paths share the same line buffers.
    parameter integer PIPELINED_GALLERY=0,
    // Constant gallery capabilities prune unused arithmetic, not only entries.
    // Legacy callers retain all operators and their one-clock behaviour.
    parameter [31:0] OPERATOR_MASK=32'hffffffff,
    parameter integer X_BITS=11,
    parameter integer Y_BITS=11
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_frame_start,
    input  wire        i_de,
    input  wire [23:0] i_rgb,
    input  wire [4:0]  i_operator,
    input  wire [7:0]  i_level,
    output reg  [23:0] o_rgb
);
    // Retain colour for the centre-aligned Sobel overlay. The older version
    // combined a centre gradient with the bottom-right pixel's colour.
    (* ram_style="bram" *) reg [23:0] line1 [0:IMG_WIDTH-1];
    (* ram_style="bram" *) reg [7:0] line2 [0:IMG_WIDTH-1];
    reg [X_BITS-1:0] x_pos;
    reg [Y_BITS-1:0] y_pos;
    reg de_d;
    reg [7:0] row0_d1,row0_d2,row1_d1,row1_d2,row2_d1,row2_d2;
    reg [23:0] centre_rgb_left;

    function [7:0] luma8;
        input [23:0] rgb;
        reg [17:0] r,g,b,sum;
        begin
            r={10'd0,rgb[23:16]}; g={10'd0,rgb[15:8]}; b={10'd0,rgb[7:0]};
            sum=(r<<6)+(r<<3)+(r<<2)+r
               +(g<<7)+(g<<4)+(g<<2)+(g<<1)
               +(b<<4)+(b<<3)+(b<<2)+b;
            luma8=sum[15:8];
        end
    endfunction
    function [15:0] abs16;
        input signed [15:0] value;
        begin abs16=value[15] ? -value : value; end
    endfunction
    function [7:0] sat8;
        input [15:0] value;
        begin sat8=(value>255)?8'hff:value[7:0]; end
    endfunction
    function [7:0] absdiff8;
        input [7:0] v0,v1;
        begin absdiff8=(v0>=v1)?v0-v1:v1-v0; end
    endfunction
    function [7:0] sat_signed8;
        input signed [15:0] value;
        begin
            if(value<0) sat_signed8=0;
            else if(value>255) sat_signed8=8'hff;
            else sat_signed8=value[7:0];
        end
    endfunction
    function [23:0] heat_colour;
        input [7:0] value;
        reg [7:0] ramp;
        begin
            if(value<64) begin ramp={value[5:0],2'b00}; heat_colour={8'd0,ramp,8'hff}; end
            else if(value<128) begin ramp={value[5:0],2'b00}; heat_colour={8'd0,8'hff,~ramp}; end
            else if(value<192) begin ramp={value[5:0],2'b00}; heat_colour={ramp,8'hff,8'd0}; end
            else begin ramp={value[5:0],2'b00}; heat_colour={8'hff,~ramp,8'd0}; end
        end
    endfunction

    wire [7:0] y=luma8(i_rgb);
    localparam integer LINE_ADDR_BITS=(IMG_WIDTH>1)?$clog2(IMG_WIDTH):1;
    reg [23:0] line1_q;
    reg [7:0] line2_q;
    reg read_column_valid;
    // Preserve undefined out-of-range reads in simulation. Invalid columns
    // are outside the active-video contract and are masked by window/DE.
    wire [23:0] p1_rgb=read_column_valid?line1_q:24'hxxxxxx;
    wire [7:0] p2=read_column_valid?line2_q:8'hxx;
    wire [7:0] p1=luma8(p1_rgb);
    // Prefetch the next active column. A falling DE edge starts a new row;
    // the first blanking edge primes column zero even with one blank cycle.
    wire [X_BITS-1:0] read_column=(!i_rst_n || i_frame_start || (!i_de && de_d)) ?
        {X_BITS{1'b0}} : i_de ? x_pos+1'b1 : x_pos;
    // The RAM data outputs have no asynchronous reset. Existing window
    // validity gates hide stale RAM data while the first two rows refill.
    always @(posedge i_clk) begin
        line1_q<=line1[read_column[LINE_ADDR_BITS-1:0]];
        line2_q<=line2[read_column[LINE_ADDR_BITS-1:0]];
        read_column_valid<=read_column<IMG_WIDTH;
        if(i_rst_n && !i_frame_start && i_de && x_pos<IMG_WIDTH) begin
            line1[x_pos[LINE_ADDR_BITS-1:0]]<=i_rgb;
            line2[x_pos[LINE_ADDR_BITS-1:0]]<=p1;
        end
    end
    wire window_valid=(x_pos>=2)&&(y_pos>=2);

    // a b c / d e f / g h i, with i being the current pixel.
    wire [7:0] a_in=row2_d2,b_in=row2_d1,c_in=p2;
    wire [7:0] d_in=row1_d2,e_in=row1_d1,f_in=p1;
    wire [7:0] g_in=row0_d2,h_in=row0_d1,i_in=y;

    // Stage 1 closes the path from HDMI input through luma and the common
    // neighbourhood statistics. The operator-specific result is computed
    // from these registered values in the following pixel clock.
    wire [11:0] sum9_in=a_in+b_in+c_in+d_in+e_in+f_in+g_in+h_in+i_in;
    wire [11:0] gaussian_sum_in=a_in+(b_in<<1)+c_in+(d_in<<1)
                                 +(e_in<<2)+(f_in<<1)+g_in+(h_in<<1)+i_in;
    wire [7:0] mean_value_in=(sum9_in+4)/9;
    // Balanced four-level reductions avoid an eight-comparator chain from
    // the first window tap to the HDMI pixel-clock register.
    wire [7:0] min_ab=(a_in<b_in)?a_in:b_in;
    wire [7:0] min_cd=(c_in<d_in)?c_in:d_in;
    wire [7:0] min_ef=(e_in<f_in)?e_in:f_in;
    wire [7:0] min_gh=(g_in<h_in)?g_in:h_in;
    wire [7:0] min_abcd=(min_ab<min_cd)?min_ab:min_cd;
    wire [7:0] min_efgh=(min_ef<min_gh)?min_ef:min_gh;
    wire [7:0] min_eight=(min_abcd<min_efgh)?min_abcd:min_efgh;
    wire [7:0] min_value_in=(min_eight<i_in)?min_eight:i_in;
    wire [7:0] max_ab=(a_in>b_in)?a_in:b_in;
    wire [7:0] max_cd=(c_in>d_in)?c_in:d_in;
    wire [7:0] max_ef=(e_in>f_in)?e_in:f_in;
    wire [7:0] max_gh=(g_in>h_in)?g_in:h_in;
    wire [7:0] max_abcd=(max_ab>max_cd)?max_ab:max_cd;
    wire [7:0] max_efgh=(max_ef>max_gh)?max_ef:max_gh;
    wire [7:0] max_eight=(max_abcd>max_efgh)?max_abcd:max_efgh;
    wire [7:0] max_value_in=(max_eight>i_in)?max_eight:i_in;
    reg [7:0] a,b,c,d,e,f,g,h,i;
    reg [11:0] sum9,gaussian_sum;
    reg [7:0] mean_value,min_value,max_value;
    reg [23:0] rgb_stage1;
    reg [4:0] operator_stage1;
    reg [7:0] level_stage1;
    reg window_valid_stage1;

    // Nine-value odd/even sorting network; exact median, no approximation.
    function [7:0] median9;
        input [7:0] va,vb,vc,vd,ve,vf,vg,vh,vi;
        reg [7:0] values[0:8]; reg [7:0] tmp;
        integer phase,k;
        begin
            values[0]=va;values[1]=vb;values[2]=vc;values[3]=vd;
            values[4]=ve;values[5]=vf;values[6]=vg;values[7]=vh;values[8]=vi;
            for(phase=0;phase<9;phase=phase+1)
                for(k=(phase%2);k<8;k=k+2)
                    if(values[k]>values[k+1]) begin
                        tmp=values[k];values[k]=values[k+1];values[k+1]=tmp;
                    end
            median9=values[4];
        end
    endfunction
    reg signed [15:0] gx,gy,lap,sharp;
    // Pair opposite taps; differences fit 9 signed bits. Balanced additions
    // preserve the exact Sobel L1 magnitude without a serial 16-bit chain.
    wire signed [8:0] diff_ca=$signed({1'b0,c})-$signed({1'b0,a});
    wire signed [8:0] diff_fd=$signed({1'b0,f})-$signed({1'b0,d});
    wire signed [8:0] diff_ig=$signed({1'b0,i})-$signed({1'b0,g});
    wire signed [8:0] diff_ga=$signed({1'b0,g})-$signed({1'b0,a});
    wire signed [8:0] diff_hb=$signed({1'b0,h})-$signed({1'b0,b});
    wire signed [8:0] diff_ic=$signed({1'b0,i})-$signed({1'b0,c});
    wire signed [9:0] outer_x=$signed(diff_ca)+$signed(diff_ig);
    wire signed [9:0] outer_y=$signed(diff_ga)+$signed(diff_ic);
    wire signed [10:0] sobel_x=$signed(outer_x)+($signed({diff_fd[8],diff_fd})<<<1);
    wire signed [10:0] sobel_y=$signed(outer_y)+($signed({diff_hb[8],diff_hb})<<<1);
    reg [15:0] magnitude,mad_sum;
    reg [7:0] result;
    reg [7:0] adaptive_limit;
    reg [23:0] result_rgb;
    reg special_rgb;
    reg [23:0] legacy_rgb;
    always @* begin
        gx=0; gy=0; lap=0; sharp=0; magnitude=0; result=i;
        mad_sum=0; result_rgb=24'd0; special_rgb=0;
        adaptive_limit=(mean_value>level_stage1)?mean_value-level_stage1:0;
        case(operator_stage1)
            4'd0: result=mean_value;                              // mean blur
            4'd1: result=gaussian_sum[11:4];                      // Gaussian 1-2-1
            4'd2: begin                                           // Roberts 2x2
                gx=$signed({1'b0,i})-$signed({1'b0,e});
                gy=$signed({1'b0,h})-$signed({1'b0,f});
                magnitude=abs16(gx)+abs16(gy); result=sat8(magnitude);
            end
            4'd3: begin                                           // Prewitt
                gx=$signed({1'b0,c})+$signed({1'b0,f})+$signed({1'b0,i})
                  -$signed({1'b0,a})-$signed({1'b0,d})-$signed({1'b0,g});
                gy=$signed({1'b0,g})+$signed({1'b0,h})+$signed({1'b0,i})
                  -$signed({1'b0,a})-$signed({1'b0,b})-$signed({1'b0,c});
                magnitude=abs16(gx)+abs16(gy); result=sat8(magnitude);
            end
            4'd4: begin                                           // Sobel
                gx=sobel_x;gy=sobel_y;
                magnitude=abs16(gx)+abs16(gy); result=sat8(magnitude);
            end
            4'd5: begin                                           // Scharr 3-10-3
                gx=3*$signed({1'b0,c})+10*$signed({1'b0,f})+3*$signed({1'b0,i})
                  -3*$signed({1'b0,a})-10*$signed({1'b0,d})-3*$signed({1'b0,g});
                gy=3*$signed({1'b0,g})+10*$signed({1'b0,h})+3*$signed({1'b0,i})
                  -3*$signed({1'b0,a})-10*$signed({1'b0,b})-3*$signed({1'b0,c});
                magnitude=abs16(gx)+abs16(gy); result=sat8(magnitude);
            end
            4'd6: begin                                           // 8-neighbour Laplacian
                lap=($signed({1'b0,e})<<<3)-$signed({1'b0,a})-$signed({1'b0,b})
                    -$signed({1'b0,c})-$signed({1'b0,d})-$signed({1'b0,f})
                    -$signed({1'b0,g})-$signed({1'b0,h})-$signed({1'b0,i});
                result=sat8(abs16(lap));
            end
            4'd7: result=(e>=adaptive_limit)?8'hff:8'h00;         // centre local threshold
            4'd8: begin                                           // unsharp mask
                sharp=($signed({1'b0,e})<<<1)-$signed({1'b0,mean_value});
                result=sharp<0?0:(sharp>255?255:sharp[7:0]);
            end
            4'd9: result=min_value;                               // grayscale erosion
            4'd10: result=max_value;                              // grayscale dilation
            4'd11: result=max_value-min_value;                    // morphology gradient
            4'd12: begin                                          // high-pass magnitude
                sharp=9*$signed({1'b0,e})-$signed({4'd0,sum9});
                result=sat8(abs16(sharp));
            end
            4'd13: begin                                          // horizontal edges
                gy=$signed({1'b0,g})+($signed({1'b0,h})<<<1)+$signed({1'b0,i})
                  -$signed({1'b0,a})-($signed({1'b0,b})<<<1)-$signed({1'b0,c});
                result=sat8(abs16(gy));
            end
            4'd14: begin                                          // vertical edges
                gx=$signed({1'b0,c})+($signed({1'b0,f})<<<1)+$signed({1'b0,i})
                  -$signed({1'b0,a})-($signed({1'b0,d})<<<1)-$signed({1'b0,g});
                result=sat8(abs16(gx));
            end
            5'd15: result=max_value-min_value;                    // local texture range
            5'd16: result=(i>=8'd128)?8'hff:8'h00;                // current-pixel threshold, no window
            5'd17: result=(i<8'd128)?8'hff:8'h00;
            5'd18: result=(e>mean_value)?e-mean_value:0;
            5'd19: result=(mean_value>e)?mean_value-e:0;
            5'd20: result=e-min_value;
            5'd21: result=max_value-e;
            5'd22: begin                                          // mean absolute deviation
                mad_sum=absdiff8(a,mean_value)+absdiff8(b,mean_value)
                       +absdiff8(c,mean_value)+absdiff8(d,mean_value)
                       +absdiff8(e,mean_value)+absdiff8(f,mean_value)
                       +absdiff8(g,mean_value)+absdiff8(h,mean_value)
                       +absdiff8(i,mean_value);
                result=(mad_sum+4)/9;
            end
            5'd23: result=((max_value-min_value)>8'd32)?8'hff:8'h00; // texture mask
            5'd24: begin                                          // diagonal / edge
                gx=($signed({1'b0,i})<<<1)+$signed({1'b0,f})+$signed({1'b0,h})
                  -($signed({1'b0,a})<<<1)-$signed({1'b0,b})-$signed({1'b0,d});
                result=sat8(abs16(gx));
            end
            5'd25: begin                                          // diagonal backslash edge
                gx=$signed({1'b0,b})+($signed({1'b0,c})<<<1)+$signed({1'b0,f})
                  -$signed({1'b0,d})-($signed({1'b0,g})<<<1)-$signed({1'b0,h});
                result=sat8(abs16(gx));
            end
            5'd26: begin                                          // combined diagonal response
                gx=($signed({1'b0,i})<<<1)+$signed({1'b0,f})+$signed({1'b0,h})
                  -($signed({1'b0,a})<<<1)-$signed({1'b0,b})-$signed({1'b0,d});
                gy=$signed({1'b0,b})+($signed({1'b0,c})<<<1)+$signed({1'b0,f})
                  -$signed({1'b0,d})-($signed({1'b0,g})<<<1)-$signed({1'b0,h});
                magnitude=abs16(gx)+abs16(gy); result=sat8(magnitude);
            end
            5'd27: begin                                          // horizontal emboss
                gx=$signed({1'b0,c})+($signed({1'b0,f})<<<1)+$signed({1'b0,i})
                  -$signed({1'b0,a})-($signed({1'b0,d})<<<1)-$signed({1'b0,g});
                sharp=16'sd128+(gx>>>3); result=sat_signed8(sharp);
            end
            5'd28: begin                                          // vertical emboss
                gy=$signed({1'b0,g})+($signed({1'b0,h})<<<1)+$signed({1'b0,i})
                  -$signed({1'b0,a})-($signed({1'b0,b})<<<1)-$signed({1'b0,c});
                sharp=16'sd128+(gy>>>3); result=sat_signed8(sharp);
            end
            5'd29: begin                                          // red Sobel overlay
                gx=$signed({1'b0,c})+($signed({1'b0,f})<<<1)+$signed({1'b0,i})
                  -$signed({1'b0,a})-($signed({1'b0,d})<<<1)-$signed({1'b0,g});
                gy=$signed({1'b0,g})+($signed({1'b0,h})<<<1)+$signed({1'b0,i})
                  -$signed({1'b0,a})-($signed({1'b0,b})<<<1)-$signed({1'b0,c});
                magnitude=abs16(gx)+abs16(gy);
                result_rgb=(magnitude>64)?24'hff2020:rgb_stage1; special_rgb=1;
            end
            5'd30: begin                                          // defect heatmap
                sharp=9*$signed({1'b0,e})-$signed({4'd0,sum9});
                result=sat8(abs16(sharp)); result_rgb=heat_colour(result); special_rgb=1;
            end
            5'd31: result=median9(a,b,c,d,e,f,g,h,i);
            default: begin                                        // combined defect mask
                sharp=9*$signed({1'b0,e})-$signed({4'd0,sum9});
                result=((abs16(sharp)>level_stage1) ||
                        ((max_value-min_value)>{level_stage1,2'b00}))?8'hff:8'h00;
            end
        endcase
        if(!window_valid_stage1 && operator_stage1!=16 && operator_stage1!=17) legacy_rgb=24'd0;
        else legacy_rgb=special_rgb?result_rgb:{result,result,result};
    end

    // Exact 19-comparison median selection topology, as used by OpenCV's
    // medianBlur_SortNet 3x3 path. Source/reference and exhaustive proof are
    // recorded in the resource-optimization validation; no approximate rank.
    // First 12 comparisons occupy the existing midpoint stage. Only lanes
    // 1..7 are live after that point, so the midpoint stores 56, not 72 bits.
    function [55:0] median_first;
        input [71:0] packed_taps;
        reg [7:0] values[0:8];reg [7:0] tmp;integer phase,k;
        begin
            for(k=0;k<9;k=k+1) values[k]=packed_taps[(8-k)*8+:8];
            // Comparisons are inserted explicitly below; blocking assignments
            // describe the combinational selection network within each stage.
            if(values[1]>values[2])begin tmp=values[1];values[1]=values[2];values[2]=tmp;end
            if(values[4]>values[5])begin tmp=values[4];values[4]=values[5];values[5]=tmp;end
            if(values[7]>values[8])begin tmp=values[7];values[7]=values[8];values[8]=tmp;end
            if(values[0]>values[1])begin tmp=values[0];values[0]=values[1];values[1]=tmp;end
            if(values[3]>values[4])begin tmp=values[3];values[3]=values[4];values[4]=tmp;end
            if(values[6]>values[7])begin tmp=values[6];values[6]=values[7];values[7]=tmp;end
            if(values[1]>values[2])begin tmp=values[1];values[1]=values[2];values[2]=tmp;end
            if(values[4]>values[5])begin tmp=values[4];values[4]=values[5];values[5]=tmp;end
            if(values[7]>values[8])begin tmp=values[7];values[7]=values[8];values[8]=tmp;end
            if(values[0]>values[3])begin tmp=values[0];values[0]=values[3];values[3]=tmp;end
            if(values[5]>values[8])begin tmp=values[5];values[5]=values[8];values[8]=tmp;end
            if(values[4]>values[7])begin tmp=values[4];values[4]=values[7];values[7]=tmp;end
            for(k=1;k<8;k=k+1) median_first[(7-k)*8+:8]=values[k];
        end
    endfunction
    function [7:0] median_last;
        input [55:0] packed_taps;
        reg [7:0] values[0:8];reg [7:0] tmp;integer phase,k;
        begin
            values[0]=0;values[8]=0;
            for(k=1;k<8;k=k+1) values[k]=packed_taps[(7-k)*8+:8];
            if(values[3]>values[6])begin tmp=values[3];values[3]=values[6];values[6]=tmp;end
            if(values[1]>values[4])begin tmp=values[1];values[1]=values[4];values[4]=tmp;end
            if(values[2]>values[5])begin tmp=values[2];values[2]=values[5];values[5]=tmp;end
            if(values[4]>values[7])begin tmp=values[4];values[4]=values[7];values[7]=tmp;end
            if(values[4]>values[2])begin tmp=values[4];values[4]=values[2];values[2]=tmp;end
            if(values[6]>values[4])begin tmp=values[6];values[6]=values[4];values[4]=tmp;end
            if(values[4]>values[2])begin tmp=values[4];values[4]=values[2];values[2]=tmp;end
            median_last=values[4];
        end
    endfunction
    wire [8:0] gallery_ab={1'b0,a}+b,gallery_cd={1'b0,c}+d;
    wire [8:0] gallery_ef={1'b0,e}+f,gallery_gh={1'b0,g}+h;
    wire [9:0] gallery_abcd={1'b0,gallery_ab}+gallery_cd;
    wire [9:0] gallery_efgh={1'b0,gallery_ef}+gallery_gh;
    wire [11:0] gallery_sum={2'b0,gallery_abcd}+gallery_efgh+i;
    wire [9:0] gaussian_row0={2'b0,a}+({2'b0,b}<<1)+c;
    wire [9:0] gaussian_row1={2'b0,d}+({2'b0,e}<<1)+f;
    wire [9:0] gaussian_row2={2'b0,g}+({2'b0,h}<<1)+i;
    wire [11:0] gallery_gauss={2'b0,gaussian_row0}+({2'b0,gaussian_row1}<<1)+gaussian_row2;
    wire [7:0] gallery_min_ab=a<b?a:b,gallery_min_cd=c<d?c:d;
    wire [7:0] gallery_min_ef=e<f?e:f,gallery_min_gh=g<h?g:h;
    wire [7:0] gallery_min_abcd=gallery_min_ab<gallery_min_cd?gallery_min_ab:gallery_min_cd;
    wire [7:0] gallery_min_efgh=gallery_min_ef<gallery_min_gh?gallery_min_ef:gallery_min_gh;
    wire [7:0] gallery_min8=gallery_min_abcd<gallery_min_efgh?gallery_min_abcd:gallery_min_efgh;
    wire [7:0] gallery_max_ab=a>b?a:b,gallery_max_cd=c>d?c:d;
    wire [7:0] gallery_max_ef=e>f?e:f,gallery_max_gh=g>h?g:h;
    wire [7:0] gallery_max_abcd=gallery_max_ab>gallery_max_cd?gallery_max_ab:gallery_max_cd;
    wire [7:0] gallery_max_efgh=gallery_max_ef>gallery_max_gh?gallery_max_ef:gallery_max_gh;
    wire [7:0] gallery_max8=gallery_max_abcd>gallery_max_efgh?gallery_max_abcd:gallery_max_efgh;
    reg [55:0] sorted_mid;
    reg [7:0] gallery_mean,gallery_gaussian,gallery_min,gallery_max,gallery_e,gallery_i,gallery_level;
    reg signed [10:0] gallery_gx,gallery_gy;
    reg [4:0] gallery_operator;reg gallery_valid;
    reg [7:0] gallery_style_strength;
    // Backward gradient refers to the CURRENT colour sample, unlike the
    // centred Sobel operator. This avoids shifting outlines over current RGB.
    wire [7:0] style_dx=absdiff8(i,h),style_dy=absdiff8(i,f);
    function [10:0] abs11;
        input signed [10:0] value;
        begin abs11=value[10]?-value:value;end
    endfunction
    wire [11:0] gallery_magnitude={1'b0,abs11(gallery_gx)}+abs11(gallery_gy);
    reg [7:0] gallery_result;
    always @* begin
        gallery_result=gallery_i;
        case(gallery_operator)
            1:gallery_result=OPERATOR_MASK[1]?gallery_gaussian:0;
            4:gallery_result=OPERATOR_MASK[4]?(gallery_magnitude>255?255:gallery_magnitude[7:0]):0;
            7:gallery_result=OPERATOR_MASK[7]?(gallery_e>=(gallery_mean>gallery_level?gallery_mean-gallery_level:0)?255:0):0;
            9:gallery_result=OPERATOR_MASK[9]?gallery_min:0;
            10:gallery_result=OPERATOR_MASK[10]?gallery_max:0;
            11:gallery_result=OPERATOR_MASK[11]?(gallery_max-gallery_min):0;
            16:gallery_result=OPERATOR_MASK[16]?(gallery_i>=128?255:0):0;
            31:gallery_result=OPERATOR_MASK[31]?median_last(sorted_mid):0;
            30:gallery_result=OPERATOR_MASK[30]?gallery_style_strength:0;
        endcase
        if(PIPELINED_GALLERY)
            o_rgb=gallery_valid || gallery_operator==16?{3{gallery_result}}:24'd0;
        else o_rgb=legacy_rgb;
    end
    always @(posedge i_clk or negedge i_rst_n)
        if(!i_rst_n) begin
            sorted_mid<=0;gallery_mean<=0;gallery_gaussian<=0;gallery_min<=0;gallery_max<=0;
            gallery_e<=0;gallery_i<=0;gallery_level<=0;gallery_gx<=0;gallery_gy<=0;
            gallery_operator<=0;gallery_valid<=0;gallery_style_strength<=0;
        end else if(PIPELINED_GALLERY) begin
            sorted_mid<=OPERATOR_MASK[31]?median_first({a,b,c,d,e,f,g,h,i}):56'd0;
            gallery_mean<=OPERATOR_MASK[7]?(gallery_sum+4)/9:8'd0;
            gallery_gaussian<=OPERATOR_MASK[1]?gallery_gauss[11:4]:8'd0;
            gallery_min<=(OPERATOR_MASK[9] || OPERATOR_MASK[11])?(gallery_min8<i?gallery_min8:i):8'd0;
            gallery_max<=(OPERATOR_MASK[10] || OPERATOR_MASK[11])?(gallery_max8>i?gallery_max8:i):8'd0;
            gallery_gx<=OPERATOR_MASK[4]?sobel_x:11'sd0;
            gallery_gy<=OPERATOR_MASK[4]?sobel_y:11'sd0;
            gallery_e<=OPERATOR_MASK[7]?e:8'd0;gallery_i<=i;
            gallery_level<=OPERATOR_MASK[7]?level_stage1:8'd0;
            gallery_operator<=operator_stage1;gallery_valid<=window_valid_stage1;
            gallery_style_strength<=OPERATOR_MASK[30]?(style_dx>style_dy?style_dx:style_dy):8'd0;
        end

    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            x_pos<=0; y_pos<=0; de_d<=0;
            row0_d1<=0; row0_d2<=0; row1_d1<=0; row1_d2<=0;
            row2_d1<=0; row2_d2<=0;
            a<=0; b<=0; c<=0; d<=0; e<=0; f<=0; g<=0; h<=0; i<=0;
            sum9<=0; gaussian_sum<=0; mean_value<=0;
            min_value<=0; max_value<=0; rgb_stage1<=0;
            operator_stage1<=0; level_stage1<=0; window_valid_stage1<=0;
            centre_rgb_left<=0;
        end else begin
            a<=a_in; b<=b_in; c<=c_in; d<=d_in; e<=e_in;
            f<=f_in; g<=g_in; h<=h_in; i<=i_in;
            sum9<=sum9_in; gaussian_sum<=gaussian_sum_in;
            mean_value<=mean_value_in; min_value<=min_value_in;
            max_value<=max_value_in; rgb_stage1<=centre_rgb_left;
            operator_stage1<=i_operator; level_stage1<=i_level;
            window_valid_stage1<=window_valid;
            de_d<=i_de;
            if(i_frame_start) begin
                x_pos<=0; y_pos<=0;
                row0_d1<=0; row0_d2<=0; row1_d1<=0; row1_d2<=0;
                row2_d1<=0; row2_d2<=0;
                centre_rgb_left<=0;
            end else if(i_de) begin
                centre_rgb_left<=p1_rgb;
                if(x_pos==0) begin
                    row0_d2<=0; row0_d1<=y;
                    row1_d2<=0; row1_d1<=p1;
                    row2_d2<=0; row2_d1<=p2;
                end else begin
                    row0_d2<=row0_d1; row0_d1<=y;
                    row1_d2<=row1_d1; row1_d1<=p1;
                    row2_d2<=row2_d1; row2_d1<=p2;
                end
                x_pos<=x_pos+1'b1;
            end else if(de_d) begin
                x_pos<=0; y_pos<=y_pos+1'b1;
                row0_d1<=0; row0_d2<=0; row1_d1<=0; row1_d2<=0;
                row2_d1<=0; row2_d2<=0;
            end
        end
    end
endmodule
