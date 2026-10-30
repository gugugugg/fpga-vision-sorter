`timescale 1ns/1ps
module micro_motion_roi #(
    parameter ROI_X=256, ROI_Y=172, ROI_W=512, ROI_H=256, SAMPLE_LOG2=2,
    parameter FAST_SHIFT=2, SLOW_SHIFT=5, GAIN_SHIFT=3
)(
    input wire clk,rst_n,vsync,hsync,de,
    input wire [23:0] rgb,
    input wire enable,
    input wire [31:0] source_frame_id,
    output reg out_vsync,out_hsync,out_de,
    output reg [23:0] out_rgb
);
    localparam X_BITS=$clog2(ROI_W)-SAMPLE_LOG2,Y_BITS=$clog2(ROI_H)-SAMPLE_LOG2;
    localparam STEP=1<<SAMPLE_LOG2,GRID_W=ROI_W/STEP;
    localparam ADDRESS_BITS=X_BITS+Y_BITS;
    localparam PIXELS=(ROI_W/STEP)*(ROI_H/STEP);
    // Two Q8.6 states per coarse cell; one update per distinct source frame.
    (* ram_style="bram" *) reg [27:0] history[0:PIXELS-1];
    reg [27:0] history_q;
    reg [10:0] x,x_d;
    reg [9:0] y,y_d;
    reg vs_d,hs_d,de_d,frame_started,enable_frame,enable_previous;
    reg fresh_frame,history_ready;
    reg [31:0] last_source_id;
    reg [23:0] rgb_d;
    reg [7:0] gray_d,gray_d2;
    wire [9:0] gray_sum={2'd0,rgb[23:16]}+{1'd0,rgb[15:8],1'b0}+{2'd0,rgb[7:0]};
    wire [7:0] gray=gray_sum[9:2];
    wire [9:0] smooth_sum={2'd0,gray_d2}+{1'd0,gray_d,1'b0}+{2'd0,gray}+10'd2;
    wire [7:0] smooth_y=de_d && de?smooth_sum[9:2]:gray_d;
    wire center_roi=de_d && x_d>=ROI_X && x_d<ROI_X+ROI_W && y_d>=ROI_Y && y_d<ROI_Y+ROI_H;
    wire [$clog2(ROI_W)-1:0] display_x=x_d-ROI_X;
    wire [$clog2(ROI_H)-1:0] display_y=y_d-ROI_Y;
    wire [X_BITS-1:0] local_x=display_x>>SAMPLE_LOG2;
    wire [Y_BITS-1:0] local_y=display_y>>SAMPLE_LOG2;
    wire first_row=(display_y & (STEP-1))==0;
    wire first_col=(display_x & (STEP-1))==0;
    wire cell_end=(display_x & (STEP-1))==STEP-1 && (display_y & (STEP-1))==STEP-1;
    (* ram_style="bram" *) reg [7:0] row_luma[0:GRID_W-1];
    reg [7:0] row_q,held_luma;
    reg top_row0,cell_end0;
    wire [ADDRESS_BITS-1:0] address={local_y,local_x};
    wire border=center_roi && (x_d==ROI_X || x_d==ROI_X+ROI_W-1 || y_d==ROI_Y || y_d==ROI_Y+ROI_H-1);
    wire final_pixel=center_roi && x_d==ROI_X+ROI_W-1 && y_d==ROI_Y+ROI_H-1;
    reg vs0,hs0,de0,roi0,border0,final0,enabled0,fresh0,ready0;
    reg [23:0] rgb0;
    reg [13:0] sample0;
    reg [ADDRESS_BITS-1:0] address0;

    wire signed [14:0] fast_old=$signed({1'b0,history_q[27:14]});
    wire signed [14:0] slow_old=$signed({1'b0,history_q[13:0]});
    wire [13:0] coarse_sample=top_row0?sample0:{row_q,6'd0};
    wire signed [14:0] sample_value=$signed({1'b0,coarse_sample});
    wire signed [14:0] fast_error=sample_value-fast_old;
    wire signed [14:0] slow_error=sample_value-slow_old;
    wire signed [14:0] fast_update=fast_old+(fast_error>>>FAST_SHIFT);
    wire signed [14:0] slow_update=slow_old+(slow_error>>>SLOW_SHIFT);
    wire [13:0] fast_next=!ready0?coarse_sample:fresh0?fast_update[13:0]:history_q[27:14];
    wire [13:0] slow_next=!ready0?coarse_sample:fresh0?slow_update[13:0]:history_q[13:0];
    wire signed [14:0] band=$signed({1'b0,fast_next})-$signed({1'b0,slow_next});
    wire signed [14:0] delta=band>>>(6-GAIN_SHIFT);

    function [7:0] add_luma;
        input [7:0] channel;
        input signed [14:0] change;
        reg signed [15:0] value;
        begin
            value=$signed({1'b0,channel})+change;
            add_luma=value<0?8'd0:value>255?8'd255:value[7:0];
        end
    endfunction

    always @(posedge clk) begin
        if(center_roi)begin
            history_q<=history[address];row_q<=row_luma[local_x];
            if(first_row && first_col)row_luma[local_x]<=smooth_y;
        end
        if(rst_n && roi0 && cell_end0 && enabled0 && (fresh0 || !ready0))
            history[address0]<={fast_next,slow_next};
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;x_d<=0;y_d<=0;vs_d<=0;hs_d<=0;de_d<=0;
            rgb_d<=0;gray_d<=0;gray_d2<=0;held_luma<=0;top_row0<=0;cell_end0<=0;
            frame_started<=0;enable_frame<=0;enable_previous<=0;
            fresh_frame<=0;history_ready<=0;last_source_id<=0;
            vs0<=0;hs0<=0;de0<=0;roi0<=0;border0<=0;final0<=0;
            enabled0<=0;fresh0<=0;ready0<=0;rgb0<=0;sample0<=0;address0<=0;
            out_vsync<=0;out_hsync<=0;out_de<=0;out_rgb<=0;
        end else begin
            vs_d<=vsync;hs_d<=hsync;de_d<=de;rgb_d<=rgb;
            gray_d<=gray;gray_d2<=gray_d;x_d<=x;y_d<=y;
            if(vsync && !vs_d) begin x<=0;y<=0;frame_started<=0;end
            else if(de) x<=x+1'b1;
            else if(de_d) begin x<=0;y<=y+1'b1;end
            if(de && !frame_started) begin
                frame_started<=1;enable_frame<=enable;enable_previous<=enable;
                fresh_frame<=source_frame_id!=last_source_id;
                last_source_id<=source_frame_id;
                if(enable!=enable_previous) history_ready<=0;
            end
            if(final0 && enabled0) history_ready<=1;
            vs0<=vs_d;hs0<=hs_d;de0<=de_d;rgb0<=rgb_d;
            roi0<=center_roi;border0<=border;final0<=final_pixel;
            enabled0<=enable_frame;fresh0<=fresh_frame;ready0<=history_ready;
            if(center_roi && first_row && first_col)held_luma<=smooth_y;
            top_row0<=first_row;cell_end0<=cell_end;
            sample0<={first_col?smooth_y:held_luma,6'd0};address0<=address;
            out_vsync<=vs0;out_hsync<=hs0;out_de<=de0;
            if(!de0) out_rgb<=0;
            else if(roi0 && enabled0 && border0) out_rgb<=24'hff40ff;
            else if(roi0 && enabled0 && ready0)
                out_rgb<={add_luma(rgb0[23:16],delta),add_luma(rgb0[15:8],delta),add_luma(rgb0[7:0],delta)};
            else out_rgb<=rgb0;
        end
    end
endmodule
