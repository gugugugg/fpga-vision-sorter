`timescale 1ns/1ps
// Self-developed RAW10 -> RGB888 bilinear ISP. Four rotating RAW8 line banks
// let the completed row be read with both neighbours while the camera writes
// the next row. All RAM reads are synchronous. No empirical line-time guess.
// Reflect-101 borders preserve Bayer parity: x=-1 -> 1, x=W -> W-2.
// Source needs >=4 blank cycles per row and >=WIDTH/4+8 at frame end to flush
// the last row. The SC520CS centred crop provides substantially more margin.
// A schedule/length violation is sticky and prevents capture acceptance.
module bayer_bilinear_4ppc #(
    parameter WIDTH=1024,HEIGHT=600,
    parameter BAYER_MODE="BGGR"
)(
    input wire i_clk,i_rst_n,i_valid,i_sof,i_last,
    input wire [39:0] i_raw,
    output reg o_valid,o_sof,o_last,
    output reg [95:0] o_rgb,
    output reg o_fault
);
    localparam GROUPS=WIDTH/4;
    localparam XB=$clog2(GROUPS),YB=$clog2(HEIGHT);
    reg [31:0] bank0[0:GROUPS-1],bank1[0:GROUPS-1];
    reg [31:0] bank2[0:GROUPS-1],bank3[0:GROUPS-1];
    reg [XB-1:0] wx,rx;
    reg [YB-1:0] wy,output_row;
    reg reading,flush_bottom,pending_last,started;
    reg [1:0] top_bank,mid_bank,bot_bank;
    reg [31:0] q0,q1,q2,q3;
    reg q_valid,q_last;
    reg [XB-1:0] q_x;
    reg [31:0] top_buf,mid_buf,bot_buf;
    reg [7:0] top_left,mid_left,bot_left;
    reg [XB-1:0] buf_x;
    wire [31:0] raw8={i_raw[39:32],i_raw[29:22],i_raw[19:12],i_raw[9:2]};
    wire [XB-1:0] write_x=i_sof?0:wx;
    wire [YB-1:0] write_y=i_sof?0:wy;
    // RAM processes deliberately have no reset or asynchronous read.
    always @(posedge i_clk) begin
        if(i_valid) case(write_y[1:0])
            0:bank0[write_x]<=raw8;1:bank1[write_x]<=raw8;
            2:bank2[write_x]<=raw8;3:bank3[write_x]<=raw8;
        endcase
        if(reading) begin
            q0<=bank0[rx];q1<=bank1[rx];q2<=bank2[rx];q3<=bank3[rx];
        end
    end
    function [31:0] select_bank;
        input [1:0] index;
        input [127:0] data;
        begin case(index)
            0:select_bank=data[31:0];1:select_bank=data[63:32];
            2:select_bank=data[95:64];3:select_bank=data[127:96];
        endcase end
    endfunction
    wire [31:0] top_q=select_bank(top_bank,{q3,q2,q1,q0});
    wire [31:0] mid_q=select_bank(mid_bank,{q3,q2,q1,q0});
    wire [31:0] bot_q=select_bank(bot_bank,{q3,q2,q1,q0});
    function [23:0] pixel;
        input [47:0] t,m,b;
        input integer lane;
        input row_odd;
        reg [7:0] a,c,d,e,f,g,h,j;
        reg [9:0] cross_sum,diag_sum;
        reg [8:0] horizontal,vertical;
        reg [7:0] red,green,blue;
        reg xp,yp;
        begin
            a=t[lane*8+:8];c=t[(lane+2)*8+:8];
            d=m[lane*8+:8];e=m[(lane+1)*8+:8];f=m[(lane+2)*8+:8];
            g=b[lane*8+:8];h=t[(lane+1)*8+:8];j=b[(lane+2)*8+:8];
            cross_sum={2'd0,d}+{2'd0,f}+{2'd0,h}+{2'd0,b[(lane+1)*8+:8]};
            diag_sum={2'd0,a}+{2'd0,c}+{2'd0,g}+{2'd0,j};
            horizontal={1'b0,d}+{1'b0,f};vertical={1'b0,h}+{1'b0,b[(lane+1)*8+:8]};
            xp=lane[0] ^ ((BAYER_MODE=="GBRG") || (BAYER_MODE=="RGGB"));
            yp=row_odd ^ ((BAYER_MODE=="GRBG") || (BAYER_MODE=="RGGB"));
            if(!yp && !xp) begin blue=e;green=cross_sum[9:2];red=diag_sum[9:2];end
            else if(yp && xp) begin red=e;green=cross_sum[9:2];blue=diag_sum[9:2];end
            else if(!yp) begin green=e;blue=horizontal[8:1];red=vertical[8:1];end
            else begin green=e;red=horizontal[8:1];blue=vertical[8:1];end
            pixel={red,green,blue};
        end
    endfunction
    wire emit=(q_valid && q_x!=0) || pending_last;
    wire [7:0] tr=pending_last?top_buf[23:16]:top_q[7:0];
    wire [7:0] mr=pending_last?mid_buf[23:16]:mid_q[7:0];
    wire [7:0] br=pending_last?bot_buf[23:16]:bot_q[7:0];
    wire [47:0] tw={tr,top_buf,top_left},mw={mr,mid_buf,mid_left},bw={br,bot_buf,bot_left};
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            wx<=0;wy<=0;rx<=0;output_row<=0;reading<=0;flush_bottom<=0;
            pending_last<=0;started<=0;top_bank<=0;mid_bank<=0;bot_bank<=0;
            q_valid<=0;q_last<=0;q_x<=0;buf_x<=0;
            top_buf<=0;mid_buf<=0;bot_buf<=0;top_left<=0;mid_left<=0;bot_left<=0;
            o_valid<=0;o_sof<=0;o_last<=0;o_rgb<=0;o_fault<=0;
        end else begin
            o_valid<=emit;o_sof<=emit && output_row==0 && buf_x==0;
            o_last<=emit && pending_last;
            if(emit) begin
                o_rgb<={pixel(tw,mw,bw,0,output_row[0]),pixel(tw,mw,bw,1,output_row[0]),
                        pixel(tw,mw,bw,2,output_row[0]),pixel(tw,mw,bw,3,output_row[0])};
            end
            q_valid<=reading;q_x<=rx;q_last<=reading && rx==GROUPS-1;
            if(reading) begin
                if(rx==GROUPS-1) reading<=0;
                else rx<=rx+1'b1;
            end
            pending_last<=q_valid && q_last;
            if(q_valid) begin
                if(q_x==0) begin
                    top_left<=top_q[15:8];mid_left<=mid_q[15:8];bot_left<=bot_q[15:8];
                end else begin
                    top_left<=top_buf[31:24];mid_left<=mid_buf[31:24];bot_left<=bot_buf[31:24];
                end
                top_buf<=top_q;mid_buf<=mid_q;bot_buf<=bot_q;buf_x<=q_x;
            end
            if(pending_last && flush_bottom) begin
                reading<=1;rx<=0;output_row<=HEIGHT-1;flush_bottom<=0;
                top_bank<=(HEIGHT-2)%4;mid_bank<=(HEIGHT-1)%4;bot_bank<=(HEIGHT-2)%4;
            end
            if(i_valid) begin
                if(i_sof) begin
                    if(reading || q_valid || pending_last) o_fault<=1;
                    wx<=1;wy<=0;started<=1;
                end else if(!started) o_fault<=1;
                else if(i_last!=(write_x==GROUPS-1)) o_fault<=1;
                if(i_last) begin
                    wx<=0;wy<=write_y==HEIGHT-1?0:write_y+1'b1;
                    if(write_y!=0) begin
                        if(reading || q_valid || pending_last) o_fault<=1;
                        reading<=1;rx<=0;output_row<=write_y-1'b1;
                        top_bank<=write_y==1?1:write_y-2'd2;
                        mid_bank<=write_y-1'b1;bot_bank<=write_y;
                        if(write_y==HEIGHT-1) begin flush_bottom<=1;started<=0;end
                    end
                end else wx<=write_x+1'b1;
            end
        end
    end
endmodule
