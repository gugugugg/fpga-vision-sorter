`timescale 1ns/1ps
// CDF-min normalization; constant frames stay identity. Build the inactive
// LUT and commit it only at frame_start. Incomplete frames retain the last LUT.
module histogram_equalizer_1ppc #(
 parameter WIDTH=1024, HEIGHT=600
)(input wire clk,rst_n,frame_start,de,input wire [7:0] y,
 output wire [7:0] equalized,output reg lut_valid);
 localparam INIT=0,WAIT_FRAME=1,COLLECT=2,DRAIN=3,MIN_READ=4,MIN_SUM=5,
            BIN_READ=6,BIN_SUM=7,DIV_START=8,DIV_WAIT=9;
 reg [19:0] hist[0:255];
 reg [7:0] lut0[0:255],lut1[0:255];
 reg active_bank,lut_ready;
 reg [7:0] bin;
 reg [20:0] cdf,cdf_min;
 reg [19:0] pixels;
 reg [3:0] state;
 reg start;wire done;wire [39:0] quotient;
 reg [39:0] numerator;reg [31:0] denominator;
 reg pending,last_valid;reg [7:0] pending_addr,last_addr;
 reg [19:0] hist_q,last_word;
 wire [7:0] hist_raddr=(state==COLLECT || state==DRAIN)?y:bin;
 wire count_write=pending && (state==COLLECT || state==DRAIN);
 wire [19:0] increment=(last_valid && last_addr==pending_addr?last_word:hist_q)+1'b1;
 wire hist_we=state==INIT || state==BIN_SUM || count_write;
 wire [7:0] hist_waddr=count_write?pending_addr:bin;
 wire [19:0] hist_wdata=count_write?increment:20'd0;
 wire lut_we=(state==BIN_SUM && denominator==0) || (state==DIV_WAIT && done);
 wire [7:0] lut_wdata=denominator==0?bin:quotient[7:0];
 wire [20:0] next_cdf=cdf+{1'b0,hist_q};
 seq_div_u40_u32 div(.i_clk(clk),.i_rst_n(rst_n),.i_start(start),
   .i_numerator(numerator),.i_denominator(denominator),
   .o_quotient(quotient),.o_done(done),.o_busy());
 assign equalized=lut_valid?(active_bank?lut1[y]:lut0[y]):y;
 always @(posedge clk) begin
   hist_q<=hist[hist_raddr];
   if(rst_n && hist_we) hist[hist_waddr]<=hist_wdata;
   if(count_write) begin last_addr<=pending_addr;last_word<=increment;end
   if(rst_n && lut_we) begin
     if(active_bank) lut0[bin]<=lut_wdata;else lut1[bin]<=lut_wdata;
   end
 end
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n) begin
     bin<=0;cdf<=0;cdf_min<=0;pixels<=0;state<=INIT;start<=0;
     lut_valid<=0;active_bank<=0;lut_ready<=0;
     pending<=0;pending_addr<=0;last_valid<=0;numerator<=0;denominator<=1;
   end else begin
     start<=0;last_valid<=count_write;
     pending<=state==COLLECT && de && !frame_start;
     if(state==COLLECT && de && !frame_start) pending_addr<=y;
     if(frame_start && lut_ready) begin
       active_bank<=~active_bank;lut_valid<=1;lut_ready<=0;
     end
     case(state)
       INIT:begin
         pixels<=0;
         if(bin==255) begin bin<=0;state<=WAIT_FRAME;end else bin<=bin+1'b1;
       end
       WAIT_FRAME:if(frame_start) begin pixels<=0;state<=COLLECT;end
       COLLECT:begin
         if(de && !frame_start) pixels<=pixels+1'b1;
         if(frame_start) begin
           bin<=0;cdf<=0;cdf_min<=0;
           if(pixels==WIDTH*HEIGHT) state<=DRAIN;else state<=INIT;
         end
       end
       DRAIN:state<=MIN_READ;
       MIN_READ:state<=MIN_SUM;
       MIN_SUM:begin
         if(hist_q!=0) begin
           cdf_min<={1'b0,hist_q};denominator<=pixels-hist_q;
           bin<=0;state<=BIN_READ;
         end else if(bin==255) begin bin<=0;state<=INIT;end
         else begin bin<=bin+1'b1;state<=MIN_READ;end
       end
       BIN_READ:state<=BIN_SUM;
       BIN_SUM:begin
         cdf<=next_cdf;
         if(denominator==0) begin
           if(bin==255) begin
             if(next_cdf==pixels) begin lut_ready<=1;state<=WAIT_FRAME;end
             else state<=INIT;
             pixels<=0;bin<=0;
           end else begin bin<=bin+1'b1;state<=BIN_READ;end
         end else begin
           numerator<=next_cdf>cdf_min?(next_cdf-cdf_min)*40'd255:40'd0;
           state<=DIV_START;
         end
       end
       DIV_START:begin start<=1;state<=DIV_WAIT;end
       DIV_WAIT:if(done) begin
         if(bin==255) begin
           if(cdf==pixels) begin lut_ready<=1;state<=WAIT_FRAME;end
           else state<=INIT;
           pixels<=0;bin<=0;
         end else begin bin<=bin+1'b1;state<=BIN_READ;end
       end
     endcase
   end
 end
endmodule
