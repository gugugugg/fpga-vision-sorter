`timescale 1ns/1ps
// Calibrated planar mapping. An 8x8 grid stores six measured joint targets per
// vertex. Bilinear interpolation is sequential and shares one small multiplier.
// This is a calibrated joint-space surface, not a general six-axis IK solver.
module visual_joint_mapper #(
 parameter IMG_WIDTH=1024,IMG_HEIGHT=600
)(
 input wire clk,rst_n,clear,
 input wire write,input wire [5:0] write_addr,input wire [71:0] write_pose,
 input wire start,input wire [10:0] x,input wire [9:0] y,
 input wire [10:0] x0,x1,input wire [9:0] y0,y1,
 output wire ready,output reg done,rejected,output reg [71:0] pose,
 output wire calibrated
);
 reg [71:0] grid[0:63];reg [63:0] loaded;
 reg [3:0] state;reg [5:0] base;reg [2:0] axis;
 reg [8:0] fx,fy;reg [2:0] gx;reg [31:0] norm_x;
 reg [10:0] hold_x;reg [9:0] hold_y;
 reg [10:0] lo_x,hi_x;reg [9:0] lo_y,hi_y;
 reg div_start;reg [39:0] dividend;reg [31:0] divisor;
 wire div_done,div_busy;wire [39:0] quotient;
 seq_div_u40_u32 divider(.i_clk(clk),.i_rst_n(rst_n),.i_start(div_start),
  .i_numerator(dividend),.i_denominator(divisor),.o_busy(div_busy),.o_done(div_done),
  .o_quotient(quotient));
 reg [71:0] p00,p10,p01,p11;
 reg signed [12:0] lhs,delta;
 reg [8:0] fraction;
 wire signed [22:0] product=delta*$signed({1'b0,fraction});
 reg signed [13:0] low_row,high_row;
 wire [11:0] c00=p00>>(axis*12),c10=p10>>(axis*12);
 wire [11:0] c01=p01>>(axis*12),c11=p11>>(axis*12);
 integer j;reg pose_ok;
 always @* begin
  pose_ok=1;
  for(j=0;j<6;j=j+1) if(((write_pose>>(j*12)) & 72'hfff)<500 ||
    ((write_pose>>(j*12)) & 72'hfff)>2500) pose_ok=0;
 end
 assign ready=state==0;assign calibrated=&loaded;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin loaded<=0;state<=0;base<=0;axis<=0;fx<=0;fy<=0;gx<=0;
   norm_x<=0;hold_x<=0;hold_y<=0;lo_x<=0;hi_x<=0;lo_y<=0;hi_y<=0;
   div_start<=0;dividend<=0;divisor<=1;done<=0;rejected<=0;pose<=0;
   p00<=0;p10<=0;p01<=0;p11<=0;lhs<=0;delta<=0;fraction<=0;low_row<=0;high_row<=0;end
  else begin
   done<=0;rejected<=0;div_start<=0;
   if(clear) begin loaded<=0;state<=0;end
   else if(write) begin
    if(ready && pose_ok) begin grid[write_addr]<=write_pose;loaded[write_addr]<=1;end
    else rejected<=1;
   end
   else case(state)
    0:if(start) begin
     if(x0>=x1 || y0>=y1 || x<x0 || x>x1 || y<y0 || y>y1 || x1>=IMG_WIDTH || y1>=IMG_HEIGHT)
      rejected<=1;
     else begin
      hold_x<=x;hold_y<=y;lo_x<=x0;hi_x<=x1;lo_y<=y0;hi_y<=y1;
      dividend<=(x-x0)*40'd1792;divisor<=x1-x0;div_start<=1;state<=1;
     end
    end
    1:if(div_done) begin
     norm_x<=quotient[31:0];dividend<=(hold_y-lo_y)*40'd1792;
     divisor<=hi_y-lo_y;div_start<=1;state<=2;
    end
    2:if(div_done) begin
     gx<=norm_x>=1792 ? 6:norm_x[10:8];fx<=norm_x>=1792 ? 9'd256:{1'b0,norm_x[7:0]};
     fy<=quotient>=1792 ? 9'd256:{1'b0,quotient[7:0]};
     base<={quotient>=1792 ? 3'd6:quotient[10:8],norm_x>=1792 ? 3'd6:norm_x[10:8]};state<=3;
    end
    3:begin
     if(!(loaded[base] && loaded[base+1] && loaded[base+8] && loaded[base+9])) begin rejected<=1;state<=0;end
     else begin p00<=grid[base];state<=4;end
    end
    4:begin p10<=grid[base+1];state<=5;end
    5:begin p01<=grid[base+8];state<=6;end
    6:begin p11<=grid[base+9];axis<=0;state<=7;end
    7:begin lhs<=$signed({1'b0,c00});delta<=$signed({1'b0,c10})-$signed({1'b0,c00});fraction<=fx;state<=8;end
    8:begin low_row<=lhs+(product>>>8);
     lhs<=$signed({1'b0,c01});delta<=$signed({1'b0,c11})-$signed({1'b0,c01});state<=9;end
    9:begin high_row<=lhs+(product>>>8);state<=10;end
    10:begin lhs<=low_row;delta<=high_row-low_row;fraction<=fy;state<=11;end
    11:begin pose[axis*12 +: 12]<=lhs+(product>>>8);
     if(axis==5) begin done<=1;state<=0;end else begin axis<=axis+1'b1;state<=7;end
    end
    default:state<=0;
   endcase
  end
 end
endmodule
