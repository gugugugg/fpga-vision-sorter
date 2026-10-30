`timescale 1ns/1ps
// A 3x3 majority removes isolated mask noise before measuring the bbox.
module part_mask_normalizer #(parameter W=128,H=75,MIN_AREA=16)(
 input wire clk,rst_n,cancel,frame_start,sample_valid,foreground,consumer_ready,
 input wire [31:0] frame_id,
 output reg busy,dropped,patch_begin,patch_we,patch_bit,patch_start,
 output reg [9:0] patch_addr,output reg [31:0] patch_id,
 output reg [7:0] bbox_x0,bbox_x1,bbox_y0,bbox_y1,
 output reg object_valid,output reg [63:0] diagnostic
);
 localparam P=W*H,IDLE=0,CAPTURE=1,FILTER_READ=2,FILTER_ADD=3,
  BEGIN_PATCH=4,READ_BIT=5,WRITE_BIT=6,FINISH=7;
 (* ram_style="bram" *) reg raw_mask[0:P-1],mask[0:P-1];
 reg [2:0] state;reg [13:0] count,area;
 reg [7:0] x,y,min_x,max_x,min_y,max_y,box_w,box_h;
 reg [9:0] index;reg bit_q,raw_q;reg [3:0] neighbour,sum;
 wire left=neighbour==0 || neighbour==3 || neighbour==6;
 wire right=neighbour==2 || neighbour==5 || neighbour==8;
 wire above=neighbour<3,below=neighbour>=6;
 wire [7:0] nx=left?x-1'b1:right?x+1'b1:x;
 wire [7:0] ny=above?y-1'b1:below?y+1'b1:y;
 wire in_bounds=!(left && x==0 || right && x==W-1 || above && y==0 || below && y==H-1);
 wire [13:0] neighbour_addr=ny*W+nx;
 wire [3:0] total=sum+(in_bounds && raw_q);
 wire foreground_filtered=total>=5;
 wire [15:0] product_x=(2*{1'b0,index[4:0]}+1)*box_w;
 wire [15:0] product_y=(2*{1'b0,index[9:5]}+1)*box_h;
 wire [7:0] sx=min_x+(product_x>>6),sy=min_y+(product_y>>6);
 wire [13:0] read_addr=sy*W+sx;
 wire [7:0] next_min_x=foreground_filtered && x<min_x?x:min_x;
 wire [7:0] next_max_x=foreground_filtered && x>max_x?x:max_x;
 wire [7:0] next_min_y=foreground_filtered && y<min_y?y:min_y;
 wire [7:0] next_max_y=foreground_filtered && y>max_y?y:max_y;
 wire [13:0] next_area=area+foreground_filtered;
 wire usable=next_area>=MIN_AREA && next_area<=P*3/4 &&
  next_max_x>=next_min_x+3 && next_max_y>=next_min_y+3 &&
  next_min_x>0 && next_max_x<W-1 && next_min_y>0 && next_max_y<H-1;
 wire [2:0] reason=next_area==0?1:next_area<MIN_AREA?2:next_area>P*3/4?3:
  (next_max_x<next_min_x+3 || next_max_y<next_min_y+3)?4:
  (next_min_x==0 || next_max_x>=W-1 || next_min_y==0 || next_max_y>=H-1)?5:0;
 always @(posedge clk)begin
  raw_q<=raw_mask[neighbour_addr];bit_q<=mask[read_addr];
  if(rst_n && !cancel && state==CAPTURE && sample_valid && !frame_start && count<P)
   raw_mask[count]<=foreground;
  if(rst_n && !cancel && state==FILTER_ADD && neighbour==8)mask[count]<=foreground_filtered;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;count<=0;area<=0;x<=0;y<=0;min_x<=W-1;max_x<=0;min_y<=H-1;max_y<=0;
   box_w<=0;box_h<=0;index<=0;busy<=0;dropped<=0;patch_begin<=0;patch_we<=0;
   patch_bit<=0;patch_start<=0;patch_addr<=0;patch_id<=0;neighbour<=0;sum<=0;
   bbox_x0<=0;bbox_x1<=0;bbox_y0<=0;bbox_y1<=0;object_valid<=0;diagnostic<=0;
  end else begin
   dropped<=0;patch_begin<=0;patch_we<=0;patch_start<=0;
   case(state)
    CAPTURE:if(sample_valid && !frame_start)begin
     count<=count+1'b1;
     if(count==P-1)begin state<=FILTER_READ;count<=0;neighbour<=0;sum<=0;end
    end
    FILTER_READ:state<=FILTER_ADD;
    FILTER_ADD:begin
     if(neighbour!=8)begin neighbour<=neighbour+1'b1;sum<=total;state<=FILTER_READ;end
     else begin
      neighbour<=0;sum<=0;area<=next_area;
      min_x<=next_min_x;max_x<=next_max_x;min_y<=next_min_y;max_y<=next_max_y;
      if(count==P-1)begin
       object_valid<=usable;diagnostic<={12'd0,usable,reason,2'd0,next_area,next_min_x,next_max_x,next_min_y,next_max_y};
       if(usable)begin
        bbox_x0<=next_min_x;bbox_x1<=next_max_x;bbox_y0<=next_min_y;bbox_y1<=next_max_y;
        box_w<=next_max_x-next_min_x+1'b1;box_h<=next_max_y-next_min_y+1'b1;
       end else begin
        min_x<=0;min_y<=0;box_w<=1;box_h<=1;bbox_x0<=0;bbox_x1<=0;bbox_y0<=0;bbox_y1<=0;
       end
       index<=0;state<=BEGIN_PATCH;
      end else begin
       count<=count+1'b1;if(x==W-1)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;
       state<=FILTER_READ;
      end
     end
    end
    BEGIN_PATCH:if(consumer_ready)begin patch_begin<=1;state<=READ_BIT;end
    READ_BIT:state<=WRITE_BIT;
    WRITE_BIT:begin
     patch_we<=1;patch_addr<=index;patch_bit<=object_valid && bit_q;
     if(index==1023)state<=FINISH;
     else begin index<=index+1'b1;state<=READ_BIT;end
    end
    FINISH:begin patch_start<=1;state<=IDLE;busy<=0;end
    default:;
   endcase
   if(frame_start)begin
    if(state==IDLE || state==CAPTURE)begin
     if(state==CAPTURE)dropped<=1;
     state<=CAPTURE;busy<=1;patch_id<=frame_id;count<=0;area<=0;x<=0;y<=0;
     min_x<=W-1;max_x<=0;min_y<=H-1;max_y<=0;object_valid<=0;neighbour<=0;sum<=0;
    end else dropped<=1;
   end
   if((state==READ_BIT || state==WRITE_BIT || state==FINISH) && !consumer_ready)begin
    state<=IDLE;busy<=0;patch_begin<=0;patch_we<=0;patch_start<=0;dropped<=1;
   end
   if(cancel)begin state<=IDLE;busy<=0;patch_begin<=0;patch_we<=0;patch_start<=0;object_valid<=0;dropped<=1;end
  end
 end
endmodule
