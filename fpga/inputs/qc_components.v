`timescale 1ns/1ps
// Exact four-connected foreground/background flood fill. Serial neighbours
// share one synchronous mask/visited read port; queue and result map are
// synchronous RAMs without reset. Load every mask pixel before start.
module qc_components #(parameter B=5)(
 input wire clk,rst_n,load,start,input wire [2*B-1:0] addr,
 input wire pixel,input wire [31:0] frame_id,
 output reg busy,done,output reg [31:0] result_frame_id,
 output reg [2*B:0] components,holes,largest_area,
 output reg [B-1:0] cx,cy,x0,x1,y0,y1,
 output reg [B-1:0] hole_x,hole_y,
 input wire [2*B-1:0] read_addr,
 output wire [2*B:0] read_label,output wire read_contour,
 output reg contour_valid,output reg [2*B-1:0] contour_addr,
 output reg [2*B:0] contour_label
);
 localparam N=1<<B,P=1<<(2*B);
 localparam IDLE=0,SCAN_READ=1,SCAN_CHECK=2,POP_READ=3,POP_COMMIT=4,
 STATS=5,NEIGH_READ=6,NEIGH_CHECK=7,PIXEL_DONE=8,REGION_DONE=9,
 FILL_READ=10,FILL_WRITE=11,DIV_X=12,WAIT_X=13,DIV_Y=14,WAIT_Y=15,
 DIV_HX=16,WAIT_HX=17,DIV_HY=18,WAIT_HY=19;
 reg mask[0:P-1],visited[0:P-1];
 reg [2*B+1:0] results[0:P-1];
 reg [2*B-1:0] queue[0:P-1];
 reg [2*B+1:0] result_q;
 assign read_label=result_q[2*B:0];assign read_contour=result_q[2*B+1];
 reg [2*B:0] head,tail,area;
 reg [2*B-1:0] scan,cur,fill;
 reg [4:0] state;reg seed,touch,edge_seen;reg [1:0] dir;
 reg [3*B:0] sx,sy,best_sx,best_sy,hole_sx,hole_sy;
 reg [2*B:0] best_hole;
 reg [B-1:0] bx0,bx1,by0,by1;
 reg [2*B-1:0] neighbor;reg in_bounds;
 wire [B-1:0] px=cur[B-1:0],py=cur[2*B-1:B];
 wire boundary=(px==0)||(px==N-1)||(py==0)||(py==N-1);
 always @* begin
  neighbor=cur;in_bounds=1;
  case(dir)
   0:begin neighbor=cur-1'b1;in_bounds=px!=0;end
   1:begin neighbor=cur+1'b1;in_bounds=px!=N-1;end
   2:begin neighbor=cur-N;in_bounds=py!=0;end
   3:begin neighbor=cur+N;in_bounds=py!=N-1;end
  endcase
 end
 wire [2*B-1:0] query_addr=(state==NEIGH_READ || state==NEIGH_CHECK)?neighbor:scan;
 wire [2*B-1:0] queue_addr=(state==FILL_READ || state==FILL_WRITE)?fill:head[2*B-1:0];
 reg mask_q,visited_q;reg [2*B-1:0] queue_q;
 wire enqueue=state==NEIGH_CHECK && in_bounds && !visited_q && mask_q==seed;
 wire new_seed=state==SCAN_CHECK && !visited_q;
 wire [2*B-1:0] queue_waddr=new_seed?{2*B{1'b0}}:tail[2*B-1:0];
 wire [2*B-1:0] queue_wdata=new_seed?scan:neighbor;
 wire [2*B-1:0] result_waddr=(state==FILL_WRITE)?queue_q:cur;
 wire [2*B:0] pixel_label=seed?(components+1'b1):{(2*B+1){1'b0}};
 wire [2*B+1:0] result_wdata=(state==FILL_WRITE)?
   {1'b0,!touch,{2*B{1'b0}}}:{edge_seen,pixel_label};
 // No memory reset; one write address and one registered read per RAM.
 always @(posedge clk) begin
  mask_q<=mask[query_addr];visited_q<=visited[query_addr];
  queue_q<=queue[queue_addr];result_q<=results[read_addr];
  if(rst_n) begin
   if(load && !busy) mask[addr]<=pixel;
   if(load && !busy) visited[addr]<=0;
   else if(new_seed || enqueue) visited[query_addr]<=1;
   if(new_seed || enqueue) queue[queue_waddr]<=queue_wdata;
   if(state==PIXEL_DONE || state==FILL_WRITE) results[result_waddr]<=result_wdata;
  end
 end
 reg div_start;reg [39:0] num;reg [31:0] den;
 wire div_done;wire [39:0] quotient;
 seq_div_u40_u32 div(.i_clk(clk),.i_rst_n(rst_n),.i_start(div_start),
 .i_numerator(num),.i_denominator(den),.o_done(div_done),.o_busy(),.o_quotient(quotient));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   busy<=0;done<=0;state<=IDLE;components<=0;holes<=0;largest_area<=0;
   cx<=0;cy<=0;x0<=0;x1<=0;y0<=0;y1<=0;hole_x<=0;hole_y<=0;
   result_frame_id<=0;scan<=0;head<=0;tail<=0;fill<=0;
   div_start<=0;num<=0;den<=1;contour_valid<=0;contour_addr<=0;contour_label<=0;
   best_sx<=0;best_sy<=0;hole_sx<=0;hole_sy<=0;best_hole<=0;
   cur<=0;seed<=0;touch<=0;edge_seen<=0;dir<=0;area<=0;sx<=0;sy<=0;
   bx0<=0;bx1<=0;by0<=0;by1<=0;
  end else begin
   done<=0;div_start<=0;contour_valid<=0;
   case(state)
    IDLE:if(start) begin
      busy<=1;scan<=0;components<=0;holes<=0;largest_area<=0;best_hole<=0;
      best_sx<=0;best_sy<=0;hole_sx<=0;hole_sy<=0;
      x0<=0;x1<=0;y0<=0;y1<=0;result_frame_id<=frame_id;state<=SCAN_READ;
    end
    SCAN_READ:state<=SCAN_CHECK;
    SCAN_CHECK:if(!visited_q) begin
      seed<=mask_q;head<=0;tail<=1;area<=0;sx<=0;sy<=0;touch<=0;
      bx0<=N-1;by0<=N-1;bx1<=0;by1<=0;state<=POP_READ;
    end else if(scan==P-1) begin
      num<=best_sx;den<=largest_area?largest_area:1;state<=DIV_X;
    end else begin scan<=scan+1'b1;state<=SCAN_READ;end
    POP_READ:if(head==tail) state<=REGION_DONE;else state<=POP_COMMIT;
    POP_COMMIT:begin cur<=queue_q;head<=head+1'b1;state<=STATS;end
    STATS:begin
      area<=area+1'b1;sx<=sx+px;sy<=sy+py;touch<=touch|boundary;
      if(px<bx0) bx0<=px;if(px>bx1) bx1<=px;
      if(py<by0) by0<=py;if(py>by1) by1<=py;
      edge_seen<=seed && boundary;dir<=0;state<=NEIGH_READ;
    end
    NEIGH_READ:state<=NEIGH_CHECK;
    NEIGH_CHECK:begin
      if(seed && in_bounds && !mask_q) edge_seen<=1;
      if(enqueue) tail<=tail+1'b1;
      if(dir==3) state<=PIXEL_DONE;else begin dir<=dir+1'b1;state<=NEIGH_READ;end
    end
    PIXEL_DONE:begin
      if(edge_seen) begin contour_valid<=1;contour_addr<=cur;contour_label<=components+1'b1;end
      state<=POP_READ;
    end
    REGION_DONE:begin
      if(seed) begin
        components<=components+1'b1;
        if(area>largest_area) begin
          largest_area<=area;best_sx<=sx;best_sy<=sy;
          x0<=bx0;x1<=bx1;y0<=by0;y1<=by1;
        end
        state<=SCAN_READ;
      end else begin
        if(!touch) begin
          holes<=holes+1'b1;
          if(area>best_hole) begin best_hole<=area;hole_sx<=sx;hole_sy<=sy;end
        end
        fill<=0;state<=FILL_READ;
      end
    end
    FILL_READ:state<=FILL_WRITE;
    FILL_WRITE:if({1'b0,fill}+1'b1==tail) state<=SCAN_READ;
      else begin fill<=fill+1'b1;state<=FILL_READ;end
    DIV_X:begin div_start<=1;state<=WAIT_X;end
    WAIT_X:if(div_done) begin cx<=quotient[B-1:0];num<=best_sy;state<=DIV_Y;end
    DIV_Y:begin div_start<=1;state<=WAIT_Y;end
    WAIT_Y:if(div_done) begin cy<=quotient[B-1:0];num<=hole_sx;den<=best_hole?best_hole:1;state<=DIV_HX;end
    DIV_HX:begin div_start<=1;state<=WAIT_HX;end
    WAIT_HX:if(div_done) begin hole_x<=quotient[B-1:0];num<=hole_sy;state<=DIV_HY;end
    DIV_HY:begin div_start<=1;state<=WAIT_HY;end
    WAIT_HY:if(div_done) begin hole_y<=quotient[B-1:0];busy<=0;done<=1;state<=IDLE;end
   endcase
  end
 end
endmodule
