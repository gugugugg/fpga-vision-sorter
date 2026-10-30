`timescale 1ns/1ps
// Small sampled analysis branch, independent of the full-resolution shared
// image operators. 32x32 bin-centre samples cover the whole configured image.
// Diagnostic maps show completed results with source frame_id and are hidden
// during recomputation. New pose command arms exactly one complete frame.
module qc_inspection_1ppc #(parameter WIDTH=1024,HEIGHT=600,B=5,
 parameter ENABLE_COMPONENTS=1,ENABLE_VOXELS=1,ENABLE_INTERNAL_MOTION=1)(
 input wire clk,rst_n,vsync,de,input wire [23:0] rgb,
 input wire [5:0] mode,input wire [31:0] frame_id,
 input wire view_valid,input wire [2:0] view_id,
 input wire teach_2d,clear_2d,
 input wire coeff_valid,coeff_clear,input wire [2:0] coeff_view,input wire signed [9:0] coeff_cos,coeff_sin,
 output wire [23:0] diagnostic_rgb,
 output reg result_valid,output wire [31:0] result_frame_id,
 output wire [2*B:0] components,holes,area,
 output wire [B-1:0] cx,cy,hole_x,hole_y,
 output reg signed [B:0] velocity_x,velocity_y,
 output wire [3*B:0] volume,missing,excess,
 output wire [B:0] size_x,size_y,size_z,
 output reg hull_valid,output wire defect,reobserve,
 output wire [2:0] suggested_view,output reg [5:0] views_loaded,
 output wire [31:0] hull_result_frame_id,
 output wire reference_2d_valid,defect_2d,
 output wire [B:0] width_2d,height_2d,
 output reg [5:0] coefficients_loaded,
 output reg overrun,output wire reference_valid
);
 localparam N=1<<B,P=1<<(2*B);
 reg vs_d,de_d,frame_seen,analysis_frame_ok;reg [10:0] x;reg [9:0] y;
 reg [B-1:0] col,row;reg [10:0] xtarget;reg [9:0] ytarget;
 reg [2*B:0] sample_count;reg [5:0] mode_frame;
 reg [31:0] id_frame,prev_id;
 reg pose_pending,capture_view;reg [2:0] pending_view,active_view;
 reg sample_d;reg [2*B-1:0] sample_addr;
 reg [7:0] current_y,previous_y_q;reg [7:0] history[0:P-1];
 reg history_valid,history_for_frame;
 wire frame_start=vsync && !vs_d;
 wire sample_now=de && frame_seen && x==xtarget && y==ytarget && sample_count<P;
 wire [2*B-1:0] addr_now={row,col};
 wire [15:0] lum=(rgb[23:16]*16'd77)+(rgb[15:8]*16'd150)+(rgb[7:0]*16'd29);
 wire [7:0] delta=current_y>=previous_y_q?current_y-previous_y_q:previous_y_q-current_y;
 wire foreground=current_y>=128;
 wire motion=history_for_frame && delta>=24;
 wire selected_mask=(ENABLE_INTERNAL_MOTION && mode_frame==15)?motion:foreground;
 wire blobs_busy,blobs_done;reg blobs_start;
 wire [B-1:0] display_col=x*N/WIDTH,display_row=y*N/HEIGHT;
 wire [2*B:0] label;wire contour;
 wire [B-1:0] x0,x1,y0,y1;
 assign width_2d=components!=0?{1'b0,x1}-{1'b0,x0}+1'b1:0;
 assign height_2d=components!=0?{1'b0,y1}-{1'b0,y0}+1'b1:0;
 reg [31:0] hull_frame_id;
 assign hull_result_frame_id=hull_frame_id;
 generate if(ENABLE_COMPONENTS) begin: g_components
 reg teach_pending;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) teach_pending<=0;
  else if(clear_2d) teach_pending<=0;
  else if(teach_2d && !(result_valid && !blobs_busy)) teach_pending<=1;
  else if(blobs_done) teach_pending<=0;
 qc_components #(.B(B)) blobs(.clk(clk),.rst_n(rst_n),
  .load(sample_d && analysis_frame_ok && !blobs_busy),.addr(sample_addr),.pixel(selected_mask),
  .start(blobs_start),.frame_id(id_frame),.busy(blobs_busy),.done(blobs_done),
  .result_frame_id(result_frame_id),.components(components),.holes(holes),
  .largest_area(area),.cx(cx),.cy(cy),.x0(x0),.x1(x1),.y0(y0),.y1(y1),
  .hole_x(hole_x),.hole_y(hole_y),.read_addr({display_row,display_col}),
  .read_label(label),.read_contour(contour),.contour_valid(),.contour_addr(),.contour_label());
 qc_reference_2d u_reference_2d(.clk(clk),.rst_n(rst_n),
  .teach((teach_2d && result_valid && !blobs_busy) || (teach_pending && blobs_done)),
  .clear(clear_2d),.published(blobs_done),
  .components(components),.holes(holes),.area(area),.width(width_2d),.height(height_2d),
  .reference_valid(reference_2d_valid),.defect(defect_2d));
 end else begin: g_no_components
  assign blobs_busy=0;assign blobs_done=0;assign result_frame_id=hull_frame_id;
  assign components=0;assign holes=0;assign area=0;
  assign cx=0;assign cy=0;assign x0=0;assign x1=0;assign y0=0;assign y1=0;
  assign hole_x=0;assign hole_y=0;assign label=0;assign contour=0;
  assign reference_2d_valid=0;assign defect_2d=0;
 end endgenerate
 wire hull_busy,hull_done,projection;reg hull_start;
 wire teach_reference=view_valid && view_id==7 && hull_valid && !hull_busy;
 wire [5:0] support;
 generate if(ENABLE_VOXELS) begin: g_voxels
 qc_voxel_carver #(.B(B)) hull(.clk(clk),.rst_n(rst_n),
  .mask_we(sample_d && capture_view && !hull_busy),.mask_view(active_view),
  .mask_addr(sample_addr),.mask_value(foreground),
  .coeff_we(coeff_valid),.coeff_view(coeff_view),.coeff_cos(coeff_cos),.coeff_sin(coeff_sin),
  .reference_we(1'b0),.reference_commit(1'b0),.reference_addr({3*B{1'b0}}),.reference_value(1'b0),
  .reference_capture(teach_reference),.reference_valid(reference_valid),
  .start(hull_start),.busy(hull_busy),.done(hull_done),
  .volume(volume),.missing(missing),.excess(excess),
  .size_x(size_x),.size_y(size_y),.size_z(size_z),
  .min_x(),.min_y(),.min_z(),.max_x(),.max_y(),.max_z(),
  .view_support(support),.suggested_view(suggested_view),.reobserve(reobserve),.defect(defect),
  .read_addr({3*B{1'b0}}),.occupied(),
  .projection_addr({display_row,display_col}),.projection(projection));
 end else begin: g_no_voxels
  assign hull_busy=0;assign hull_done=0;assign projection=0;assign support=0;
  assign volume=0;assign missing=0;assign excess=0;
  assign size_x=0;assign size_y=0;assign size_z=0;
  assign reference_valid=0;assign suggested_view=0;assign reobserve=0;assign defect=0;
 end endgenerate
 reg [B-1:0] prev_cx,prev_cy;reg tracked;
 reg [5:0] render_mode;
 always @(posedge clk) render_mode<=mode;
 assign diagnostic_rgb=!(render_mode==18?hull_valid:result_valid)?24'd0:
   render_mode==13?(contour?24'hffffff:24'd0):
   render_mode==14?(label[2*B]?24'hff8020:24'd0):
   render_mode==18?(hull_valid && projection?24'h40d0ff:24'd0):
   label[2*B]?24'd0:label==0?24'd0:
   render_mode==15?24'hff2020:{label[4:0],3'b100,label[6:2],3'b100,label[8:4],3'b100};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   vs_d<=0;de_d<=0;frame_seen<=0;analysis_frame_ok<=0;x<=0;y<=0;col<=0;row<=0;
   xtarget<=WIDTH/(2*N);ytarget<=HEIGHT/(2*N);sample_count<=0;
   mode_frame<=0;id_frame<=0;prev_id<=0;sample_d<=0;sample_addr<=0;
   current_y<=0;previous_y_q<=0;history_valid<=0;history_for_frame<=0;
   pose_pending<=0;capture_view<=0;pending_view<=0;active_view<=0;
   views_loaded<=0;hull_start<=0;hull_valid<=0;blobs_start<=0;hull_frame_id<=0;
   coefficients_loaded<=0;
   result_valid<=0;overrun<=0;prev_cx<=0;prev_cy<=0;tracked<=0;
   velocity_x<=0;velocity_y<=0;
  end else begin
   vs_d<=vsync;de_d<=de;sample_d<=sample_now;
   hull_start<=0;blobs_start<=0;
   if(ENABLE_VOXELS && coeff_clear) begin coefficients_loaded<=0;views_loaded<=0;hull_valid<=0;pose_pending<=0;end
   else if(ENABLE_VOXELS && coeff_valid && coeff_view<6) begin
    if(hull_busy) overrun<=1;
    else begin coefficients_loaded[coeff_view]<=1;views_loaded<=0;hull_valid<=0;pose_pending<=0;end
   end
   if(ENABLE_VOXELS && view_valid) begin
    if(view_id==6) begin
     views_loaded<=0;hull_valid<=0;pose_pending<=0;
     if(!ENABLE_COMPONENTS) result_valid<=0;
    end
    else if(view_id<6) begin pending_view<=view_id;pose_pending<=1;end
    else if(teach_reference) begin
     views_loaded<=0;hull_valid<=0;pose_pending<=0;
     if(!ENABLE_COMPONENTS) result_valid<=0;
    end
   end
   if(frame_start) begin
    frame_seen<=1;x<=0;y<=0;col<=0;row<=0;sample_count<=0;
    xtarget<=WIDTH/(2*N);ytarget<=HEIGHT/(2*N);
    capture_view<=0;overrun<=blobs_busy;analysis_frame_ok<=!blobs_busy;
   end
   if(de && !frame_start) begin
    x<=x+1'b1;
    if(x==0 && y==0) begin
      id_frame<=frame_id;mode_frame<=mode;
      history_for_frame<=history_valid;
      if(ENABLE_VOXELS && pose_pending && !hull_busy) begin
        active_view<=pending_view;capture_view<=1;pose_pending<=0;
      end
    end
    if(x==WIDTH-1 && y==HEIGHT-1 && frame_seen) begin
      if(ENABLE_COMPONENTS) begin
       if((sample_count==P || (sample_now && sample_count==P-1)) && analysis_frame_ok && !blobs_busy) begin blobs_start<=1;history_valid<=1;end
       else overrun<=1;
      end
      if(capture_view && (sample_count==P || (sample_now && sample_count==P-1)) && !hull_busy) begin
        views_loaded[active_view]<=1;
        if((views_loaded | (6'b1<<active_view))==6'b111111) hull_start<=1;
      end
    end
   end else if(de_d && !de) begin x<=0;y<=y+1'b1;end
   if(sample_now) begin
    if(ENABLE_INTERNAL_MOTION) begin previous_y_q<=history[addr_now];history[addr_now]<=lum[15:8];end
    current_y<=lum[15:8];sample_addr<=addr_now;sample_count<=sample_count+1'b1;
    if(col==N-1) begin
      col<=0;row<=row+1'b1;xtarget<=WIDTH/(2*N);
      ytarget<=((row+1)*HEIGHT)/N+HEIGHT/(2*N);
    end else begin col<=col+1'b1;xtarget<=((col+1)*WIDTH)/N+WIDTH/(2*N);end
   end
   if(blobs_start) result_valid<=0;
   if(hull_start) begin
    hull_valid<=0;hull_frame_id<=id_frame;
    if(!ENABLE_COMPONENTS) result_valid<=0;
   end
   if(blobs_done) begin
    result_valid<=1;
    if(result_frame_id!=prev_id) begin
      if(tracked && area!=0) begin
       velocity_x<=$signed({1'b0,cx})-$signed({1'b0,prev_cx});
       velocity_y<=$signed({1'b0,cy})-$signed({1'b0,prev_cy});
      end else begin velocity_x<=0;velocity_y<=0;end
      tracked<=area!=0;prev_cx<=cx;prev_cy<=cy;prev_id<=result_frame_id;
    end
   end
   if(hull_done && views_loaded==6'b111111) begin
    hull_valid<=1;
    if(!ENABLE_COMPONENTS) result_valid<=1;
   end
   // Invalidate last, including a coefficient command coincident with the
   // final sampled pixel. No partial frame can revalidate an old projection.
   if(ENABLE_VOXELS && (coeff_clear || (coeff_valid && coeff_view<6 && !hull_busy))) begin
    views_loaded<=0;hull_valid<=0;pose_pending<=0;capture_view<=0;hull_start<=0;
   end
  end
 end
endmodule
