`timescale 1ns/1ps
// Large Chinese state labels and boxes share the three-clock video pipeline.
module part_template_overlay #(
 parameter WIDTH=1024,ROI_X=256,ROI_Y=150,ROI_W=512,ROI_H=300,CLK_HZ=52002000,
 parameter TEXT_FILE="template_labels.dath",
 parameter FOOTER_FILE="capture_footer.hex",
 parameter FONT_FILE="template_font.hex",GW=128,GH=75
)(input wire clk,rst_n,vsync,hsync,de,input wire [23:0] rgb,
 input wire [31:0] frame_id,prediction_id,capture_id,
 input wire show_overlay,capture_mode,input wire [2:0] capture_notice,
 input wire [1:0] predicted_class,input wire foreground_valid,input wire [2:0] reject_reason,
 input wire store_busy,input wire [2:0] store_error,input wire [2:0] stored,
 input wire [15:0] sort_status,
 input wire background_valid,stable,last_error,track_valid,
 input wire [1:0] loaded,class_id,operation,input wire [10:0] cuboid_errors,wedge_errors,
 input wire [13:0] motion_cells,input wire [7:0] x0,x1,y0,y1,mx0,mx1,my0,my1,
 output reg out_vsync,out_hsync,out_de,output reg [23:0] out_rgb);
 reg vs_d;reg [10:0] x;reg [9:0] y;
 reg [207:0] display_bundle;
 wire [1:0] f_predicted;wire f_foreground;wire [2:0] f_reject;
 wire [2:0] f_notice;
 wire f_capture_mode,f_store_busy;wire [2:0] f_store_error;wire [2:0] f_stored;
 wire f_show;
 wire [15:0] f_sort;
 wire f_bg,f_stable,f_error,f_track;wire[1:0]f_loaded,f_class,f_operation;
 wire[10:0]f_ce,f_we;wire[13:0]f_cells;
 wire[7:0]f_x0,f_x1,f_y0,f_y1,f_mx0,f_mx1,f_my0,f_my1;
 wire[31:0]f_prediction,f_capture;
 assign {f_predicted,f_foreground,f_reject,f_notice,f_capture_mode,f_store_busy,f_store_error,f_stored,f_show,f_sort,f_bg,f_loaded,f_stable,f_class,f_operation,f_error,f_ce,f_we,f_cells,f_track,
  f_x0,f_x1,f_y0,f_y1,f_mx0,f_mx1,f_my0,f_my1,f_prediction,f_capture}=display_bundle;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)display_bundle<=0;
  else if(vsync && !vs_d)display_bundle<={predicted_class,foreground_valid,reject_reason,capture_notice,capture_mode,store_busy,store_error,stored,show_overlay,sort_status,background_valid,loaded,stable,class_id,operation,last_error,
   cuboid_errors,wedge_errors,motion_cells,track_valid,x0,x1,y0,y1,mx0,mx1,my0,my1,prediction_id,capture_id};

 reg [31:0] last_id,source_age;
 wire alive=source_age<CLK_HZ/2;
 wire [31:0] result_age=frame_id-f_prediction;
 wire fresh=result_age<=2 && alive;
 (* ram_style="bram" *) reg [7:0] footer[0:8191];
 initial $readmemh(FOOTER_FILE,footer);
 wire banner_x_inside=x>=384 && x<640;
 wire type_zone=banner_x_inside && y>=496 && y<528;
 wire state_zone=banner_x_inside && y>=536 && y<568;
 wire banner_zone=f_capture_mode?(banner_x_inside && y>=528 && y<560):(type_zone || state_zone);
 wire footer_zone=f_capture_mode?((y>=576 && y<592) || banner_zone):(f_show && banner_zone);
 wire [1:0] shown_type=(!f_bg || f_loaded!=3)?2'd3:f_stable?f_class:fresh?f_predicted:2'd0;
 reg [3:0] recognition_state;
 always @*begin
  if(f_sort[12:8]==22)recognition_state=9;
  else if(f_sort[12:8]==21)recognition_state=8;
  else if(f_sort[0] && f_sort[12:8]!=1)recognition_state=7;
  else if(f_store_busy)recognition_state=10;
  else if(!f_bg || f_loaded!=3)recognition_state=6;
  else if(!f_foreground)recognition_state=0;
  else if(f_stable && fresh)recognition_state=3;
  else if(f_reject==6)recognition_state=4;
  else if(f_reject==5)recognition_state=5;
  else if(f_predicted!=0 || f_sort[12:8]==1)recognition_state=2;
  else recognition_state=1;
 end
 wire [7:0] banner_x=x-384;
 wire [4:0] banner_y=f_capture_mode?y-528:type_zone?y-496:y-536;
 wire [4:0] banner_page=f_capture_mode?{2'b01,f_notice}:type_zone?{3'b100,shown_type}:(5'd20+recognition_state);
 wire [12:0] footer_addr=banner_zone?{banner_page,banner_y[4:1],banner_x[7:4]}:{2'b0,y[3:0],x[9:3]};
 wire [2:0] banner_ink=f_capture_mode?(f_notice==5?1:f_notice==6?2:f_notice==4?3:0):
   type_zone?(shown_type==0 || shown_type==3?3:0):recognition_state==3?1:recognition_state==9?2:
   recognition_state==4 || recognition_state==5?3:0;
 reg [7:0] footer_q;reg [2:0] footer_col;reg footer_d;
 reg [2:0] footer_ink_d,footer_ink_e;
 wire [23:0] footer_ink=footer_ink_e==1?24'h40ff60:footer_ink_e==2?24'hff6050:
   footer_ink_e==3?24'hffff40:footer_ink_e==4?24'hffff80:24'h40dfff;
 always @(posedge clk)footer_q<=footer[footer_addr];
 reg roi_d,target_d,motion_d;
 reg [23:0] rgb_d,rgb_e;reg vd,hd,dd,ve,he,de_e;
 reg roi_e,target_e,motion_e,footer_e;
 reg [7:0] footer_q2;reg [2:0] footer_col2;
 wire roi_border=(x==ROI_X || x==ROI_X+ROI_W-1) && y>=ROI_Y && y<ROI_Y+ROI_H ||
  (y==ROI_Y || y==ROI_Y+ROI_H-1) && x>=ROI_X && x<ROI_X+ROI_W;
 // Keep the expanded seven-pixel grid mapping in LUT add/subtract logic.
 function [10:0] x_step(input [10:0] i);
  begin x_step=(ROI_W/GW==7)?(i<<3)-i:i*(ROI_W/GW);end
 endfunction
 function [9:0] y_step(input [9:0] i);
  begin y_step=(ROI_H/GH==7)?(i<<3)-i:i*(ROI_H/GH);end
 endfunction
 wire [10:0] bx0=ROI_X+x_step({3'd0,f_x0}),bx1=ROI_X+x_step(f_x1+11'd1)-1'b1;
 wire [9:0] by0=ROI_Y+y_step({2'd0,f_y0}),by1=ROI_Y+y_step(f_y1+10'd1)-1'b1;
 wire [10:0] mbx0=ROI_X+x_step({3'd0,f_mx0}),mbx1=ROI_X+x_step(f_mx1+11'd1)-1'b1;
 wire [9:0] mby0=ROI_Y+y_step({2'd0,f_my0}),mby1=ROI_Y+y_step(f_my1+10'd1)-1'b1;
 wire target_border=fresh && f_bg && f_x1>f_x0 && f_y1>f_y0 &&
  (((x==bx0 || x==bx1) && y>=by0 && y<=by1) || ((y==by0 || y==by1) && x>=bx0 && x<=bx1));
 wire motion_border=alive && f_track &&
  (((x==mbx0 || x==mbx1) && y>=mby0 && y<=mby1) || ((y==mby0 || y==mby1) && x>=mbx0 && x<=mbx1));
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin
   vs_d<=0;x<=0;y<=0;last_id<=0;source_age<=CLK_HZ/2;
   footer_e<=0;footer_q2<=0;footer_col2<=0;roi_e<=0;target_e<=0;motion_e<=0;rgb_e<=0;ve<=0;he<=0;de_e<=0;
   footer_d<=0;footer_col<=0;roi_d<=0;target_d<=0;motion_d<=0;rgb_d<=0;vd<=0;hd<=0;dd<=0;
   footer_ink_d<=0;footer_ink_e<=0;
   out_vsync<=0;out_hsync<=0;out_de<=0;out_rgb<=0;
  end else begin
   vs_d<=vsync;
   if(source_age<CLK_HZ/2)source_age<=source_age+1'b1;
   if(frame_id!=last_id)begin source_age<=0;last_id<=frame_id;end
   if(vsync && !vs_d)begin x<=0;y<=0;end
   else if(de)begin if(x==WIDTH-1)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;end
   footer_d<=footer_zone;footer_col<=banner_zone?banner_x[3:1]:x[2:0];
   footer_ink_d<=banner_zone?banner_ink:3'd4;footer_ink_e<=footer_ink_d;
   roi_d<=f_show && roi_border;target_d<=f_show && target_border;motion_d<=f_show && motion_border;
   rgb_d<=rgb;vd<=vsync;hd<=hsync;dd<=de;
   rgb_e<=rgb_d;ve<=vd;he<=hd;de_e<=dd;
   roi_e<=roi_d;target_e<=target_d;motion_e<=motion_d;
   footer_e<=footer_d;footer_q2<=footer_q;footer_col2<=footer_col;
   out_vsync<=ve;out_hsync<=he;out_de<=de_e;
   out_rgb<=!de_e?0:(footer_e && footer_q2[7-footer_col2])?footer_ink:
     motion_e?24'hff4040:target_e?(f_stable?24'h40ff60:24'hffb040):roi_e?24'h40dfff:rgb_e;
  end
endmodule
