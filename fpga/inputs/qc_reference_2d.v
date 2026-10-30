`timescale 1ns/1ps
// Teach completed silhouette descriptors, then check topology, area and size.
// Area tolerance is 12%, extent tolerance two sampled bins. Values are bins,
// not millimetres. No quality verdict is valid before a reference is taught.
module qc_reference_2d(
 input wire clk,rst_n,teach,clear,published,
 input wire [10:0] components,holes,area,
 input wire [5:0] width,height,
 output reg reference_valid,output reg defect
);
 reg [10:0] ref_components,ref_holes,ref_area;
 reg [5:0] ref_width,ref_height;
 wire [10:0] area_delta=area>=ref_area?area-ref_area:ref_area-area;
 wire [5:0] width_delta=width>=ref_width?width-ref_width:ref_width-width;
 wire [5:0] height_delta=height>=ref_height?height-ref_height:ref_height-height;
 wire [17:0] allowed_area=ref_area*18'd12/100;
 always @(posedge clk or negedge rst_n)
  if(!rst_n) begin reference_valid<=0;defect<=0;ref_components<=0;ref_holes<=0;
   ref_area<=0;ref_width<=0;ref_height<=0;end
  else if(clear) begin reference_valid<=0;defect<=0;end
  else if(teach && components!=0 && area!=0) begin
   ref_components<=components;ref_holes<=holes;ref_area<=area;
   ref_width<=width;ref_height<=height;reference_valid<=1;defect<=0;
  end else if(published) defect<=reference_valid && (components!=ref_components ||
   holes!=ref_holes || area_delta>allowed_area || width_delta>2 || height_delta>2);
endmodule
