`timescale 1ns/1ps
// Calibrated orthographic yaw visual hull, six explicit views. Q8 signed
// coefficients project centred XY; Z maps to mask row. Masks contain filled
// silhouettes, not just boundary pixels. Default angles: 0,60,120,180,240,300.
// N=32 default; B=6 supports 64^3 with unchanged interfaces. One view read per
// cycle; one voxel is committed after all views. Single synchronous mask RAM.
module qc_voxel_carver #(parameter B=5, VIEWS=6)(
 input wire clk,rst_n,mask_we,input wire [2:0] mask_view,
 input wire [2*B-1:0] mask_addr,input wire mask_value,
 input wire coeff_we,input wire [2:0] coeff_view,
 input wire signed [9:0] coeff_cos,coeff_sin,
 input wire reference_we,reference_commit,input wire [3*B-1:0] reference_addr,
 input wire reference_value,input wire start,
 input wire reference_capture,output wire reference_valid,
 output reg busy,done,output reg [3*B:0] volume,missing,excess,
 output reg [B:0] size_x,size_y,size_z,
 output reg [B-1:0] min_x,min_y,min_z,max_x,max_y,max_z,
 output reg [5:0] view_support,output reg [2:0] suggested_view,
 output reg reobserve,defect,
 input wire [3*B-1:0] read_addr,output reg occupied,
 input wire [2*B-1:0] projection_addr,output reg projection
);
 localparam N=1<<B,P=1<<(2*B),TOTAL=1<<(3*B);
 reg masks[0:VIEWS*P-1];reg voxels[0:TOTAL-1];reg reference[0:TOTAL-1];
 reg front[0:P-1];reg ref_valid,have_hull;
 reg [3*B-1:0] teach_addr;reg reference_q;
 assign reference_valid=ref_valid;
 reg signed [9:0] cosines[0:VIEWS-1],sines[0:VIEWS-1];
 reg [3*B-1:0] voxel;reg [2:0] view;reg keep,mask_q,in_range_q;
 reg [2*B-1:0] clear_addr;reg [3:0] state;
 reg [3*B:0] rejected[0:VIEWS-1];
 reg [3*B:0] smallest_reject;reg [2:0] search_view;
 wire [B-1:0] vx=voxel[B-1:0],vy=voxel[2*B-1:B],vz=voxel[3*B-1:2*B];
 wire signed [B+1:0] xx=$signed({1'b0,vx,1'b0})-(N-1);
 wire signed [B+1:0] yy=$signed({1'b0,vy,1'b0})-(N-1);
 wire signed [B+12:0] projected=xx*cosines[view]+yy*sines[view];
 wire signed [B+12:0] u=(projected>>>9)+(N/2);
 wire in_bounds=(u>=0)&&(u<N);
 wire ref_box=(vx>=N/4 && vx<3*N/4 && vy>=N/4 && vy<3*N/4 && vz>=N/4 && vz<3*N/4);
 wire expected=ref_valid?reference_q:ref_box;
 wire [3*B-1:0] voxel_read_addr=(state==8 || state==9)?teach_addr:read_addr;
 wire reference_write=(!busy && reference_we)||(state==9);
 wire [3*B-1:0] reference_write_addr=state==9?teach_addr:reference_addr;
 wire reference_write_value=state==9?occupied:reference_value;
 integer k;
 always @(posedge clk) begin
  occupied<=voxels[voxel_read_addr];projection<=front[projection_addr];
  reference_q<=reference[voxel];
  if(rst_n) begin
   if(state==4) voxels[voxel]<=keep;
   if(reference_write) reference[reference_write_addr]<=reference_write_value;
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   busy<=0;done<=0;state<=0;ref_valid<=0;have_hull<=0;teach_addr<=0;
   voxel<=0;view<=0;clear_addr<=0;
   volume<=0;missing<=0;excess<=0;size_x<=0;size_y<=0;size_z<=0;
   min_x<=N-1;min_y<=N-1;min_z<=N-1;max_x<=0;max_y<=0;max_z<=0;
   defect<=0;reobserve<=0;suggested_view<=0;view_support<=0;
   keep<=1;mask_q<=0;in_range_q<=0;search_view<=0;smallest_reject<=0;
   for(k=0;k<VIEWS;k=k+1) rejected[k]<=0;
   cosines[0]<=256;sines[0]<=0;cosines[1]<=128;sines[1]<=222;
   cosines[2]<=-128;sines[2]<=222;cosines[3]<=-256;sines[3]<=0;
   cosines[4]<=-128;sines[4]<=-222;cosines[5]<=128;sines[5]<=-222;
  end else begin
   done<=0;
   if(!busy) begin
    if(mask_we && mask_view<VIEWS) masks[mask_view*P+mask_addr]<=mask_value;
    if(coeff_we && coeff_view<VIEWS) begin cosines[coeff_view]<=coeff_cos;sines[coeff_view]<=coeff_sin;end
    if(reference_commit) ref_valid<=1;
   end
   case(state)
    0:if(reference_capture && have_hull) begin
      busy<=1;ref_valid<=0;teach_addr<=0;state<=8;
    end else if(start) begin
      have_hull<=0;
      busy<=1;clear_addr<=0;voxel<=0;view<=0;keep<=1;state<=1;
      volume<=0;missing<=0;excess<=0;defect<=0;view_support<=0;
      min_x<=N-1;min_y<=N-1;min_z<=N-1;max_x<=0;max_y<=0;max_z<=0;
      for(k=0;k<VIEWS;k=k+1) rejected[k]<=0;
    end
    1:begin
      front[clear_addr]<=0;
      if(clear_addr==P-1) state<=2;else clear_addr<=clear_addr+1'b1;
    end
    2:begin
      mask_q<=masks[view*P+{vz,u[B-1:0]}];in_range_q<=in_bounds;state<=3;
    end
    3:begin
      keep<=keep && in_range_q && mask_q;
      if(!in_range_q || !mask_q) rejected[view]<=rejected[view]+1'b1;
      if(view==VIEWS-1) state<=4;else begin view<=view+1'b1;state<=2;end
    end
    4:begin
      if(keep) begin
        volume<=volume+1'b1;front[{vz,vx}]<=1;
        if(vx<min_x)min_x<=vx;if(vx>max_x)max_x<=vx;
        if(vy<min_y)min_y<=vy;if(vy>max_y)max_y<=vy;
        if(vz<min_z)min_z<=vz;if(vz>max_z)max_z<=vz;
      end
      if(expected && !keep) missing<=missing+1'b1;
      if(!expected && keep) excess<=excess+1'b1;
      if(voxel==TOTAL-1) state<=5;
      else begin voxel<=voxel+1'b1;view<=0;keep<=1;state<=2;end
    end
    5:begin
      size_x<=volume?{1'b0,max_x}-{1'b0,min_x}+1'b1:0;
      size_y<=volume?{1'b0,max_y}-{1'b0,min_y}+1'b1:0;
      size_z<=volume?{1'b0,max_z}-{1'b0,min_z}+1'b1:0;
      // Demo reference tolerance: 4 voxels. Counts remain available for a
      // calibrated application-specific tolerance at integration time.
      defect<=(missing>4)||(excess>4);
      search_view<=0;smallest_reject<=TOTAL;suggested_view<=0;state<=6;
    end
    6:begin
      view_support[search_view]<=rejected[search_view]!=0;
      if(rejected[search_view]<smallest_reject) begin
        smallest_reject<=rejected[search_view];suggested_view<=search_view;
      end
      if(search_view==VIEWS-1) state<=7;else search_view<=search_view+1'b1;
    end
    7:begin
      // An empty intersection or a view that excludes no voxel needs another
      // observation. This is coverage confidence, not a probability claim.
      reobserve<=!(&view_support)||(volume==0);done<=1;busy<=0;have_hull<=1;state<=0;
    end
    // Explicit teaching of a completed hull. One synchronous voxel read and
    // reference write per two clocks; neither memory has a reset or async read.
    8:state<=9;
    9:if(teach_addr==TOTAL-1) begin ref_valid<=1;busy<=0;state<=0;end
      else begin teach_addr<=teach_addr+1'b1;state<=8;end
   endcase
  end
 end
endmodule
