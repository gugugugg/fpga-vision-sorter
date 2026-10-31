`timescale 1ns/1ps
module tb_templates;
reg clk=0,rst=0,cancel=0,tb=0,tc=0,tw=0,tbit=0,commit=0,pb=0,pw=0,pbit=0,start=0;
reg[9:0] ta=0,pa=0;reg[31:0] fid=0;
wire ready,busy,done,error;wire[1:0] loaded,result_class;
wire[10:0] ce,we;wire[31:0] rid;wire[3:0] reason;
reg cm[0:1023],wm[0:1023],pm[0:1023];
always #5 clk=~clk;
part_binary_template dut(.clk(clk),.rst_n(rst),.cancel(cancel),.clear_templates(1'b0),.teach_begin(tb),.teach_class(tc),
 .teach_we(tw),.teach_bit(tbit),.teach_commit(commit),.teach_addr(ta),
 .patch_begin(pb),.patch_we(pw),.patch_bit(pbit),.start(start),.patch_addr(pa),.frame_id(fid),
 .ready(ready),.busy(busy),.done(done),.error(error),.templates_loaded(loaded),
 .result_class(result_class),.cuboid_errors(ce),.wedge_errors(we),.result_frame_id(rid),.reject_reason(reason));
task teach(input integer slot);
begin
 @(negedge clk);tc=slot;tb=1;@(negedge clk);tb=0;
 for(integer k=0;k<1024;k=k+1)begin tw=1;ta=k;tbit=slot?wm[k]:cm[k];@(negedge clk);end
 tw=0;commit=1;@(negedge clk);commit=0;
end endtask
task load_patch(input integer n);
reg[255:0] filename;
begin
 case(n)
 0:$readmemh("patch0.hex",pm);1:$readmemh("patch1.hex",pm);2:$readmemh("patch2.hex",pm);
 3:$readmemh("patch3.hex",pm);4:$readmemh("patch4.hex",pm);5:$readmemh("patch5.hex",pm);
 endcase
 @(negedge clk);fid=n+10;pb=1;@(negedge clk);pb=0;
 for(integer k=0;k<1024;k=k+1)begin pw=1;pa=k;pbit=pm[k];@(negedge clk);end
 pw=0;start=1;@(negedge clk);start=0;
end endtask
initial begin
 $readmemh("cube.hex",cm);$readmemh("wedge.hex",wm);
 repeat(3)@(negedge clk);rst=1;
 // Unloaded ROMs must not fabricate a classification.
 load_patch(0);if(!done || result_class!=0 || reason!=3)$fatal(1,"unloaded template accepted");
 teach(0);teach(1);if(loaded!=3)$fatal(1,"template teach not committed");
load_patch(0); wait(done); #1;
if(result_class!=1 || ce!=0 || we!=496 || reason!=0 || rid!=10)
 $fatal(1,"case0 class=%0d cube=%0d wedge=%0d reason=%0d id=%0d",result_class,ce,we,reason,rid);
@(negedge clk);
load_patch(1); wait(done); #1;
if(result_class!=2 || ce!=496 || we!=0 || reason!=0 || rid!=11)
 $fatal(1,"case1 class=%0d cube=%0d wedge=%0d reason=%0d id=%0d",result_class,ce,we,reason,rid);
@(negedge clk);
load_patch(2); wait(done); #1;
if(result_class!=0 || ce!=1024 || we!=1024 || reason!=4 || rid!=12)
 $fatal(1,"case2 class=%0d cube=%0d wedge=%0d reason=%0d id=%0d",result_class,ce,we,reason,rid);
@(negedge clk);
load_patch(3); wait(done); #1;
if(result_class!=1 || ce!=70 || we!=498 || reason!=0 || rid!=13)
 $fatal(1,"case3 class=%0d cube=%0d wedge=%0d reason=%0d id=%0d",result_class,ce,we,reason,rid);
@(negedge clk);
load_patch(4); wait(done); #1;
if(result_class!=2 || ce!=498 || we!=70 || reason!=0 || rid!=14)
 $fatal(1,"case4 class=%0d cube=%0d wedge=%0d reason=%0d id=%0d",result_class,ce,we,reason,rid);
@(negedge clk);
load_patch(5); wait(done); #1;
if(result_class!=2 || ce!=576 || we!=512 || reason!=0 || rid!=15)
 $fatal(1,"case5 class=%0d cube=%0d wedge=%0d reason=%0d id=%0d",result_class,ce,we,reason,rid);
@(negedge clk);
 // Reject partial or reordered patches.
 @(negedge clk);pb=1;@(negedge clk);pb=0;pw=1;pa=8;pbit=1;
 @(negedge clk);pw=0;start=1;@(negedge clk);start=0;
 if(!done || !error || reason!=2 || result_class!=0)$fatal(1,"partial/reordered patch accepted");
 // Cancellation cannot leak an old inference result.
 load_patch(0);cancel=1;@(negedge clk);cancel=0;
 repeat(2200)begin @(negedge clk);if(done || busy || result_class!=0)$fatal(1,"cancel leaked result");end
 // Similar templates still select the smaller distance.
 for(integer k=0;k<1024;k=k+1)wm[k]=k>=32;
 teach(1);load_patch(0);wait(done);#1;
 if(result_class!=1 || ce!=0 || we!=32 || reason!=0)$fatal(1,"nearest similar template not selected");
 $display("PASS templates: exact/noisy/large-distance nearest classes, empty, untrained, malformed, cancel, close templates");$finish;
end
initial begin #2000000;$fatal(1,"timeout");end
endmodule
