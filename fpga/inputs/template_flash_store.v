`timescale 1ns/1ps
// Independent user-Flash records: two 128-byte silhouettes and one RGB565 background.
// Payload is written and verified before its record is reported saved.
module template_flash_store #(
 parameter CLK_HZ=52002000, POWER_CYCLES=CLK_HZ/200, SPI_HALF=8
)(
 input wire clk,rst_n,cancel,
 input wire capture_begin,capture_we,capture_bit,
 input wire [9:0] capture_addr,
 input wire save_request,input wire [1:0] save_class,
 output wire background_active,output reg background_load_begin,background_load_we,background_load_done,
 output reg [13:0] background_addr,output reg [15:0] background_pixel,input wire [15:0] background_q,
 output reg load_begin,load_class,load_we,load_bit,load_commit,
 output reg [9:0] load_addr,
 output wire busy,output reg present,output reg [2:0] saved,
 output reg [2:0] error,
 output reg flash_cs_n,output wire flash_clk,flash_mosi,input wire flash_miso
);
 localparam POWER=0,ID_START=1,ID_READ=2,ID_BYTE=3,LOAD_START=4,
 LOAD_READ=5,LOAD_BYTE=6,LOAD_BITS=7,LOAD_COMMIT=8,LOAD_NEXT=9,IDLE=10,
 WREN_ERASE=11,ERASE_GAP=12,ERASE_CMD=13,ERASE_SENT=14,
 POLL_CMD=15,POLL_READ=16,POLL_BYTE=17,
 WREN_DATA=18,DATA_GAP=19,DATA_CMD=20,DATA_PREP=21,DATA_SEND=22,DATA_BYTE=23,
 WREN_TAG=24,TAG_GAP=25,TAG_CMD=26,TAG_SEND=27,TAG_BYTE=28,VERIFY=29,
 CMD_SEND=30,CMD_BYTE=31,WAIT_BYTE=32,GAP=33,FAIL=34;
 reg [5:0] state,byte_return,cmd_return,gap_return;
 reg [17:0] power_count;
 reg [1:0] slot;reg verify_read,header_ok;
 reg [2:0] erase_index;reg [7:0] background_high;
 reg [1:0] phase,cmd_index,tag_index,gap_count;
 reg [2:0] cmd_length,bit_index;
 reg [14:0] index;reg [7:0] read_byte,pack;
 reg [31:0] cmd_word;
 (* ram_style="bram" *) reg [7:0] buffer[0:127];
 reg [6:0] buffer_addr;
 reg [7:0] buffer_q;
 reg byte_start;reg [7:0] byte_tx;wire byte_done;wire [7:0] byte_rx;
 (* async_reg="true" *) reg miso_sync0,miso_sync1;
 wire [23:0] sector={10'd0,slot,12'd0};
 wire [14:0] payload_start=slot==2?15'd256:15'd16;
 wire [14:0] payload_last=slot==2?15'd19455:15'd143;
 wire [14:0] save_last=slot==2?15'd19199:15'd127;
 wire [7:0] expected_byte=slot==2?(index[0]?background_q[7:0]:background_q[15:8]):buffer_q;
 assign background_active=busy && slot==2;
 assign busy=state!=IDLE;
 template_spi_byte #(.HALF(SPI_HALF)) u_byte(
  .clk(clk),.rst_n(rst_n),.start(byte_start),.tx(byte_tx),.done(byte_done),.rx(byte_rx),
  .sclk(flash_clk),.mosi(flash_mosi),.miso(miso_sync1));
 function [7:0] tag(input [1:0] n,input [1:0] class_id);
  begin case(n)0:tag="T";1:tag="M";2:tag=8'h01;3:tag={6'd0,class_id};endcase end
 endfunction
 task send_byte(input [7:0] value,input [5:0] next_state);
  begin byte_tx<=value;byte_start<=1;byte_return<=next_state;state<=WAIT_BYTE;end
 endtask
 task transaction(input [7:0] code,input [23:0] address,input addressed,input [5:0] next_state);
  begin flash_cs_n<=0;cmd_word<={code,address};cmd_index<=0;cmd_length<=addressed?4:1;
   cmd_return<=next_state;state<=CMD_SEND;end
 endtask
 task finish_transaction(input [5:0] next_state);
  begin flash_cs_n<=1;gap_count<=0;gap_return<=next_state;state<=GAP;end
 endtask
 always @(posedge clk)begin
  buffer_q<=buffer[buffer_addr];
  if(capture_we && capture_addr[2:0]==7)buffer[capture_addr[9:3]]<={pack[6:0],capture_bit};
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=POWER;power_count<=0;slot<=0;verify_read<=0;header_ok<=1;
   phase<=0;cmd_index<=0;tag_index<=0;gap_count<=0;cmd_length<=0;bit_index<=0;
   index<=0;read_byte<=0;pack<=0;cmd_word<=0;buffer_addr<=0;
   byte_start<=0;byte_tx<=8'hff;byte_return<=IDLE;cmd_return<=IDLE;gap_return<=IDLE;
   load_begin<=0;load_class<=0;load_we<=0;load_bit<=0;load_commit<=0;load_addr<=0;
   background_load_begin<=0;background_load_we<=0;background_load_done<=0;background_addr<=0;background_pixel<=0;background_high<=0;erase_index<=0;
   present<=0;saved<=0;error<=0;flash_cs_n<=1;miso_sync0<=1;miso_sync1<=1;
  end else begin
   miso_sync0<=flash_miso;miso_sync1<=miso_sync0;
   byte_start<=0;load_begin<=0;load_we<=0;load_commit<=0;
   background_load_begin<=0;background_load_we<=0;background_load_done<=0;
   if(capture_begin)pack<=0;
   if(capture_we)pack<={pack[6:0],capture_bit};
   case(state)
    POWER:if(power_count==POWER_CYCLES-1)state<=ID_START;else power_count<=power_count+1'b1;
    ID_START:begin index<=0;transaction(8'h9f,0,0,ID_READ);end
    ID_READ:send_byte(8'hff,ID_BYTE);
    ID_BYTE:begin
     if(index==0)present<=byte_rx!=0 && byte_rx!=8'hff;
     if(index==2)finish_transaction(LOAD_START);else begin index<=index+1'b1;state<=ID_READ;end
    end
    LOAD_START:if(!present)begin error<=1;state<=IDLE;end else begin
     index<=0;header_ok<=1;load_class<=slot[0];verify_read<=0;transaction(8'h03,sector,1,LOAD_READ);
    end
    LOAD_READ:send_byte(8'hff,LOAD_BYTE);
    LOAD_BYTE:begin
     if(index<4)begin
      if(byte_rx!=tag(index[1:0],slot))header_ok<=0;
      if(index==3 && (!header_ok || byte_rx!=tag(3,slot)))begin
       if(verify_read)begin error<=2;finish_transaction(IDLE);end
       else finish_transaction(LOAD_NEXT);
      end else begin
       if(index==3 && !verify_read)begin
        if(slot==2)begin background_load_begin<=1;background_addr<=0;end
        else begin load_begin<=1;load_addr<=0;end
       end
       index<=index+1'b1;state<=LOAD_READ;
      end
     end else if(index<payload_start)begin index<=index+1'b1;state<=LOAD_READ;end
     else if(verify_read)begin
      if(byte_rx!=expected_byte)begin error<=2;finish_transaction(IDLE);end
      else if(index==payload_last)begin saved[slot]<=1;finish_transaction(IDLE);end
      else begin
       index<=index+1'b1;
       if(slot==2)background_addr<=(index+15'd1-15'd256)>>1;
       else buffer_addr<=buffer_addr+1'b1;
       state<=LOAD_READ;
      end
     end else if(slot==2)begin
      if(!index[0])begin background_high<=byte_rx;index<=index+1'b1;state<=LOAD_READ;end
      else begin
       background_pixel<={background_high,byte_rx};background_load_we<=1;
       if(index==payload_last)state<=LOAD_COMMIT;else begin index<=index+1'b1;state<=LOAD_READ;end
      end
     end else begin read_byte<=byte_rx;bit_index<=0;state<=LOAD_BITS;end
    end
    LOAD_BITS:begin
     load_we<=1;load_bit<=read_byte[7-bit_index];
     if(bit_index==7)begin
      if(index==payload_last)state<=LOAD_COMMIT;
      else begin index<=index+1'b1;state<=LOAD_READ;end
     end else bit_index<=bit_index+1'b1;
     // The registered address advances after the previous write is consumed.
     if(load_we)load_addr<=load_addr+1'b1;
    end
    LOAD_COMMIT:begin if(slot==2)background_load_done<=1;else load_commit<=1;saved[slot]<=1;finish_transaction(LOAD_NEXT);end
    LOAD_NEXT:if(slot!=2)begin slot<=slot+1'b1;state<=LOAD_START;end else state<=IDLE;
    IDLE:if(save_request)begin
     slot<=save_class;erase_index<=0;error<=0;
     if(present)begin saved[save_class]<=0;state<=WREN_ERASE;end else error<=1;
    end
    WREN_ERASE:transaction(8'h06,0,0,ERASE_GAP);
    ERASE_GAP:finish_transaction(ERASE_CMD);
    ERASE_CMD:begin phase<=0;transaction(8'h20,sector+{9'd0,erase_index,12'd0},1,ERASE_SENT);end
    ERASE_SENT:finish_transaction(POLL_CMD);
    POLL_CMD:transaction(8'h05,0,0,POLL_READ);
    POLL_READ:send_byte(8'hff,POLL_BYTE);
    POLL_BYTE:if(byte_rx==8'hff)begin error<=1;finish_transaction(IDLE);end
     else if(byte_rx[0])finish_transaction(POLL_CMD);
     else if(phase==0)begin
      if(slot==2 && erase_index<4)begin erase_index<=erase_index+1'b1;finish_transaction(WREN_ERASE);end
      else begin index<=0;finish_transaction(WREN_DATA);end
     end else if(phase==1)begin
      if(index<save_last)begin index<=index+1'b1;finish_transaction(WREN_DATA);end
      else finish_transaction(WREN_TAG);
     end else finish_transaction(VERIFY);
    WREN_DATA:transaction(8'h06,0,0,DATA_GAP);
    DATA_GAP:finish_transaction(DATA_CMD);
    DATA_CMD:begin phase<=1;buffer_addr<=0;background_addr<=index[14:1];transaction(8'h02,sector+payload_start+index,1,DATA_PREP);end
    DATA_PREP:state<=DATA_SEND;
    DATA_SEND:send_byte(slot==2?(index[0]?background_q[7:0]:background_q[15:8]):buffer_q,DATA_BYTE);
    DATA_BYTE:if(index[6:0]==127)finish_transaction(POLL_CMD);
     else begin index<=index+1'b1;buffer_addr<=buffer_addr+1'b1;background_addr<=(index+15'd1)>>1;state<=DATA_PREP;end
    WREN_TAG:transaction(8'h06,0,0,TAG_GAP);
    TAG_GAP:finish_transaction(TAG_CMD);
    TAG_CMD:begin phase<=2;tag_index<=0;transaction(8'h02,sector,1,TAG_SEND);end
    TAG_SEND:send_byte(tag(tag_index,slot),TAG_BYTE);
    TAG_BYTE:if(tag_index==3)finish_transaction(POLL_CMD);else begin tag_index<=tag_index+1'b1;state<=TAG_SEND;end
    VERIFY:begin verify_read<=1;index<=0;buffer_addr<=0;background_addr<=0;header_ok<=1;transaction(8'h03,sector,1,LOAD_READ);end
    CMD_SEND:send_byte(cmd_word[31:24],CMD_BYTE);
    CMD_BYTE:if(cmd_index==cmd_length-1)state<=cmd_return;
     else begin cmd_index<=cmd_index+1'b1;cmd_word<=cmd_word<<8;state<=CMD_SEND;end
    WAIT_BYTE:if(byte_done)state<=byte_return;
    GAP:if(gap_count==3)state<=gap_return;else gap_count<=gap_count+1'b1;
   endcase
   if(background_load_we)background_addr<=background_addr+1'b1;
   if(load_we && state!=LOAD_BITS)load_addr<=load_addr+1'b1;
   if(cancel && state!=IDLE)begin flash_cs_n<=1;state<=IDLE;error<=3;load_we<=0;load_commit<=0;background_load_we<=0;background_load_done<=0;end
  end
 end
endmodule

module template_spi_byte #(parameter HALF=8)(
 input wire clk,rst_n,start,input wire [7:0] tx,
 output reg done,output reg [7:0] rx,output reg sclk,mosi,input wire miso
);
 reg busy;reg [2:0] divider,bit_index;reg [7:0] shift,tx_latched;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)begin busy<=0;done<=0;rx<=0;sclk<=0;mosi<=1;divider<=0;bit_index<=0;shift<=0;tx_latched<=0;end
  else begin
   done<=0;
   if(!busy)begin
    sclk<=0;
    if(start)begin busy<=1;divider<=0;bit_index<=7;shift<=0;tx_latched<=tx;mosi<=tx[7];end
   end else if(divider==HALF-1)begin
    divider<=0;sclk<=~sclk;
    if(!sclk)shift<={shift[6:0],miso};
    else if(bit_index==0)begin busy<=0;done<=1;rx<=shift;mosi<=1;end
    else begin bit_index<=bit_index-1'b1;mosi<=tx_latched[bit_index-1'b1];end
   end else divider<=divider+1'b1;
  end
endmodule
