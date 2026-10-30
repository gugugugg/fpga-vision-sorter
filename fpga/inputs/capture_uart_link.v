`timescale 1ns/1ps
// Byte mailbox: acknowledgement is delayed until the UART byte has actually
// finished. TX uses a fractional divider (exact average 3 Mbaud at 50 MHz).
module capture_uart_link #(
    parameter CLK_HZ=50000000, parameter BAUD=3000000,
    parameter DEBOUNCE_CYCLES=1000000
)(
    input wire i_rst_n,i_sys_clk,i_ddr_clk,i_key_n,i_rx,
    input wire i_command_active,
    output reg o_command_byte_valid,output reg [7:0] o_command_byte,
    output wire o_tx, output reg o_capture,output reg o_resend,
    input wire i_byte_valid,input wire [7:0] i_byte,
    output reg o_view_valid,output reg [2:0] o_view_id,
    output reg o_teach_2d,o_clear_2d,
    output wire o_byte_ready
);
    reg req,ack;
    wire sys_rst_n,ddr_rst_n;
    reset_release sys_reset(i_sys_clk,i_rst_n,sys_rst_n);
    reset_release ddr_reset(i_ddr_clk,i_rst_n,ddr_rst_n);
    reg [7:0] hold;
    (* async_reg="true",syn_preserve=1 *) reg ack0,ack1,req0,req1;
    reg [7:0] hold0,hold1;
    reg [9:0] tx_shift;
    reg [3:0] tx_bits;
    reg [25:0] tx_phase;
    reg tx_busy;
    assign o_tx=tx_busy?tx_shift[0]:1'b1;
    assign o_byte_ready=req==ack1;
    always @(posedge i_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin req<=0;hold<=0;ack0<=0;ack1<=0;end
        else begin
            ack0<=ack;ack1<=ack0;
            if(i_byte_valid && o_byte_ready) begin hold<=i_byte;req<=~req;end
        end
    end
    (* async_reg="true",syn_preserve=1 *) reg [1:0] key_sync,rx_sync;
    reg key_state;
    reg [19:0] debounce;
    reg key_toggle,resend_toggle;
    localparam RX_DIV=(CLK_HZ+BAUD/2)/BAUD;
    reg [9:0] rx_count;
    reg [3:0] rx_bit;
    reg [7:0] rx_data;
    reg [1:0] rx_state;
    always @(posedge i_sys_clk or negedge sys_rst_n) begin
        if(!sys_rst_n) begin
            req0<=0;req1<=0;hold0<=0;hold1<=0;ack<=0;
            tx_shift<=10'h3ff;tx_bits<=0;tx_phase<=0;tx_busy<=0;
            key_sync<=3;key_state<=1;debounce<=0;key_toggle<=0;
            rx_sync<=3;rx_state<=0;rx_count<=0;rx_bit<=0;rx_data<=0;
            resend_toggle<=0;o_view_valid<=0;o_view_id<=0;o_command_byte_valid<=0;o_command_byte<=0;o_teach_2d<=0;o_clear_2d<=0;
        end else begin
            o_view_valid<=0;o_command_byte_valid<=0;o_teach_2d<=0;o_clear_2d<=0;
            req0<=req;req1<=req0;hold0<=hold;hold1<=hold0;
            key_sync<={key_sync[0],i_key_n};
            rx_sync<={rx_sync[0],i_rx};
            if(key_sync[1]==key_state) debounce<=0;
            else if(debounce==DEBOUNCE_CYCLES-1) begin
                key_state<=key_sync[1];debounce<=0;
                if(!key_sync[1]) key_toggle<=~key_toggle;
            end else debounce<=debounce+1'b1;
            if(!tx_busy && req1!=ack) begin
                tx_shift<={1'b1,hold1,1'b0};tx_bits<=0;tx_phase<=0;tx_busy<=1;
            end else if(tx_busy) begin
                if(tx_phase>=CLK_HZ-BAUD) begin
                    tx_phase<=tx_phase-(CLK_HZ-BAUD);
                    tx_shift<={1'b1,tx_shift[9:1]};tx_bits<=tx_bits+1'b1;
                    if(tx_bits==9) begin tx_busy<=0;ack<=req1;end
                end else tx_phase<=tx_phase+BAUD;
            end
            case(rx_state)
                0:if(!rx_sync[1]) begin rx_count<=RX_DIV/2;rx_state<=1;end
                1:if(rx_count==0) begin
                    if(!rx_sync[1]) begin rx_count<=RX_DIV-1;rx_bit<=0;rx_state<=2;end
                    else rx_state<=0;
                end else rx_count<=rx_count-1'b1;
                2:if(rx_count==0) begin
                    rx_data[rx_bit]<=rx_sync[1];rx_count<=RX_DIV-1;
                    if(rx_bit==7) rx_state<=3;else rx_bit<=rx_bit+1'b1;
                end else rx_count<=rx_count-1'b1;
                3:if(rx_count==0) begin
                    if(rx_sync[1]) begin o_command_byte_valid<=1;o_command_byte<=rx_data;end
                    if(rx_sync[1] && i_command_active!==1'b1 && rx_data==8'h52) resend_toggle<=~resend_toggle;
                    // A0..A5: stable view, A6: restart, A7: teach completed hull.
                    if(rx_sync[1] && i_command_active!==1'b1 && rx_data>=8'ha0 && rx_data<=8'ha7) begin
                        o_view_valid<=1;o_view_id<=rx_data[2:0];
                    end
                    if(rx_sync[1] && i_command_active!==1'b1 && rx_data==8'ha8) o_teach_2d<=1;
                    if(rx_sync[1] && i_command_active!==1'b1 && rx_data==8'ha9) o_clear_2d<=1;
                    rx_state<=0;
                end else rx_count<=rx_count-1'b1;
            endcase
        end
    end
    (* async_reg="true",syn_preserve=1 *) reg [2:0] key_cdc,resend_cdc;
    always @(posedge i_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin key_cdc<=0;resend_cdc<=0;o_capture<=0;o_resend<=0;end
        else begin
            key_cdc<={key_cdc[1:0],key_toggle};
            resend_cdc<={resend_cdc[1:0],resend_toggle};
            o_capture<=key_cdc[2]^key_cdc[1];
            o_resend<=resend_cdc[2]^resend_cdc[1];
        end
    end
endmodule
