`timescale 1ns/1ps

// Four active-low board buttons: next mode, previous mode, threshold up/down.
module vision_mode_ctrl #(
    parameter CLK_HZ      = 50000000,
    parameter DEBOUNCE_MS = 20,
    parameter THRESH_MIN  = 16,
    parameter THRESH_MAX  = 255,
    parameter THRESH_STEP = 16,
    parameter MAX_MODE    = 63,
    parameter ENABLE_REMOTE = 0,
    parameter ENABLE_MENU = 0,
    parameter [63:0] MODE_BITMAP = 64'hffffffffffffffff
)(
    input  wire       i_clk,
    input  wire       i_rst_n,
    input  wire [3:0] i_key_n,
    input wire i_menu_enabled,
    input wire i_capture_mode,output wire [3:0] o_key_press,
    output reg o_menu_open,
    output reg [5:0] o_menu_cursor,o_menu_first,
    input wire i_remote_mode_valid,
    input wire [5:0] i_remote_mode,
    input wire i_remote_threshold_valid,
    input wire [9:0] i_remote_threshold,
    output reg o_remote_rejected,
    output reg  [5:0] o_mode,
    output reg  [9:0] o_edge_threshold
);
    localparam COUNT_MAX = (CLK_HZ/1000)*DEBOUNCE_MS;
    (* async_reg="true",syn_preserve=1 *) reg [3:0] key_sync0,key_sync1;
    reg [3:0] key_state;
    reg [3:0] key_state_d;
    reg [31:0] debounce_count;
    wire [3:0] key_press = key_state_d & ~key_state;
    assign o_key_press=key_press;
    (* async_reg="true",syn_preserve=1 *) reg menu_sync0,menu_sync1;
    reg menu_enabled,menu_armed;
    reg [31:0] menu_debounce;
    function integer mode_count;
        input [63:0] mask;
        integer j;
        begin mode_count=0;for(j=0;j<=MAX_MODE;j=j+1) mode_count=mode_count+mask[j];end
    endfunction
    localparam MENU_COUNT=mode_count(MODE_BITMAP);
    localparam MENU_LAST_FIRST=MENU_COUNT>7?MENU_COUNT-7:0;
    function [5:0] ordinal;
        input [5:0] id;
        integer j,n;
        begin n=0;for(j=0;j<=MAX_MODE;j=j+1) if(j<id && MODE_BITMAP[j]) n=n+1;ordinal=n;end
    endfunction
    function [5:0] mode_at;
        input [5:0] index;
        integer j,n;
        begin mode_at=0;n=0;for(j=0;j<=MAX_MODE;j=j+1) if(MODE_BITMAP[j]) begin
            if(n==index) mode_at=j;n=n+1;end end
    endfunction
    wire [5:0] cursor_up=o_menu_cursor==0?MENU_COUNT-1:o_menu_cursor-1'b1;
    wire [5:0] cursor_down=o_menu_cursor==MENU_COUNT-1?0:o_menu_cursor+1'b1;

    // Keep public mode IDs stable while skipping uninstantiated algorithms.
    function [5:0] step_mode;
        input [5:0] current;
        input backwards;
        integer n, candidate;
        reg found;
        begin
            candidate=current;found=0;step_mode=current;
            for(n=0;n<64;n=n+1) begin
                if(backwards) candidate=(candidate==0)?MAX_MODE:candidate-1;
                else candidate=(candidate==MAX_MODE)?0:candidate+1;
                if(!found && MODE_BITMAP[candidate]) begin
                    step_mode=candidate;found=1;
                end
            end
        end
    endfunction

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            key_sync0 <= 4'hf;
            key_sync1 <= 4'hf;
            key_state <= 4'hf;
            key_state_d <= 4'hf;
            debounce_count <= 0;
            o_mode <= 0;
            o_edge_threshold <= 10'd128;
            o_remote_rejected <= 0;
            menu_sync0<=0;menu_sync1<=0;menu_enabled<=0;menu_debounce<=0;
            menu_armed<=0;o_menu_open<=0;o_menu_cursor<=0;o_menu_first<=0;
        end else begin
            o_remote_rejected <= 0;
            key_sync0 <= i_key_n;
            key_sync1 <= key_sync0;
            key_state_d <= key_state;
            if (key_sync1 == key_state) begin
                debounce_count <= 0;
            end else if (debounce_count == COUNT_MAX-1) begin
                key_state <= key_sync1;
                debounce_count <= 0;
            end else begin
                debounce_count <= debounce_count + 1'b1;
            end

            if(ENABLE_MENU) begin
                menu_sync0<=i_menu_enabled;menu_sync1<=menu_sync0;
                if(menu_sync1==menu_enabled) menu_debounce<=0;
                else if(menu_debounce==COUNT_MAX-1) begin
                    menu_enabled<=menu_sync1;menu_debounce<=0;
                    menu_armed<=0;o_menu_open<=0;
                end else menu_debounce<=menu_debounce+1'b1;
                // A mode change or reset while a key is held must not turn
                // that same press into a confirm/open event. Release first.
                if(!menu_enabled || menu_sync1!=menu_enabled) begin
                    menu_armed<=0;o_menu_open<=0;
                end else if(key_state==4'hf && key_sync1==4'hf && i_key_n==4'hf)
                    menu_armed<=1;
                if(menu_enabled && menu_sync1==menu_enabled && menu_armed && !i_capture_mode) begin
                    // Cancel takes precedence over confirm and navigation.
                    if(key_press[3]) o_menu_open<=0;
                    else if(key_press[2]) begin
                        if(o_menu_open) begin o_mode<=mode_at(o_menu_cursor);o_menu_open<=0;end
                        else begin
                            o_menu_open<=1;o_menu_cursor<=ordinal(o_mode);
                            o_menu_first<=ordinal(o_mode)>MENU_LAST_FIRST?MENU_LAST_FIRST:ordinal(o_mode);
                        end
                    end else if(!o_menu_open && (o_mode==8 || o_mode==9) && key_press[0] && !key_press[1]) begin
                        o_edge_threshold<=o_edge_threshold+THRESH_STEP>=THRESH_MAX ? THRESH_MAX : o_edge_threshold+THRESH_STEP;
                    end else if(!o_menu_open && (o_mode==8 || o_mode==9) && key_press[1] && !key_press[0]) begin
                        o_edge_threshold<=o_edge_threshold<=THRESH_MIN+THRESH_STEP ? THRESH_MIN : o_edge_threshold-THRESH_STEP;
                    end else if(o_menu_open && key_press[0] && !key_press[1]) begin
                        o_menu_cursor<=cursor_up;
                        if(o_menu_cursor==0) o_menu_first<=MENU_LAST_FIRST;
                        else if(cursor_up<o_menu_first) o_menu_first<=cursor_up;
                    end else if(o_menu_open && key_press[1] && !key_press[0]) begin
                        o_menu_cursor<=cursor_down;
                        if(o_menu_cursor==MENU_COUNT-1) o_menu_first<=0;
                        else if(cursor_down>=o_menu_first+7) o_menu_first<=cursor_down-6;
                    end
                end
            end else begin
            if (key_press[0])
                o_mode <= step_mode(o_mode,1'b0);
            else if (key_press[1])
                o_mode <= step_mode(o_mode,1'b1);

            if (key_press[2]) begin
                if (o_edge_threshold + THRESH_STEP >= THRESH_MAX)
                    o_edge_threshold <= THRESH_MAX;
                else
                    o_edge_threshold <= o_edge_threshold + THRESH_STEP;
            end else if (key_press[3]) begin
                if (o_edge_threshold <= THRESH_MIN + THRESH_STEP)
                    o_edge_threshold <= THRESH_MIN;
                else
                    o_edge_threshold <= o_edge_threshold - THRESH_STEP;
            end
            end
            if(i_capture_mode)begin o_menu_open<=0;menu_armed<=0;o_mode<=19;end
            // Frame-boundary pulses are generated by the network CDC bridge.
            // Remote commands take precedence over a coincident button edge.
            if(ENABLE_REMOTE && i_remote_mode_valid) begin
                if(i_remote_mode <= MAX_MODE && MODE_BITMAP[i_remote_mode]) begin
                    o_mode <= i_remote_mode;
                    if(ENABLE_MENU) begin o_menu_open<=0;o_menu_cursor<=ordinal(i_remote_mode);end
                end
                else o_remote_rejected <= 1;
            end
            if(ENABLE_REMOTE && i_remote_threshold_valid) begin
                if(i_remote_threshold >= THRESH_MIN && i_remote_threshold <= THRESH_MAX)
                    o_edge_threshold <= i_remote_threshold;
                else o_remote_rejected <= 1;
            end
        end
    end
endmodule
