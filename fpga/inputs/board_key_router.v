`timescale 1ns/1ps
// HX1P35A manual p21: physical SW4 = SW3 net = D11; UP/ON=1.
// Reset assertion remains asynchronous; reset_release synchronizes release
// separately in every existing clock domain. SW4 is independently synchronized
// and debounced by vision_mode_ctrl before it enables menu key events.
module board_key_router #(parameter ENABLE_MENU=0)(
    input wire i_sw4,i_key3_n,i_key4_n,
    input wire [1:0] i_keys12_n,
    output wire o_reset_n,o_menu_enabled,o_capture_key_n,
    output wire [3:0] o_keys_n
);
    assign o_reset_n=ENABLE_MENU ? (i_key3_n | ~i_sw4) : i_key3_n;
    assign o_menu_enabled=ENABLE_MENU && !i_sw4;
    assign o_keys_n=ENABLE_MENU ? {i_key4_n,i_key3_n,i_keys12_n} : {2'b11,i_keys12_n};
    assign o_capture_key_n=ENABLE_MENU ? 1'b1 : i_key4_n;
endmodule
