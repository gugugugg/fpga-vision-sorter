`timescale 1ns/1ps
// Deliberately bounded flat JSON schema, accepting host key order/whitespace.
// Unknown/duplicate keys, escapes, negative/fractional values are rejected.
// HELLO/PING/STOP use empty objects. STREAM requires exactly six fields.
module network_vq_json(
    input wire i_clk,i_rst_n,i_start,i_stream,
    input wire [10:0] i_size,
    output wire [10:0] o_addr,input wire [7:0] i_byte,input wire i_byte_valid,
    output reg o_done,o_ok,o_original
);
    localparam OPEN=0,KEY=1,KSTR=2,COLON=3,VALUE=4,VSTR=5,NUM=6,
               SEP=7,TAIL=8,FINISH=9;
    reg busy,stream; reg [3:0] state;
    reg [10:0] pos,size; reg [95:0] token;
    reg [3:0] n,key; reg [5:0] seen; reg [31:0] number;
    reg original; reg [31:0] width,height,fmt;
    reg fps_eq5,source_eq1;
    wire ws=i_byte==8'h20 || i_byte==9 || i_byte==10 || i_byte==13;
    wire digit=i_byte>="0" && i_byte<="9";
    assign o_addr=pos;
    task fail;
        begin busy<=0;o_done<=1;o_ok<=0;end
    endtask
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            busy<=0;state<=OPEN;pos<=0;size<=0;stream<=0;token<=0;n<=0;
            key<=0;seen<=0;number<=0;original<=0;fps_eq5<=0;source_eq1<=0;
            width<=0;height<=0;fmt<=0;o_done<=0;o_ok<=0;o_original<=0;
        end else begin
            o_done<=0;
            if(i_start && !busy) begin
                busy<=1;state<=OPEN;pos<=0;size<=i_size;stream<=i_stream;
                seen<=0;o_ok<=0;original<=0;
            end else if(busy) begin
                if(state==FINISH) begin
                    busy<=0;o_done<=1;o_original<=original;
                    o_ok<=stream?(seen==6'b111111 && fps_eq5 && source_eq1 &&
                        width==(original?1024:512) && height==(original?600:300) &&
                        fmt==(original?1:2)):(seen==0);
                end else if(pos>=size) begin
                    if(state==TAIL) state<=FINISH;else fail;
                end else if(i_byte_valid) begin
                    pos<=pos+1'b1;
                    case(state)
                        OPEN:if(!ws) begin if(i_byte=="{") state<=KEY;else fail;end
                        KEY:if(!ws) begin
                            if(i_byte=="}" && seen==0) state<=TAIL;
                            else if(i_byte=="\"") begin state<=KSTR;token<=0;n<=0;end
                            else fail;
                        end
                        KSTR:if(i_byte=="\"") begin
                            state<=COLON;
                            case(token)
                                "profile":key<=0;"fps":key<=1;"source":key<=2;
                                "width":key<=3;"height":key<=4;"pixel_format":key<=5;
                                default:fail;
                            endcase
                        end else if(n==12 || i_byte<32 || i_byte=="\\") fail;
                        else begin token<={token[87:0],i_byte};n<=n+1'b1;end
                        COLON:if(!ws) begin
                            if(i_byte!=":" || seen[key] || !stream) fail;
                            else begin seen[key]<=1;state<=VALUE;end
                        end
                        VALUE:if(!ws) begin
                            if(key==0 && i_byte=="\"") begin state<=VSTR;token<=0;n<=0;end
                            else if(key!=0 && digit) begin number<=i_byte-"0";n<=1;state<=NUM;end
                            else fail;
                        end
                        VSTR:if(i_byte=="\"") begin
                            state<=SEP;
                            if(token=="preview") original<=0;
                            else if(token=="original") original<=1;else fail;
                        end else if(n==12 || i_byte<32 || i_byte=="\\") fail;
                        else begin token<={token[87:0],i_byte};n<=n+1'b1;end
                        NUM:if(digit) begin
                            if(n==6 || (n==1 && number==0)) fail;
                            else begin number<=number*10+i_byte-"0";n<=n+1'b1;end
                        end else begin
                            case(key)
                                1:fps_eq5<=(number==32'd5);2:source_eq1<=(number==32'd1);3:width<=number;
                                4:height<=number;5:fmt<=number;
                            endcase
                            state<=SEP;pos<=pos; // Reprocess delimiter.
                        end
                        SEP:if(!ws) begin
                            if(i_byte==",") state<=KEY;
                            else if(i_byte=="}") state<=TAIL;else fail;
                        end
                        TAIL:if(!ws) fail;
                        default:fail;
                    endcase
                end
            end
        end
    end
endmodule
