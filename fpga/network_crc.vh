// IEEE reflected CRC32 (internal state; complement on wire) and CCITT-FALSE.
function [31:0] crc32_byte;
    input [31:0] c; input [7:0] b;
    reg [31:0] v; integer k;
    begin
        v=c^b;
        for(k=0;k<8;k=k+1) v=v[0]?(v>>1)^32'hedb88320:v>>1;
        crc32_byte=v;
    end
endfunction
function [15:0] crc16_byte;
    input [15:0] c; input [7:0] b;
    reg [15:0] v; integer k;
    begin
        v=c^{b,8'd0};
        for(k=0;k<8;k=k+1) v=v[15]?(v<<1)^16'h1021:v<<1;
        crc16_byte=v;
    end
endfunction
