


// BSP reader adapted for frame selection, domain resets and draining old reads.
// Original vendor source is retained unchanged.
module board_frame_reader (
    output wire o_frame_start,
    input wire        I_rst_n,

    input wire        I_ddr_clk,
    output wire       O_video_out_rd_busy,  
    input wire        I_video_in_wr_busy,   
    input wire[1:0]   I_video_out_rp,
    output wire       O_ddr_user_rd_en,
    output reg[24:0]  O_ddr_user_addr,
    input wire        I_ddr_user_ready,
    input wire        I_ddr_user_rd_valid,
    input wire[127:0] I_ddr_user_rd_data,

    input wire        I_dsi_clk,
    input wire        I_video_vsync,
    input wire        I_video_rd_en,  
    output reg[23:0]  O_vdieo_data
);
    wire ddr_rst_n,pixel_rst_n;
    reset_release ddr_reset(I_ddr_clk,I_rst_n,ddr_rst_n);
    reset_release pixel_reset(I_dsi_clk,I_rst_n,pixel_rst_n);

    reg [9:0] pending_reads;
    // Each accepted read owns one future FIFO slot until its response arrives.
    // DDR arbitration can delay a burst far beyond the demo's 40-cycle margin.
    // Register the admission test; reserve eight slots for stale occupancy and
    // the grant pipeline. Do not interrupt a burst after it has been admitted.
    reg room_for_burst;
    reg have_frame;
    reg         S_fifo_rst;
    reg[7:0]    S_fifo_rst_cnt;
    wire[8:0]   S_fifo_wr_num;       
    wire        S_fifo_rd_en;        
    wire[127:0] S_fifo_rd_data;      
    (* async_reg="true",syn_preserve=1 *) reg S_video_vsync_1d,S_video_vsync_2d;
    reg         S_video_vsync_3d;
    wire        S_video_frame_start;   
    wire        S_ddr_rd_trig;       
    reg         S_ddr_rd_valid;      
    reg[9:0]    S_ddr_rd_cnt;        
    reg[3:0]    S_fifo_rd_cnt;       
    reg         S_video_rd_en_1d;
    reg[3:0]    S_fifo_rd_cnt_1d;     
    reg[127:0]  S_fifo_rd_data_1d;    
	wire        S_fifo_emtpy;     



    localparam IMAGE_BASE_ADDR_0 = 25'd0;
    localparam IMAGE_BASE_ADDR_1 = 25'd7000000;
    localparam IMAGE_BASE_ADDR_2 = 25'd14000000;
    localparam IMAGE_BASE_ADDR_3 = 25'd21000000;
    wire [10:0] reserved_slots={2'd0,S_fifo_wr_num}+{1'b0,pending_reads};
    always @(posedge I_ddr_clk or negedge ddr_rst_n)
        if(!ddr_rst_n)room_for_burst<=0;
        else room_for_burst<=reserved_slots<=11'd263; // 511 - 240 - 8


    always @(posedge I_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin S_video_vsync_1d<=0;S_video_vsync_2d<=0;S_video_vsync_3d<=0;end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <=S_video_vsync_1d;
            S_video_vsync_3d <= S_video_vsync_2d;
        end
    end

    assign S_video_frame_start = ~S_video_vsync_3d & S_video_vsync_2d;
    assign o_frame_start = S_video_frame_start;

    // Do not mix responses from the previous frame into a newly reset FIFO.
    always @(posedge I_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) begin pending_reads<=0;have_frame<=0;end
        else begin
            if(S_video_frame_start) have_frame<=1;
            case({O_ddr_user_rd_en,I_ddr_user_rd_valid})
              2'b10:pending_reads<=pending_reads+1'b1;
              2'b01:if(pending_reads!=0)pending_reads<=pending_reads-1'b1;
              default:pending_reads<=pending_reads;
            endcase
        end
    end


    always @(posedge I_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n)
            S_fifo_rst <= 1'b0;
        else
            if(S_video_frame_start)
                S_fifo_rst <= 1'b1;
            else if(S_fifo_rst_cnt == 'd10 && pending_reads==0)
                S_fifo_rst <= 1'b0;
            else
                S_fifo_rst <= S_fifo_rst;
    end


    always @(posedge I_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) S_fifo_rst_cnt<=0;
        else if(S_video_frame_start || pending_reads!=0) S_fifo_rst_cnt <= 0;
        else if(S_fifo_rst)
            S_fifo_rst_cnt <= S_fifo_rst_cnt + 'd1;
        else
            S_fifo_rst_cnt <= 'd0;
    end

    /*
        这里需要注意fifo的剩余空间一定要大于将要写入的长度，因为ddr读出的时候会有至少40个时钟周期的延时，
        例如每次从ddr读出240个长度的数据，那么fifo的剩余空间最少要设为280，否则会引起fifo写满丢失数据。
    */
    assign S_ddr_rd_trig = room_for_burst && (!I_video_in_wr_busy) && (!S_ddr_rd_valid) && (!S_fifo_rst) && ddr_rst_n && have_frame && !S_video_frame_start;

    /*
        fifo的深度是512，实际的深度是510，因此需要注意每次读出的长度避免为256，否则连续两次写入会引起fifo写满，丢失数据
    */
    always @(posedge I_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n)
            S_ddr_rd_valid <= 1'b0;
        else
			if(S_video_frame_start) S_ddr_rd_valid <= 1'b0;
            else if(O_ddr_user_rd_en && S_ddr_rd_cnt == 'd239)
                S_ddr_rd_valid <= 1'b0;
            else if(S_ddr_rd_trig)
                S_ddr_rd_valid <= 1'b1;
            else
                S_ddr_rd_valid <= S_ddr_rd_valid;
    end

    assign O_ddr_user_rd_en = S_ddr_rd_valid & I_ddr_user_ready & ~S_fifo_rst & ~S_video_frame_start;

    always @(posedge I_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n) S_ddr_rd_cnt<=0;
        else if(S_video_frame_start) S_ddr_rd_cnt<=0;
        else if(S_ddr_rd_valid)
            begin
                if(O_ddr_user_rd_en)
                    S_ddr_rd_cnt <= S_ddr_rd_cnt + 'd1;
                else
                    S_ddr_rd_cnt <= S_ddr_rd_cnt;
            end
        else    
            S_ddr_rd_cnt <= 'd0;
    end


    always @(posedge I_ddr_clk or negedge ddr_rst_n) begin
        if(!ddr_rst_n)
            O_ddr_user_addr <= 'd0;
        else
            if(S_video_frame_start)
                begin
                    case(I_video_out_rp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                    endcase
                end
            else if(O_ddr_user_rd_en)
                O_ddr_user_addr <= O_ddr_user_addr + 'd8;
            else    
                O_ddr_user_addr <= O_ddr_user_addr;
    end

    
    assign O_video_out_rd_busy = S_ddr_rd_trig | S_ddr_rd_valid;


    w128_d512_fifo U_w128_d512_fifo(
        .rst        ( S_fifo_rst | !I_rst_n ),

        .clkw       ( I_ddr_clk           ),     
        .we         ( I_ddr_user_rd_valid ),
        .di         ( I_ddr_user_rd_data  ),
        .wrusedw    ( S_fifo_wr_num       ),
        .afull      (),
        .full_flag  (),
 
        .clkr       ( I_dsi_clk           ),
        .re         ( S_fifo_rd_en        ), 
        .dout       ( S_fifo_rd_data      ),  
        .rdusedw    (), 
        .valid      (),  
        .empty_flag ( S_fifo_emtpy        ),
        .aempty     ()
    );


   always @(posedge I_dsi_clk or negedge pixel_rst_n) begin
       if(!pixel_rst_n) S_fifo_rd_cnt<=0;
       else if(I_video_rd_en)
           S_fifo_rd_cnt <= S_fifo_rd_cnt + 'd1;
       else
           S_fifo_rd_cnt <= 'd0;
   end

   assign S_fifo_rd_en = I_video_rd_en && (S_fifo_rd_cnt == 'd0)  ? 1'b1 :
                         I_video_rd_en && (S_fifo_rd_cnt == 'd5)  ? 1'b1 :
                         I_video_rd_en && (S_fifo_rd_cnt == 'd10) ? 1'b1 : 1'b0;

   always @(posedge I_dsi_clk or negedge pixel_rst_n) begin
       if(!pixel_rst_n) begin S_video_rd_en_1d<=0;S_fifo_rd_cnt_1d<=0;S_fifo_rd_data_1d<=0;end
       else begin
           S_video_rd_en_1d  <= I_video_rd_en;
           S_fifo_rd_cnt_1d  <= S_fifo_rd_cnt;
           S_fifo_rd_data_1d <= S_fifo_rd_data;
       end
   end

   always @(posedge I_dsi_clk or negedge pixel_rst_n) begin
       if(!pixel_rst_n) O_vdieo_data<=0;
       else if(S_video_rd_en_1d)
           begin
               case(S_fifo_rd_cnt_1d)
                   'd0: O_vdieo_data  <= S_fifo_rd_data[127:104];
                   'd1: O_vdieo_data  <= S_fifo_rd_data[103:80];
                   'd2: O_vdieo_data  <= S_fifo_rd_data[79:56];
                   'd3: O_vdieo_data  <= S_fifo_rd_data[55:32];
                   'd4: O_vdieo_data  <= S_fifo_rd_data[31:8];
                   'd5: O_vdieo_data  <= {S_fifo_rd_data_1d[7:0],S_fifo_rd_data[127:112]};
                   'd6: O_vdieo_data  <= S_fifo_rd_data[111:88];
                   'd7: O_vdieo_data  <= S_fifo_rd_data[87:64];
                   'd8: O_vdieo_data  <= S_fifo_rd_data[63:40];
                   'd9: O_vdieo_data  <= S_fifo_rd_data[39:16];
                   'd10: O_vdieo_data <= {S_fifo_rd_data_1d[15:0],S_fifo_rd_data[127:120]};
                   'd11: O_vdieo_data <= S_fifo_rd_data[119:96];
                   'd12: O_vdieo_data <= S_fifo_rd_data[95:72];
                   'd13: O_vdieo_data <= S_fifo_rd_data[71:48];
                   'd14: O_vdieo_data <= S_fifo_rd_data[47:24];
                   'd15: O_vdieo_data <= S_fifo_rd_data[23:0];
               endcase
           end
       else
           O_vdieo_data <= 'd0;
   end

    
endmodule


