`timescale 1ns/1ps

// Lightweight 1PPC HDMI overlay: official Anlogic logo, status text, bbox and centre.
module vision_osd_1ppc #(
    parameter IMG_WIDTH  = 1024,
    parameter IMG_HEIGHT = 600,
    parameter STAGE2_INFO = 0,
    parameter QC_INFO = 0,
    parameter ARM_INFO = 0,
    parameter NORMAL_IMAGE_AWB = 0
)(
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_vsync,
    input  wire        i_hsync,
    input  wire        i_de,
    input  wire [23:0] i_rgb,
    input  wire [5:0]  i_mode,
    input  wire [15:0] i_pipeline_cycles,
    input wire i_latency_valid,input wire [15:0] i_latency_ms,
    input wire i_qc_valid,input wire [31:0] i_qc_frame_id,
    input wire [10:0] i_qc_components,i_qc_holes,
    input wire [4:0] i_qc_x,i_qc_y,
    input wire [4:0] i_hole_x,i_hole_y,
    input wire signed [5:0] i_velocity_x,i_velocity_y,
    input wire i_hull_valid,input wire [15:0] i_volume,
    input wire i_reference_valid,
    input wire [7:0] i_display_fps,
    input wire [7:0] i_capture_fps,i_board_status,
    input wire [63:0] i_csi_diag,
    input wire [31:0] i_source_frame_id,
    input wire [7:0] i_arm_status,input wire [8:0] i_arm_points,
    input wire [5:0] i_dim_x,i_dim_y,i_dim_z,
    input wire i_defect,i_reobserve,input wire [2:0] i_suggested_view,
    input  wire        i_ae_stable,
    input  wire        i_awb_valid,
    input  wire        i_target_valid,
    input  wire [3:0]  i_target_class,
    input  wire [10:0] i_center_x,
    input  wire [9:0]  i_center_y,
    input  wire [10:0] i_bbox_x0,
    input  wire [10:0] i_bbox_x1,
    input  wire [9:0]  i_bbox_y0,
    input  wire [9:0]  i_bbox_y1,
    output reg         o_vsync,
    output reg         o_hsync,
    output reg         o_de,
    output reg  [23:0] o_rgb
);
    reg [10:0] x_pos;
    reg [9:0]  y_pos;
    reg de_d;

    localparam [10:0] LOGO_W  = 11'd160;
    localparam [9:0]  LOGO_FIRST_ROW = 10'd34;
    localparam [9:0]  LOGO_END_ROW = 10'd128;
    localparam [10:0] PANEL_X = 11'd168;
    localparam [10:0] PANEL_W = 11'd368;

    wire [10:0] panel_x = x_pos - PANEL_X;
    wire [7:0] text_col = panel_x[10:3];
    wire [3:0] text_row = y_pos[6:3];
    wire [2:0] glyph_x = panel_x[2:0];
    wire [2:0] glyph_y = y_pos[2:0];

    // The official logo occupies only rows 34..127. Its pixel values remain
    // exact, while the cropped synchronous ROM replaces a large case decoder.
    wire logo_window = (x_pos < LOGO_W)
                       && (y_pos >= LOGO_FIRST_ROW) && (y_pos < LOGO_END_ROW);
    wire [14:0] logo_addr = ({y_pos,7'b0} + {y_pos,5'b0}) + x_pos;
    wire [13:0] logo_crop_addr = logo_addr - 15'd5440;
    wire [24:0] official_logo_pixel;

    anlogic_logo_bram u_anlogic_logo_rom(
        .I_clk   (i_clk),
        .I_addr  (logo_crop_addr),
        .O_pixel (official_logo_pixel)
    );

    // Convert latency to decimal only when it changes. Keeping / and % out of
    // the per-pixel combinational path protects HDMI timing closure.
    reg [15:0] latency_cached;
    reg [7:0] lat_h, lat_t, lat_o;
    reg [7:0] fps_cached,fps_h,fps_t,fps_o;
    reg [7:0] cfps_cached,cfps_h,cfps_t,cfps_o;
    always @(posedge i_clk or negedge i_rst_n)
      if(!i_rst_n) begin cfps_cached<=255;cfps_h<="0";cfps_t<="0";cfps_o<="0";end
      else if(i_capture_fps!=cfps_cached) begin
        cfps_cached<=i_capture_fps;cfps_h<=8'd48+i_capture_fps/100;
        cfps_t<=8'd48+(i_capture_fps%100)/10;cfps_o<=8'd48+i_capture_fps%10;
      end
    always @(posedge i_clk or negedge i_rst_n)
      if(!i_rst_n) begin fps_cached<=255;fps_h<="0";fps_t<="0";fps_o<="0";end
      else if(i_display_fps!=fps_cached) begin
        fps_cached<=i_display_fps;fps_h<=8'd48+i_display_fps/100;
        fps_t<=8'd48+(i_display_fps%100)/10;fps_o<=8'd48+i_display_fps%10;
      end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            latency_cached <= 16'hffff;
            lat_h <= "0";
            lat_t <= "0";
            lat_o <= "0";
        end else if (i_latency_ms != latency_cached) begin
            latency_cached <= i_latency_ms;
            lat_h <= 8'd48 + ((i_latency_ms % 16'd1000) / 16'd100);
            lat_t <= 8'd48 + (((i_latency_ms % 16'd1000) % 16'd100) / 16'd10);
            lat_o <= 8'd48 + ((i_latency_ms % 16'd1000) % 16'd10);
        end
    end

    function [7:0] hexchar;
      input [3:0] nibble;
      begin hexchar=nibble<10?"0"+nibble:"A"+nibble-10;end
    endfunction
    reg [7:0] char_code;
    reg [7:0] mode_c0, mode_c1, mode_c2, mode_c3;
    reg [7:0] shape_c0, shape_c1, shape_c2;
    reg [7:0] color_char;

    always @(*) begin
        mode_c0="R"; mode_c1="A"; mode_c2="W"; mode_c3=" ";
        case (i_mode)
            6'd0:begin mode_c0="R"; mode_c1="A"; mode_c2="W"; mode_c3=" "; end
            6'd1:begin mode_c0="A"; mode_c1="E"; mode_c2=" "; mode_c3=" "; end
            6'd2:begin mode_c0="A"; mode_c1="W"; mode_c2="B"; mode_c3=" "; end
            6'd3:begin mode_c0="H"; mode_c1="E"; mode_c2="Q"; mode_c3=" "; end
            6'd4:begin mode_c0="C"; mode_c1="N"; mode_c2="T"; mode_c3="R"; end
            6'd5:begin mode_c0="G"; mode_c1="A"; mode_c2="U"; mode_c3="S"; end
            6'd6:begin mode_c0="M"; mode_c1="E"; mode_c2="D"; mode_c3="N"; end
            6'd7:begin mode_c0="S"; mode_c1="O"; mode_c2="B"; mode_c3="L"; end
            6'd8:begin mode_c0="B"; mode_c1="I"; mode_c2="N"; mode_c3=" "; end
            6'd9:begin mode_c0="A"; mode_c1="D"; mode_c2="P"; mode_c3="T"; end
            6'd10:begin mode_c0="E"; mode_c1="R"; mode_c2="O"; mode_c3="D"; end
            6'd11:begin mode_c0="D"; mode_c1="I"; mode_c2="L"; mode_c3="A"; end
            6'd12:begin mode_c0="C"; mode_c1="C"; mode_c2="L"; mode_c3=" "; end
            6'd13:begin mode_c0="C"; mode_c1="O"; mode_c2="N"; mode_c3="T"; end
            6'd14:begin mode_c0="O"; mode_c1="R"; mode_c2="I"; mode_c3="G"; end
            6'd15:begin mode_c0="M"; mode_c1="O"; mode_c2="T"; mode_c3="N"; end
            6'd16:begin mode_c0="P"; mode_c1="C"; mode_c2="L"; mode_c3="R"; end
            6'd17:begin mode_c0="S"; mode_c1="A"; mode_c2="T"; mode_c3=" "; end
            6'd18:begin mode_c0="E"; mode_c1="V"; mode_c2="M"; mode_c3=" "; end
            6'd19:begin mode_c0="S";mode_c1="O";mode_c2="R";mode_c3="T";end
            6'd20:begin mode_c0="C"; mode_c1="E"; mode_c2="L"; mode_c3=" "; end
            6'd21:begin mode_c0="S"; mode_c1="M"; mode_c2="T"; mode_c3="H"; end
            6'd22:begin mode_c0="F"; mode_c1="L"; mode_c2="A"; mode_c3="T"; end
            6'd23:begin mode_c0="I"; mode_c1="N"; mode_c2="K"; mode_c3=" "; end
            default:begin mode_c0="?";mode_c1="?";mode_c2="?";mode_c3="?";end
        endcase

        case (i_target_class[3:2])
            2'd1: color_char="R";
            2'd2: color_char="G";
            2'd3: color_char="B";
            default: color_char="-";
        endcase
        case (i_target_class[1:0])
            2'd0: begin shape_c0="L"; shape_c1="N"; shape_c2="G"; end
            2'd1: begin shape_c0="R"; shape_c1="E"; shape_c2="C"; end
            2'd2: begin shape_c0="R"; shape_c1="N"; shape_c2="D"; end
            default: begin shape_c0="I"; shape_c1="R"; shape_c2="R"; end
        endcase

        char_code = " ";
        case (text_row)
            0: case (text_col)
                0:char_code="A"; 1:char_code="N"; 2:char_code="L";
                3:char_code="O"; 4:char_code="G"; 5:char_code="I";
                6:char_code="C";
                default: char_code=" ";
            endcase
            1: case (text_col)
                0:char_code="M"; 1:char_code="O"; 2:char_code="D";
                3:char_code="E"; 4:char_code=":"; 5:char_code=mode_c0;
                6:char_code=mode_c1; 7:char_code=mode_c2; 8:char_code=mode_c3;
                default: char_code=" ";
            endcase
            2: case (text_col)
                0:char_code="L"; 1:char_code="A"; 2:char_code="T";
                3:char_code=":"; 4:char_code=i_latency_valid?lat_h:"-"; 5:char_code=i_latency_valid?lat_t:"-";
                6:char_code=i_latency_valid?lat_o:"-"; 7:char_code="M"; 8:char_code="S";
                default: char_code=" ";
            endcase
            3: case (text_col)
                0:char_code="A"; 1:char_code="E"; 2:char_code=":";
                3:char_code=i_mode==14?"H":"A"; 4:char_code=i_mode==14?"O":"U";
                5:char_code=i_mode==14?"L":"T"; 6:char_code=i_mode==14?"D":"O";
                8:char_code=i_mode==14?" ":i_ae_stable ? "O" : "R";
                9:char_code=i_mode==14?" ":i_ae_stable ? "K" : "U";
                10:char_code=i_mode==14?" ":i_ae_stable ? " " : "N";
                default: char_code=" ";
            endcase
            4: case (text_col)
                0:char_code="A"; 1:char_code="W"; 2:char_code="B"; 3:char_code=":";
                4:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? "B" : "A";
                5:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? "Y" : "U";
                6:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? "P" : "T";
                7:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? "A" : "O";
                8:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? "S" : " ";
                9:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? "S" : (i_awb_valid ? "O" : "R");
                10:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? " " : (i_awb_valid ? "K" : "U");
                11:char_code=(i_mode==14 || (i_mode==0 && !NORMAL_IMAGE_AWB)) ? " " : (i_awb_valid ? " " : "N");
                default: char_code=" ";
            endcase
            5: case (text_col)
                0:char_code="T"; 1:char_code="A"; 2:char_code="R"; 3:char_code="G";
                4:char_code="E"; 5:char_code="T"; 6:char_code=":";
                7:char_code=i_target_valid ? color_char : "-";
                9:char_code=i_target_valid ? shape_c0 : "-";
                10:char_code=i_target_valid ? shape_c1 : "-";
                11:char_code=i_target_valid ? shape_c2 : "-";
                default: char_code=" ";
            endcase
            6:case(text_col)
              0:char_code="S";1:char_code="R";2:char_code="C";3:char_code=":";
              // Video modes use the displayed DDR frame. Analysis modes retain
              // their own result frame and show dashes until that result exists.
              4,5,6,7,8,9,10,11:char_code=!(STAGE2_INFO && i_mode==15) && !(QC_INFO && (i_mode==12 || i_mode==13 || i_mode==14 || i_mode==18))?
                hexchar(i_source_frame_id >> ((11-text_col)*4)):
                i_qc_valid?hexchar(i_qc_frame_id >> ((11-text_col)*4)):"-";
              default:char_code=" ";endcase
            7:case(text_col)
              0:char_code="C";1:char_code="C";2:char_code="L";3:char_code=":";
              4:char_code=hexchar({1'b0,i_qc_components[10:8]});5:char_code=hexchar(i_qc_components[7:4]);6:char_code=hexchar(i_qc_components[3:0]);
              8:char_code="H";9:char_code="O";10:char_code="L";11:char_code="E";12:char_code=":";
              13:char_code=hexchar({1'b0,i_qc_holes[10:8]});14:char_code=hexchar(i_qc_holes[7:4]);15:char_code=hexchar(i_qc_holes[3:0]);
              default:char_code=" ";endcase
            8:case(text_col)
              0:char_code="X";1:char_code=":";2:char_code=hexchar({3'b0,i_qc_x[4]});3:char_code=hexchar(i_qc_x[3:0]);
              5:char_code="Y";6:char_code=":";7:char_code=hexchar({3'b0,i_qc_y[4]});8:char_code=hexchar(i_qc_y[3:0]);
              11:char_code="H";12:char_code="X";13:char_code=":";14:char_code=hexchar({3'b0,i_hole_x[4]});15:char_code=hexchar(i_hole_x[3:0]);
              18:char_code="H";19:char_code="Y";20:char_code=":";21:char_code=hexchar({3'b0,i_hole_y[4]});22:char_code=hexchar(i_hole_y[3:0]);
              default:char_code=" ";endcase
            9:case(text_col)
              0:char_code="V";1:char_code="O";2:char_code="L";3:char_code=":";
              4,5,6,7:char_code=i_hull_valid?hexchar(i_volume >> ((7-text_col)*4)):"-";
              default:char_code=" ";endcase
            10:case(text_col)
              0:char_code="D";1:char_code="I";2:char_code="M";3:char_code=":";
              4:char_code=hexchar({2'b0,i_dim_x[5:4]});5:char_code=hexchar(i_dim_x[3:0]);
              7:char_code=hexchar({2'b0,i_dim_y[5:4]});8:char_code=hexchar(i_dim_y[3:0]);
              10:char_code=hexchar({2'b0,i_dim_z[5:4]});11:char_code=hexchar(i_dim_z[3:0]);
              default:char_code=" ";endcase
            11:case(text_col)
              0:char_code="Q";1:char_code="C";2:char_code=":";
              3:char_code=i_hull_valid?(i_defect?"F":"O"):"-";
              4:char_code=i_hull_valid?(i_defect?"A":"K"):"-";
              5:char_code=i_hull_valid && i_defect?"I":" ";6:char_code=i_hull_valid && i_defect?"L":" ";
              9:char_code="R";10:char_code="E";11:char_code="F";12:char_code=":";
              13:char_code=i_reference_valid?"U":"D";14:char_code=i_reference_valid?"S":"E";
              15:char_code=i_reference_valid?"E":"M";16:char_code=i_reference_valid?"R":"O";
              default:char_code=" ";endcase
            12:case(text_col)
              0:char_code="V";1:char_code="I";2:char_code="E";3:char_code="W";4:char_code=":";
              5:char_code=i_reobserve?hexchar({1'b0,i_suggested_view}):"-";
              default:char_code=" ";endcase
            13:case(text_col)
              0:char_code="V";1:char_code="X";2:char_code=":";
              3:char_code=hexchar({2'b0,i_velocity_x[5:4]});4:char_code=hexchar(i_velocity_x[3:0]);
              6:char_code="V";7:char_code="Y";8:char_code=":";
              9:char_code=hexchar({2'b0,i_velocity_y[5:4]});10:char_code=hexchar(i_velocity_y[3:0]);
              default:char_code=" ";endcase
            14:case(text_col)
              0:char_code="D";1:char_code="F";2:char_code="P";3:char_code="S";4:char_code=":";
              5:char_code=fps_h;6:char_code=fps_t;7:char_code=fps_o;
              9:char_code="C";10:char_code="F";11:char_code="P";12:char_code="S";13:char_code=":";
              14:char_code=cfps_h;15:char_code=cfps_t;16:char_code=cfps_o;
              18:char_code="B";19:char_code="S";20:char_code="P";21:char_code=":";
              22:char_code=hexchar(i_board_status[7:4]);23:char_code=hexchar(i_board_status[3:0]);
              default:char_code=" ";endcase
            15:case(text_col)
              0:char_code="C";
              1:char_code="S";
              2:char_code="I";
              3:char_code=":";
              4:char_code=hexchar(i_csi_diag[63:60]);
              5:char_code=hexchar(i_csi_diag[59:56]);
              7:char_code="D";
              8:char_code="T";
              9:char_code=":";
              10:char_code=hexchar(i_csi_diag[55:52]);
              11:char_code=hexchar(i_csi_diag[51:48]);
              13:char_code="W";
              14:char_code=":";
              15:char_code=hexchar(i_csi_diag[47:44]);
              16:char_code=hexchar(i_csi_diag[43:40]);
              17:char_code=hexchar(i_csi_diag[39:36]);
              18:char_code=hexchar(i_csi_diag[35:32]);
              20:char_code="L";
              21:char_code=":";
              22:char_code=hexchar(i_csi_diag[31:28]);
              23:char_code=hexchar(i_csi_diag[27:24]);
              24:char_code=hexchar(i_csi_diag[23:20]);
              25:char_code=hexchar(i_csi_diag[19:16]);
              27:char_code="C";
              28:char_code=":";
              29:char_code=hexchar(i_csi_diag[15:12]);
              30:char_code=hexchar(i_csi_diag[11:8]);
              31:char_code=hexchar(i_csi_diag[7:4]);
              32:char_code=hexchar(i_csi_diag[3:0]);
              default:char_code=" ";endcase
            default: char_code=" ";
        endcase
        if(STAGE2_INFO && !(QC_INFO && (i_mode==12 || i_mode==13 || i_mode==14 || i_mode==18))) begin
          // Excluded QC/3D fields must not appear as zero-valued measurements.
          if(text_row==7) begin
            char_code=" ";
            if(i_mode==15) case(text_col)
              0:char_code="M";1:char_code="O";2:char_code="V";3:char_code=":";
              4:char_code=i_qc_valid?hexchar({1'b0,i_qc_components[10:8]}):"-";
              5:char_code=i_qc_valid?hexchar(i_qc_components[7:4]):"-";
              6:char_code=i_qc_valid?hexchar(i_qc_components[3:0]):"-";
              8:char_code="C";9:char_code="E";10:char_code="L";11:char_code="L";12:char_code="S";
              default:char_code=" ";endcase
          end
          if(text_row==8) begin
            char_code=" ";
            if(i_mode==2) case(text_col)
              0:char_code="X";1:char_code=":";
              2:char_code=i_target_valid?hexchar({1'b0,i_center_x[10:8]}):"-";
              3:char_code=i_target_valid?hexchar(i_center_x[7:4]):"-";
              4:char_code=i_target_valid?hexchar(i_center_x[3:0]):"-";
              6:char_code="Y";7:char_code=":";
              8:char_code=i_target_valid?hexchar({2'b0,i_center_y[9:8]}):"-";
              9:char_code=i_target_valid?hexchar(i_center_y[7:4]):"-";
              10:char_code=i_target_valid?hexchar(i_center_y[3:0]):"-";
              default:char_code=" ";endcase
          end
          if(text_row>=9 && text_row<=13) char_code=" ";
        end
        if(QC_INFO && (i_mode==12 || i_mode==13 || i_mode==14)) begin
          if(text_row==9) case(text_col)
           0:char_code="A";1:char_code="R";2:char_code="E";3:char_code="A";4:char_code=":";
           5,6,7:char_code=i_qc_valid?hexchar(i_volume >> ((7-text_col)*4)):"-";
           default:char_code=" ";endcase
          if(text_row==10) case(text_col)
           0:char_code="W";1:char_code=":";2:char_code=hexchar({2'd0,i_dim_x[5:4]});3:char_code=hexchar(i_dim_x[3:0]);
           5:char_code="H";6:char_code=":";7:char_code=hexchar({2'd0,i_dim_y[5:4]});8:char_code=hexchar(i_dim_y[3:0]);
           default:char_code=" ";endcase
          if(text_row==12) char_code=" ";
          if(text_row==11 && !i_reference_valid) case(text_col)
           13:char_code="N";14:char_code="O";15:char_code="N";16:char_code="E";default:begin end
          endcase
        end
        if(ARM_INFO && text_row==13) case(text_col)
          0:char_code="A";1:char_code="R";2:char_code="M";3:char_code=":";
          4:char_code=hexchar(i_arm_status[7:4]);5:char_code=hexchar(i_arm_status[3:0]);
          7:char_code="P";8:char_code="T";9:char_code="S";10:char_code=":";
          11:char_code=hexchar({3'd0,i_arm_points[8]});12:char_code=hexchar(i_arm_points[7:4]);
          13:char_code=hexchar(i_arm_points[3:0]);default:char_code=" ";endcase
    end

    function [4:0] glyph_row;
        input [7:0] ch;
        input [2:0] row;
        reg [34:0] bits;
        begin
            case (ch)
                "A": bits={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "B": bits={5'b11110,5'b10001,5'b10001,5'b11110,5'b10001,5'b10001,5'b11110};
                "C": bits={5'b01111,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b01111};
                "D": bits={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
                "E": bits={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
                "F": bits={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
                "Q": bits={5'b01110,5'b10001,5'b10001,5'b10001,5'b10101,5'b10010,5'b01101};
                "G": bits={5'b01111,5'b10000,5'b10000,5'b10111,5'b10001,5'b10001,5'b01111};
                "H": bits={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
                "I": bits={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b11111};
                "K": bits={5'b10001,5'b10010,5'b10100,5'b11000,5'b10100,5'b10010,5'b10001};
                "L": bits={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
                "M": bits={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
                "N": bits={5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001};
                "O": bits={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "P": bits={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
                "R": bits={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
                "S": bits={5'b01111,5'b10000,5'b10000,5'b01110,5'b00001,5'b00001,5'b11110};
                "T": bits={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
                "U": bits={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
                "V": bits={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01010,5'b00100};
                "W": bits={5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b10101,5'b01010};
                "X": bits={5'b10001,5'b10001,5'b01010,5'b00100,5'b01010,5'b10001,5'b10001};
                "Y": bits={5'b10001,5'b10001,5'b01010,5'b00100,5'b00100,5'b00100,5'b00100};
                "0": bits={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
                "1": bits={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
                "2": bits={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
                "3": bits={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
                "4": bits={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
                "5": bits={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
                "6": bits={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
                "7": bits={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
                "8": bits={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
                "9": bits={5'b01110,5'b10001,5'b10001,5'b01111,5'b00001,5'b00001,5'b01110};
                ":": bits={5'b00000,5'b00100,5'b00100,5'b00000,5'b00100,5'b00100,5'b00000};
                "-": bits={5'b00000,5'b00000,5'b00000,5'b11111,5'b00000,5'b00000,5'b00000};
                default: bits=0;
            endcase
            case (row)
                0:glyph_row=bits[34:30]; 1:glyph_row=bits[29:25];
                2:glyph_row=bits[24:20]; 3:glyph_row=bits[19:15];
                4:glyph_row=bits[14:10]; 5:glyph_row=bits[9:5];
                6:glyph_row=bits[4:0]; default:glyph_row=0;
            endcase
        end
    endfunction

    wire [4:0] glyph_bits = glyph_row(char_code,glyph_y);
    wire text_pixel = (x_pos >= PANEL_X) && (x_pos < (PANEL_X + PANEL_W))
                      && (y_pos < 10'd128)
                      && (glyph_x < 5) && glyph_bits[4-glyph_x];
    wire detection_overlay = (i_mode == 6'd2) && i_target_valid;
    wire bbox_h = detection_overlay && (x_pos>=i_bbox_x0) && (x_pos<=i_bbox_x1)
                  && ((y_pos==i_bbox_y0)||(y_pos==i_bbox_y0+1)
                      ||(y_pos==i_bbox_y1)||(y_pos+1==i_bbox_y1));
    wire bbox_v = detection_overlay && (y_pos>=i_bbox_y0) && (y_pos<=i_bbox_y1)
                  && ((x_pos==i_bbox_x0)||(x_pos==i_bbox_x0+1)
                      ||(x_pos==i_bbox_x1)||(x_pos+1==i_bbox_x1));
    wire [10:0] centre_dx = (x_pos>=i_center_x) ? (x_pos-i_center_x) : (i_center_x-x_pos);
    wire [9:0] centre_dy = (y_pos>=i_center_y) ? (y_pos-i_center_y) : (i_center_y-y_pos);
    wire centre = detection_overlay
                  && (((centre_dy==0)&&(centre_dx<=6))
                      ||((centre_dx==0)&&(centre_dy<=6)));
    wire [23:0] target_colour = (i_target_class[3:2]==1) ? 24'hff3030
                                : (i_target_class[3:2]==2) ? 24'h30ff50
                                : 24'h3090ff;
    reg logo_window_stage1,logo_window_stage2;
    reg vs_stage1,hs_stage1,de_stage1;
    reg [23:0] fallback_rgb_stage1,fallback_rgb_stage2;
    always @(*) begin
        if (!o_de)
            o_rgb = 24'd0;
        else if (logo_window_stage2 && official_logo_pixel[24])
            o_rgb = official_logo_pixel[23:0];
        else
            o_rgb = fallback_rgb_stage2;
    end
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            x_pos<=0; y_pos<=0; de_d<=0;
            logo_window_stage1<=0; fallback_rgb_stage1<=0;
            logo_window_stage2<=0;fallback_rgb_stage2<=0;vs_stage1<=0;hs_stage1<=0;de_stage1<=0;
            o_vsync<=0; o_hsync<=0; o_de<=0;
        end else begin
            logo_window_stage1 <= i_de && logo_window;
            vs_stage1<=i_vsync;hs_stage1<=i_hsync;de_stage1<=i_de;
            logo_window_stage2<=logo_window_stage1;fallback_rgb_stage2<=fallback_rgb_stage1;
            o_vsync <= vs_stage1;
            o_hsync <= hs_stage1;
            o_de <= de_stage1;
            de_d <= i_de;
            if (i_vsync) begin
                x_pos <= 0;
                y_pos <= 0;
            end else if (i_de) begin
                if (x_pos == IMG_WIDTH-1) x_pos<=0; else x_pos<=x_pos+1'b1;
            end else if (de_d) begin
                x_pos <= 0;
                if (y_pos == IMG_HEIGHT-1) y_pos<=0; else y_pos<=y_pos+1'b1;
            end

            if (!i_de)
                fallback_rgb_stage1 <= 0;
            else if (text_pixel)
                fallback_rgb_stage1 <= 24'hfff200;
            else if (bbox_h || bbox_v || centre)
                fallback_rgb_stage1 <= target_colour;
            else
                fallback_rgb_stage1 <= i_rgb;

        end
    end
endmodule
