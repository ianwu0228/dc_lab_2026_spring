module DE2_115 (
	input CLOCK_50,
	input CLOCK2_50,
	input CLOCK3_50,
	input ENETCLK_25,
	input SMA_CLKIN,
	output SMA_CLKOUT,
	output [8:0] LEDG,
	output [17:0] LEDR,
	input [3:0] KEY,
	input [17:0] SW,
	output [6:0] HEX0,
	output [6:0] HEX1,
	output [6:0] HEX2,
	output [6:0] HEX3,
	output [6:0] HEX4,
	output [6:0] HEX5,
	output [6:0] HEX6,
	output [6:0] HEX7,
	output LCD_BLON,
	inout [7:0] LCD_DATA,
	output LCD_EN,
	output LCD_ON,
	output LCD_RS,
	output LCD_RW,
	output UART_CTS,
	input UART_RTS,
	input UART_RXD,
	output UART_TXD,
	inout PS2_CLK,
	inout PS2_DAT,
	inout PS2_CLK2,
	inout PS2_DAT2,
	output SD_CLK,
	inout SD_CMD,
	inout [3:0] SD_DAT,
	input SD_WP_N,
	output [7:0] VGA_B,
	output VGA_BLANK_N,
	output VGA_CLK,
	output [7:0] VGA_G,
	output VGA_HS,
	output [7:0] VGA_R,
	output VGA_SYNC_N,
	output VGA_VS,
	input AUD_ADCDAT,
	inout AUD_ADCLRCK,
	inout AUD_BCLK,
	output AUD_DACDAT,
	inout AUD_DACLRCK,
	output AUD_XCK,
	output EEP_I2C_SCLK,
	inout EEP_I2C_SDAT,
	output I2C_SCLK,
	inout I2C_SDAT,
	output ENET0_GTX_CLK,
	input ENET0_INT_N,
	output ENET0_MDC,
	input ENET0_MDIO,
	output ENET0_RST_N,
	input ENET0_RX_CLK,
	input ENET0_RX_COL,
	input ENET0_RX_CRS,
	input [3:0] ENET0_RX_DATA,
	input ENET0_RX_DV,
	input ENET0_RX_ER,
	input ENET0_TX_CLK,
	output [3:0] ENET0_TX_DATA,
	output ENET0_TX_EN,
	output ENET0_TX_ER,
	input ENET0_LINK100,
	output ENET1_GTX_CLK,
	input ENET1_INT_N,
	output ENET1_MDC,
	input ENET1_MDIO,
	output ENET1_RST_N,
	input ENET1_RX_CLK,
	input ENET1_RX_COL,
	input ENET1_RX_CRS,
	input [3:0] ENET1_RX_DATA,
	input ENET1_RX_DV,
	input ENET1_RX_ER,
	input ENET1_TX_CLK,
	output [3:0] ENET1_TX_DATA,
	output ENET1_TX_EN,
	output ENET1_TX_ER,
	input ENET1_LINK100,
	input TD_CLK27,
	input [7:0] TD_DATA,
	input TD_HS,
	output TD_RESET_N,
	input TD_VS,
	inout [15:0] OTG_DATA,
	output [1:0] OTG_ADDR,
	output OTG_CS_N,
	output OTG_WR_N,
	output OTG_RD_N,
	input OTG_INT,
	output OTG_RST_N,
	input IRDA_RXD,
	output [12:0] DRAM_ADDR,
	output [1:0] DRAM_BA,
	output DRAM_CAS_N,
	output DRAM_CKE,
	output DRAM_CLK,
	output DRAM_CS_N,
	inout [31:0] DRAM_DQ,
	output [3:0] DRAM_DQM,
	output DRAM_RAS_N,
	output DRAM_WE_N,
	output [19:0] SRAM_ADDR,
	output SRAM_CE_N,
	inout [15:0] SRAM_DQ,
	output SRAM_LB_N,
	output SRAM_OE_N,
	output SRAM_UB_N,
	output SRAM_WE_N,
	output [22:0] FL_ADDR,
	output FL_CE_N,
	inout [7:0] FL_DQ,
	output FL_OE_N,
	output FL_RST_N,
	input FL_RY,
	output FL_WE_N,
	output FL_WP_N,
	inout [35:0] GPIO,
	input HSMC_CLKIN_P1,
	input HSMC_CLKIN_P2,
	input HSMC_CLKIN0,
	output HSMC_CLKOUT_P1,
	output HSMC_CLKOUT_P2,
	output HSMC_CLKOUT0,
	inout [3:0] HSMC_D,
	input [16:0] HSMC_RX_D_P,
	output [16:0] HSMC_TX_D_P,
	inout [6:0] EX_IO
);

logic key0down;
logic key1down;
logic key3down;
logic key0down_d;
logic key1down_d;

Debounce deb0(
    .i_in(KEY[0]),
    .i_rst_n(1'b1),
    .i_clk(CLOCK_50),
    .o_debounced(key0down)
);

Debounce deb1(
    .i_in(KEY[1]),
    .i_rst_n(1'b1),
    .i_clk(CLOCK_50),
    .o_debounced(key1down)
);

Debounce deb3(
    .i_in(KEY[3]),
    .i_rst_n(1'b1),
    .i_clk(CLOCK_50),
    .o_debounced(key3down)
);

always @(posedge CLOCK_50 or negedge key3down) begin
    if (!key3down) begin
        key0down_d <= 1'b1;
        key1down_d <= 1'b1;
    end else begin
        key0down_d <= key0down;
        key1down_d <= key1down;
    end
end

wire calibration_start_pulse  = key0down_d && !key0down;
wire calibration_finish_pulse = key1down_d && !key1down;

assign HEX4 = '1;
assign HEX5 = '1;
assign HEX6 = '1;

// =====================================================================
// QMC5883P 磁力計硬體連接與三態緩衝器實作
// =====================================================================

// --- 四顆磁力計的獨立 I2C bus，依序使用 GPIO[0] ~ GPIO[7] ---
wire [3:0]  qmc_scl;
wire [3:0]  qmc_sda_out;
wire [3:0]  qmc_sda_in;
wire [3:0]  qmc_sda_dir;
wire [15:0] mag1_x, mag1_y, mag1_z;
wire [15:0] mag2_x, mag2_y, mag2_z;
wire [15:0] mag3_x, mag3_y, mag3_z;
wire [15:0] mag4_x, mag4_y, mag4_z;
wire [3:0]  qmc_sample_valid;
localparam [3:0] ACTIVE_SENSOR_MASK = 4'b1111;
wire [4:0]  qmc_dbg_state [0:3];
wire [3:0]  qmc_dbg_err;
wire [7:0]  qmc_dbg_chip_id [0:3];
wire [3:0]  qmc_dbg_chip_id_valid;
wire [3:0]  qmc_dbg_chip_id_ok;
wire [3:0]  qmc_dbg_init_done;
wire [3:0]  qmc_dbg_ack_error_latched;

assign GPIO[0] = qmc_scl[0];
assign GPIO[1] = (qmc_sda_dir[0] && (qmc_sda_out[0] == 1'b0)) ? 1'b0 : 1'bz;
assign qmc_sda_in[0] = GPIO[1];

assign GPIO[2] = qmc_scl[1];
assign GPIO[3] = (qmc_sda_dir[1] && (qmc_sda_out[1] == 1'b0)) ? 1'b0 : 1'bz;
assign qmc_sda_in[1] = GPIO[3];

assign GPIO[4] = qmc_scl[2];
assign GPIO[5] = (qmc_sda_dir[2] && (qmc_sda_out[2] == 1'b0)) ? 1'b0 : 1'bz;
assign qmc_sda_in[2] = GPIO[5];

assign GPIO[6] = qmc_scl[3];
assign GPIO[7] = (qmc_sda_dir[3] && (qmc_sda_out[3] == 1'b0)) ? 1'b0 : 1'bz;
assign qmc_sda_in[3] = GPIO[7];


// =====================================================================
// 實體化控制大腦 (完全平行獨立運作)
// =====================================================================

// Module name remains qmc5883l_ctrl, but the file is now fixed for QMC5883P.
qmc5883l_ctrl u_qmc_1 (
    .clk(CLOCK_50),       
    .rst_n(key3down),       
    .i2c_scl(qmc_scl[0]),
    .sda_out(qmc_sda_out[0]),
    .sda_in(qmc_sda_in[0]),
    .sda_dir(qmc_sda_dir[0]),
    .mag_x(mag1_x),
    .mag_y(mag1_y),
    .mag_z(mag1_z),
    .sample_valid(qmc_sample_valid[0]),
    
    .dbg_state(qmc_dbg_state[0]),
    .dbg_err(qmc_dbg_err[0]),
    .dbg_chip_id(qmc_dbg_chip_id[0]),
    .dbg_chip_id_valid(qmc_dbg_chip_id_valid[0]),
    .dbg_chip_id_ok(qmc_dbg_chip_id_ok[0]),
    .dbg_init_done(qmc_dbg_init_done[0]),
    .dbg_ack_error_latched(qmc_dbg_ack_error_latched[0])
);

qmc5883l_ctrl u_qmc_2 (
    .clk(CLOCK_50),
    .rst_n(key3down),
    .i2c_scl(qmc_scl[1]),
    .sda_out(qmc_sda_out[1]),
    .sda_in(qmc_sda_in[1]),
    .sda_dir(qmc_sda_dir[1]),
    .mag_x(mag2_x),
    .mag_y(mag2_y),
    .mag_z(mag2_z),
    .sample_valid(qmc_sample_valid[1]),

    .dbg_state(qmc_dbg_state[1]),
    .dbg_err(qmc_dbg_err[1]),
    .dbg_chip_id(qmc_dbg_chip_id[1]),
    .dbg_chip_id_valid(qmc_dbg_chip_id_valid[1]),
    .dbg_chip_id_ok(qmc_dbg_chip_id_ok[1]),
    .dbg_init_done(qmc_dbg_init_done[1]),
    .dbg_ack_error_latched(qmc_dbg_ack_error_latched[1])
);

qmc5883l_ctrl u_qmc_3 (
    .clk(CLOCK_50),
    .rst_n(key3down),
    .i2c_scl(qmc_scl[2]),
    .sda_out(qmc_sda_out[2]),
    .sda_in(qmc_sda_in[2]),
    .sda_dir(qmc_sda_dir[2]),
    .mag_x(mag3_x),
    .mag_y(mag3_y),
    .mag_z(mag3_z),
    .sample_valid(qmc_sample_valid[2]),

    .dbg_state(qmc_dbg_state[2]),
    .dbg_err(qmc_dbg_err[2]),
    .dbg_chip_id(qmc_dbg_chip_id[2]),
    .dbg_chip_id_valid(qmc_dbg_chip_id_valid[2]),
    .dbg_chip_id_ok(qmc_dbg_chip_id_ok[2]),
    .dbg_init_done(qmc_dbg_init_done[2]),
    .dbg_ack_error_latched(qmc_dbg_ack_error_latched[2])
);

qmc5883l_ctrl u_qmc_4 (
    .clk(CLOCK_50),
    .rst_n(key3down),
    .i2c_scl(qmc_scl[3]),
    .sda_out(qmc_sda_out[3]),
    .sda_in(qmc_sda_in[3]),
    .sda_dir(qmc_sda_dir[3]),
    .mag_x(mag4_x),
    .mag_y(mag4_y),
    .mag_z(mag4_z),
    .sample_valid(qmc_sample_valid[3]),

    .dbg_state(qmc_dbg_state[3]),
    .dbg_err(qmc_dbg_err[3]),
    .dbg_chip_id(qmc_dbg_chip_id[3]),
    .dbg_chip_id_valid(qmc_dbg_chip_id_valid[3]),
    .dbg_chip_id_ok(qmc_dbg_chip_id_ok[3]),
    .dbg_init_done(qmc_dbg_init_done[3]),
    .dbg_ack_error_latched(qmc_dbg_ack_error_latched[3])
);

// =====================================================================
// FPGA-only magnetometer calibration
// =====================================================================
wire               calibration_collecting;
wire               calibration_calculating;
wire               calibration_done;
wire signed [15:0] cal_s1_offset_x, cal_s1_offset_y, cal_s1_offset_z;
wire signed [15:0] cal_s2_offset_x, cal_s2_offset_y, cal_s2_offset_z;
wire signed [15:0] cal_s3_offset_x, cal_s3_offset_y, cal_s3_offset_z;
wire signed [15:0] cal_s4_offset_x, cal_s4_offset_y, cal_s4_offset_z;
wire        [31:0] cal_s1_scale_x_q16, cal_s1_scale_y_q16, cal_s1_scale_z_q16;
wire        [31:0] cal_s2_scale_x_q16, cal_s2_scale_y_q16, cal_s2_scale_z_q16;
wire        [31:0] cal_s3_scale_x_q16, cal_s3_scale_y_q16, cal_s3_scale_z_q16;
wire        [31:0] cal_s4_scale_x_q16, cal_s4_scale_y_q16, cal_s4_scale_z_q16;

wire signed [15:0] cal_mag1_x, cal_mag1_y, cal_mag1_z;
wire signed [15:0] cal_mag2_x, cal_mag2_y, cal_mag2_z;
wire signed [15:0] cal_mag3_x, cal_mag3_y, cal_mag3_z;
wire signed [15:0] cal_mag4_x, cal_mag4_y, cal_mag4_z;
wire signed [31:0] cal_mag1_x_gauss_q16, cal_mag1_y_gauss_q16, cal_mag1_z_gauss_q16;
wire signed [31:0] cal_mag2_x_gauss_q16, cal_mag2_y_gauss_q16, cal_mag2_z_gauss_q16;
wire signed [31:0] cal_mag3_x_gauss_q16, cal_mag3_y_gauss_q16, cal_mag3_z_gauss_q16;
wire signed [31:0] cal_mag4_x_gauss_q16, cal_mag4_y_gauss_q16, cal_mag4_z_gauss_q16;

mag_calibration_manager u_calibration_manager (
    .clk                (CLOCK_50),
    .rst_n              (key3down),
    .start_calibration  (calibration_start_pulse),
    .finish_calibration (calibration_finish_pulse),
    .active_sensor_mask (ACTIVE_SENSOR_MASK),
    .sample_valid       (qmc_sample_valid),

    .s1_x (mag1_x), .s1_y (mag1_y), .s1_z (mag1_z),
    .s2_x (mag2_x), .s2_y (mag2_y), .s2_z (mag2_z),
    .s3_x (mag3_x), .s3_y (mag3_y), .s3_z (mag3_z),
    .s4_x (mag4_x), .s4_y (mag4_y), .s4_z (mag4_z),

    .collecting       (calibration_collecting),
    .calculating      (calibration_calculating),
    .calibration_done (calibration_done),

    .s1_offset_x (cal_s1_offset_x), .s1_offset_y (cal_s1_offset_y),
    .s1_offset_z (cal_s1_offset_z),
    .s2_offset_x (cal_s2_offset_x), .s2_offset_y (cal_s2_offset_y),
    .s2_offset_z (cal_s2_offset_z),
    .s3_offset_x (cal_s3_offset_x), .s3_offset_y (cal_s3_offset_y),
    .s3_offset_z (cal_s3_offset_z),
    .s4_offset_x (cal_s4_offset_x), .s4_offset_y (cal_s4_offset_y),
    .s4_offset_z (cal_s4_offset_z),

    .s1_scale_x_q16 (cal_s1_scale_x_q16), .s1_scale_y_q16 (cal_s1_scale_y_q16),
    .s1_scale_z_q16 (cal_s1_scale_z_q16),
    .s2_scale_x_q16 (cal_s2_scale_x_q16), .s2_scale_y_q16 (cal_s2_scale_y_q16),
    .s2_scale_z_q16 (cal_s2_scale_z_q16),
    .s3_scale_x_q16 (cal_s3_scale_x_q16), .s3_scale_y_q16 (cal_s3_scale_y_q16),
    .s3_scale_z_q16 (cal_s3_scale_z_q16),
    .s4_scale_x_q16 (cal_s4_scale_x_q16), .s4_scale_y_q16 (cal_s4_scale_y_q16),
    .s4_scale_z_q16 (cal_s4_scale_z_q16)
);

mag_calibrator u_calibrator_1 (
    .raw_x (mag1_x), .raw_y (mag1_y), .raw_z (mag1_z),
    .offset_x (cal_s1_offset_x), .offset_y (cal_s1_offset_y), .offset_z (cal_s1_offset_z),
    .scale_x_q16 (cal_s1_scale_x_q16), .scale_y_q16 (cal_s1_scale_y_q16),
    .scale_z_q16 (cal_s1_scale_z_q16),
    .corrected_x (cal_mag1_x), .corrected_y (cal_mag1_y), .corrected_z (cal_mag1_z),
    .corrected_x_gauss_q16 (cal_mag1_x_gauss_q16),
    .corrected_y_gauss_q16 (cal_mag1_y_gauss_q16),
    .corrected_z_gauss_q16 (cal_mag1_z_gauss_q16)
);

mag_calibrator u_calibrator_2 (
    .raw_x (mag2_x), .raw_y (mag2_y), .raw_z (mag2_z),
    .offset_x (cal_s2_offset_x), .offset_y (cal_s2_offset_y), .offset_z (cal_s2_offset_z),
    .scale_x_q16 (cal_s2_scale_x_q16), .scale_y_q16 (cal_s2_scale_y_q16),
    .scale_z_q16 (cal_s2_scale_z_q16),
    .corrected_x (cal_mag2_x), .corrected_y (cal_mag2_y), .corrected_z (cal_mag2_z),
    .corrected_x_gauss_q16 (cal_mag2_x_gauss_q16),
    .corrected_y_gauss_q16 (cal_mag2_y_gauss_q16),
    .corrected_z_gauss_q16 (cal_mag2_z_gauss_q16)
);

mag_calibrator u_calibrator_3 (
    .raw_x (mag3_x), .raw_y (mag3_y), .raw_z (mag3_z),
    .offset_x (cal_s3_offset_x), .offset_y (cal_s3_offset_y), .offset_z (cal_s3_offset_z),
    .scale_x_q16 (cal_s3_scale_x_q16), .scale_y_q16 (cal_s3_scale_y_q16),
    .scale_z_q16 (cal_s3_scale_z_q16),
    .corrected_x (cal_mag3_x), .corrected_y (cal_mag3_y), .corrected_z (cal_mag3_z),
    .corrected_x_gauss_q16 (cal_mag3_x_gauss_q16),
    .corrected_y_gauss_q16 (cal_mag3_y_gauss_q16),
    .corrected_z_gauss_q16 (cal_mag3_z_gauss_q16)
);

mag_calibrator u_calibrator_4 (
    .raw_x (mag4_x), .raw_y (mag4_y), .raw_z (mag4_z),
    .offset_x (cal_s4_offset_x), .offset_y (cal_s4_offset_y), .offset_z (cal_s4_offset_z),
    .scale_x_q16 (cal_s4_scale_x_q16), .scale_y_q16 (cal_s4_scale_y_q16),
    .scale_z_q16 (cal_s4_scale_z_q16),
    .corrected_x (cal_mag4_x), .corrected_y (cal_mag4_y), .corrected_z (cal_mag4_z),
    .corrected_x_gauss_q16 (cal_mag4_x_gauss_q16),
    .corrected_y_gauss_q16 (cal_mag4_y_gauss_q16),
    .corrected_z_gauss_q16 (cal_mag4_z_gauss_q16)
);

// =====================================================================
// QMC5883P bring-up interface
// =====================================================================
// KEY[0]     = restart calibration collection.
// KEY[1]     = finish collection and calculate coefficients.
// SW[4]      = display calibrated values after calculation is complete.
// SW[3:2]    = selected sensor: 00, 01, 10, 11 select sensors 1, 2, 3, 4.
// SW[1:0]    = selected axis: 00, 01, 10 select X, Y, Z.
// LEDG[0]    = all active sensors initialized successfully.
// LEDG[3:1]  = calibration collecting, calculating, done.
// LEDR[17:0] = absolute selected-axis field strength relative to configured range.
logic signed [15:0] selected_mag_x, selected_mag_y, selected_mag_z;
logic signed [15:0] selected_cal_mag_x, selected_cal_mag_y, selected_cal_mag_z;

always_comb begin
    case (SW[3:2])
        2'b00: begin
            selected_mag_x                 = mag1_x;
            selected_mag_y                 = mag1_y;
            selected_mag_z                 = mag1_z;
            selected_cal_mag_x             = cal_mag1_x;
            selected_cal_mag_y             = cal_mag1_y;
            selected_cal_mag_z             = cal_mag1_z;
        end
        2'b01: begin
            selected_mag_x                 = mag2_x;
            selected_mag_y                 = mag2_y;
            selected_mag_z                 = mag2_z;
            selected_cal_mag_x             = cal_mag2_x;
            selected_cal_mag_y             = cal_mag2_y;
            selected_cal_mag_z             = cal_mag2_z;
        end
        2'b10: begin
            selected_mag_x                 = mag3_x;
            selected_mag_y                 = mag3_y;
            selected_mag_z                 = mag3_z;
            selected_cal_mag_x             = cal_mag3_x;
            selected_cal_mag_y             = cal_mag3_y;
            selected_cal_mag_z             = cal_mag3_z;
        end
        default: begin
            selected_mag_x                 = mag4_x;
            selected_mag_y                 = mag4_y;
            selected_mag_z                 = mag4_z;
            selected_cal_mag_x             = cal_mag4_x;
            selected_cal_mag_y             = cal_mag4_y;
            selected_cal_mag_z             = cal_mag4_z;
        end
    endcase
end

assign LEDG[0]   = ((qmc_dbg_init_done & ACTIVE_SENSOR_MASK) ==
                    ACTIVE_SENSOR_MASK);
assign LEDG[1]   = calibration_collecting;
assign LEDG[2]   = calibration_calculating;
assign LEDG[3]   = calibration_done;
assign LEDG[8:4] = 5'd0;

// =====================================================================
// 觀察與驗證機制：SW[3:2] 選擇感測器，SW[1:0] 選擇 X, Y, Z 軸
// =====================================================================
wire use_calibrated_display = SW[4] && calibration_done;
logic signed [15:0] display_mag_data;

always_comb begin
    case (SW[1:0])
        2'b00:   display_mag_data = use_calibrated_display ? selected_cal_mag_x : selected_mag_x;
        2'b01:   display_mag_data = use_calibrated_display ? selected_cal_mag_y : selected_mag_y;
        2'b10:   display_mag_data = use_calibrated_display ? selected_cal_mag_z : selected_mag_z;
        default: display_mag_data = use_calibrated_display ? selected_cal_mag_x : selected_mag_x;
    endcase
end

function automatic [17:0] raw_to_led_bar;
    input signed [15:0] raw_field;
    reg [15:0] field_magnitude;
    reg [20:0] scaled_strength;
    reg [4:0]  led_count;
    begin
        if (raw_field == 16'sh8000)
            field_magnitude = 16'd32768;
        else if (raw_field < 0)
            field_magnitude = -raw_field;
        else
            field_magnitude = raw_field;

        if (field_magnitude == 0) begin
            raw_to_led_bar = 18'd0;
        end else if (field_magnitude >= 16'd32768) begin
            raw_to_led_bar = 18'h3FFFF;
        end else begin
            scaled_strength = field_magnitude * 5'd18 + 15'd32767;
            led_count = scaled_strength >> 15;
            raw_to_led_bar = (18'h1 << led_count) - 1'b1;
        end
    end
endfunction

assign LEDR = raw_to_led_bar($signed(display_mag_data));

// 將選中的 16-bit 原始資料以 16 進位輸出至 HEX0 ~ HEX3 七段顯示器
HexTo7Seg hex_dec_0 (.i_hex(display_mag_data[3:0]),   .o_seg(HEX0));
HexTo7Seg hex_dec_1 (.i_hex(display_mag_data[7:4]),   .o_seg(HEX1));
HexTo7Seg hex_dec_2 (.i_hex(display_mag_data[11:8]),  .o_seg(HEX2));
HexTo7Seg hex_dec_3 (.i_hex(display_mag_data[15:12]), .o_seg(HEX3));
HexTo7Seg hex_sensor_number (.i_hex({2'b00, SW[3:2]} + 4'd1), .o_seg(HEX7));



// =====================================================================
// LCD display for QMC5883P magnetic field values
// =====================================================================

assign LCD_ON   = 1'b1;
assign LCD_BLON = 1'b1;

// Convert 4-bit hex value to ASCII character
function automatic [7:0] hex_to_ascii;
    input [3:0] hex;
    begin
        if (hex < 4'd10)
            hex_to_ascii = 8'h30 + hex;               // 0-9
        else
            hex_to_ascii = 8'h41 + (hex - 4'd10);     // A-F
    end
endfunction

// Convert 16-bit value to four ASCII hex characters
function automatic [31:0] word_to_hex_ascii;
    input [15:0] value;
    begin
        word_to_hex_ascii = {
            hex_to_ascii(value[15:12]),
            hex_to_ascii(value[11:8]),
            hex_to_ascii(value[7:4]),
            hex_to_ascii(value[3:0])
        };
    end
endfunction

// Convert signed 16-bit value to a sign and four hexadecimal magnitude digits.
function automatic [39:0] signed_word_to_hex_ascii;
    input signed [15:0] value;
    reg [15:0] magnitude;
    begin
        if (value < 0) begin
            magnitude = (~value) + 1'b1;
            signed_word_to_hex_ascii = {"-", word_to_hex_ascii(magnitude)};
        end else begin
            magnitude = value;
            signed_word_to_hex_ascii = {"+", word_to_hex_ascii(magnitude)};
        end
    end
endfunction

wire [127:0] lcd_line1;
wire [127:0] lcd_line2;
wire [15:0]  lcd_mag_x = use_calibrated_display ? selected_cal_mag_x : selected_mag_x;
wire [15:0]  lcd_mag_y = use_calibrated_display ? selected_cal_mag_y : selected_mag_y;
wire [15:0]  lcd_mag_z = use_calibrated_display ? selected_cal_mag_z : selected_mag_z;

// Exactly 16 characters:
// "X=+1234 Y=-5678 "
assign lcd_line1 = {
    "X=",
    signed_word_to_hex_ascii(lcd_mag_x),
    " Y=",
    signed_word_to_hex_ascii(lcd_mag_y),
    " "
};

// Exactly 16 characters:
// "Z=+1234         "
assign lcd_line2 = {
    "Z=",
    signed_word_to_hex_ascii(lcd_mag_z),
    "         "
};

lcd_1602_controller u_lcd (
    .clk      (CLOCK_50),
    .rst_n    (key3down),

    .line1    (lcd_line1),
    .line2    (lcd_line2),

    .lcd_data (LCD_DATA),
    .lcd_rs   (LCD_RS),
    .lcd_rw   (LCD_RW),
    .lcd_en   (LCD_EN)
);

// =====================================================================
// RS232 UART stream for all four sensors
// =====================================================================
wire signed [15:0] uart_s1_x =
    use_calibrated_display ? cal_mag1_x : $signed(mag1_x);
wire signed [15:0] uart_s1_y =
    use_calibrated_display ? cal_mag1_y : $signed(mag1_y);
wire signed [15:0] uart_s1_z =
    use_calibrated_display ? cal_mag1_z : $signed(mag1_z);
wire signed [15:0] uart_s2_x =
    use_calibrated_display ? cal_mag2_x : $signed(mag2_x);
wire signed [15:0] uart_s2_y =
    use_calibrated_display ? cal_mag2_y : $signed(mag2_y);
wire signed [15:0] uart_s2_z =
    use_calibrated_display ? cal_mag2_z : $signed(mag2_z);
wire signed [15:0] uart_s3_x =
    use_calibrated_display ? cal_mag3_x : $signed(mag3_x);
wire signed [15:0] uart_s3_y =
    use_calibrated_display ? cal_mag3_y : $signed(mag3_y);
wire signed [15:0] uart_s3_z =
    use_calibrated_display ? cal_mag3_z : $signed(mag3_z);
wire signed [15:0] uart_s4_x =
    use_calibrated_display ? cal_mag4_x : $signed(mag4_x);
wire signed [15:0] uart_s4_y =
    use_calibrated_display ? cal_mag4_y : $signed(mag4_y);
wire signed [15:0] uart_s4_z =
    use_calibrated_display ? cal_mag4_z : $signed(mag4_z);

mag_uart_streamer u_mag_uart_streamer (
    .clk      (CLOCK_50),
    .rst_n    (key3down),
    .s1_x     (uart_s1_x),
    .s1_y     (uart_s1_y),
    .s1_z     (uart_s1_z),
    .s2_x     (uart_s2_x),
    .s2_y     (uart_s2_y),
    .s2_z     (uart_s2_z),
    .s3_x     (uart_s3_x),
    .s3_y     (uart_s3_y),
    .s3_z     (uart_s3_z),
    .s4_x     (uart_s4_x),
    .s4_y     (uart_s4_y),
    .s4_z     (uart_s4_z),
    .uart_txd (UART_TXD)
);

assign UART_CTS = 1'b0;

// =====================================================================
// VGA dashboard: 640x480 @ 60 Hz, four sensor Gauss values and H2 traces
// =====================================================================
wire               vga_pixel_clk;
wire               vga_frame_start;
wire [9:0]         vga_pixel_x;
wire [9:0]         vga_pixel_y;
wire               vga_active_video;
wire               vga_hsync_n;
wire               vga_vsync_n;
wire               vga_text_pixel_on;
wire               vga_graph_axis_pixel_on;
wire               vga_graph_plot_s1_pixel_on;
wire               vga_graph_plot_s2_pixel_on;
wire               vga_graph_plot_s3_pixel_on;
wire               vga_graph_plot_s4_pixel_on;
wire        [31:0] vga_sensor1_magnitude_squared_gauss_q16;
wire        [31:0] vga_sensor2_magnitude_squared_gauss_q16;
wire        [31:0] vga_sensor3_magnitude_squared_gauss_q16;
wire        [31:0] vga_sensor4_magnitude_squared_gauss_q16;
wire signed [15:0] vga_source_x_q10;
wire signed [15:0] vga_source_y_q10;
wire signed [15:0] vga_source_z_q10;
wire               vga_source_valid;
wire signed [15:0] vga_sensor1_x =
    use_calibrated_display ? cal_mag1_x : $signed(mag1_x);
wire signed [15:0] vga_sensor1_y =
    use_calibrated_display ? cal_mag1_y : $signed(mag1_y);
wire signed [15:0] vga_sensor1_z =
    use_calibrated_display ? cal_mag1_z : $signed(mag1_z);
wire signed [15:0] vga_sensor2_x =
    use_calibrated_display ? cal_mag2_x : $signed(mag2_x);
wire signed [15:0] vga_sensor2_y =
    use_calibrated_display ? cal_mag2_y : $signed(mag2_y);
wire signed [15:0] vga_sensor2_z =
    use_calibrated_display ? cal_mag2_z : $signed(mag2_z);
wire signed [15:0] vga_sensor3_x =
    use_calibrated_display ? cal_mag3_x : $signed(mag3_x);
wire signed [15:0] vga_sensor3_y =
    use_calibrated_display ? cal_mag3_y : $signed(mag3_y);
wire signed [15:0] vga_sensor3_z =
    use_calibrated_display ? cal_mag3_z : $signed(mag3_z);
wire signed [15:0] vga_sensor4_x =
    use_calibrated_display ? cal_mag4_x : $signed(mag4_x);
wire signed [15:0] vga_sensor4_y =
    use_calibrated_display ? cal_mag4_y : $signed(mag4_y);
wire signed [15:0] vga_sensor4_z =
    use_calibrated_display ? cal_mag4_z : $signed(mag4_z);

mag_magnitude_squared u_sensor1_magnitude_squared (
    .field_x_counts                (vga_sensor1_x),
    .field_y_counts                (vga_sensor1_y),
    .field_z_counts                (vga_sensor1_z),
    .magnitude_squared_gauss_q16   (vga_sensor1_magnitude_squared_gauss_q16)
);

mag_magnitude_squared u_sensor2_magnitude_squared (
    .field_x_counts                (vga_sensor2_x),
    .field_y_counts                (vga_sensor2_y),
    .field_z_counts                (vga_sensor2_z),
    .magnitude_squared_gauss_q16   (vga_sensor2_magnitude_squared_gauss_q16)
);

mag_magnitude_squared u_sensor3_magnitude_squared (
    .field_x_counts                (vga_sensor3_x),
    .field_y_counts                (vga_sensor3_y),
    .field_z_counts                (vga_sensor3_z),
    .magnitude_squared_gauss_q16   (vga_sensor3_magnitude_squared_gauss_q16)
);

mag_magnitude_squared u_sensor4_magnitude_squared (
    .field_x_counts                (vga_sensor4_x),
    .field_y_counts                (vga_sensor4_y),
    .field_z_counts                (vga_sensor4_z),
    .magnitude_squared_gauss_q16   (vga_sensor4_magnitude_squared_gauss_q16)
);

mag_source_tracker u_mag_source_tracker (
    .clk                     (CLOCK_50),
    .rst_n                   (key3down),
    .update                  (vga_frame_start),
    .sensor1_h2_gauss_q16    (vga_sensor1_magnitude_squared_gauss_q16),
    .sensor2_h2_gauss_q16    (vga_sensor2_magnitude_squared_gauss_q16),
    .sensor3_h2_gauss_q16    (vga_sensor3_magnitude_squared_gauss_q16),
    .sensor4_h2_gauss_q16    (vga_sensor4_magnitude_squared_gauss_q16),
    .source_x_q10            (vga_source_x_q10),
    .source_y_q10            (vga_source_y_q10),
    .source_z_q10            (vga_source_z_q10),
    .source_valid            (vga_source_valid)
);

vga_timing_640x480 u_vga_timing (
    .clk_50       (CLOCK_50),
    .rst_n        (key3down),
    .pixel_clk    (vga_pixel_clk),
    .frame_start  (vga_frame_start),
    .pixel_x      (vga_pixel_x),
    .pixel_y      (vga_pixel_y),
    .active_video (vga_active_video),
    .hsync_n      (vga_hsync_n),
    .vsync_n      (vga_vsync_n)
);

vga_four_sensor_dashboard u_vga_dashboard (
    .clk                     (CLOCK_50),
    .rst_n                   (key3down),
    .frame_start             (vga_frame_start),
    .active_video            (vga_active_video),
    .pixel_x                 (vga_pixel_x),
    .pixel_y                 (vga_pixel_y),
    .sensor1_x               (vga_sensor1_x),
    .sensor1_y               (vga_sensor1_y),
    .sensor1_z               (vga_sensor1_z),
    .sensor2_x               (vga_sensor2_x),
    .sensor2_y               (vga_sensor2_y),
    .sensor2_z               (vga_sensor2_z),
    .sensor3_x               (vga_sensor3_x),
    .sensor3_y               (vga_sensor3_y),
    .sensor3_z               (vga_sensor3_z),
    .sensor4_x               (vga_sensor4_x),
    .sensor4_y               (vga_sensor4_y),
    .sensor4_z               (vga_sensor4_z),
    .sensor1_h2_gauss_q16    (vga_sensor1_magnitude_squared_gauss_q16),
    .sensor2_h2_gauss_q16    (vga_sensor2_magnitude_squared_gauss_q16),
    .sensor3_h2_gauss_q16    (vga_sensor3_magnitude_squared_gauss_q16),
    .sensor4_h2_gauss_q16    (vga_sensor4_magnitude_squared_gauss_q16),
    .source_x_q10            (vga_source_x_q10),
    .source_y_q10            (vga_source_y_q10),
    .source_z_q10            (vga_source_z_q10),
    .source_valid            (vga_source_valid),
    .calibrated_mode         (use_calibrated_display),
    .calibration_collecting  (calibration_collecting),
    .calibration_calculating (calibration_calculating),
    .calibration_done        (calibration_done),
    .text_pixel_on           (vga_text_pixel_on),
    .graph_axis_pixel_on     (vga_graph_axis_pixel_on),
    .graph_plot_s1_pixel_on  (vga_graph_plot_s1_pixel_on),
    .graph_plot_s2_pixel_on  (vga_graph_plot_s2_pixel_on),
    .graph_plot_s3_pixel_on  (vga_graph_plot_s3_pixel_on),
    .graph_plot_s4_pixel_on  (vga_graph_plot_s4_pixel_on)
);

assign VGA_CLK     = vga_pixel_clk;
assign VGA_HS      = vga_hsync_n;
assign VGA_VS      = vga_vsync_n;
assign VGA_BLANK_N = vga_active_video;
assign VGA_SYNC_N  = 1'b0;
assign VGA_R       = vga_text_pixel_on       ? 8'hFF :
                     vga_graph_plot_s2_pixel_on ? 8'hFF :
                     vga_graph_plot_s4_pixel_on ? 8'hFF :
                     vga_graph_axis_pixel_on ? 8'h60 : 8'h00;
assign VGA_G       = vga_text_pixel_on       ? 8'hFF :
                     vga_graph_plot_s1_pixel_on ? 8'hFF :
                     vga_graph_plot_s4_pixel_on ? 8'hFF :
                     vga_graph_axis_pixel_on ? 8'h60 : 8'h00;
assign VGA_B       = vga_text_pixel_on       ? 8'hFF :
                     vga_graph_plot_s3_pixel_on ? 8'hFF :
                     vga_graph_axis_pixel_on ? 8'h60 : 8'h00;

endmodule
