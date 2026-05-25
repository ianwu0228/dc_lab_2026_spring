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

logic key3down;


Debounce deb3(
    .i_in(KEY[3]),
    .i_rst_n(1'b1),
    .i_clk(CLOCK_50),
    .o_debounced(key3down)
);

assign HEX4 = '1;
assign HEX5 = '1;
assign HEX6 = '1;
assign HEX7 = '1;

// =====================================================================
// QMC5883P 磁力計硬體連接與三態緩衝器實作
// =====================================================================

// --- 磁力計 1 內部訊號與三態控制 (使用 GPIO[0] 與 GPIO[1]) ---
wire        qmc1_scl;
wire        qmc1_sda_out;
wire        qmc1_sda_in;
wire        qmc1_sda_dir;
wire [15:0] mag1_x, mag1_y, mag1_z;
wire [4:0]  qmc1_dbg_state;
wire        qmc1_dbg_err;
wire [7:0]  qmc1_dbg_chip_id;
wire        qmc1_dbg_chip_id_valid;
wire        qmc1_dbg_chip_id_ok;
wire        qmc1_dbg_init_done;
wire        qmc1_dbg_ack_error_latched;

assign GPIO[0] = qmc1_scl;
assign GPIO[1] = (qmc1_sda_dir && (qmc1_sda_out == 1'b0)) ? 1'b0 : 1'bz; // I2C SDA open-drain: drive low only; release for high/ACK/read
assign qmc1_sda_in = GPIO[1];


// =====================================================================
// 實體化控制大腦 (完全平行獨立運作)
// =====================================================================

// 磁力計 1 控制器：module name remains qmc5883l_ctrl, but the file is now fixed for QMC5883P.
qmc5883l_ctrl u_qmc_1 (
    .clk(CLOCK_50),       
    .rst_n(key3down),       
    .i2c_scl(qmc1_scl),
    .sda_out(qmc1_sda_out),
    .sda_in(qmc1_sda_in),
    .sda_dir(qmc1_sda_dir),
    .mag_x(mag1_x),
    .mag_y(mag1_y),
    .mag_z(mag1_z),
    
    .dbg_state(qmc1_dbg_state),
    .dbg_err(qmc1_dbg_err),
    .dbg_chip_id(qmc1_dbg_chip_id),
    .dbg_chip_id_valid(qmc1_dbg_chip_id_valid),
    .dbg_chip_id_ok(qmc1_dbg_chip_id_ok),
    .dbg_init_done(qmc1_dbg_init_done),
    .dbg_ack_error_latched(qmc1_dbg_ack_error_latched)
);

// =====================================================================
// Debug LED mapping for QMC5883P bring-up
// =====================================================================
// LEDR[7:0]   = chip ID value. Expected QMC5883P chip ID is 8'h80, so LEDR[7] should be ON.
// LEDR[8]     = chip ID has been read at least once.
// LEDR[9]     = chip ID equals 8'h80.
// LEDR[14:10] = controller FSM state[4:0].
// LEDR[15]    = initialization completed: CONTROL1 and CONTROL2 were written.
// LEDR[16]    = live ACK error pulse from I2C master.
// LEDR[17]    = latched ACK error; stays ON after any ACK error until successful chip ID.
assign LEDR[7:0]   = qmc1_dbg_chip_id;
assign LEDR[8]     = qmc1_dbg_chip_id_valid;
assign LEDR[9]     = qmc1_dbg_chip_id_ok;
assign LEDR[14:10] = qmc1_dbg_state;
assign LEDR[15]    = qmc1_dbg_init_done;
assign LEDR[16]    = qmc1_dbg_err;
assign LEDR[17]    = qmc1_dbg_ack_error_latched;

// =====================================================================
// 觀察與驗證機制：單顆感測器測試，利用 SW[1:0] 切換顯示 X, Y, Z 軸的數值
// =====================================================================
logic [15:0] display_mag_data;

always_comb begin
    case (SW[1:0])
        2'b00:   display_mag_data = mag1_x; // SW[1:0] 為 00 時：顯示 X 軸磁場
        2'b01:   display_mag_data = mag1_y; // SW[1:0] 為 01 時：顯示 Y 軸磁場
        2'b10:   display_mag_data = mag1_z; // SW[1:0] 為 10 時：顯示 Z 軸磁場
        default: display_mag_data = mag1_x;
    endcase
end

// 將選中的 16-bit 原始資料以 16 進位輸出至 HEX0 ~ HEX3 七段顯示器
HexTo7Seg hex_dec_0 (.i_hex(display_mag_data[3:0]),   .o_seg(HEX0));
HexTo7Seg hex_dec_1 (.i_hex(display_mag_data[7:4]),   .o_seg(HEX1));
HexTo7Seg hex_dec_2 (.i_hex(display_mag_data[11:8]),  .o_seg(HEX2));
HexTo7Seg hex_dec_3 (.i_hex(display_mag_data[15:12]), .o_seg(HEX3));



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

wire [127:0] lcd_line1;
wire [127:0] lcd_line2;
wire [31:0]  chip_id_ascii;

assign chip_id_ascii = word_to_hex_ascii({8'h00, qmc1_dbg_chip_id});

// Exactly 16 characters:
// "X=1234 Y=5678   "
assign lcd_line1 = {
    "X=",
    word_to_hex_ascii(mag1_x),
    " Y=",
    word_to_hex_ascii(mag1_y),
    "   "
};

// Exactly 16 characters:
// "Z=1234 ID=80  "
assign lcd_line2 = {
    "Z=",
    word_to_hex_ascii(mag1_z),
    " ID=",
    chip_id_ascii[15:0],
    "  "
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

endmodule