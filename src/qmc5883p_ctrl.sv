module qmc5883l_ctrl (
    input  wire        clk,       // 50 MHz system clock
    input  wire        rst_n,     // Active-low reset

    // I2C physical lines — connect to top-level tri-state logic
    output wire        i2c_scl,
    output wire        sda_out,
    input  wire        sda_in,
    output wire        sda_dir,

    // Decoded 16-bit magnetic field output, two's complement, LSB first
    output reg  [15:0] mag_x,
    output reg  [15:0] mag_y,
    output reg  [15:0] mag_z,
    output reg         sample_valid,

    // Debug outputs — connect to LEDs or SignalTap in Quartus
    output wire [4:0]  dbg_state,
    output wire        dbg_err,
    output wire [7:0]  dbg_chip_id,
    output wire        dbg_chip_id_valid,
    output wire        dbg_chip_id_ok,
    output wire        dbg_init_done,
    output reg         dbg_ack_error_latched
);

    // =========================================================================
    // i2c_master interface signals
    // =========================================================================
    reg  [2:0] i2c_cmd;
    reg        i2c_go;
    reg  [7:0] i2c_tx_data;
    wire [7:0] i2c_rx_data;
    wire       i2c_done;
    wire       i2c_ack_err;

    i2c_master u_i2c (
        .clk_50m  (clk),
        .rst_n    (rst_n),
        .cmd      (i2c_cmd),
        .go       (i2c_go),
        .tx_data  (i2c_tx_data),
        .rx_data  (i2c_rx_data),
        .done     (i2c_done),
        .ack_err  (i2c_ack_err),
        .i2c_scl  (i2c_scl),
        .sda_out  (sda_out),
        .sda_in   (sda_in),
        .sda_dir  (sda_dir)
    );

    // =========================================================================
    // I2C command aliases. Must match i2c_master.sv.
    // =========================================================================
    localparam CMD_IDLE  = 3'd0;
    localparam CMD_START = 3'd1;
    localparam CMD_WRITE = 3'd2;
    localparam CMD_RACK  = 3'd3;  // read byte, send ACK: more bytes follow
    localparam CMD_RNACK = 3'd4;  // read byte, send NACK: last byte
    localparam CMD_STOP  = 3'd5;

    // =========================================================================
    // QMC5883P address and registers
    // =========================================================================
    localparam QMC_ADDR_W     = 8'h58;  // 7-bit address 0x2C, write byte
    localparam QMC_ADDR_R     = 8'h59;  // 7-bit address 0x2C, read byte

    localparam REG_CHIP_ID    = 8'h00;
    localparam REG_DATA_START = 8'h01;  // X_LSB, then X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    localparam REG_CTRL1      = 8'h0A;
    localparam REG_CTRL2      = 8'h0B;

    localparam CHIP_ID_EXPECT = 8'h80;

    // // CONTROL1 = 0x1F:
    // // bits [1:0] mode = 3 continuous
    // // bits [3:2] ODR  = 3 200 Hz
    // // bits [5:4] OSR  = 1 OSR=4
    // // bits [7:6] DSR  = 0 DSR=1
    // localparam CTRL1_VALUE    = 8'h1F;

    // // CONTROL2 = 0x0C:
    // // bits [1:0] set/reset = 0 set/reset on
    // // bits [3:2] range     = 3 +-2G
    // localparam CTRL2_VALUE    = 8'h0C;

    // ====================================================================================
    // ============================================================
    // QMC5883P user configuration
    // ============================================================

    // --------------------
    // MODE bits [1:0]
    // --------------------
    localparam [1:0] QMC_MODE_SUSPEND    = 2'b00;
    localparam [1:0] QMC_MODE_NORMAL     = 2'b01;
    localparam [1:0] QMC_MODE_SINGLE     = 2'b10;
    localparam [1:0] QMC_MODE_CONTINUOUS = 2'b11;

    // --------------------
    // ODR bits [3:2]
    // --------------------
    localparam [1:0] QMC_ODR_10HZ  = 2'b00;
    localparam [1:0] QMC_ODR_50HZ  = 2'b01;
    localparam [1:0] QMC_ODR_100HZ = 2'b10;
    localparam [1:0] QMC_ODR_200HZ = 2'b11;

    // --------------------
    // OSR bits [5:4]
    // --------------------
    localparam [1:0] QMC_OSR_8 = 2'b00;
    localparam [1:0] QMC_OSR_4 = 2'b01;
    localparam [1:0] QMC_OSR_2 = 2'b10;
    localparam [1:0] QMC_OSR_1 = 2'b11;

    // --------------------
    // DSR bits [7:6]
    // --------------------
    localparam [1:0] QMC_DSR_1 = 2'b00;
    localparam [1:0] QMC_DSR_2 = 2'b01;
    localparam [1:0] QMC_DSR_4 = 2'b10;
    localparam [1:0] QMC_DSR_8 = 2'b11;

    // --------------------
    // RANGE bits [3:2] of CTRL2
    // --------------------
    localparam [1:0] QMC_RANGE_30G = 2'b00;
    localparam [1:0] QMC_RANGE_12G = 2'b01;
    localparam [1:0] QMC_RANGE_8G  = 2'b10;
    localparam [1:0] QMC_RANGE_2G  = 2'b11;

    // --------------------
    // SET/RESET bits [1:0] of CTRL2
    // --------------------
    localparam [1:0] QMC_SETRESET_ON      = 2'b00;
    localparam [1:0] QMC_SETRESET_SETONLY = 2'b01;
    localparam [1:0] QMC_SETRESET_OFF     = 2'b10;

    // ============================================================
    // Select desired sensor configuration here
    // ============================================================

    localparam [1:0] CFG_MODE     = QMC_MODE_CONTINUOUS;
    localparam [1:0] CFG_ODR      = QMC_ODR_200HZ;
    localparam [1:0] CFG_OSR      = QMC_OSR_4;
    localparam [1:0] CFG_DSR      = QMC_DSR_1;
    localparam [1:0] CFG_RANGE    = QMC_RANGE_2G;
    localparam [1:0] CFG_SETRESET = QMC_SETRESET_ON;

    localparam [7:0] CTRL1_VALUE = {
        CFG_DSR,      // bits [7:6]
        CFG_OSR,      // bits [5:4]
        CFG_ODR,      // bits [3:2]
        CFG_MODE      // bits [1:0]
    };

    localparam [7:0] CTRL2_VALUE = {
        4'b0000,       // bits [7:4], unused/reserved
        CFG_RANGE,     // bits [3:2]
        CFG_SETRESET   // bits [1:0]
    };

    // 20 ms at 50 MHz = 1,000,000 cycles
    localparam DELAY_20MS = 20'd1_000_000;
    reg [19:0] delay_cnt;

    // Read exactly once per configured 200 Hz sensor output period.  Reading
    // faster can return duplicate samples, which corrupts frequency analysis.
    localparam SAMPLE_PERIOD_CYCLES = 18'd250_000;
    reg [17:0] sample_period_cnt;
    reg        sample_due;

    // =========================================================================
    // FSM state encoding
    // =========================================================================
    localparam [5:0]
        // Read chip ID first. Expected value is 0x80.
        S_CHIP_START      = 6'd0,
        S_CHIP_ADDRW      = 6'd1,
        S_CHIP_REG        = 6'd2,
        S_CHIP_RSTART     = 6'd3,
        S_CHIP_ADDRR      = 6'd4,
        S_CHIP_READ       = 6'd5,
        S_CHIP_STOP       = 6'd6,
        S_CHIP_CHECK      = 6'd7,

        // Write CONTROL1: register 0x0A = 0x1F
        S_INIT1_START     = 6'd8,
        S_INIT1_ADDR      = 6'd9,
        S_INIT1_REG       = 6'd10,
        S_INIT1_VAL       = 6'd11,
        S_INIT1_STOP      = 6'd12,
        S_INIT1_WAIT      = 6'd13,

        // Write CONTROL2: register 0x0B = 0x0C
        S_INIT2_START     = 6'd14,
        S_INIT2_ADDR      = 6'd15,
        S_INIT2_REG       = 6'd16,
        S_INIT2_VAL       = 6'd17,
        S_INIT2_STOP      = 6'd18,
        S_INIT2_WAIT      = 6'd19,

        // Wait before first reading
        S_WAIT_20MS       = 6'd20,

        // Continuous 6-byte data read from register 0x01
        S_RD_START        = 6'd21,
        S_RD_ADDRW        = 6'd22,
        S_RD_REG          = 6'd23,
        S_RD_RSTART       = 6'd24,
        S_RD_ADDRR        = 6'd25,
        S_RD_XLSB_GO      = 6'd26,
        S_RD_XLSB_LAT     = 6'd27,
        S_RD_XMSB_LAT     = 6'd28,
        S_RD_YLSB_LAT     = 6'd29,
        S_RD_YMSB_LAT     = 6'd30,
        S_RD_ZLSB_LAT     = 6'd31,
        S_RD_ZMSB_LAT     = 6'd32,
        S_ASSEMBLE        = 6'd33,

        S_ERROR_RETRY     = 6'd34,
        S_WAIT_SAMPLE     = 6'd35;

    reg [5:0] state;

    // Raw byte buffers
    reg [7:0] chip_id;
    reg       chip_id_valid;
    reg       init_done;
    reg [7:0] buf_xl, buf_xh;
    reg [7:0] buf_yl, buf_yh;
    reg [7:0] buf_zl, buf_zh;

    // =========================================================================
    // Main FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_CHIP_START;
            i2c_go      <= 1'b0;
            i2c_cmd     <= CMD_IDLE;
            i2c_tx_data <= 8'd0;
            delay_cnt   <= 20'd0;
            sample_period_cnt <= 18'd0;
            sample_due  <= 1'b0;
            chip_id     <= 8'd0;
            chip_id_valid <= 1'b0;
            init_done   <= 1'b0;
            dbg_ack_error_latched <= 1'b0;
            mag_x       <= 16'd0;
            mag_y       <= 16'd0;
            mag_z       <= 16'd0;
            sample_valid <= 1'b0;
            buf_xl      <= 8'd0;
            buf_xh      <= 8'd0;
            buf_yl      <= 8'd0;
            buf_yh      <= 8'd0;
            buf_zl      <= 8'd0;
            buf_zh      <= 8'd0;
        end else begin
            // Default: do not start a new I2C command unless a state below requests it.
            i2c_go <= 1'b0;
            sample_valid <= 1'b0;

            if (init_done) begin
                if (sample_period_cnt == SAMPLE_PERIOD_CYCLES - 1'b1) begin
                    sample_period_cnt <= 18'd0;
                    sample_due <= 1'b1;
                end else begin
                    sample_period_cnt <= sample_period_cnt + 1'b1;
                end
            end else begin
                sample_period_cnt <= 18'd0;
                sample_due <= 1'b0;
            end

            // If any write/address byte gets NACKed, retry from chip-ID read.
            if (i2c_done && i2c_ack_err) begin
                dbg_ack_error_latched <= 1'b1;
                init_done <= 1'b0;
                state <= S_ERROR_RETRY;
            end else begin
                case (state)
                    // ---------------------------------------------------------
                    // Read QMC5883P chip ID: START, 0x58, 0x00, RSTART, 0x59, read, STOP
                    // ---------------------------------------------------------
                    S_CHIP_START: begin
                        i2c_cmd <= CMD_START;
                        i2c_go  <= 1'b1;
                        state   <= S_CHIP_ADDRW;
                    end

                    S_CHIP_ADDRW: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= QMC_ADDR_W;
                            i2c_go      <= 1'b1;
                            state       <= S_CHIP_REG;
                        end
                    end

                    S_CHIP_REG: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= REG_CHIP_ID;
                            i2c_go      <= 1'b1;
                            state       <= S_CHIP_RSTART;
                        end
                    end

                    S_CHIP_RSTART: begin
                        if (i2c_done) begin
                            i2c_cmd <= CMD_START;   // repeated START
                            i2c_go  <= 1'b1;
                            state   <= S_CHIP_ADDRR;
                        end
                    end

                    S_CHIP_ADDRR: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= QMC_ADDR_R;
                            i2c_go      <= 1'b1;
                            state       <= S_CHIP_READ;
                        end
                    end

                    S_CHIP_READ: begin
                        if (i2c_done) begin
                            i2c_cmd <= CMD_RNACK;   // only one byte, so NACK after read
                            i2c_go  <= 1'b1;
                            state   <= S_CHIP_STOP;
                        end
                    end

                    S_CHIP_STOP: begin
                        if (i2c_done) begin
                            chip_id <= i2c_rx_data;
                            chip_id_valid <= 1'b1;
                            i2c_cmd <= CMD_STOP;
                            i2c_go  <= 1'b1;
                            state   <= S_CHIP_CHECK;
                        end
                    end

                    S_CHIP_CHECK: begin
                        if (i2c_done) begin
                            if (chip_id == CHIP_ID_EXPECT) begin
                                dbg_ack_error_latched <= 1'b0;
                                state <= S_INIT1_START;
                            end else begin
                                state <= S_ERROR_RETRY;
                            end
                        end
                    end

                    // ---------------------------------------------------------
                    // Write CONTROL1: 0x0A = 0x1F
                    // ---------------------------------------------------------
                    S_INIT1_START: begin
                        i2c_cmd <= CMD_START;
                        i2c_go  <= 1'b1;
                        state   <= S_INIT1_ADDR;
                    end

                    S_INIT1_ADDR: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= QMC_ADDR_W;
                            i2c_go      <= 1'b1;
                            state       <= S_INIT1_REG;
                        end
                    end

                    S_INIT1_REG: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= REG_CTRL1;
                            i2c_go      <= 1'b1;
                            state       <= S_INIT1_VAL;
                        end
                    end

                    S_INIT1_VAL: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= CTRL1_VALUE;
                            i2c_go      <= 1'b1;
                            state       <= S_INIT1_STOP;
                        end
                    end

                    S_INIT1_STOP: begin
                        if (i2c_done) begin
                            i2c_cmd <= CMD_STOP;
                            i2c_go  <= 1'b1;
                            state   <= S_INIT1_WAIT;
                        end
                    end

                    S_INIT1_WAIT: begin
                        if (i2c_done) begin
                            state <= S_INIT2_START;
                        end
                    end

                    // ---------------------------------------------------------
                    // Write CONTROL2: 0x0B = 0x0C
                    // ---------------------------------------------------------
                    S_INIT2_START: begin
                        i2c_cmd <= CMD_START;
                        i2c_go  <= 1'b1;
                        state   <= S_INIT2_ADDR;
                    end

                    S_INIT2_ADDR: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= QMC_ADDR_W;
                            i2c_go      <= 1'b1;
                            state       <= S_INIT2_REG;
                        end
                    end

                    S_INIT2_REG: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= REG_CTRL2;
                            i2c_go      <= 1'b1;
                            state       <= S_INIT2_VAL;
                        end
                    end

                    S_INIT2_VAL: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= CTRL2_VALUE;
                            i2c_go      <= 1'b1;
                            state       <= S_INIT2_STOP;
                        end
                    end

                    S_INIT2_STOP: begin
                        if (i2c_done) begin
                            i2c_cmd <= CMD_STOP;
                            i2c_go  <= 1'b1;
                            state   <= S_INIT2_WAIT;
                        end
                    end

                    S_INIT2_WAIT: begin
                        if (i2c_done) begin
                            init_done <= 1'b1;
                            delay_cnt <= 20'd0;
                            state     <= S_WAIT_20MS;
                        end
                    end

                    S_WAIT_20MS: begin
                        if (delay_cnt == DELAY_20MS) begin
                            state <= S_RD_START;
                        end else begin
                            delay_cnt <= delay_cnt + 1'b1;
                        end
                    end

                    // ---------------------------------------------------------
                    // Read six magnetic data bytes from register 0x01
                    // Sequence: XL, XH, YL, YH, ZL, ZH
                    // ---------------------------------------------------------
                    S_RD_START: begin
                        sample_due <= 1'b0;
                        i2c_cmd <= CMD_START;
                        i2c_go  <= 1'b1;
                        state   <= S_RD_ADDRW;
                    end

                    S_RD_ADDRW: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= QMC_ADDR_W;
                            i2c_go      <= 1'b1;
                            state       <= S_RD_REG;
                        end
                    end

                    S_RD_REG: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= REG_DATA_START;
                            i2c_go      <= 1'b1;
                            state       <= S_RD_RSTART;
                        end
                    end

                    S_RD_RSTART: begin
                        if (i2c_done) begin
                            i2c_cmd <= CMD_START;   // repeated START
                            i2c_go  <= 1'b1;
                            state   <= S_RD_ADDRR;
                        end
                    end

                    S_RD_ADDRR: begin
                        if (i2c_done) begin
                            i2c_cmd     <= CMD_WRITE;
                            i2c_tx_data <= QMC_ADDR_R;
                            i2c_go      <= 1'b1;
                            state       <= S_RD_XLSB_GO;
                        end
                    end

                    S_RD_XLSB_GO: begin
                        if (i2c_done) begin
                            i2c_cmd <= CMD_RACK;
                            i2c_go  <= 1'b1;
                            state   <= S_RD_XLSB_LAT;
                        end
                    end

                    S_RD_XLSB_LAT: begin
                        if (i2c_done) begin
                            buf_xl  <= i2c_rx_data;
                            i2c_cmd <= CMD_RACK;
                            i2c_go  <= 1'b1;
                            state   <= S_RD_XMSB_LAT;
                        end
                    end

                    S_RD_XMSB_LAT: begin
                        if (i2c_done) begin
                            buf_xh  <= i2c_rx_data;
                            i2c_cmd <= CMD_RACK;
                            i2c_go  <= 1'b1;
                            state   <= S_RD_YLSB_LAT;
                        end
                    end

                    S_RD_YLSB_LAT: begin
                        if (i2c_done) begin
                            buf_yl  <= i2c_rx_data;
                            i2c_cmd <= CMD_RACK;
                            i2c_go  <= 1'b1;
                            state   <= S_RD_YMSB_LAT;
                        end
                    end

                    S_RD_YMSB_LAT: begin
                        if (i2c_done) begin
                            buf_yh  <= i2c_rx_data;
                            i2c_cmd <= CMD_RACK;
                            i2c_go  <= 1'b1;
                            state   <= S_RD_ZLSB_LAT;
                        end
                    end

                    S_RD_ZLSB_LAT: begin
                        if (i2c_done) begin
                            buf_zl  <= i2c_rx_data;
                            i2c_cmd <= CMD_RNACK;   // last byte is next, so NACK after Z_MSB
                            i2c_go  <= 1'b1;
                            state   <= S_RD_ZMSB_LAT;
                        end
                    end

                    S_RD_ZMSB_LAT: begin
                        if (i2c_done) begin
                            buf_zh  <= i2c_rx_data;
                            i2c_cmd <= CMD_STOP;
                            i2c_go  <= 1'b1;
                            state   <= S_ASSEMBLE;
                        end
                    end

                    S_ASSEMBLE: begin
                        if (i2c_done) begin
                            mag_x <= {buf_xh, buf_xl};
                            mag_y <= {buf_yh, buf_yl};
                            mag_z <= {buf_zh, buf_zl};
                            sample_valid <= 1'b1;
                            state <= S_WAIT_SAMPLE;
                        end
                    end

                    S_WAIT_SAMPLE: begin
                        if (sample_due)
                            state <= S_RD_START;
                    end

                    S_ERROR_RETRY: begin
                        // Small delay before retrying, useful when the sensor is still powering up.
                        if (delay_cnt == DELAY_20MS) begin
                            delay_cnt <= 20'd0;
                            state     <= S_CHIP_START;
                        end else begin
                            delay_cnt <= delay_cnt + 1'b1;
                        end
                    end

                    default: begin
                        state <= S_CHIP_START;
                    end
                endcase
            end
        end
    end

    assign dbg_state         = state[4:0];
    assign dbg_err           = i2c_ack_err;
    assign dbg_chip_id       = chip_id;
    assign dbg_chip_id_valid = chip_id_valid;
    assign dbg_chip_id_ok    = chip_id_valid && (chip_id == CHIP_ID_EXPECT);
    assign dbg_init_done     = init_done;

endmodule
