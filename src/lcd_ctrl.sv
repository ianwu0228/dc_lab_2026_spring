module lcd_1602_controller (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [127:0] line1,   // 16 ASCII characters
    input  wire [127:0] line2,   // 16 ASCII characters

    output reg  [7:0]  lcd_data,
    output reg         lcd_rs,
    output reg         lcd_rw,
    output reg         lcd_en
);

    // ============================================================
    // LCD command/data writer for DE2-115 16x2 LCD
    // 50 MHz clock assumed
    // ============================================================

    localparam S_POWER_WAIT  = 5'd0;
    localparam S_INIT_1      = 5'd1;
    localparam S_INIT_2      = 5'd2;
    localparam S_INIT_3      = 5'd3;
    localparam S_INIT_4      = 5'd4;
    localparam S_INIT_5      = 5'd5;
    localparam S_SET_LINE1   = 5'd6;
    localparam S_WRITE_LINE1 = 5'd7;
    localparam S_SET_LINE2   = 5'd8;
    localparam S_WRITE_LINE2 = 5'd9;
    localparam S_REFRESH     = 5'd10;
    localparam S_PULSE_HIGH  = 5'd11;
    localparam S_PULSE_LOW   = 5'd12;

    reg [4:0]  state;
    reg [4:0]  return_state;
    reg [31:0] delay_cnt;
    reg [4:0]  char_index;

    reg [7:0] current_byte;
    reg       current_rs;

    // ------------------------------------------------------------
    // Extract one ASCII character from a 16-character packed line.
    // line[127:120] = character 0
    // line[119:112] = character 1
    // ...
    // ------------------------------------------------------------
    function [7:0] get_char;
        input [127:0] line;
        input [4:0] index;
        begin
            get_char = line[127 - index * 8 -: 8];
        end
    endfunction

    // ------------------------------------------------------------
    // Main LCD FSM
    // ------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_POWER_WAIT;
            return_state <= S_POWER_WAIT;
            delay_cnt    <= 32'd0;
            char_index   <= 5'd0;

            lcd_data     <= 8'h00;
            lcd_rs       <= 1'b0;
            lcd_rw       <= 1'b0;
            lcd_en       <= 1'b0;

            current_byte <= 8'h00;
            current_rs   <= 1'b0;
        end else begin
            case (state)

                // Wait about 20 ms after reset
                S_POWER_WAIT: begin
                    lcd_en <= 1'b0;
                    lcd_rw <= 1'b0;

                    if (delay_cnt < 32'd1_000_000) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end else begin
                        delay_cnt <= 32'd0;
                        state <= S_INIT_1;
                    end
                end

                // Function set: 8-bit, 2-line, 5x8 dots
                S_INIT_1: begin
                    current_byte <= 8'h38;
                    current_rs   <= 1'b0;
                    return_state <= S_INIT_2;
                    delay_cnt    <= 32'd0;
                    state        <= S_PULSE_HIGH;
                end

                // Display ON, cursor OFF
                S_INIT_2: begin
                    current_byte <= 8'h0C;
                    current_rs   <= 1'b0;
                    return_state <= S_INIT_3;
                    delay_cnt    <= 32'd0;
                    state        <= S_PULSE_HIGH;
                end

                // Clear display
                S_INIT_3: begin
                    current_byte <= 8'h01;
                    current_rs   <= 1'b0;
                    return_state <= S_INIT_4;
                    delay_cnt    <= 32'd0;
                    state        <= S_PULSE_HIGH;
                end

                // Wait after clear display
                S_INIT_4: begin
                    if (delay_cnt < 32'd100_000) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end else begin
                        delay_cnt <= 32'd0;
                        state <= S_INIT_5;
                    end
                end

                // Entry mode: cursor increment
                S_INIT_5: begin
                    current_byte <= 8'h06;
                    current_rs   <= 1'b0;
                    return_state <= S_SET_LINE1;
                    delay_cnt    <= 32'd0;
                    state        <= S_PULSE_HIGH;
                end

                // Set LCD cursor to line 1
                S_SET_LINE1: begin
                    char_index   <= 5'd0;
                    current_byte <= 8'h80;
                    current_rs   <= 1'b0;
                    return_state <= S_WRITE_LINE1;
                    delay_cnt    <= 32'd0;
                    state        <= S_PULSE_HIGH;
                end

                // Write 16 characters to line 1
                S_WRITE_LINE1: begin
                    if (char_index < 5'd16) begin
                        current_byte <= get_char(line1, char_index);
                        current_rs   <= 1'b1;
                        return_state <= S_WRITE_LINE1;
                        char_index   <= char_index + 1'b1;
                        delay_cnt    <= 32'd0;
                        state        <= S_PULSE_HIGH;
                    end else begin
                        state <= S_SET_LINE2;
                    end
                end

                // Set LCD cursor to line 2
                S_SET_LINE2: begin
                    char_index   <= 5'd0;
                    current_byte <= 8'hC0;
                    current_rs   <= 1'b0;
                    return_state <= S_WRITE_LINE2;
                    delay_cnt    <= 32'd0;
                    state        <= S_PULSE_HIGH;
                end

                // Write 16 characters to line 2
                S_WRITE_LINE2: begin
                    if (char_index < 5'd16) begin
                        current_byte <= get_char(line2, char_index);
                        current_rs   <= 1'b1;
                        return_state <= S_WRITE_LINE2;
                        char_index   <= char_index + 1'b1;
                        delay_cnt    <= 32'd0;
                        state        <= S_PULSE_HIGH;
                    end else begin
                        delay_cnt <= 32'd0;
                        state <= S_REFRESH;
                    end
                end

                // Refresh display about 5 times per second
                S_REFRESH: begin
                    if (delay_cnt < 32'd10_000_000) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end else begin
                        delay_cnt <= 32'd0;
                        state <= S_SET_LINE1;
                    end
                end

                // Generate LCD EN high pulse
                S_PULSE_HIGH: begin
                    lcd_data <= current_byte;
                    lcd_rs   <= current_rs;
                    lcd_rw   <= 1'b0;       // always write
                    lcd_en   <= 1'b1;

                    if (delay_cnt < 32'd1000) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end else begin
                        delay_cnt <= 32'd0;
                        state <= S_PULSE_LOW;
                    end
                end

                // Generate LCD EN low pulse and small delay
                S_PULSE_LOW: begin
                    lcd_en <= 1'b0;

                    if (delay_cnt < 32'd3000) begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end else begin
                        delay_cnt <= 32'd0;
                        state <= return_state;
                    end
                end

                default: begin
                    state <= S_POWER_WAIT;
                end

            endcase
        end
    end

endmodule