module vga_sensor1_dashboard (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               frame_start,
    input  wire               active_video,
    input  wire [9:0]         pixel_x,
    input  wire [9:0]         pixel_y,

    input  wire signed [15:0] sensor_x,
    input  wire signed [15:0] sensor_y,
    input  wire signed [15:0] sensor_z,
    input  wire               calibrated_mode,
    input  wire               calibration_collecting,
    input  wire               calibration_calculating,
    input  wire               calibration_done,

    output wire               pixel_on
);

    reg signed [15:0] snapshot_x;
    reg signed [15:0] snapshot_y;
    reg signed [15:0] snapshot_z;
    reg               snapshot_calibrated_mode;
    reg               snapshot_collecting;
    reg               snapshot_calculating;
    reg               snapshot_done;

    wire [5:0] text_column = pixel_x[9:4];
    wire [4:0] text_row    = pixel_y[8:4];
    wire [2:0] glyph_column = pixel_x[3:1];
    wire [2:0] glyph_row    = pixel_y[3:1];

    reg  [7:0]   character;
    reg  [255:0] line_text;
    reg  [87:0]  status_text;
    wire [7:0]   font_pixels;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapshot_x               <= 16'sd0;
            snapshot_y               <= 16'sd0;
            snapshot_z               <= 16'sd0;
            snapshot_calibrated_mode <= 1'b0;
            snapshot_collecting      <= 1'b0;
            snapshot_calculating     <= 1'b0;
            snapshot_done            <= 1'b0;
        end else if (frame_start) begin
            snapshot_x               <= sensor_x;
            snapshot_y               <= sensor_y;
            snapshot_z               <= sensor_z;
            snapshot_calibrated_mode <= calibrated_mode;
            snapshot_collecting      <= calibration_collecting;
            snapshot_calculating     <= calibration_calculating;
            snapshot_done            <= calibration_done;
        end
    end

    function automatic [7:0] hex_to_ascii;
        input [3:0] hex;
        begin
            if (hex < 4'd10)
                hex_to_ascii = "0" + hex;
            else
                hex_to_ascii = "A" + (hex - 4'd10);
        end
    endfunction

    function automatic [39:0] signed_word_to_hex_ascii;
        input signed [15:0] value;
        reg [15:0] magnitude;
        begin
            if (value < 0) begin
                magnitude = (~value) + 1'b1;
                signed_word_to_hex_ascii = {
                    "-",
                    hex_to_ascii(magnitude[15:12]),
                    hex_to_ascii(magnitude[11:8]),
                    hex_to_ascii(magnitude[7:4]),
                    hex_to_ascii(magnitude[3:0])
                };
            end else begin
                magnitude = value;
                signed_word_to_hex_ascii = {
                    "+",
                    hex_to_ascii(magnitude[15:12]),
                    hex_to_ascii(magnitude[11:8]),
                    hex_to_ascii(magnitude[7:4]),
                    hex_to_ascii(magnitude[3:0])
                };
            end
        end
    endfunction

    function automatic [7:0] string_character;
        input [255:0] text;
        input [5:0]   index;
        begin
            if (index < 6'd32)
                string_character = text[255 - (index * 8) -: 8];
            else
                string_character = " ";
        end
    endfunction

    always @* begin
        if (snapshot_collecting)
            status_text = "COLLECTING ";
        else if (snapshot_calculating)
            status_text = "CALCULATING";
        else if (snapshot_done)
            status_text = "DONE       ";
        else
            status_text = "READY      ";

        line_text = {32{" "}};

        case (text_row)
            5'd2:  line_text = {"MAGNETOMETER MONITOR", {12{" "}}};
            5'd4:  line_text = {
                "MODE: ",
                snapshot_calibrated_mode ? "CALIBRATED" : "RAW       ",
                {16{" "}}
            };
            5'd6:  line_text = {"SENSOR 1", {24{" "}}};
            5'd8:  line_text = {
                "X = ", signed_word_to_hex_ascii(snapshot_x), {23{" "}}
            };
            5'd9:  line_text = {
                "Y = ", signed_word_to_hex_ascii(snapshot_y), {23{" "}}
            };
            5'd10: line_text = {
                "Z = ", signed_word_to_hex_ascii(snapshot_z), {23{" "}}
            };
            5'd13: line_text = {
                "CALIBRATION: ", status_text, {8{" "}}
            };
            default: line_text = {32{" "}};
        endcase

        character = string_character(line_text, text_column);
    end

    vga_font_rom u_font (
        .character (character),
        .glyph_row (glyph_row),
        .pixels    (font_pixels)
    );

    assign pixel_on = active_video && font_pixels[3'd7 - glyph_column];

endmodule
