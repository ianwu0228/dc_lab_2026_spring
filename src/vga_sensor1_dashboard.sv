module vga_four_sensor_dashboard (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               frame_start,
    input  wire               active_video,
    input  wire [9:0]         pixel_x,
    input  wire [9:0]         pixel_y,

    input  wire signed [15:0] sensor1_x,
    input  wire signed [15:0] sensor1_y,
    input  wire signed [15:0] sensor1_z,
    input  wire signed [15:0] sensor2_x,
    input  wire signed [15:0] sensor2_y,
    input  wire signed [15:0] sensor2_z,
    input  wire signed [15:0] sensor3_x,
    input  wire signed [15:0] sensor3_y,
    input  wire signed [15:0] sensor3_z,
    input  wire signed [15:0] sensor4_x,
    input  wire signed [15:0] sensor4_y,
    input  wire signed [15:0] sensor4_z,
    input  wire        [31:0] sensor1_h2_gauss_q16,
    input  wire        [31:0] sensor2_h2_gauss_q16,
    input  wire        [31:0] sensor3_h2_gauss_q16,
    input  wire        [31:0] sensor4_h2_gauss_q16,
    input  wire signed [15:0] source_x_q10,
    input  wire signed [15:0] source_y_q10,
    input  wire signed [15:0] source_z_q10,
    input  wire               source_valid,
    input  wire               calibrated_mode,
    input  wire               calibration_collecting,
    input  wire               calibration_calculating,
    input  wire               calibration_done,

    output wire               text_pixel_on,
    output wire               graph_axis_pixel_on,
    output wire               graph_plot_s1_pixel_on,
    output wire               graph_plot_s2_pixel_on,
    output wire               graph_plot_s3_pixel_on,
    output wire               graph_plot_s4_pixel_on
);

    localparam [9:0] CENTER_X = 10'd320;
    localparam [9:0] CENTER_Y = 10'd320;
    localparam [9:0] S1_X = 10'd260;
    localparam [9:0] S1_Y = 10'd380;
    localparam [9:0] S2_X = 10'd380;
    localparam [9:0] S2_Y = 10'd380;
    localparam [9:0] S3_X = 10'd260;
    localparam [9:0] S3_Y = 10'd260;
    localparam [9:0] S4_X = 10'd380;
    localparam [9:0] S4_Y = 10'd260;

    reg signed [15:0] snapshot_source_x_q10;
    reg signed [15:0] snapshot_source_y_q10;
    reg signed [15:0] snapshot_source_z_q10;
    reg               snapshot_source_valid;
    reg               snapshot_calibrated_mode;
    reg               snapshot_collecting;
    reg               snapshot_calculating;
    reg               snapshot_done;

    wire [5:0] text_column = pixel_x[9:4];
    wire [4:0] text_row    = pixel_y[8:4];
    wire [2:0] glyph_column = pixel_x[3:1];
    wire [2:0] glyph_row    = pixel_y[3:1];

    reg  [7:0]   character;
    reg  [319:0] line_text;
    reg  [87:0]  status_text;
    reg  [7:0]   font_pixels;
    reg  [63:0]  font_bitmap;

    wire [9:0] source_screen_x =
        project_x(snapshot_source_x_q10);
    wire [9:0] source_screen_y =
        project_y(snapshot_source_y_q10, snapshot_source_z_q10);
    wire [9:0] shadow_screen_x =
        project_x(snapshot_source_x_q10);
    wire [9:0] shadow_screen_y =
        project_y(snapshot_source_y_q10, 16'sd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapshot_source_x_q10 <= 16'sd0;
            snapshot_source_y_q10 <= 16'sd0;
            snapshot_source_z_q10 <= 16'sd0;
            snapshot_source_valid <= 1'b0;
            snapshot_calibrated_mode <= 1'b0;
            snapshot_collecting <= 1'b0;
            snapshot_calculating <= 1'b0;
            snapshot_done <= 1'b0;
        end else if (frame_start) begin
            snapshot_source_x_q10 <= source_x_q10;
            snapshot_source_y_q10 <= source_y_q10;
            snapshot_source_z_q10 <= source_z_q10;
            snapshot_source_valid <= source_valid;
            snapshot_calibrated_mode <= calibrated_mode;
            snapshot_collecting <= calibration_collecting;
            snapshot_calculating <= calibration_calculating;
            snapshot_done <= calibration_done;
        end
    end

    function automatic [9:0] project_x;
        input signed [15:0] x_q10;
        reg signed [26:0] x_value;
        begin
            x_value = $signed({1'b0, CENTER_X}) +
                      ((x_q10 * 8'sd120) >>> 10);
            if (x_value < 0)
                project_x = 10'd0;
            else if (x_value > 27'sd639)
                project_x = 10'd639;
            else
                project_x = x_value[9:0];
        end
    endfunction

    function automatic [9:0] project_y;
        input signed [15:0] y_q10;
        input signed [15:0] z_q10;
        reg signed [26:0] y_value;
        begin
            y_value = $signed({1'b0, CENTER_Y}) -
                      ((y_q10 * 8'sd120) >>> 10) -
                      ((z_q10 * 8'sd80) >>> 10);
            if (y_value < 0)
                project_y = 10'd0;
            else if (y_value > 27'sd479)
                project_y = 10'd479;
            else
                project_y = y_value[9:0];
        end
    endfunction

    function automatic point_near;
        input [9:0] center_x;
        input [9:0] center_y;
        input [3:0] radius;
        reg [10:0] dx;
        reg [10:0] dy;
        begin
            dx = (pixel_x >= center_x) ? (pixel_x - center_x) :
                 (center_x - pixel_x);
            dy = (pixel_y >= center_y) ? (pixel_y - center_y) :
                 (center_y - pixel_y);
            point_near = (dx <= radius) && (dy <= radius);
        end
    endfunction

    function automatic line_near;
        input [9:0] x0;
        input [9:0] y0;
        input [9:0] x1;
        input [9:0] y1;
        reg signed [20:0] area_term;
        reg [10:0] min_x;
        reg [10:0] max_x;
        reg [10:0] min_y;
        reg [10:0] max_y;
        begin
            min_x = (x0 < x1) ? x0 : x1;
            max_x = (x0 < x1) ? x1 : x0;
            min_y = (y0 < y1) ? y0 : y1;
            max_y = (y0 < y1) ? y1 : y0;
            area_term =
                ($signed({1'b0, pixel_x}) - $signed({1'b0, x0})) *
                ($signed({1'b0, y1}) - $signed({1'b0, y0})) -
                ($signed({1'b0, pixel_y}) - $signed({1'b0, y0})) *
                ($signed({1'b0, x1}) - $signed({1'b0, x0}));
            if (area_term < 0)
                area_term = -area_term;
            line_near = (pixel_x >= min_x - 1'b1) &&
                        (pixel_x <= max_x + 1'b1) &&
                        (pixel_y >= min_y - 1'b1) &&
                        (pixel_y <= max_y + 1'b1) &&
                        (area_term < 21'sd420);
        end
    endfunction

    function automatic [7:0] decimal_digit_ascii;
        input [3:0] digit;
        begin
            decimal_digit_ascii = "0" + digit;
        end
    endfunction

    function automatic [7:0] fixed_digit;
        input [15:0] value_q10;
        input [1:0]  digit_index;
        reg [31:0] scaled;
        reg [31:0] milli;
        begin
            scaled = {22'd0, value_q10[9:0]} * 32'd1000;
            milli = scaled >> 10;
            case (digit_index)
                2'd0: fixed_digit = decimal_digit_ascii((milli / 100) % 10);
                2'd1: fixed_digit = decimal_digit_ascii((milli / 10) % 10);
                default: fixed_digit = decimal_digit_ascii(milli % 10);
            endcase
        end
    endfunction

    function automatic [47:0] q10_to_ascii;
        input signed [15:0] value_q10;
        reg [15:0] abs_value_q10;
        reg [7:0] sign_char;
        reg [7:0] integer_char;
        begin
            if (value_q10 < 0) begin
                abs_value_q10 = -value_q10;
                sign_char = "-";
            end else begin
                abs_value_q10 = value_q10;
                sign_char = "+";
            end
            if ((abs_value_q10 >> 10) > 16'd9)
                integer_char = "9";
            else
                integer_char = decimal_digit_ascii(abs_value_q10[13:10]);
            q10_to_ascii = {
                sign_char,
                integer_char,
                ".",
                fixed_digit(abs_value_q10, 2'd0),
                fixed_digit(abs_value_q10, 2'd1),
                fixed_digit(abs_value_q10, 2'd2)
            };
        end
    endfunction

    function automatic [7:0] string_character;
        input [319:0] text;
        input [5:0]   index;
        begin
            if (index < 6'd40)
                string_character = text[319 - (index * 8) -: 8];
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

        line_text = {40{" "}};

        case (text_row)
            5'd1: line_text = {"MAGNET SOURCE POSITION", {18{" "}}};
            5'd3: line_text = {
                "MODE ",
                snapshot_calibrated_mode ? "CAL " : "RAW ",
                "CALIB ",
                status_text,
                {14{" "}}
            };
            5'd5: line_text = {
                "X=", q10_to_ascii(snapshot_source_x_q10),
                " Y=", q10_to_ascii(snapshot_source_y_q10),
                " Z=", q10_to_ascii(snapshot_source_z_q10),
                {14{" "}}
            };
            5'd7: line_text = {"24MM SQUARE SENSOR PLANE", {16{" "}}};
            5'd8: line_text = {"KEY2 BG NO MAGNET", {23{" "}}};
            5'd28: line_text = {"BLUE SENSOR  RED SOURCE  YELLOW SHADOW", {2{" "}}};
            default: line_text = {40{" "}};
        endcase

        character = string_character(line_text, text_column);
    end

    always @* begin
        case (character)
            "0": font_bitmap = 64'h3C666E7666663C00;
            "1": font_bitmap = 64'h1838181818187E00;
            "2": font_bitmap = 64'h3C66060C18307E00;
            "3": font_bitmap = 64'h3C66061C06663C00;
            "4": font_bitmap = 64'h0C1C3C6C7E0C0C00;
            "5": font_bitmap = 64'h7E607C0606663C00;
            "6": font_bitmap = 64'h1C30607C66663C00;
            "7": font_bitmap = 64'h7E66060C18181800;
            "8": font_bitmap = 64'h3C66663C66663C00;
            "9": font_bitmap = 64'h3C66663E060C3800;
            "A": font_bitmap = 64'h183C66667E666600;
            "B": font_bitmap = 64'h7C66667C66667C00;
            "C": font_bitmap = 64'h3C66606060663C00;
            "D": font_bitmap = 64'h786C6666666C7800;
            "E": font_bitmap = 64'h7E60607C60607E00;
            "F": font_bitmap = 64'h7E60607C60606000;
            "G": font_bitmap = 64'h3C66606E66663C00;
            "H": font_bitmap = 64'h6666667E66666600;
            "I": font_bitmap = 64'h3C18181818183C00;
            "L": font_bitmap = 64'h6060606060607E00;
            "M": font_bitmap = 64'h63777F6B63636300;
            "N": font_bitmap = 64'h66767E7E6E666600;
            "O": font_bitmap = 64'h3C66666666663C00;
            "P": font_bitmap = 64'h7C66667C60606000;
            "Q": font_bitmap = 64'h3C6666666E3C0E00;
            "R": font_bitmap = 64'h7C66667C6C666600;
            "S": font_bitmap = 64'h3C66603C06663C00;
            "T": font_bitmap = 64'h7E5A181818183C00;
            "U": font_bitmap = 64'h6666666666663C00;
            "V": font_bitmap = 64'h66666666663C1800;
            "W": font_bitmap = 64'h6363636B7F776300;
            "X": font_bitmap = 64'h66663C183C666600;
            "Y": font_bitmap = 64'h6666663C18183C00;
            "Z": font_bitmap = 64'h7E060C1830607E00;
            "+": font_bitmap = 64'h0018187E18180000;
            "-": font_bitmap = 64'h0000007E00000000;
            "=": font_bitmap = 64'h00007E007E000000;
            ":": font_bitmap = 64'h0018180018180000;
            ".": font_bitmap = 64'h0000000000181800;
            ",": font_bitmap = 64'h0000000018183000;
            "(": font_bitmap = 64'h0C18303030180C00;
            ")": font_bitmap = 64'h30180C0C0C183000;
            default: font_bitmap = 64'd0;
        endcase

        case (glyph_row)
            3'd0: font_pixels = font_bitmap[63:56];
            3'd1: font_pixels = font_bitmap[55:48];
            3'd2: font_pixels = font_bitmap[47:40];
            3'd3: font_pixels = font_bitmap[39:32];
            3'd4: font_pixels = font_bitmap[31:24];
            3'd5: font_pixels = font_bitmap[23:16];
            3'd6: font_pixels = font_bitmap[15:8];
            default: font_pixels = font_bitmap[7:0];
        endcase
    end

    wire square_s1_s2 = line_near(S1_X, S1_Y, S2_X, S2_Y);
    wire square_s1_s3 = line_near(S1_X, S1_Y, S3_X, S3_Y);
    wire square_s2_s4 = line_near(S2_X, S2_Y, S4_X, S4_Y);
    wire square_s3_s4 = line_near(S3_X, S3_Y, S4_X, S4_Y);
    wire source_height_line = snapshot_source_valid &&
                              line_near(source_screen_x,
                                        source_screen_y,
                                        shadow_screen_x,
                                        shadow_screen_y);

    assign text_pixel_on = active_video &&
                           font_pixels[3'd7 - glyph_column];
    assign graph_axis_pixel_on = active_video &&
                                 (square_s1_s2 || square_s1_s3 ||
                                  square_s2_s4 || square_s3_s4 ||
                                  source_height_line);
    assign graph_plot_s1_pixel_on = 1'b0;
    assign graph_plot_s2_pixel_on = active_video && snapshot_source_valid &&
                                    point_near(source_screen_x,
                                               source_screen_y,
                                               4'd4);
    assign graph_plot_s3_pixel_on = active_video &&
                                    (point_near(S1_X, S1_Y, 4'd4) ||
                                     point_near(S2_X, S2_Y, 4'd4) ||
                                     point_near(S3_X, S3_Y, 4'd4) ||
                                     point_near(S4_X, S4_Y, 4'd4));
    assign graph_plot_s4_pixel_on = active_video && snapshot_source_valid &&
                                    point_near(shadow_screen_x,
                                               shadow_screen_y,
                                               4'd2);

endmodule
