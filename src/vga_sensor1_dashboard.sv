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
    input  wire        [31:0] sensor1_h2_75hz_gauss_q16,
    input  wire        [31:0] sensor2_h2_75hz_gauss_q16,
    input  wire        [31:0] sensor3_h2_75hz_gauss_q16,
    input  wire        [31:0] sensor4_h2_75hz_gauss_q16,
    input  wire        [31:0] sensor1_h2_45hz_gauss_q16,
    input  wire        [31:0] sensor2_h2_45hz_gauss_q16,
    input  wire        [31:0] sensor3_h2_45hz_gauss_q16,
    input  wire        [31:0] sensor4_h2_45hz_gauss_q16,
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

    localparam [9:0] GRAPH_LEFT   = 10'd96;
    localparam [9:0] GRAPH_RIGHT  = 10'd607;
    localparam [9:0] GRAPH_TOP    = 10'd336;
    localparam [9:0] GRAPH_BOTTOM = 10'd447;
    localparam [7:0] GRAPH_HEIGHT = 8'd111;
    localparam [31:0] GRAPH_FULL_SCALE_Q16 = 32'd1_048_576; // 16.000 G^2

    reg signed [15:0] snapshot_s1_x, snapshot_s1_y, snapshot_s1_z;
    reg signed [15:0] snapshot_s2_x, snapshot_s2_y, snapshot_s2_z;
    reg signed [15:0] snapshot_s3_x, snapshot_s3_y, snapshot_s3_z;
    reg signed [15:0] snapshot_s4_x, snapshot_s4_y, snapshot_s4_z;
    reg        [31:0] snapshot_s1_h2_75hz_gauss_q16;
    reg        [31:0] snapshot_s2_h2_75hz_gauss_q16;
    reg        [31:0] snapshot_s3_h2_75hz_gauss_q16;
    reg        [31:0] snapshot_s4_h2_75hz_gauss_q16;
    reg        [31:0] snapshot_s1_h2_45hz_gauss_q16;
    reg        [31:0] snapshot_s2_h2_45hz_gauss_q16;
    reg        [31:0] snapshot_s3_h2_45hz_gauss_q16;
    reg        [31:0] snapshot_s4_h2_45hz_gauss_q16;
    reg               snapshot_calibrated_mode;
    reg               snapshot_collecting;
    reg               snapshot_calculating;
    reg               snapshot_done;
    reg        [7:0]  plot_history_s1 [0:511];
    reg        [7:0]  plot_history_s2 [0:511];
    reg        [7:0]  plot_history_s3 [0:511];
    reg        [7:0]  plot_history_s4 [0:511];
    reg        [8:0]  history_write_index;
    reg        [9:0]  history_valid_count;

    wire [5:0] text_column = pixel_x[9:4];
    wire [4:0] text_row    = pixel_y[8:4];
    wire [2:0] glyph_column = pixel_x[3:1];
    wire [2:0] glyph_row    = pixel_y[3:1];

    reg  [7:0]   character;
    reg  [319:0] line_text;
    reg  [87:0]  status_text;
    reg  [7:0]   font_pixels;
    reg  [63:0]  font_bitmap;
    wire [9:0]   graph_x = pixel_x - GRAPH_LEFT;
    wire         graph_area = (pixel_x >= GRAPH_LEFT) &&
                              (pixel_x <= GRAPH_RIGHT) &&
                              (pixel_y >= GRAPH_TOP) &&
                              (pixel_y <= GRAPH_BOTTOM);
    wire         history_full = (history_valid_count == 10'd512);
    wire [9:0]   empty_history_columns = 10'd512 - history_valid_count;
    wire         graph_history_valid =
        history_full || (graph_x >= empty_history_columns);
    wire [8:0]   graph_history_index =
        history_full
            ? history_write_index + graph_x[8:0]
            : graph_x[8:0] - empty_history_columns[8:0];
    wire [9:0]   graph_plot_y_s1 =
        GRAPH_BOTTOM - {2'd0, plot_history_s1[graph_history_index]};
    wire [9:0]   graph_plot_y_s2 =
        GRAPH_BOTTOM - {2'd0, plot_history_s2[graph_history_index]};
    wire [9:0]   graph_plot_y_s3 =
        GRAPH_BOTTOM - {2'd0, plot_history_s3[graph_history_index]};
    wire [9:0]   graph_plot_y_s4 =
        GRAPH_BOTTOM - {2'd0, plot_history_s4[graph_history_index]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            snapshot_s1_x <= 16'sd0;
            snapshot_s1_y <= 16'sd0;
            snapshot_s1_z <= 16'sd0;
            snapshot_s2_x <= 16'sd0;
            snapshot_s2_y <= 16'sd0;
            snapshot_s2_z <= 16'sd0;
            snapshot_s3_x <= 16'sd0;
            snapshot_s3_y <= 16'sd0;
            snapshot_s3_z <= 16'sd0;
            snapshot_s4_x <= 16'sd0;
            snapshot_s4_y <= 16'sd0;
            snapshot_s4_z <= 16'sd0;
            snapshot_s1_h2_75hz_gauss_q16 <= 32'd0;
            snapshot_s2_h2_75hz_gauss_q16 <= 32'd0;
            snapshot_s3_h2_75hz_gauss_q16 <= 32'd0;
            snapshot_s4_h2_75hz_gauss_q16 <= 32'd0;
            snapshot_s1_h2_45hz_gauss_q16 <= 32'd0;
            snapshot_s2_h2_45hz_gauss_q16 <= 32'd0;
            snapshot_s3_h2_45hz_gauss_q16 <= 32'd0;
            snapshot_s4_h2_45hz_gauss_q16 <= 32'd0;
            snapshot_calibrated_mode <= 1'b0;
            snapshot_collecting <= 1'b0;
            snapshot_calculating <= 1'b0;
            snapshot_done <= 1'b0;
            history_write_index <= 9'd0;
            history_valid_count <= 10'd0;
        end else if (frame_start) begin
            snapshot_s1_x <= sensor1_x;
            snapshot_s1_y <= sensor1_y;
            snapshot_s1_z <= sensor1_z;
            snapshot_s2_x <= sensor2_x;
            snapshot_s2_y <= sensor2_y;
            snapshot_s2_z <= sensor2_z;
            snapshot_s3_x <= sensor3_x;
            snapshot_s3_y <= sensor3_y;
            snapshot_s3_z <= sensor3_z;
            snapshot_s4_x <= sensor4_x;
            snapshot_s4_y <= sensor4_y;
            snapshot_s4_z <= sensor4_z;
            snapshot_s1_h2_75hz_gauss_q16 <= sensor1_h2_75hz_gauss_q16;
            snapshot_s2_h2_75hz_gauss_q16 <= sensor2_h2_75hz_gauss_q16;
            snapshot_s3_h2_75hz_gauss_q16 <= sensor3_h2_75hz_gauss_q16;
            snapshot_s4_h2_75hz_gauss_q16 <= sensor4_h2_75hz_gauss_q16;
            snapshot_s1_h2_45hz_gauss_q16 <= sensor1_h2_45hz_gauss_q16;
            snapshot_s2_h2_45hz_gauss_q16 <= sensor2_h2_45hz_gauss_q16;
            snapshot_s3_h2_45hz_gauss_q16 <= sensor3_h2_45hz_gauss_q16;
            snapshot_s4_h2_45hz_gauss_q16 <= sensor4_h2_45hz_gauss_q16;
            snapshot_calibrated_mode <= calibrated_mode;
            snapshot_collecting <= calibration_collecting;
            snapshot_calculating <= calibration_calculating;
            snapshot_done <= calibration_done;
            plot_history_s1[history_write_index] <=
                magnitude_to_plot_level(sensor1_h2_75hz_gauss_q16);
            plot_history_s2[history_write_index] <=
                magnitude_to_plot_level(sensor2_h2_75hz_gauss_q16);
            plot_history_s3[history_write_index] <=
                magnitude_to_plot_level(sensor3_h2_75hz_gauss_q16);
            plot_history_s4[history_write_index] <=
                magnitude_to_plot_level(sensor4_h2_75hz_gauss_q16);
            history_write_index <= history_write_index + 1'b1;

            if (!history_full)
                history_valid_count <= history_valid_count + 1'b1;
        end
    end

    function automatic [7:0] decimal_digit_ascii;
        input [3:0] digit;
        begin
            decimal_digit_ascii = "0" + digit;
        end
    endfunction

    function automatic [7:0] magnitude_to_plot_level;
        input [31:0] magnitude_q16;
        reg [39:0] scaled_magnitude;
        begin
            if (magnitude_q16 >= GRAPH_FULL_SCALE_Q16) begin
                magnitude_to_plot_level = GRAPH_HEIGHT;
            end else begin
                scaled_magnitude = magnitude_q16 * GRAPH_HEIGHT;
                magnitude_to_plot_level = scaled_magnitude >> 20;
            end
        end
    endfunction

    function automatic signed [31:0] counts_to_milligauss;
        input signed [15:0] counts;
        reg signed [31:0] extended_counts;
        begin
            extended_counts = counts;
            counts_to_milligauss = extended_counts / 32'sd15;
        end
    endfunction

    function automatic [47:0] milligauss_to_ascii;
        input signed [31:0] milligauss;
        reg [31:0] abs_mg;
        reg [31:0] saturated_mg;
        reg [3:0]  ones;
        reg [3:0]  tenths;
        reg [3:0]  hundredths;
        reg [3:0]  thousandths;
        reg [7:0]  sign_character;
        begin
            if (milligauss < 0)
                abs_mg = -milligauss;
            else
                abs_mg = milligauss;

            sign_character = (milligauss < 0) ? "-" : "+";
            saturated_mg = (abs_mg > 32'd9999) ? 32'd9999 : abs_mg;
            ones = (saturated_mg / 32'd1000) % 10;
            tenths = (saturated_mg / 32'd100) % 10;
            hundredths = (saturated_mg / 32'd10) % 10;
            thousandths = saturated_mg % 10;

            milligauss_to_ascii = {
                sign_character,
                decimal_digit_ascii(ones),
                ".",
                decimal_digit_ascii(tenths),
                decimal_digit_ascii(hundredths),
                decimal_digit_ascii(thousandths)
            };
        end
    endfunction

    function automatic [47:0] h2_q16_to_ascii;
        input [31:0] h2_q16;
        reg [63:0] scaled_h2;
        reg [31:0] milli_h2;
        reg [3:0]  tens;
        reg [3:0]  ones;
        reg [3:0]  tenths;
        reg [3:0]  hundredths;
        reg [3:0]  thousandths;
        begin
            scaled_h2 = h2_q16 * 32'd1000;
            milli_h2 = scaled_h2 >> 16;
            if (milli_h2 > 32'd99999)
                milli_h2 = 32'd99999;

            tens = (milli_h2 / 32'd10000) % 10;
            ones = (milli_h2 / 32'd1000) % 10;
            tenths = (milli_h2 / 32'd100) % 10;
            hundredths = (milli_h2 / 32'd10) % 10;
            thousandths = milli_h2 % 10;

            h2_q16_to_ascii = {
                decimal_digit_ascii(tens),
                decimal_digit_ascii(ones),
                ".",
                decimal_digit_ascii(tenths),
                decimal_digit_ascii(hundredths),
                decimal_digit_ascii(thousandths)
            };
        end
    endfunction

    function automatic [319:0] sensor_axis_line;
        input [7:0] sensor_digit;
        input signed [15:0] x_counts;
        input signed [15:0] y_counts;
        input signed [15:0] z_counts;
        begin
            sensor_axis_line = {
                "S", sensor_digit,
                " X", milligauss_to_ascii(counts_to_milligauss(x_counts)),
                " Y", milligauss_to_ascii(counts_to_milligauss(y_counts)),
                " Z", milligauss_to_ascii(counts_to_milligauss(z_counts)),
                {14{" "}}
            };
        end
    endfunction

    function automatic [319:0] sensor_h2_line;
        input [7:0] sensor_digit;
        input [31:0] h2_75hz_q16;
        input [31:0] h2_45hz_q16;
        begin
            sensor_h2_line = {
                "S", sensor_digit,
                " H75 ", h2_q16_to_ascii(h2_75hz_q16),
                " H45 ", h2_q16_to_ascii(h2_45hz_q16),
                {16{" "}}
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
            5'd1:  line_text = {"QMC5883P GAUSS DASHBOARD", {16{" "}}};
            5'd3:  line_text = {
                "MODE ",
                snapshot_calibrated_mode ? "CAL " : "RAW ",
                "CALIB ",
                status_text,
                {15{" "}}
            };
            5'd5:  line_text = {"AXES GAUSS H75 H45 IN GAUSS^2", {11{" "}}};
            5'd7:  line_text = sensor_axis_line(
                "1",
                snapshot_s1_x,
                snapshot_s1_y,
                snapshot_s1_z
            );
            5'd8:  line_text = sensor_h2_line(
                "1",
                snapshot_s1_h2_75hz_gauss_q16,
                snapshot_s1_h2_45hz_gauss_q16
            );
            5'd10: line_text = sensor_axis_line(
                "2",
                snapshot_s2_x,
                snapshot_s2_y,
                snapshot_s2_z
            );
            5'd11: line_text = sensor_h2_line(
                "2",
                snapshot_s2_h2_75hz_gauss_q16,
                snapshot_s2_h2_45hz_gauss_q16
            );
            5'd13: line_text = sensor_axis_line(
                "3",
                snapshot_s3_x,
                snapshot_s3_y,
                snapshot_s3_z
            );
            5'd14: line_text = sensor_h2_line(
                "3",
                snapshot_s3_h2_75hz_gauss_q16,
                snapshot_s3_h2_45hz_gauss_q16
            );
            5'd16: line_text = sensor_axis_line(
                "4",
                snapshot_s4_x,
                snapshot_s4_y,
                snapshot_s4_z
            );
            5'd17: line_text = sensor_h2_line(
                "4",
                snapshot_s4_h2_75hz_gauss_q16,
                snapshot_s4_h2_45hz_gauss_q16
            );
            5'd19: line_text = {"H75 GRAPH 0-16 G^2  S1 S2 S3 S4", {9{" "}}};
            5'd28: line_text = {"S1 GREEN S2 RED S3 BLUE S4 YELLOW", {7{" "}}};
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
            "^": font_bitmap = 64'h183C660000000000;
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

    assign text_pixel_on = active_video &&
                           font_pixels[3'd7 - glyph_column];
    assign graph_axis_pixel_on = active_video && graph_area &&
                                 ((pixel_x == GRAPH_LEFT) ||
                                  (pixel_y == GRAPH_BOTTOM));
    assign graph_plot_s1_pixel_on = active_video && graph_area &&
                                    graph_history_valid &&
                                    ((pixel_y == graph_plot_y_s1) ||
                                     (pixel_y == graph_plot_y_s1 + 1'b1));
    assign graph_plot_s2_pixel_on = active_video && graph_area &&
                                    graph_history_valid &&
                                    ((pixel_y == graph_plot_y_s2) ||
                                     (pixel_y == graph_plot_y_s2 + 1'b1));
    assign graph_plot_s3_pixel_on = active_video && graph_area &&
                                    graph_history_valid &&
                                    ((pixel_y == graph_plot_y_s3) ||
                                     (pixel_y == graph_plot_y_s3 + 1'b1));
    assign graph_plot_s4_pixel_on = active_video && graph_area &&
                                    graph_history_valid &&
                                    ((pixel_y == graph_plot_y_s4) ||
                                     (pixel_y == graph_plot_y_s4 + 1'b1));

endmodule
