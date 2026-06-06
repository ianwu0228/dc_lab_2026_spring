module mag_source_tracker (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        update,
    input  wire        baseline_capture,
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
    input  wire [31:0] sensor1_h2_gauss_q16,
    input  wire [31:0] sensor2_h2_gauss_q16,
    input  wire [31:0] sensor3_h2_gauss_q16,
    input  wire [31:0] sensor4_h2_gauss_q16,
    output reg  signed [15:0] source_x_q10,
    output reg  signed [15:0] source_y_q10,
    output reg  signed [15:0] source_z_q10,
    output reg         source_valid
);

    // Permanent-magnet DC tracker:
    //   1. KEY[2] captures the DC background vector for each sensor.
    //   2. Runtime field strength is |H_now - H_background|^2.
    //   3. A grid search finds the point whose inverse-sixth distance ratios
    //      best match the four measured strengths.
    //
    // Coordinate system:
    //   S3(-12,+12,0)        S4(+12,+12,0)
    //
    //   S1(-12,-12,0)        S2(+12,-12,0)
    //
    // Output Q10 values are normalized by SENSOR_SIDE_MM:
    //   1024 = 24 mm.
    localparam signed [15:0] SENSOR_SIDE_MM = 16'sd24;
    localparam signed [15:0] SENSOR_HALF_MM = 16'sd12;
    localparam signed [15:0] S1_X_MM = -SENSOR_HALF_MM;
    localparam signed [15:0] S1_Y_MM = -SENSOR_HALF_MM;
    localparam signed [15:0] S2_X_MM =  SENSOR_HALF_MM;
    localparam signed [15:0] S2_Y_MM = -SENSOR_HALF_MM;
    localparam signed [15:0] S3_X_MM = -SENSOR_HALF_MM;
    localparam signed [15:0] S3_Y_MM =  SENSOR_HALF_MM;
    localparam signed [15:0] S4_X_MM =  SENSOR_HALF_MM;
    localparam signed [15:0] S4_Y_MM =  SENSOR_HALF_MM;

    localparam signed [15:0] X_MIN_MM = -16'sd48;
    localparam signed [15:0] X_MAX_MM =  16'sd48;
    localparam signed [15:0] Y_MIN_MM = -16'sd48;
    localparam signed [15:0] Y_MAX_MM =  16'sd48;
    localparam signed [15:0] Z_MIN_MM =   16'sd4;
    localparam signed [15:0] Z_MAX_MM =  16'sd120;
    localparam signed [15:0] COARSE_STEP_MM = 16'sd4;
    localparam signed [15:0] REFINE_STEP_MM = 16'sd1;
    localparam signed [15:0] REFINE_RADIUS_MM = 16'sd3;

    localparam [31:0] MIN_TOTAL_SIGNAL = 32'd1000;
    localparam [2:0]  STRENGTH_FILTER_SHIFT = 3'd3;
    localparam [2:0]  POSITION_FILTER_SHIFT = 3'd2;

    localparam [3:0]
        S_IDLE        = 4'd0,
        S_COARSE_EVAL = 4'd1,
        S_REFINE_INIT = 4'd2,
        S_REFINE_EVAL = 4'd3,
        S_OUTPUT      = 4'd4;

    reg [3:0] state;

    reg signed [15:0] bg1_x, bg1_y, bg1_z;
    reg signed [15:0] bg2_x, bg2_y, bg2_z;
    reg signed [15:0] bg3_x, bg3_y, bg3_z;
    reg signed [15:0] bg4_x, bg4_y, bg4_z;

    reg [31:0] filtered_s1, filtered_s2, filtered_s3, filtered_s4;
    reg [31:0] h2_s1, h2_s2, h2_s3, h2_s4;
    reg [1:0]  reference_sensor;

    reg signed [15:0] candidate_x_mm;
    reg signed [15:0] candidate_y_mm;
    reg signed [15:0] candidate_z_mm;
    reg signed [15:0] best_x_mm;
    reg signed [15:0] best_y_mm;
    reg signed [15:0] best_z_mm;
    reg signed [15:0] refine_x_min_mm;
    reg signed [15:0] refine_x_max_mm;
    reg signed [15:0] refine_y_min_mm;
    reg signed [15:0] refine_y_max_mm;
    reg signed [15:0] refine_z_min_mm;
    reg signed [15:0] refine_z_max_mm;
    reg [95:0] best_score;

    wire [31:0] signal_s1 = vector_delta_square(
        sensor1_x, sensor1_y, sensor1_z, bg1_x, bg1_y, bg1_z);
    wire [31:0] signal_s2 = vector_delta_square(
        sensor2_x, sensor2_y, sensor2_z, bg2_x, bg2_y, bg2_z);
    wire [31:0] signal_s3 = vector_delta_square(
        sensor3_x, sensor3_y, sensor3_z, bg3_x, bg3_y, bg3_z);
    wire [31:0] signal_s4 = vector_delta_square(
        sensor4_x, sensor4_y, sensor4_z, bg4_x, bg4_y, bg4_z);

    wire [33:0] filtered_sum =
        {2'd0, filtered_s1} +
        {2'd0, filtered_s2} +
        {2'd0, filtered_s3} +
        {2'd0, filtered_s4};

    wire [1:0] strongest_sensor =
        (filtered_s1 >= filtered_s2 &&
         filtered_s1 >= filtered_s3 &&
         filtered_s1 >= filtered_s4) ? 2'd0 :
        (filtered_s2 >= filtered_s3 &&
         filtered_s2 >= filtered_s4) ? 2'd1 :
        (filtered_s3 >= filtered_s4) ? 2'd2 : 2'd3;

    wire [95:0] candidate_score_value = candidate_score(
        candidate_x_mm,
        candidate_y_mm,
        candidate_z_mm,
        h2_s1,
        h2_s2,
        h2_s3,
        h2_s4,
        reference_sensor
    );

    wire coarse_last =
        (candidate_x_mm >= X_MAX_MM) &&
        (candidate_y_mm >= Y_MAX_MM) &&
        (candidate_z_mm >= Z_MAX_MM);

    wire refine_last =
        (candidate_x_mm >= refine_x_max_mm) &&
        (candidate_y_mm >= refine_y_max_mm) &&
        (candidate_z_mm >= refine_z_max_mm);

    function automatic [31:0] square_signed_18;
        input signed [17:0] value;
        reg [17:0] magnitude;
        reg [35:0] square;
        begin
            magnitude = value[17] ? ((~value) + 1'b1) : value;
            square = {18'd0, magnitude} * {18'd0, magnitude};
            if (square[35:32] != 4'd0)
                square_signed_18 = 32'hFFFFFFFF;
            else
                square_signed_18 = square[31:0];
        end
    endfunction

    function automatic [31:0] vector_delta_square;
        input signed [15:0] x_now;
        input signed [15:0] y_now;
        input signed [15:0] z_now;
        input signed [15:0] x_bg;
        input signed [15:0] y_bg;
        input signed [15:0] z_bg;
        reg signed [17:0] dx;
        reg signed [17:0] dy;
        reg signed [17:0] dz;
        reg [33:0] sum;
        begin
            dx = {{2{x_now[15]}}, x_now} - {{2{x_bg[15]}}, x_bg};
            dy = {{2{y_now[15]}}, y_now} - {{2{y_bg[15]}}, y_bg};
            dz = {{2{z_now[15]}}, z_now} - {{2{z_bg[15]}}, z_bg};
            sum = {2'd0, square_signed_18(dx)} +
                  {2'd0, square_signed_18(dy)} +
                  {2'd0, square_signed_18(dz)};
            if (sum[33:32] != 2'd0)
                vector_delta_square = 32'hFFFFFFFF;
            else
                vector_delta_square = sum[31:0];
        end
    endfunction

    function automatic [31:0] smooth_unsigned;
        input [31:0] old_value;
        input [31:0] new_value;
        reg [31:0] delta;
        reg [31:0] step;
        begin
            if (new_value >= old_value) begin
                delta = new_value - old_value;
                step = delta >> STRENGTH_FILTER_SHIFT;
                smooth_unsigned = old_value + ((delta != 32'd0 && step == 32'd0) ? 32'd1 : step);
            end else begin
                delta = old_value - new_value;
                step = delta >> STRENGTH_FILTER_SHIFT;
                smooth_unsigned = old_value - ((delta != 32'd0 && step == 32'd0) ? 32'd1 : step);
            end
        end
    endfunction

    function automatic [31:0] square_signed_16;
        input signed [15:0] value;
        reg [15:0] magnitude;
        reg [31:0] magnitude_32;
        begin
            magnitude = value[15] ? ((~value) + 1'b1) : value;
            magnitude_32 = {16'd0, magnitude};
            square_signed_16 = magnitude_32 * magnitude_32;
        end
    endfunction

    function automatic [47:0] r6_scaled;
        input [31:0] r2;
        reg [63:0] r4;
        reg [95:0] r6;
        reg [95:0] shifted;
        begin
            r4 = {32'd0, r2} * {32'd0, r2};
            r6 = {32'd0, r4} * {64'd0, r2};
            shifted = r6 >> 8;
            if (shifted[95:48] != 48'd0)
                r6_scaled = {48{1'b1}};
            else
                r6_scaled = shifted[47:0];
        end
    endfunction

    function automatic [95:0] pair_score;
        input [31:0] h2_ref;
        input [31:0] h2_other;
        input [47:0] r6_ref;
        input [47:0] r6_other;
        reg [127:0] left_product;
        reg [127:0] right_product;
        reg [127:0] diff;
        reg [127:0] scaled_diff;
        begin
            // Permanent magnet strength-only approximation:
            //   |H|^2 ~= K / r^6
            // K is unknown, so compare K estimates:
            //   Href^2 * rref^6 ~= Hother^2 * rother^6
            left_product = {32'd0, h2_ref} * {16'd0, r6_ref};
            right_product = {32'd0, h2_other} * {16'd0, r6_other};
            diff = (left_product >= right_product) ?
                   (left_product - right_product) :
                   (right_product - left_product);
            scaled_diff = diff >> 16;

            if (scaled_diff[127:96] != 32'd0)
                pair_score = {96{1'b1}};
            else
                pair_score = scaled_diff[95:0];
        end
    endfunction

    function automatic [95:0] add3_score;
        input [95:0] a;
        input [95:0] b;
        input [95:0] c;
        reg [98:0] sum;
        begin
            sum = {3'd0, a} + {3'd0, b} + {3'd0, c};
            if (sum[98:96] != 3'd0)
                add3_score = {96{1'b1}};
            else
                add3_score = sum[95:0];
        end
    endfunction

    function automatic [95:0] candidate_score;
        input signed [15:0] x_mm;
        input signed [15:0] y_mm;
        input signed [15:0] z_mm;
        input [31:0] m1;
        input [31:0] m2;
        input [31:0] m3;
        input [31:0] m4;
        input [1:0]  ref_sel;
        reg signed [15:0] dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4;
        reg [31:0] z2;
        reg [31:0] r2_1, r2_2, r2_3, r2_4;
        reg [47:0] r6_1, r6_2, r6_3, r6_4;
        begin
            dx1 = x_mm - S1_X_MM;
            dy1 = y_mm - S1_Y_MM;
            dx2 = x_mm - S2_X_MM;
            dy2 = y_mm - S2_Y_MM;
            dx3 = x_mm - S3_X_MM;
            dy3 = y_mm - S3_Y_MM;
            dx4 = x_mm - S4_X_MM;
            dy4 = y_mm - S4_Y_MM;

            z2 = square_signed_16(z_mm);
            r2_1 = square_signed_16(dx1) + square_signed_16(dy1) + z2;
            r2_2 = square_signed_16(dx2) + square_signed_16(dy2) + z2;
            r2_3 = square_signed_16(dx3) + square_signed_16(dy3) + z2;
            r2_4 = square_signed_16(dx4) + square_signed_16(dy4) + z2;
            r6_1 = r6_scaled(r2_1);
            r6_2 = r6_scaled(r2_2);
            r6_3 = r6_scaled(r2_3);
            r6_4 = r6_scaled(r2_4);

            case (ref_sel)
                2'd0: candidate_score = add3_score(
                    pair_score(m1, m2, r6_1, r6_2),
                    pair_score(m1, m3, r6_1, r6_3),
                    pair_score(m1, m4, r6_1, r6_4)
                );
                2'd1: candidate_score = add3_score(
                    pair_score(m2, m1, r6_2, r6_1),
                    pair_score(m2, m3, r6_2, r6_3),
                    pair_score(m2, m4, r6_2, r6_4)
                );
                2'd2: candidate_score = add3_score(
                    pair_score(m3, m1, r6_3, r6_1),
                    pair_score(m3, m2, r6_3, r6_2),
                    pair_score(m3, m4, r6_3, r6_4)
                );
                default: candidate_score = add3_score(
                    pair_score(m4, m1, r6_4, r6_1),
                    pair_score(m4, m2, r6_4, r6_2),
                    pair_score(m4, m3, r6_4, r6_3)
                );
            endcase
        end
    endfunction

    function automatic signed [15:0] max_signed_16;
        input signed [15:0] a;
        input signed [15:0] b;
        begin
            max_signed_16 = (a > b) ? a : b;
        end
    endfunction

    function automatic signed [15:0] min_signed_16;
        input signed [15:0] a;
        input signed [15:0] b;
        begin
            min_signed_16 = (a < b) ? a : b;
        end
    endfunction

    function automatic signed [15:0] mm_to_q10;
        input signed [15:0] mm_value;
        reg signed [31:0] scaled;
        begin
            scaled = mm_value * 32'sd1024;
            mm_to_q10 = scaled / SENSOR_SIDE_MM;
        end
    endfunction

    function automatic signed [15:0] smooth_signed_q10;
        input signed [15:0] old_value;
        input signed [15:0] new_value;
        reg signed [16:0] delta;
        begin
            delta = {new_value[15], new_value} - {old_value[15], old_value};
            smooth_signed_q10 = old_value + (delta >>> POSITION_FILTER_SHIFT);
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            bg1_x <= 16'sd0; bg1_y <= 16'sd0; bg1_z <= 16'sd0;
            bg2_x <= 16'sd0; bg2_y <= 16'sd0; bg2_z <= 16'sd0;
            bg3_x <= 16'sd0; bg3_y <= 16'sd0; bg3_z <= 16'sd0;
            bg4_x <= 16'sd0; bg4_y <= 16'sd0; bg4_z <= 16'sd0;
            filtered_s1 <= 32'd0;
            filtered_s2 <= 32'd0;
            filtered_s3 <= 32'd0;
            filtered_s4 <= 32'd0;
            h2_s1 <= 32'd0;
            h2_s2 <= 32'd0;
            h2_s3 <= 32'd0;
            h2_s4 <= 32'd0;
            reference_sensor <= 2'd0;
            candidate_x_mm <= 16'sd0;
            candidate_y_mm <= 16'sd0;
            candidate_z_mm <= Z_MIN_MM;
            best_x_mm <= 16'sd0;
            best_y_mm <= 16'sd0;
            best_z_mm <= Z_MIN_MM;
            refine_x_min_mm <= 16'sd0;
            refine_x_max_mm <= 16'sd0;
            refine_y_min_mm <= 16'sd0;
            refine_y_max_mm <= 16'sd0;
            refine_z_min_mm <= Z_MIN_MM;
            refine_z_max_mm <= Z_MIN_MM;
            best_score <= {96{1'b1}};
            source_x_q10 <= 16'sd0;
            source_y_q10 <= 16'sd0;
            source_z_q10 <= 16'sd0;
            source_valid <= 1'b0;
        end else begin
            if (baseline_capture) begin
                bg1_x <= sensor1_x; bg1_y <= sensor1_y; bg1_z <= sensor1_z;
                bg2_x <= sensor2_x; bg2_y <= sensor2_y; bg2_z <= sensor2_z;
                bg3_x <= sensor3_x; bg3_y <= sensor3_y; bg3_z <= sensor3_z;
                bg4_x <= sensor4_x; bg4_y <= sensor4_y; bg4_z <= sensor4_z;
                filtered_s1 <= 32'd0;
                filtered_s2 <= 32'd0;
                filtered_s3 <= 32'd0;
                filtered_s4 <= 32'd0;
                source_valid <= 1'b0;
                state <= S_IDLE;
            end else begin
                case (state)
                    S_IDLE: begin
                        if (update) begin
                            filtered_s1 <= smooth_unsigned(filtered_s1, signal_s1);
                            filtered_s2 <= smooth_unsigned(filtered_s2, signal_s2);
                            filtered_s3 <= smooth_unsigned(filtered_s3, signal_s3);
                            filtered_s4 <= smooth_unsigned(filtered_s4, signal_s4);

                            if (filtered_sum >= {2'd0, MIN_TOTAL_SIGNAL}) begin
                                h2_s1 <= filtered_s1;
                                h2_s2 <= filtered_s2;
                                h2_s3 <= filtered_s3;
                                h2_s4 <= filtered_s4;
                                reference_sensor <= strongest_sensor;
                                candidate_x_mm <= X_MIN_MM;
                                candidate_y_mm <= Y_MIN_MM;
                                candidate_z_mm <= Z_MIN_MM;
                                best_score <= {96{1'b1}};
                                state <= S_COARSE_EVAL;
                            end else begin
                                source_valid <= 1'b0;
                            end
                        end
                    end

                    S_COARSE_EVAL: begin
                        if (candidate_score_value < best_score) begin
                            best_score <= candidate_score_value;
                            best_x_mm <= candidate_x_mm;
                            best_y_mm <= candidate_y_mm;
                            best_z_mm <= candidate_z_mm;
                        end

                        if (coarse_last) begin
                            state <= S_REFINE_INIT;
                        end else if (candidate_z_mm < Z_MAX_MM) begin
                            candidate_z_mm <= candidate_z_mm + COARSE_STEP_MM;
                        end else begin
                            candidate_z_mm <= Z_MIN_MM;
                            if (candidate_y_mm < Y_MAX_MM) begin
                                candidate_y_mm <= candidate_y_mm + COARSE_STEP_MM;
                            end else begin
                                candidate_y_mm <= Y_MIN_MM;
                                candidate_x_mm <= candidate_x_mm + COARSE_STEP_MM;
                            end
                        end
                    end

                    S_REFINE_INIT: begin
                        refine_x_min_mm <= max_signed_16(best_x_mm - REFINE_RADIUS_MM, X_MIN_MM);
                        refine_x_max_mm <= min_signed_16(best_x_mm + REFINE_RADIUS_MM, X_MAX_MM);
                        refine_y_min_mm <= max_signed_16(best_y_mm - REFINE_RADIUS_MM, Y_MIN_MM);
                        refine_y_max_mm <= min_signed_16(best_y_mm + REFINE_RADIUS_MM, Y_MAX_MM);
                        refine_z_min_mm <= max_signed_16(best_z_mm - REFINE_RADIUS_MM, Z_MIN_MM);
                        refine_z_max_mm <= min_signed_16(best_z_mm + REFINE_RADIUS_MM, Z_MAX_MM);
                        candidate_x_mm <= max_signed_16(best_x_mm - REFINE_RADIUS_MM, X_MIN_MM);
                        candidate_y_mm <= max_signed_16(best_y_mm - REFINE_RADIUS_MM, Y_MIN_MM);
                        candidate_z_mm <= max_signed_16(best_z_mm - REFINE_RADIUS_MM, Z_MIN_MM);
                        best_score <= {96{1'b1}};
                        state <= S_REFINE_EVAL;
                    end

                    S_REFINE_EVAL: begin
                        if (candidate_score_value < best_score) begin
                            best_score <= candidate_score_value;
                            best_x_mm <= candidate_x_mm;
                            best_y_mm <= candidate_y_mm;
                            best_z_mm <= candidate_z_mm;
                        end

                        if (refine_last) begin
                            state <= S_OUTPUT;
                        end else if (candidate_z_mm < refine_z_max_mm) begin
                            candidate_z_mm <= candidate_z_mm + REFINE_STEP_MM;
                        end else begin
                            candidate_z_mm <= refine_z_min_mm;
                            if (candidate_y_mm < refine_y_max_mm) begin
                                candidate_y_mm <= candidate_y_mm + REFINE_STEP_MM;
                            end else begin
                                candidate_y_mm <= refine_y_min_mm;
                                candidate_x_mm <= candidate_x_mm + REFINE_STEP_MM;
                            end
                        end
                    end

                    S_OUTPUT: begin
                        if (source_valid) begin
                            source_x_q10 <= smooth_signed_q10(source_x_q10, mm_to_q10(best_x_mm));
                            source_y_q10 <= smooth_signed_q10(source_y_q10, mm_to_q10(best_y_mm));
                            source_z_q10 <= smooth_signed_q10(source_z_q10, mm_to_q10(best_z_mm));
                        end else begin
                            source_x_q10 <= mm_to_q10(best_x_mm);
                            source_y_q10 <= mm_to_q10(best_y_mm);
                            source_z_q10 <= mm_to_q10(best_z_mm);
                        end
                        source_valid <= 1'b1;
                        state <= S_IDLE;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
