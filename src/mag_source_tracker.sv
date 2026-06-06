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

    // Permanent-magnet DC visualization tracker.
    //
    // KEY[2] captures the background DC magnetic vector with no magnet nearby.
    // Runtime strength is |H_now - H_background|^2 for each sensor.
    //
    // For a passive magnet, a physically exact solve needs a nonlinear dipole
    // fit over position and magnetic moment. This module instead computes a
    // stable relative position for VGA display:
    //   X/Y: baseline-subtracted strength balance over the 24 mm square.
    //   Z:   heuristic from how concentrated the strongest sensor is.
    //
    // Output Q10 units:
    //   1024 = one sensor side length = 24 mm.

    localparam [31:0] MIN_TOTAL_SIGNAL = 32'd4_000;
    localparam [2:0]  STRENGTH_FILTER_SHIFT = 3'd4;
    localparam [2:0]  POSITION_FILTER_SHIFT = 3'd4;
    localparam [11:0] XY_BALANCE_GAIN_Q10 = 12'd1024;
    localparam [11:0] MAX_RATIO_SCALE_Q10 = 12'd4096;
    localparam signed [15:0] Z_HIGH_Q10 = 16'sd2560; // 2.5 units
    localparam signed [15:0] Z_LOW_Q10  = 16'sd512;  // 0.5 units

    localparam [3:0]
        S_IDLE    = 4'd0,
        S_X_START = 4'd1,
        S_X_WAIT  = 4'd2,
        S_Y_START = 4'd3,
        S_Y_WAIT  = 4'd4,
        S_Z_START = 4'd5,
        S_Z_WAIT  = 4'd6,
        S_OUTPUT  = 4'd7;

    reg [3:0] state;

    reg signed [15:0] bg1_x, bg1_y, bg1_z;
    reg signed [15:0] bg2_x, bg2_y, bg2_z;
    reg signed [15:0] bg3_x, bg3_y, bg3_z;
    reg signed [15:0] bg4_x, bg4_y, bg4_z;

    reg [31:0] filtered_s1, filtered_s2, filtered_s3, filtered_s4;
    reg [31:0] solve_s1, solve_s2, solve_s3, solve_s4;
    reg [31:0] solve_total;
    reg [31:0] solve_max;
    reg        balance_sign;
    reg signed [15:0] next_x_q10;
    reg signed [15:0] next_y_q10;
    reg signed [15:0] next_z_q10;

    reg         divider_start;
    reg  [47:0] divider_numerator;
    reg  [31:0] divider_denominator;
    wire        divider_busy;
    wire        divider_done;
    wire [47:0] divider_quotient;

    wire [31:0] signal_s1 = vector_delta_square(
        sensor1_x, sensor1_y, sensor1_z, bg1_x, bg1_y, bg1_z);
    wire [31:0] signal_s2 = vector_delta_square(
        sensor2_x, sensor2_y, sensor2_z, bg2_x, bg2_y, bg2_z);
    wire [31:0] signal_s3 = vector_delta_square(
        sensor3_x, sensor3_y, sensor3_z, bg3_x, bg3_y, bg3_z);
    wire [31:0] signal_s4 = vector_delta_square(
        sensor4_x, sensor4_y, sensor4_z, bg4_x, bg4_y, bg4_z);

    wire [31:0] next_s1 = smooth_unsigned(filtered_s1, signal_s1);
    wire [31:0] next_s2 = smooth_unsigned(filtered_s2, signal_s2);
    wire [31:0] next_s3 = smooth_unsigned(filtered_s3, signal_s3);
    wire [31:0] next_s4 = smooth_unsigned(filtered_s4, signal_s4);

    wire [33:0] next_total_wide =
        {2'd0, next_s1} +
        {2'd0, next_s2} +
        {2'd0, next_s3} +
        {2'd0, next_s4};

    wire [31:0] next_total =
        (next_total_wide[33:32] != 2'd0) ? 32'hFFFFFFFF :
        next_total_wide[31:0];

    wire [31:0] next_max_12 = (next_s1 >= next_s2) ? next_s1 : next_s2;
    wire [31:0] next_max_34 = (next_s3 >= next_s4) ? next_s3 : next_s4;
    wire [31:0] next_max = (next_max_12 >= next_max_34) ?
                            next_max_12 : next_max_34;

    wire [33:0] solve_left_wide   = {2'd0, solve_s1} + {2'd0, solve_s3};
    wire [33:0] solve_right_wide  = {2'd0, solve_s2} + {2'd0, solve_s4};
    wire [33:0] solve_bottom_wide = {2'd0, solve_s1} + {2'd0, solve_s2};
    wire [33:0] solve_top_wide    = {2'd0, solve_s3} + {2'd0, solve_s4};

    wire [31:0] solve_left =
        (solve_left_wide[33:32] != 2'd0) ? 32'hFFFFFFFF :
        solve_left_wide[31:0];
    wire [31:0] solve_right =
        (solve_right_wide[33:32] != 2'd0) ? 32'hFFFFFFFF :
        solve_right_wide[31:0];
    wire [31:0] solve_bottom =
        (solve_bottom_wide[33:32] != 2'd0) ? 32'hFFFFFFFF :
        solve_bottom_wide[31:0];
    wire [31:0] solve_top =
        (solve_top_wide[33:32] != 2'd0) ? 32'hFFFFFFFF :
        solve_top_wide[31:0];

    wire signed [32:0] x_balance =
        $signed({1'b0, solve_right}) - $signed({1'b0, solve_left});
    wire signed [32:0] y_balance =
        $signed({1'b0, solve_top}) - $signed({1'b0, solve_bottom});

    unsigned_divider u_tracker_divider (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (divider_start),
        .numerator   (divider_numerator),
        .denominator (divider_denominator),
        .busy        (divider_busy),
        .done        (divider_done),
        .quotient    (divider_quotient)
    );

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

    function automatic [47:0] scaled_abs_balance;
        input signed [32:0] balance;
        reg [32:0] magnitude;
        reg [47:0] magnitude_48;
        begin
            magnitude = balance[32] ? ((~balance) + 1'b1) : balance;
            magnitude_48 = {15'd0, magnitude};
            scaled_abs_balance = magnitude_48 * {36'd0, XY_BALANCE_GAIN_Q10};
        end
    endfunction

    function automatic signed [15:0] signed_q10_from_quotient;
        input        sign_value;
        input [47:0] quotient;
        reg signed [15:0] magnitude;
        begin
            if (quotient[47:15] != 33'd0)
                magnitude = 16'sh7FFF;
            else
                magnitude = {1'b0, quotient[14:0]};

            signed_q10_from_quotient = sign_value ? -magnitude : magnitude;
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

    function automatic signed [15:0] z_from_ratio;
        input [47:0] ratio_q10;
        reg signed [16:0] z_value;
        begin
            // ratio_q10 ~= strongest_sensor * 4096 / total.
            // Equal strengths: ratio ~= 1024 -> high Z.
            // One dominant sensor: ratio approaches 4096 -> lower Z.
            if (ratio_q10 <= 48'd1024)
                z_from_ratio = Z_HIGH_Q10;
            else begin
                z_value = Z_HIGH_Q10 -
                          (($signed({1'b0, ratio_q10[15:0]}) -
                            17'sd1024) >>> 1);
                if (z_value < Z_LOW_Q10)
                    z_from_ratio = Z_LOW_Q10;
                else
                    z_from_ratio = z_value[15:0];
            end
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
            solve_s1 <= 32'd0;
            solve_s2 <= 32'd0;
            solve_s3 <= 32'd0;
            solve_s4 <= 32'd0;
            solve_total <= 32'd1;
            solve_max <= 32'd0;
            balance_sign <= 1'b0;
            next_x_q10 <= 16'sd0;
            next_y_q10 <= 16'sd0;
            next_z_q10 <= Z_HIGH_Q10;
            source_x_q10 <= 16'sd0;
            source_y_q10 <= 16'sd0;
            source_z_q10 <= 16'sd0;
            source_valid <= 1'b0;
            divider_start <= 1'b0;
            divider_numerator <= 48'd0;
            divider_denominator <= 32'd1;
        end else begin
            divider_start <= 1'b0;

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
                            filtered_s1 <= next_s1;
                            filtered_s2 <= next_s2;
                            filtered_s3 <= next_s3;
                            filtered_s4 <= next_s4;

                            if (next_total >= MIN_TOTAL_SIGNAL) begin
                                solve_s1 <= next_s1;
                                solve_s2 <= next_s2;
                                solve_s3 <= next_s3;
                                solve_s4 <= next_s4;
                                solve_total <= next_total;
                                solve_max <= next_max;
                                state <= S_X_START;
                            end else begin
                                source_valid <= 1'b0;
                            end
                        end
                    end

                    S_X_START: begin
                        balance_sign <= x_balance[32];
                        divider_numerator <= scaled_abs_balance(x_balance);
                        divider_denominator <= (solve_total == 32'd0) ?
                                               32'd1 : solve_total;
                        divider_start <= 1'b1;
                        state <= S_X_WAIT;
                    end

                    S_X_WAIT: begin
                        if (divider_done) begin
                            next_x_q10 <= signed_q10_from_quotient(
                                balance_sign,
                                divider_quotient
                            );
                            state <= S_Y_START;
                        end
                    end

                    S_Y_START: begin
                        balance_sign <= y_balance[32];
                        divider_numerator <= scaled_abs_balance(y_balance);
                        divider_denominator <= (solve_total == 32'd0) ?
                                               32'd1 : solve_total;
                        divider_start <= 1'b1;
                        state <= S_Y_WAIT;
                    end

                    S_Y_WAIT: begin
                        if (divider_done) begin
                            next_y_q10 <= signed_q10_from_quotient(
                                balance_sign,
                                divider_quotient
                            );
                            state <= S_Z_START;
                        end
                    end

                    S_Z_START: begin
                        divider_numerator <= {16'd0, solve_max} *
                                             {36'd0, MAX_RATIO_SCALE_Q10};
                        divider_denominator <= (solve_total == 32'd0) ?
                                               32'd1 : solve_total;
                        divider_start <= 1'b1;
                        state <= S_Z_WAIT;
                    end

                    S_Z_WAIT: begin
                        if (divider_done) begin
                            next_z_q10 <= z_from_ratio(divider_quotient);
                            state <= S_OUTPUT;
                        end
                    end

                    S_OUTPUT: begin
                        if (source_valid) begin
                            source_x_q10 <= smooth_signed_q10(source_x_q10, next_x_q10);
                            source_y_q10 <= smooth_signed_q10(source_y_q10, next_y_q10);
                            source_z_q10 <= smooth_signed_q10(source_z_q10, next_z_q10);
                        end else begin
                            source_x_q10 <= next_x_q10;
                            source_y_q10 <= next_y_q10;
                            source_z_q10 <= next_z_q10;
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
