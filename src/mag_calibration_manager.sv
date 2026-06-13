module mag_calibration_manager (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start_calibration,
    input  wire               finish_calibration,
    input  wire [3:0]         active_sensor_mask,
    input  wire [3:0]         sample_valid,

    input  wire signed [15:0] s1_x,
    input  wire signed [15:0] s1_y,
    input  wire signed [15:0] s1_z,
    input  wire signed [15:0] s2_x,
    input  wire signed [15:0] s2_y,
    input  wire signed [15:0] s2_z,
    input  wire signed [15:0] s3_x,
    input  wire signed [15:0] s3_y,
    input  wire signed [15:0] s3_z,
    input  wire signed [15:0] s4_x,
    input  wire signed [15:0] s4_y,
    input  wire signed [15:0] s4_z,

    output wire               collecting,
    output wire               calculating,
    output wire               calibration_done,
    output wire               calibration_error,

    output wire signed [15:0] s1_offset_x,
    output wire signed [15:0] s1_offset_y,
    output wire signed [15:0] s1_offset_z,
    output wire signed [15:0] s2_offset_x,
    output wire signed [15:0] s2_offset_y,
    output wire signed [15:0] s2_offset_z,
    output wire signed [15:0] s3_offset_x,
    output wire signed [15:0] s3_offset_y,
    output wire signed [15:0] s3_offset_z,
    output wire signed [15:0] s4_offset_x,
    output wire signed [15:0] s4_offset_y,
    output wire signed [15:0] s4_offset_z,

    output wire [31:0]        s1_scale_x_q16,
    output wire [31:0]        s1_scale_y_q16,
    output wire [31:0]        s1_scale_z_q16,
    output wire [31:0]        s2_scale_x_q16,
    output wire [31:0]        s2_scale_y_q16,
    output wire [31:0]        s2_scale_z_q16,
    output wire [31:0]        s3_scale_x_q16,
    output wire [31:0]        s3_scale_y_q16,
    output wire [31:0]        s3_scale_z_q16,
    output wire [31:0]        s4_scale_x_q16,
    output wire [31:0]        s4_scale_y_q16,
    output wire [31:0]        s4_scale_z_q16
);

    localparam [2:0]
        S_IDLE        = 3'd0,
        S_COLLECT     = 3'd1,
        S_PREPARE     = 3'd2,
        S_MEAN_START  = 3'd3,
        S_MEAN_WAIT   = 3'd4,
        S_SCALE_START = 3'd5,
        S_SCALE_WAIT  = 3'd6,
        S_DONE        = 3'd7;

    localparam [31:0] UNITY_SCALE_Q16 = 32'd65_536;
    // At +/-2 G sensitivity, Earth's field normally produces a radius of
    // several thousand counts. Smaller spans indicate incomplete rotation.
    localparam [15:0] MIN_VALID_RADIUS = 16'd2_500;

    reg [2:0] state;
    reg [3:0] sensors_seen;
    reg [3:0] axis_index;
    reg [19:0] radius_sum;
    reg [15:0] target_radius;
    reg        coverage_error;

    reg signed [15:0] min_value [0:11];
    reg signed [15:0] max_value [0:11];
    reg signed [15:0] offset_value [0:11];
    reg        [15:0] radius_value [0:11];
    reg        [31:0] scale_value_q16 [0:11];

    wire signed [15:0] raw_value [0:11];
    wire [11:0] axis_active;
    wire [11:0] axis_sample_valid;
    wire [2:0] active_sensor_count =
        {2'd0, active_sensor_mask[0]} +
        {2'd0, active_sensor_mask[1]} +
        {2'd0, active_sensor_mask[2]} +
        {2'd0, active_sensor_mask[3]};
    wire [4:0] active_axis_count =
        {2'd0, active_sensor_count} + {active_sensor_count, 1'b0};

    assign raw_value[0]  = s1_x;
    assign raw_value[1]  = s1_y;
    assign raw_value[2]  = s1_z;
    assign raw_value[3]  = s2_x;
    assign raw_value[4]  = s2_y;
    assign raw_value[5]  = s2_z;
    assign raw_value[6]  = s3_x;
    assign raw_value[7]  = s3_y;
    assign raw_value[8]  = s3_z;
    assign raw_value[9]  = s4_x;
    assign raw_value[10] = s4_y;
    assign raw_value[11] = s4_z;

    assign axis_active = {
        {3{active_sensor_mask[3]}},
        {3{active_sensor_mask[2]}},
        {3{active_sensor_mask[1]}},
        {3{active_sensor_mask[0]}}
    };

    assign axis_sample_valid = axis_active & {
        {3{sample_valid[3]}},
        {3{sample_valid[2]}},
        {3{sample_valid[1]}},
        {3{sample_valid[0]}}
    };

    wire signed [16:0] current_min_extended =
        {min_value[axis_index][15], min_value[axis_index]};
    wire signed [16:0] current_max_extended =
        {max_value[axis_index][15], max_value[axis_index]};
    wire signed [16:0] current_extrema_sum =
        current_min_extended + current_max_extended;
    wire        [16:0] current_extrema_span =
        current_max_extended - current_min_extended;
    wire        [15:0] current_radius = current_extrema_span[16:1];

    reg         divider_start;
    reg  [47:0] divider_numerator;
    reg  [31:0] divider_denominator;
    wire        divider_busy;
    wire        divider_done;
    wire [47:0] divider_quotient;

    unsigned_divider u_divider (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (divider_start),
        .numerator   (divider_numerator),
        .denominator (divider_denominator),
        .busy        (divider_busy),
        .done        (divider_done),
        .quotient    (divider_quotient)
    );

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            sensors_seen <= 4'd0;
            axis_index <= 4'd0;
            radius_sum <= 20'd0;
            target_radius <= 16'd0;
            coverage_error <= 1'b0;
            divider_start <= 1'b0;
            divider_numerator <= 48'd0;
            divider_denominator <= 32'd0;

            for (i = 0; i < 12; i = i + 1) begin
                min_value[i] <= 16'sh7FFF;
                max_value[i] <= 16'sh8000;
                offset_value[i] <= 16'sd0;
                radius_value[i] <= 16'd0;
                scale_value_q16[i] <= UNITY_SCALE_Q16;
            end
        end else begin
            divider_start <= 1'b0;

            if (start_calibration) begin
                state <= S_COLLECT;
                sensors_seen <= 4'd0;
                axis_index <= 4'd0;
                radius_sum <= 20'd0;
                target_radius <= 16'd0;
                coverage_error <= 1'b0;

                for (i = 0; i < 12; i = i + 1) begin
                    min_value[i] <= 16'sh7FFF;
                    max_value[i] <= 16'sh8000;
                    offset_value[i] <= 16'sd0;
                    radius_value[i] <= 16'd0;
                    scale_value_q16[i] <= UNITY_SCALE_Q16;
                end
            end else begin
                case (state)
                    S_IDLE: begin
                        // Hold unity correction until the first calibration.
                    end

                    S_COLLECT: begin
                        sensors_seen <= sensors_seen |
                                        (sample_valid & active_sensor_mask);

                        for (i = 0; i < 12; i = i + 1) begin
                            if (axis_sample_valid[i]) begin
                                if (raw_value[i] < min_value[i])
                                    min_value[i] <= raw_value[i];
                                if (raw_value[i] > max_value[i])
                                    max_value[i] <= raw_value[i];
                            end
                        end

                        if (finish_calibration &&
                            (active_sensor_mask != 4'b0000) &&
                            ((sensors_seen & active_sensor_mask) ==
                             active_sensor_mask)) begin
                            axis_index <= 4'd0;
                            radius_sum <= 20'd0;
                            state <= S_PREPARE;
                        end
                    end

                    S_PREPARE: begin
                        if (axis_active[axis_index]) begin
                            offset_value[axis_index] <= current_extrema_sum >>> 1;
                            radius_value[axis_index] <= current_radius;
                            radius_sum <= radius_sum + current_radius;
                            if (current_radius < MIN_VALID_RADIUS)
                                coverage_error <= 1'b1;
                        end else begin
                            offset_value[axis_index] <= 16'sd0;
                            radius_value[axis_index] <= 16'd0;
                        end

                        if (axis_index == 4'd11) begin
                            state <= S_MEAN_START;
                        end else begin
                            axis_index <= axis_index + 1'b1;
                        end
                    end

                    S_MEAN_START: begin
                        divider_numerator <= {28'd0, radius_sum};
                        divider_denominator <= {27'd0, active_axis_count};
                        divider_start <= 1'b1;
                        state <= S_MEAN_WAIT;
                    end

                    S_MEAN_WAIT: begin
                        if (divider_done) begin
                            target_radius <= divider_quotient[15:0];
                            axis_index <= 4'd0;
                            state <= S_SCALE_START;
                        end
                    end

                    S_SCALE_START: begin
                        if (radius_value[axis_index] == 0) begin
                            scale_value_q16[axis_index] <= UNITY_SCALE_Q16;

                            if (axis_index == 4'd11)
                                state <= S_DONE;
                            else
                                axis_index <= axis_index + 1'b1;
                        end else begin
                            divider_numerator <= {target_radius, 16'd0};
                            divider_denominator <= {16'd0, radius_value[axis_index]};
                            divider_start <= 1'b1;
                            state <= S_SCALE_WAIT;
                        end
                    end

                    S_SCALE_WAIT: begin
                        if (divider_done) begin
                            if ((divider_quotient[31:0] < 32'd32_768) ||
                                (divider_quotient[31:0] > 32'd131_072)) begin
                                scale_value_q16[axis_index] <= UNITY_SCALE_Q16;
                                coverage_error <= 1'b1;
                            end else begin
                                scale_value_q16[axis_index] <= divider_quotient[31:0];
                            end

                            if (axis_index == 4'd11) begin
                                state <= S_DONE;
                            end else begin
                                axis_index <= axis_index + 1'b1;
                                state <= S_SCALE_START;
                            end
                        end
                    end

                    S_DONE: begin
                        // Hold coefficients until calibration is restarted.
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

    assign collecting = (state == S_COLLECT);
    assign calculating = (state == S_PREPARE) ||
                         (state == S_MEAN_START) ||
                         (state == S_MEAN_WAIT) ||
                         (state == S_SCALE_START) ||
                         (state == S_SCALE_WAIT);
    assign calibration_done = (state == S_DONE) && !coverage_error;
    assign calibration_error = (state == S_DONE) && coverage_error;

    assign s1_offset_x = offset_value[0];
    assign s1_offset_y = offset_value[1];
    assign s1_offset_z = offset_value[2];
    assign s2_offset_x = offset_value[3];
    assign s2_offset_y = offset_value[4];
    assign s2_offset_z = offset_value[5];
    assign s3_offset_x = offset_value[6];
    assign s3_offset_y = offset_value[7];
    assign s3_offset_z = offset_value[8];
    assign s4_offset_x = offset_value[9];
    assign s4_offset_y = offset_value[10];
    assign s4_offset_z = offset_value[11];

    assign s1_scale_x_q16 = scale_value_q16[0];
    assign s1_scale_y_q16 = scale_value_q16[1];
    assign s1_scale_z_q16 = scale_value_q16[2];
    assign s2_scale_x_q16 = scale_value_q16[3];
    assign s2_scale_y_q16 = scale_value_q16[4];
    assign s2_scale_z_q16 = scale_value_q16[5];
    assign s3_scale_x_q16 = scale_value_q16[6];
    assign s3_scale_y_q16 = scale_value_q16[7];
    assign s3_scale_z_q16 = scale_value_q16[8];
    assign s4_scale_x_q16 = scale_value_q16[9];
    assign s4_scale_y_q16 = scale_value_q16[10];
    assign s4_scale_z_q16 = scale_value_q16[11];

endmodule
