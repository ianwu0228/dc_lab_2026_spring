module mag_source_tracker (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        update,
    input  wire [31:0] sensor1_h2_gauss_q16,
    input  wire [31:0] sensor2_h2_gauss_q16,
    input  wire [31:0] sensor3_h2_gauss_q16,
    input  wire [31:0] sensor4_h2_gauss_q16,
    output reg  signed [15:0] source_x_q10,
    output reg  signed [15:0] source_y_q10,
    output reg  signed [15:0] source_z_q10,
    output reg         source_valid
);

    localparam [15:0] ONE_Q10 = 16'd1024;
    localparam [31:0] MIN_TOTAL_H2_Q16 = 32'd4096; // 0.0625 G^2 total
    // Finexus Eq. 4: |H|^2 = K * r^-6 * (3cos(theta)^2 + 1).
    // This hardware path estimates r^2 with cbrt(K / |H|^2), using K as a
    // calibration constant in sensor-spacing units. Tune this for the coil.
    localparam [31:0] FIELD_K_Q16 = 32'd65536; // 1.0 G^2 at one unit, Q16.16

    localparam [3:0]
        S_IDLE     = 4'd0,
        S_D1_START = 4'd1,
        S_D1_WAIT  = 4'd2,
        S_D2_START = 4'd3,
        S_D2_WAIT  = 4'd4,
        S_D3_START = 4'd5,
        S_D3_WAIT  = 4'd6,
        S_D4_START = 4'd7,
        S_D4_WAIT  = 4'd8;

    reg [3:0] state;
    reg [31:0] h2_s1, h2_s2, h2_s3, h2_s4;
    reg [15:0] d1_sq_q10;
    reg [15:0] d2_sq_q10;
    reg [15:0] d3_sq_q10;
    reg [15:0] d4_sq_q10;

    reg         divider_start;
    reg  [47:0] divider_numerator;
    reg  [31:0] divider_denominator;
    wire        divider_busy;
    wire        divider_done;
    wire [47:0] divider_quotient;

    wire [31:0] current_sum =
        sensor1_h2_gauss_q16 +
        sensor2_h2_gauss_q16 +
        sensor3_h2_gauss_q16 +
        sensor4_h2_gauss_q16;

    unsigned_divider u_position_divider (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (divider_start),
        .numerator   (divider_numerator),
        .denominator (divider_denominator),
        .busy        (divider_busy),
        .done        (divider_done),
        .quotient    (divider_quotient)
    );

    function automatic [15:0] cube_root_q10;
        input [47:0] value_q16;
        integer i;
        reg [15:0] low_value;
        reg [15:0] high_value;
        reg [15:0] mid_value;
        reg [47:0] mid_cube;
        reg [47:0] target_q30;
        begin
            if (value_q16[47:34] != 14'd0)
                target_q30 = 48'hFFFFFFFFFFFF;
            else
                target_q30 = {value_q16[33:0], 14'd0};
            low_value = 16'd0;
            high_value = 16'd8192;
            for (i = 0; i < 13; i = i + 1) begin
                mid_value = (low_value + high_value + 1'b1) >> 1;
                mid_cube = mid_value * mid_value * mid_value;
                if (mid_cube <= target_q30)
                    low_value = mid_value;
                else
                    high_value = mid_value - 1'b1;
            end
            cube_root_q10 = low_value;
        end
    endfunction

    function automatic signed [15:0] clamp_signed_q10;
        input signed [31:0] value;
        begin
            if (value > 32'sd32767)
                clamp_signed_q10 = 16'sd32767;
            else if (value < -32'sd32768)
                clamp_signed_q10 = 16'sh8000;
            else
                clamp_signed_q10 = value[15:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            h2_s1 <= 32'd0;
            h2_s2 <= 32'd0;
            h2_s3 <= 32'd0;
            h2_s4 <= 32'd0;
            d1_sq_q10 <= ONE_Q10;
            d2_sq_q10 <= ONE_Q10;
            d3_sq_q10 <= ONE_Q10;
            d4_sq_q10 <= ONE_Q10;
            source_x_q10 <= 16'sd0;
            source_y_q10 <= 16'sd0;
            source_z_q10 <= 16'sd0;
            source_valid <= 1'b0;
            divider_start <= 1'b0;
            divider_numerator <= 48'd0;
            divider_denominator <= 32'd1;
        end else begin
            divider_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (update && (current_sum >= MIN_TOTAL_H2_Q16)) begin
                        h2_s1 <= sensor1_h2_gauss_q16;
                        h2_s2 <= sensor2_h2_gauss_q16;
                        h2_s3 <= sensor3_h2_gauss_q16;
                        h2_s4 <= sensor4_h2_gauss_q16;
                        source_valid <= 1'b0;
                        state <= S_D1_START;
                    end
                end

                S_D1_START: begin
                    divider_numerator <= {FIELD_K_Q16, 16'd0};
                    divider_denominator <= h2_s1;
                    divider_start <= 1'b1;
                    state <= S_D1_WAIT;
                end

                S_D1_WAIT: begin
                    if (divider_done) begin
                        d1_sq_q10 <= cube_root_q10(divider_quotient);
                        state <= S_D2_START;
                    end
                end

                S_D2_START: begin
                    divider_numerator <= {FIELD_K_Q16, 16'd0};
                    divider_denominator <= h2_s2;
                    divider_start <= 1'b1;
                    state <= S_D2_WAIT;
                end

                S_D2_WAIT: begin
                    if (divider_done) begin
                        d2_sq_q10 <= cube_root_q10(divider_quotient);
                        state <= S_D3_START;
                    end
                end

                S_D3_START: begin
                    divider_numerator <= {FIELD_K_Q16, 16'd0};
                    divider_denominator <= h2_s3;
                    divider_start <= 1'b1;
                    state <= S_D3_WAIT;
                end

                S_D3_WAIT: begin
                    if (divider_done) begin
                        d3_sq_q10 <= cube_root_q10(divider_quotient);
                        state <= S_D4_START;
                    end
                end

                S_D4_START: begin
                    divider_numerator <= {FIELD_K_Q16, 16'd0};
                    divider_denominator <= h2_s4;
                    divider_start <= 1'b1;
                    state <= S_D4_WAIT;
                end

                S_D4_WAIT: begin
                    if (divider_done) begin
                        d4_sq_q10 <= cube_root_q10(divider_quotient);
                        // Finexus sensor layout:
                        // S1=(0,0,0), S2=(-1,1,0), S3=(1,1,0), S4=(0,0,1).
                        source_x_q10 <= clamp_signed_q10(
                            ($signed({1'b0, d2_sq_q10}) -
                             $signed({1'b0, d3_sq_q10})) >>> 2);
                        source_y_q10 <= clamp_signed_q10(
                            $signed({1'b0, ONE_Q10}) -
                            (($signed({1'b0, d2_sq_q10}) +
                              $signed({1'b0, d3_sq_q10}) -
                              ($signed({1'b0, d1_sq_q10}) <<< 1)) >>> 2));
                        source_z_q10 <= clamp_signed_q10(
                            ($signed({1'b0, d1_sq_q10}) +
                             $signed({1'b0, ONE_Q10}) -
                             $signed({1'b0, cube_root_q10(divider_quotient)})) >>> 1);
                        source_valid <= 1'b1;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
