module mag_lockin_vector_75hz #(
    parameter integer WINDOW_SAMPLES = 200
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_tick,
    input  wire signed [15:0] field_x_counts,
    input  wire signed [15:0] field_y_counts,
    input  wire signed [15:0] field_z_counts,
    input  wire signed [15:0] sine_q15,
    input  wire signed [15:0] cosine_q15,

    output reg         [31:0] carrier_l2_squared_gauss_q16,
    output reg                result_valid
);

    // Rolling coherent lock-in window. The window storage is intentionally
    // written as synchronous RAM so Quartus can map it to M9K blocks instead
    // of implementing thousands of product-history registers in LABs.
    localparam [3:0]
        S_COLLECT    = 4'd0,
        S_UPDATE_ACC = 4'd1,
        S_CALC_XI    = 4'd2,
        S_CALC_XQ    = 4'd3,
        S_CALC_YI    = 4'd4,
        S_CALC_YQ    = 4'd5,
        S_CALC_ZI    = 4'd6,
        S_CALC_ZQ    = 4'd7;

    // The original scale used a 55-bit shift for N=200. Since power scales
    // with N^2, N=100 needs two fewer shift bits.
    localparam integer POWER_SHIFT =
        (WINDOW_SAMPLES == 100) ? 53 : 55;

    reg [3:0] state;
    reg [7:0] sample_index;
    reg [7:0] sample_count;
    reg       window_full;

    reg signed [39:0] x_i_acc, x_q_acc;
    reg signed [39:0] y_i_acc, y_q_acc;
    reg signed [39:0] z_i_acc, z_q_acc;
    reg        [81:0] power_sum;

    reg signed [31:0] new_x_i_product, new_x_q_product;
    reg signed [31:0] new_y_i_product, new_y_q_product;
    reg signed [31:0] new_z_i_product, new_z_q_product;
    reg signed [31:0] old_x_i_product, old_x_q_product;
    reg signed [31:0] old_y_i_product, old_y_q_product;
    reg signed [31:0] old_z_i_product, old_z_q_product;

    (* ramstyle = "M9K" *) reg signed [31:0] x_i_window [0:WINDOW_SAMPLES-1];
    (* ramstyle = "M9K" *) reg signed [31:0] x_q_window [0:WINDOW_SAMPLES-1];
    (* ramstyle = "M9K" *) reg signed [31:0] y_i_window [0:WINDOW_SAMPLES-1];
    (* ramstyle = "M9K" *) reg signed [31:0] y_q_window [0:WINDOW_SAMPLES-1];
    (* ramstyle = "M9K" *) reg signed [31:0] z_i_window [0:WINDOW_SAMPLES-1];
    (* ramstyle = "M9K" *) reg signed [31:0] z_q_window [0:WINDOW_SAMPLES-1];

    wire signed [31:0] x_i_product = field_x_counts * cosine_q15;
    wire signed [31:0] x_q_product = field_x_counts * sine_q15;
    wire signed [31:0] y_i_product = field_y_counts * cosine_q15;
    wire signed [31:0] y_q_product = field_y_counts * sine_q15;
    wire signed [31:0] z_i_product = field_z_counts * cosine_q15;
    wire signed [31:0] z_q_product = field_z_counts * sine_q15;

    wire signed [39:0] next_x_i =
        x_i_acc + {{8{new_x_i_product[31]}}, new_x_i_product} -
        (window_full ? {{8{old_x_i_product[31]}}, old_x_i_product} : 40'sd0);
    wire signed [39:0] next_x_q =
        x_q_acc + {{8{new_x_q_product[31]}}, new_x_q_product} -
        (window_full ? {{8{old_x_q_product[31]}}, old_x_q_product} : 40'sd0);
    wire signed [39:0] next_y_i =
        y_i_acc + {{8{new_y_i_product[31]}}, new_y_i_product} -
        (window_full ? {{8{old_y_i_product[31]}}, old_y_i_product} : 40'sd0);
    wire signed [39:0] next_y_q =
        y_q_acc + {{8{new_y_q_product[31]}}, new_y_q_product} -
        (window_full ? {{8{old_y_q_product[31]}}, old_y_q_product} : 40'sd0);
    wire signed [39:0] next_z_i =
        z_i_acc + {{8{new_z_i_product[31]}}, new_z_i_product} -
        (window_full ? {{8{old_z_i_product[31]}}, old_z_i_product} : 40'sd0);
    wire signed [39:0] next_z_q =
        z_q_acc + {{8{new_z_q_product[31]}}, new_z_q_product} -
        (window_full ? {{8{old_z_q_product[31]}}, old_z_q_product} : 40'sd0);

    reg signed [39:0] square_operand;
    wire signed [79:0] square_signed = square_operand * square_operand;
    wire        [79:0] square_value = square_signed;
    wire        [81:0] final_power_sum = power_sum + {2'd0, square_value};
    wire        [81:0] scaled_final_power = final_power_sum >> POWER_SHIFT;

    always @(*) begin
        case (state)
            S_CALC_XI: square_operand = x_i_acc;
            S_CALC_XQ: square_operand = x_q_acc;
            S_CALC_YI: square_operand = y_i_acc;
            S_CALC_YQ: square_operand = y_q_acc;
            S_CALC_ZI: square_operand = z_i_acc;
            default:   square_operand = z_q_acc;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_COLLECT;
            sample_index <= 8'd0;
            sample_count <= 8'd0;
            window_full <= 1'b0;
            x_i_acc <= 40'sd0;
            x_q_acc <= 40'sd0;
            y_i_acc <= 40'sd0;
            y_q_acc <= 40'sd0;
            z_i_acc <= 40'sd0;
            z_q_acc <= 40'sd0;
            power_sum <= 82'd0;
            new_x_i_product <= 32'sd0;
            new_x_q_product <= 32'sd0;
            new_y_i_product <= 32'sd0;
            new_y_q_product <= 32'sd0;
            new_z_i_product <= 32'sd0;
            new_z_q_product <= 32'sd0;
            old_x_i_product <= 32'sd0;
            old_x_q_product <= 32'sd0;
            old_y_i_product <= 32'sd0;
            old_y_q_product <= 32'sd0;
            old_z_i_product <= 32'sd0;
            old_z_q_product <= 32'sd0;
            carrier_l2_squared_gauss_q16 <= 32'd0;
            result_valid <= 1'b0;
        end else begin
            result_valid <= 1'b0;

            case (state)
                S_COLLECT: begin
                    if (sample_tick) begin
                        new_x_i_product <= x_i_product;
                        new_x_q_product <= x_q_product;
                        new_y_i_product <= y_i_product;
                        new_y_q_product <= y_q_product;
                        new_z_i_product <= z_i_product;
                        new_z_q_product <= z_q_product;

                        old_x_i_product <= x_i_window[sample_index];
                        old_x_q_product <= x_q_window[sample_index];
                        old_y_i_product <= y_i_window[sample_index];
                        old_y_q_product <= y_q_window[sample_index];
                        old_z_i_product <= z_i_window[sample_index];
                        old_z_q_product <= z_q_window[sample_index];

                        x_i_window[sample_index] <= x_i_product;
                        x_q_window[sample_index] <= x_q_product;
                        y_i_window[sample_index] <= y_i_product;
                        y_q_window[sample_index] <= y_q_product;
                        z_i_window[sample_index] <= z_i_product;
                        z_q_window[sample_index] <= z_q_product;

                        state <= S_UPDATE_ACC;
                    end
                end

                S_UPDATE_ACC: begin
                    x_i_acc <= next_x_i;
                    x_q_acc <= next_x_q;
                    y_i_acc <= next_y_i;
                    y_q_acc <= next_y_q;
                    z_i_acc <= next_z_i;
                    z_q_acc <= next_z_q;

                    if (sample_index == WINDOW_SAMPLES - 1)
                        sample_index <= 8'd0;
                    else
                        sample_index <= sample_index + 1'b1;

                    if (window_full) begin
                        power_sum <= 82'd0;
                        state <= S_CALC_XI;
                    end else begin
                        if (sample_count == WINDOW_SAMPLES - 1) begin
                            window_full <= 1'b1;
                            power_sum <= 82'd0;
                            state <= S_CALC_XI;
                        end else begin
                            sample_count <= sample_count + 1'b1;
                            state <= S_COLLECT;
                        end
                    end
                end

                S_CALC_XI: begin
                    power_sum <= {2'd0, square_value};
                    state <= S_CALC_XQ;
                end

                S_CALC_XQ: begin
                    power_sum <= power_sum + {2'd0, square_value};
                    state <= S_CALC_YI;
                end

                S_CALC_YI: begin
                    power_sum <= power_sum + {2'd0, square_value};
                    state <= S_CALC_YQ;
                end

                S_CALC_YQ: begin
                    power_sum <= power_sum + {2'd0, square_value};
                    state <= S_CALC_ZI;
                end

                S_CALC_ZI: begin
                    power_sum <= power_sum + {2'd0, square_value};
                    state <= S_CALC_ZQ;
                end

                S_CALC_ZQ: begin
                    if (scaled_final_power[81:32] != 50'd0)
                        carrier_l2_squared_gauss_q16 <= 32'hFFFFFFFF;
                    else
                        carrier_l2_squared_gauss_q16 <= scaled_final_power[31:0];
                    result_valid <= 1'b1;
                    state <= S_COLLECT;
                end

                default: state <= S_COLLECT;
            endcase
        end
    end

endmodule
