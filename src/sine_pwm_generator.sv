module sine_pwm_generator #(
    parameter integer PHASE_WIDTH = 32,
    parameter integer PWM_BITS = 10
) (
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire [PHASE_WIDTH-1:0]   phase_increment,
    output reg                      pwm_out,
    output reg  [PWM_BITS-1:0]      pwm_duty
);

    // Numerically controlled oscillator feeding a sine-shaped PWM duty cycle.
    //
    // Output frequency:
    //   f_out = phase_increment * f_clk / 2^PHASE_WIDTH
    //
    // For f_clk=50 MHz and PHASE_WIDTH=32:
    //   phase_increment = round(f_out * 2^32 / 50_000_000)
    //
    // 75 Hz:
    //   phase_increment = 6442
    //
    // With PWM_BITS=10 and f_clk=50 MHz:
    //   PWM carrier = 50 MHz / 1024 = 48.828 kHz.

    localparam [PWM_BITS-1:0] PWM_CENTER = 10'd512;

    reg [PHASE_WIDTH-1:0] phase_accumulator;
    reg [PWM_BITS-1:0]    pwm_counter;

    wire [7:0] sine_phase = phase_accumulator[PHASE_WIDTH-1 -: 8];
    wire [1:0] quadrant = sine_phase[7:6];
    wire [5:0] quarter_index = sine_phase[5:0];

    reg signed [10:0] sine_amplitude;
    reg signed [11:0] duty_signed;

    function automatic [9:0] quarter_sine_q10;
        input [5:0] index;
        begin
            case (index)
                6'd0:  quarter_sine_q10 = 10'd0;
                6'd1:  quarter_sine_q10 = 10'd13;
                6'd2:  quarter_sine_q10 = 10'd25;
                6'd3:  quarter_sine_q10 = 10'd38;
                6'd4:  quarter_sine_q10 = 10'd50;
                6'd5:  quarter_sine_q10 = 10'd63;
                6'd6:  quarter_sine_q10 = 10'd75;
                6'd7:  quarter_sine_q10 = 10'd87;
                6'd8:  quarter_sine_q10 = 10'd100;
                6'd9:  quarter_sine_q10 = 10'd112;
                6'd10: quarter_sine_q10 = 10'd124;
                6'd11: quarter_sine_q10 = 10'd136;
                6'd12: quarter_sine_q10 = 10'd148;
                6'd13: quarter_sine_q10 = 10'd160;
                6'd14: quarter_sine_q10 = 10'd172;
                6'd15: quarter_sine_q10 = 10'd184;
                6'd16: quarter_sine_q10 = 10'd196;
                6'd17: quarter_sine_q10 = 10'd207;
                6'd18: quarter_sine_q10 = 10'd218;
                6'd19: quarter_sine_q10 = 10'd230;
                6'd20: quarter_sine_q10 = 10'd241;
                6'd21: quarter_sine_q10 = 10'd252;
                6'd22: quarter_sine_q10 = 10'd263;
                6'd23: quarter_sine_q10 = 10'd273;
                6'd24: quarter_sine_q10 = 10'd284;
                6'd25: quarter_sine_q10 = 10'd294;
                6'd26: quarter_sine_q10 = 10'd304;
                6'd27: quarter_sine_q10 = 10'd314;
                6'd28: quarter_sine_q10 = 10'd324;
                6'd29: quarter_sine_q10 = 10'd334;
                6'd30: quarter_sine_q10 = 10'd343;
                6'd31: quarter_sine_q10 = 10'd352;
                6'd32: quarter_sine_q10 = 10'd361;
                6'd33: quarter_sine_q10 = 10'd370;
                6'd34: quarter_sine_q10 = 10'd379;
                6'd35: quarter_sine_q10 = 10'd387;
                6'd36: quarter_sine_q10 = 10'd395;
                6'd37: quarter_sine_q10 = 10'd403;
                6'd38: quarter_sine_q10 = 10'd410;
                6'd39: quarter_sine_q10 = 10'd418;
                6'd40: quarter_sine_q10 = 10'd425;
                6'd41: quarter_sine_q10 = 10'd432;
                6'd42: quarter_sine_q10 = 10'd438;
                6'd43: quarter_sine_q10 = 10'd445;
                6'd44: quarter_sine_q10 = 10'd451;
                6'd45: quarter_sine_q10 = 10'd456;
                6'd46: quarter_sine_q10 = 10'd462;
                6'd47: quarter_sine_q10 = 10'd467;
                6'd48: quarter_sine_q10 = 10'd472;
                6'd49: quarter_sine_q10 = 10'd477;
                6'd50: quarter_sine_q10 = 10'd481;
                6'd51: quarter_sine_q10 = 10'd485;
                6'd52: quarter_sine_q10 = 10'd489;
                6'd53: quarter_sine_q10 = 10'd492;
                6'd54: quarter_sine_q10 = 10'd496;
                6'd55: quarter_sine_q10 = 10'd499;
                6'd56: quarter_sine_q10 = 10'd501;
                6'd57: quarter_sine_q10 = 10'd503;
                6'd58: quarter_sine_q10 = 10'd505;
                6'd59: quarter_sine_q10 = 10'd507;
                6'd60: quarter_sine_q10 = 10'd509;
                6'd61: quarter_sine_q10 = 10'd510;
                6'd62: quarter_sine_q10 = 10'd510;
                default: quarter_sine_q10 = 10'd511;
            endcase
        end
    endfunction

    always @(*) begin
        case (quadrant)
            2'd0:
                sine_amplitude = $signed({1'b0, quarter_sine_q10(quarter_index)});
            2'd1:
                sine_amplitude = $signed({1'b0, quarter_sine_q10(~quarter_index)});
            2'd2:
                sine_amplitude = -$signed({1'b0, quarter_sine_q10(quarter_index)});
            default:
                sine_amplitude = -$signed({1'b0, quarter_sine_q10(~quarter_index)});
        endcase

        duty_signed = $signed({1'b0, PWM_CENTER}) + sine_amplitude;

        if (duty_signed < 12'sd0)
            pwm_duty = {PWM_BITS{1'b0}};
        else if (duty_signed > 12'sd1023)
            pwm_duty = {PWM_BITS{1'b1}};
        else
            pwm_duty = duty_signed[PWM_BITS-1:0];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_accumulator <= {PHASE_WIDTH{1'b0}};
            pwm_counter <= {PWM_BITS{1'b0}};
            pwm_out <= 1'b0;
        end else begin
            phase_accumulator <= phase_accumulator + phase_increment;
            pwm_counter <= pwm_counter + 1'b1;
            pwm_out <= (pwm_counter < pwm_duty);
        end
    end

endmodule
