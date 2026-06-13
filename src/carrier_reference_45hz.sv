module carrier_reference_45hz (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_tick,
    output reg signed [15:0]  sine_q15,
    output reg signed [15:0]  cosine_q15
);

    // At Fs=200 Hz, a 45 Hz reference advances by 9/40 of a cycle per sample.
    reg [5:0] phase_index;

    always @(*) begin
        case (phase_index)
            6'd0:  begin sine_q15 =  16'sd0;     cosine_q15 =  16'sd32767; end
            6'd1:  begin sine_q15 =  16'sd5126;  cosine_q15 =  16'sd32364; end
            6'd2:  begin sine_q15 =  16'sd10126; cosine_q15 =  16'sd31163; end
            6'd3:  begin sine_q15 =  16'sd14876; cosine_q15 =  16'sd29196; end
            6'd4:  begin sine_q15 =  16'sd19260; cosine_q15 =  16'sd26509; end
            6'd5:  begin sine_q15 =  16'sd23170; cosine_q15 =  16'sd23170; end
            6'd6:  begin sine_q15 =  16'sd26509; cosine_q15 =  16'sd19260; end
            6'd7:  begin sine_q15 =  16'sd29196; cosine_q15 =  16'sd14876; end
            6'd8:  begin sine_q15 =  16'sd31163; cosine_q15 =  16'sd10126; end
            6'd9:  begin sine_q15 =  16'sd32364; cosine_q15 =  16'sd5126;  end
            6'd10: begin sine_q15 =  16'sd32767; cosine_q15 =  16'sd0;     end
            6'd11: begin sine_q15 =  16'sd32364; cosine_q15 = -16'sd5126;  end
            6'd12: begin sine_q15 =  16'sd31163; cosine_q15 = -16'sd10126; end
            6'd13: begin sine_q15 =  16'sd29196; cosine_q15 = -16'sd14876; end
            6'd14: begin sine_q15 =  16'sd26509; cosine_q15 = -16'sd19260; end
            6'd15: begin sine_q15 =  16'sd23170; cosine_q15 = -16'sd23170; end
            6'd16: begin sine_q15 =  16'sd19260; cosine_q15 = -16'sd26509; end
            6'd17: begin sine_q15 =  16'sd14876; cosine_q15 = -16'sd29196; end
            6'd18: begin sine_q15 =  16'sd10126; cosine_q15 = -16'sd31163; end
            6'd19: begin sine_q15 =  16'sd5126;  cosine_q15 = -16'sd32364; end
            6'd20: begin sine_q15 =  16'sd0;     cosine_q15 = -16'sd32767; end
            6'd21: begin sine_q15 = -16'sd5126;  cosine_q15 = -16'sd32364; end
            6'd22: begin sine_q15 = -16'sd10126; cosine_q15 = -16'sd31163; end
            6'd23: begin sine_q15 = -16'sd14876; cosine_q15 = -16'sd29196; end
            6'd24: begin sine_q15 = -16'sd19260; cosine_q15 = -16'sd26509; end
            6'd25: begin sine_q15 = -16'sd23170; cosine_q15 = -16'sd23170; end
            6'd26: begin sine_q15 = -16'sd26509; cosine_q15 = -16'sd19260; end
            6'd27: begin sine_q15 = -16'sd29196; cosine_q15 = -16'sd14876; end
            6'd28: begin sine_q15 = -16'sd31163; cosine_q15 = -16'sd10126; end
            6'd29: begin sine_q15 = -16'sd32364; cosine_q15 = -16'sd5126;  end
            6'd30: begin sine_q15 = -16'sd32767; cosine_q15 =  16'sd0;     end
            6'd31: begin sine_q15 = -16'sd32364; cosine_q15 =  16'sd5126;  end
            6'd32: begin sine_q15 = -16'sd31163; cosine_q15 =  16'sd10126; end
            6'd33: begin sine_q15 = -16'sd29196; cosine_q15 =  16'sd14876; end
            6'd34: begin sine_q15 = -16'sd26509; cosine_q15 =  16'sd19260; end
            6'd35: begin sine_q15 = -16'sd23170; cosine_q15 =  16'sd23170; end
            6'd36: begin sine_q15 = -16'sd19260; cosine_q15 =  16'sd26509; end
            6'd37: begin sine_q15 = -16'sd14876; cosine_q15 =  16'sd29196; end
            6'd38: begin sine_q15 = -16'sd10126; cosine_q15 =  16'sd31163; end
            default: begin sine_q15 = -16'sd5126; cosine_q15 =  16'sd32364; end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            phase_index <= 6'd0;
        else if (sample_tick) begin
            if (phase_index >= 6'd31)
                phase_index <= phase_index - 6'd31;
            else
                phase_index <= phase_index + 6'd9;
        end
    end

endmodule
