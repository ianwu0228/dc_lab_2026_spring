module carrier_reference_75hz (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               sample_tick,
    output reg signed [15:0]  sine_q15,
    output reg signed [15:0]  cosine_q15
);

    // At Fs=200 Hz, a 75 Hz reference advances by 3/8 of a cycle per sample.
    // The exact eight-sample sequence avoids phase-accumulator frequency error.
    reg [2:0] phase_index;

    always @(*) begin
        case (phase_index)
            3'd0: begin sine_q15 =  16'sd0;      cosine_q15 =  16'sd32767;  end
            3'd1: begin sine_q15 =  16'sd23170;  cosine_q15 =  16'sd23170;  end
            3'd2: begin sine_q15 =  16'sd32767;  cosine_q15 =  16'sd0;      end
            3'd3: begin sine_q15 =  16'sd23170;  cosine_q15 = -16'sd23170;  end
            3'd4: begin sine_q15 =  16'sd0;      cosine_q15 =  16'sh8000;   end
            3'd5: begin sine_q15 = -16'sd23170;  cosine_q15 = -16'sd23170;  end
            3'd6: begin sine_q15 =  16'sh8000;   cosine_q15 =  16'sd0;      end
            default: begin
                sine_q15 = -16'sd23170;
                cosine_q15 = 16'sd23170;
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            phase_index <= 3'd0;
        else if (sample_tick)
            phase_index <= phase_index + 3'd3;
    end

endmodule
