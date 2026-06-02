module vga_timing_640x480 (
    input  wire       clk_50,
    input  wire       rst_n,
    output wire       pixel_clk,
    output reg        frame_start,
    output wire [9:0] pixel_x,
    output wire [9:0] pixel_y,
    output wire       active_video,
    output wire       hsync_n,
    output wire       vsync_n
);

    // 640x480 VGA timing at approximately 60 Hz using a 25 MHz pixel clock.
    localparam H_VISIBLE = 10'd640;
    localparam H_FRONT   = 10'd16;
    localparam H_SYNC    = 10'd96;
    localparam H_TOTAL   = 10'd800;
    localparam V_VISIBLE = 10'd480;
    localparam V_FRONT   = 10'd10;
    localparam V_SYNC    = 10'd2;
    localparam V_TOTAL   = 10'd525;

    reg       pixel_phase;
    reg [9:0] h_count;
    reg [9:0] v_count;

    assign pixel_clk = pixel_phase;
    assign pixel_x = h_count;
    assign pixel_y = v_count;

    assign active_video = (h_count < H_VISIBLE) &&
                          (v_count < V_VISIBLE);
    assign hsync_n = !((h_count >= (H_VISIBLE + H_FRONT)) &&
                       (h_count <  (H_VISIBLE + H_FRONT + H_SYNC)));
    assign vsync_n = !((v_count >= (V_VISIBLE + V_FRONT)) &&
                       (v_count <  (V_VISIBLE + V_FRONT + V_SYNC)));

    // Counters advance on VGA_CLK falling edges. RGB signals then remain stable
    // before the monitor samples them on the following rising edge.
    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
            pixel_phase <= 1'b0;
            h_count     <= 10'd0;
            v_count     <= 10'd0;
            frame_start <= 1'b0;
        end else begin
            pixel_phase <= ~pixel_phase;
            frame_start <= 1'b0;

            if (pixel_phase) begin
                if (h_count == H_TOTAL - 1'b1) begin
                    h_count <= 10'd0;

                    if (v_count == V_TOTAL - 1'b1) begin
                        v_count <= 10'd0;
                        frame_start <= 1'b1;
                    end else begin
                        v_count <= v_count + 1'b1;
                    end
                end else begin
                    h_count <= h_count + 1'b1;
                end
            end
        end
    end

endmodule
