module vga_font_rom (
    input  wire [7:0] character,
    input  wire [2:0] glyph_row,
    output reg  [7:0] pixels
);

    reg [63:0] bitmap;

    // Compact 8x8 uppercase font for the dashboard labels and hexadecimal data.
    always @* begin
        case (character)
            "0": bitmap = 64'h3C666E7666663C00;
            "1": bitmap = 64'h1838181818187E00;
            "2": bitmap = 64'h3C66060C18307E00;
            "3": bitmap = 64'h3C66061C06663C00;
            "4": bitmap = 64'h0C1C3C6C7E0C0C00;
            "5": bitmap = 64'h7E607C0606663C00;
            "6": bitmap = 64'h1C30607C66663C00;
            "7": bitmap = 64'h7E66060C18181800;
            "8": bitmap = 64'h3C66663C66663C00;
            "9": bitmap = 64'h3C66663E060C3800;
            "A": bitmap = 64'h183C66667E666600;
            "B": bitmap = 64'h7C66667C66667C00;
            "C": bitmap = 64'h3C66606060663C00;
            "D": bitmap = 64'h786C6666666C7800;
            "E": bitmap = 64'h7E60607C60607E00;
            "F": bitmap = 64'h7E60607C60606000;
            "G": bitmap = 64'h3C66606E66663C00;
            "H": bitmap = 64'h6666667E66666600;
            "I": bitmap = 64'h3C18181818183C00;
            "L": bitmap = 64'h6060606060607E00;
            "M": bitmap = 64'h63777F6B63636300;
            "N": bitmap = 64'h66767E7E6E666600;
            "O": bitmap = 64'h3C66666666663C00;
            "P": bitmap = 64'h7C66667C60606000;
            "Q": bitmap = 64'h3C6666666E3C0E00;
            "R": bitmap = 64'h7C66667C6C666600;
            "S": bitmap = 64'h3C66603C06663C00;
            "T": bitmap = 64'h7E5A181818183C00;
            "U": bitmap = 64'h6666666666663C00;
            "V": bitmap = 64'h66666666663C1800;
            "W": bitmap = 64'h6363636B7F776300;
            "X": bitmap = 64'h66663C183C666600;
            "Y": bitmap = 64'h6666663C18183C00;
            "Z": bitmap = 64'h7E060C1830607E00;
            "+": bitmap = 64'h0018187E18180000;
            "-": bitmap = 64'h0000007E00000000;
            "=": bitmap = 64'h00007E007E000000;
            ":": bitmap = 64'h0018180018180000;
            ".": bitmap = 64'h0000000000181800;
            "^": bitmap = 64'h183C660000000000;
            default: bitmap = 64'd0;
        endcase

        case (glyph_row)
            3'd0: pixels = bitmap[63:56];
            3'd1: pixels = bitmap[55:48];
            3'd2: pixels = bitmap[47:40];
            3'd3: pixels = bitmap[39:32];
            3'd4: pixels = bitmap[31:24];
            3'd5: pixels = bitmap[23:16];
            3'd6: pixels = bitmap[15:8];
            default: pixels = bitmap[7:0];
        endcase
    end

endmodule
