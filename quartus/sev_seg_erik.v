/*
File: sev_seg_erik.v
Modified by: Erik Van Schijndel
Course; ECE 643
Instructor: Don Gruenbacher
*/
module sev_seg_erik(x_in, segs);
    input [4:0] x_in;
    output [6:0] segs;

    assign segs =
        (x_in == 5'h00) ? 7'b1000000 : // 0
        (x_in == 5'h01) ? 7'b1111001 : // 1
        (x_in == 5'h02) ? 7'b0100100 : // 2
        (x_in == 5'h03) ? 7'b0110000 : // 3
        (x_in == 5'h04) ? 7'b0011001 : // 4
        (x_in == 5'h05) ? 7'b0010010 : // 5
        (x_in == 5'h06) ? 7'b0000010 : // 6 
        (x_in == 5'h07) ? 7'b1111000 : // 7
        (x_in == 5'h08) ? 7'b0000000 : // 8
        (x_in == 5'h09) ? 7'b0010000 : // 9
        (x_in == 5'h0A) ? 7'b0001000 : // A
        (x_in == 5'h0B) ? 7'b0000011 : // B
        (x_in == 5'h0C) ? 7'b1000110 : // C
        (x_in == 5'h0D) ? 7'b0100001 : // D
        (x_in == 5'h0E) ? 7'b0000110 : // E
        (x_in == 5'h0F) ? 7'b0001110 : // F
        (x_in == 5'h10) ? 7'b1000010 : // G
        (x_in == 5'h11) ? 7'b0001001 : // H
        (x_in == 5'h12) ? 7'b1111001 : // I
        (x_in == 5'h13) ? 7'b1100001 : // J
        (x_in == 5'h14) ? 7'b0111111 : // K (Cannot display (-))
        (x_in == 5'h15) ? 7'b1000111 : // L
        (x_in == 5'h16) ? 7'b0111111 : // M (Cannot display (-))
        (x_in == 5'h17) ? 7'b0101011 : // N
        (x_in == 5'h18) ? 7'b0100011 : // O
        (x_in == 5'h19) ? 7'b0001100 : // P
        (x_in == 5'h1A) ? 7'b0011000 : // Q
        (x_in == 5'h1B) ? 7'b0101111 : // R
        (x_in == 5'h1C) ? 7'b0010010 : // S
        (x_in == 5'h1D) ? 7'b0000111 : // T
        (x_in == 5'h1E) ? 7'b1000001 : // U
        (x_in == 5'h1F) ? 7'b0111111 : // V (Cannot display (-))
        (x_in == 5'h20) ? 7'b0111111 : // W (Cannot display (-))
        (x_in == 5'h21) ? 7'b0111111 : // X (Cannot display (-))
        (x_in == 5'h22) ? 7'b0010001 : // Y
        (x_in == 5'h23) ? 7'b0100100 : // Z
        7'b0111111 ; // Default (all off)

endmodule
