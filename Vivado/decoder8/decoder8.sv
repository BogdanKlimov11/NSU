`timescale 1ns / 1ps

module decoder8(
    input [2:0] code,
    output logic [7:0] q
    );
    
//assign q[0] = !code[2] & !code[1] & !code[0];
//assign q[1] = !code[2] & !code[1] & code[0];
//assign q[2] = !code[2] & code[1] & !code[0];
//assign q[3] = !code[2] & code[1] & code[0];
//assign q[4] = code[2] & !code[1] & !code[0];
//assign q[5] = code[2] & !code[1] & code[0];
//assign q[6] = code[2] & code[1] & !code[0];
//assign q[7] = code[2] & code[1] & code[0];

always_comb
    case (code)
        3'b000 : q = 8'b00000001;
        3'b001 : q = 8'b00000010;
        3'b010 : q = 8'b00000100;
        3'b011 : q = 8'b00001000;
        3'b100 : q = 8'b00010000;
        3'b101 : q = 8'b00100000;
        3'b110 : q = 8'b01000000;
        3'b111 : q = 8'b10000000;
    endcase

endmodule
