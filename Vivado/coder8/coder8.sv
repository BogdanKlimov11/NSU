`timescale 1ns / 1ps

module coder8(
    input [7:0] in,
    output logic [2:0] code
    );
    
//assign code[0] = in[7] | in[5] | in[3] | in[1];
//assign code[1] = in[2] | in[3] | in[6] | in[7];
//assign code[2] = in[4] | in[5] | in[6] | in[7];

always_comb
    case (in)
        8'b00000001 : code = 3'b000;
        8'b00000010 : code = 3'b001;
        8'b00000100 : code = 3'b010;
        8'b00001000 : code = 3'b011;
        8'b00010000 : code = 3'b100;
        8'b00100000 : code = 3'b101;
        8'b01000000 : code = 3'b110;
        8'b10000000 : code = 3'b111;
    endcase
    
endmodule