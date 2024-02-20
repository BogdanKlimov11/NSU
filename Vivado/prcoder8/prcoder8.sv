`timescale 1ns / 1ps

module prcoder8(
    input [7:0] in,
    output logic [2:0] code
    );
    
//always_comb
//    if (in[7] == 1'b1)
//        code = 3'b111;
//    else if (in[6] == 1'b1)
//        code = 3'b110;
//    else if (in[5] == 1'b1)
//        code = 3'b101;
//    else if (in[4] == 1'b1)
//        code = 3'b100;
//    else if (in[3] == 1'b1)
//        code = 3'b011;                
//    else if (in[2] == 1'b1)
//        code = 3'b010;
//    else if (in[1] == 1'b1)
//        code = 3'b001;
//    else
//        code = 3'b000;

always_comb
    casex (in)
        8'b1xxxxxxx: code = 3'b111;
        8'b01xxxxxx: code = 3'b110;
        8'b001xxxxx: code = 3'b101;
        8'b0001xxxx: code = 3'b100;
        8'b00001xxx: code = 3'b011;
        8'b000001xx: code = 3'b010;
        8'b0000001x: code = 3'b001;
        8'b0000000x: code = 3'b000;
    endcase

endmodule;