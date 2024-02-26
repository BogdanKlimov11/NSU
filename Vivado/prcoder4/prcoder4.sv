`timescale 1ns / 1ps

module prcoder4(
    input [3:0] in,
    output logic [1:0] code
    );

//always_comb
//    if (in[3] == 1'b1)
//        code = 2'b11;                
//    else if (in[2] == 1'b1)
//        code = 2'b10;
//    else if (in[1] == 1'b1)
//        code = 2'b01;
//    else
//        code = 2'b00;

always_comb
    casex (in)
        4'b1xxx: code = 2'b11;
        4'b01xx: code = 2'b10;
        4'b001x: code = 2'b01;
        4'b000x: code = 2'b00;
    endcase

endmodule
