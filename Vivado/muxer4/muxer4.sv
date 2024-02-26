`timescale 1ns / 1ps

// 2 bit muxer
module muxer2(
    input [1:0] in,
    input sel,
    output logic q
    );
    
//assign q = !sel & in[0] |
//            sel & in[1];

always_comb
    if (sel == 1'b0)
        q = in[0];
    else
        q = in[1];
    
endmodule

// 4 bit muxer
module muxer4(
    input [3:0] in,
    input [1:0] sel,
    output logic q
    );
    
//assign q = !sel[1] & !sel[0] & in[0] |
//           !sel[1] &  sel[0] & in[1] |
//            sel[1] & !sel[0] & in[2] |
//            sel[1] &  sel[0] & in[3];

//always_comb
//    if (sel[1] == 1'b0)
//        if (sel[0] == 1'b0)
//            q = in[0];
//        else
//            q = in[1];
//    else
//        if (sel[0] == 1'b0)
//            q = in[2];
//        else
//            q = in[3];

//always_comb
//    if (sel == 2'b00)
//        q = in[0];
//    else if (sel == 2'b01)
//        q = in[1];
//    else if (sel == 2'b10)
//        q = in[2];
//    else
//        q = in[3];

//always_comb
//    case (sel)
//        2'b00: q = in[0];
//        2'b01: q = in[1];
//        2'b10: q = in[2];
//        2'b11: q = in[3];
//    endcase;

logic [1:0] m;

muxer2 M0(.in(in[1:0]), .sel(sel[0]), .q(m[0]));
muxer2 M1(.in(in[3:2]), .sel(sel[0]), .q(m[1]));
muxer2 M2(.in(m[1:0]), .sel(sel[1]), .q(q));

    
endmodule
