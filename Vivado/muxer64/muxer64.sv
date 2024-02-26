`timescale 1ns / 1ps

// 8 bit muxer
module muxer8(
    input [7:0] in,
    input [2:0] sel,
    output logic q
    );

//always_comb
//    case (sel)
//        3'b000: q = in[0];
//        3'b001: q = in[1];
//        3'b010: q = in[2];
//        3'b011: q = in[3];
//        3'b100: q = in[4];
//        3'b101: q = in[5];
//        3'b110: q = in[6];
//        3'b111: q = in[7];
//    endcase;
 
//assign q = !sel[2] & !sel[1] & !sel[0] & in[0]
//           | !sel[2] & !sel[1] & sel[0] & in[1]
//           | !sel[2] & sel[1] & !sel[0] & in[2]
//           | !sel[2] & sel[1] & sel[0] & in[3]
//           | sel[2] & !sel[1] & !sel[0] & in[4]
//           | sel[2] & !sel[1] & sel[0] & in[5]
//           | sel[2] & sel[1] & !sel[0] & in[6]
//           | sel[2] & sel[1] & sel[0] & in[7];

always_comb
    if (sel == 3'b000)
        q = in[0];
    else if (sel == 3'b001)
         q = in[1];
    else if (sel == 3'b010)
            q = in[2];
    else if(sel == 3'b011)
            q = in[3];
    else if (sel == 3'b100)
            q = in[4];
    else if (sel == 3'b101)
            q = in[5];
    else if (sel == 3'b110)
            q = in[6];
            else
            q = in[7];

endmodule

// 64 bit muxer
module muxer64(
    input [63:0] in,
    input [5:0] sel,
    output q
    );

logic [7:0] m;

muxer8 M0 (.in(in[7:0]), .sel(sel[2:0]), .q(m[0]));
muxer8 M1 (.in(in[15:8]), .sel(sel[2:0]), .q(m[1]));
muxer8 M2 (.in(in[23:16]), .sel(sel[2:0]), .q(m[2]));
muxer8 M3 (.in(in[31:24]), .sel(sel[2:0]), .q(m[3]));
muxer8 M4 (.in(in[39:32]), .sel(sel[2:0]), .q(m[4]));
muxer8 M5 (.in(in[47:40]), .sel(sel[2:0]), .q(m[5]));
muxer8 M6 (.in(in[55:48]), .sel(sel[2:0]), .q(m[6]));
muxer8 M7 (.in(in[63:56]), .sel(sel[2:0]), .q(m[7]));
muxer8 M8 (.in(m[7:0]), .sel(sel[5:3]), .q(q));

endmodule