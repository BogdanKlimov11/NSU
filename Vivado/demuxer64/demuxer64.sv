`timescale 1ns / 1ps

// 8 bit demuxer
module demuxer8(
    input logic in,
    input logic [2:0] sel,
    output logic [7:0] out
    );

always_comb begin
    out = 8'b00000000;
    if (sel == 3'b000)
        out[0] = in;
    else if (sel == 3'b001)
        out[1] = in;
    else if (sel == 3'b010)
        out[2] = in;
    else if (sel == 3'b011)
        out[3] = in;
    else if (sel == 3'b100)
        out[4] = in;
    else if (sel == 3'b101)
        out[5] = in;
    else if (sel == 3'b110)
        out[6] = in;
    else
        out[7] = in;
end

//always_comb begin
//    out = 8'b00000000;
//    case (sel)
//        3'b000: out[0] = in;
//        3'b001: out[1] = in;
//        3'b010: out[2] = in;
//        3'b011: out[3] = in;
//        3'b100: out[4] = in;
//        3'b101: out[5] = in;
//        3'b110: out[6] = in;
//        3'b111: out[7] = in;
//    endcase;
//end

//assign out = (sel == 3'b000) ? {7'b0000000, in} :
//             (sel == 3'b001) ? {6'b000000, in, 1'b0} :
//             (sel == 3'b010) ? {5'b00000, in, 2'b00} :
//             (sel == 3'b011) ? {4'b0000, in, 3'b000} :
//             (sel == 3'b100) ? {3'b000, in, 4'b0000} :
//             (sel == 3'b101) ? {2'b00, in, 5'b00000} :
//             (sel == 3'b110) ? {1'b0, in, 6'b000000} :
//             {in, 7'b0000000};

endmodule

// 64 bit demuxer
module demuxer64(
    input logic in,
    input logic [5:0] sel,
    output logic [63:0] out
    );

logic [7:0] m;

demuxer8 D0 (.in(in), .sel(sel[5:3]), .out(m));
demuxer8 D1 (.in(m[0]), .sel(sel[2:0]), .out(out[7:0]));
demuxer8 D2 (.in(m[1]), .sel(sel[2:0]), .out(out[15:8]));
demuxer8 D3 (.in(m[2]), .sel(sel[2:0]), .out(out[23:16]));
demuxer8 D4 (.in(m[3]), .sel(sel[2:0]), .out(out[31:24]));
demuxer8 D5 (.in(m[4]), .sel(sel[2:0]), .out(out[39:32]));
demuxer8 D6 (.in(m[5]), .sel(sel[2:0]), .out(out[47:40]));
demuxer8 D7 (.in(m[6]), .sel(sel[2:0]), .out(out[55:48]));
demuxer8 D8 (.in(m[7]), .sel(sel[2:0]), .out(out[63:56]));

endmodule
