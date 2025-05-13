`timescale 1ns / 1ps

// 2 bit demuxer
module demuxer2(
    input logic in,
    input logic sel,
    output logic [1:0] out
    );
    
always_comb begin
    out = 2'b00;
    if (sel == 1'b0)
        out[0] = in;
    else
        out[1] = in;
end

//assign out = sel ? {in, 1'b0} : {1'b0, in};

//always_comb
//    case (sel)
//        1'b0: out = {1'b0, in};
//        1'b1: out = {in, 1'b0};
//        default: out = 2'b00;
//    endcase;

endmodule

// 4 bit demuxer
module demuxer4(
    input logic in,
    input logic [1:0] sel,
    output logic [3:0] out
    );
    
logic [1:0] m;

demuxer2 D0(.in(in), .sel(sel[1]), .out(m));
demuxer2 D1(.in(m[0]), .sel(sel[0]), .out(out[1:0]));
demuxer2 D2(.in(m[1]), .sel(sel[0]), .out(out[3:2]));

//always_comb begin
//    out = 4'b0000;
//    if (sel[1] == 1'b0)
//        if (sel[0] == 1'b0)
//            out[0] = in;
//        else
//            out[1] = in;
//    else
//        if (sel[0] == 1'b0)
//            out[2] = in;
//        else
//            out[3] = in;
//end

//always_comb begin
//    out = 4'b0000;
//    case (sel)
//        2'b00: out[0] = in;
//        2'b01: out[1] = in;
//        2'b10: out[2] = in;
//        2'b11: out[3] = in;
//    endcase;
//end

//assign out = (sel == 2'b00) ? {3'b000, in} :
//             (sel == 2'b01) ? {2'b00, in, 1'b0} :
//             (sel == 2'b10) ? {1'b0, in, 2'b00} :
//             {in, 3'b000};

endmodule
