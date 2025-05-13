`timescale 1ns / 1ps

// TestBench for 2 bit muxer
module demuxer2_TB();

logic in;
logic sel;
logic [1:0] out;

demuxer2 DUT(.in(in), .sel(sel), .out(out));

initial begin
    in = 1; sel = 0; #10;
    in = 1; sel = 1; #10;
    in = 0; sel = 0; #10;
    in = 0; sel = 1; #10;
    $finish;
end

endmodule

// TestBench for 4 bit muxer
module demuxer4_TB();

logic in;
logic [1:0] sel;
logic [3:0] out;

demuxer4 DUT(.in(in), .sel(sel), .out(out));

initial begin
    in = 1; sel = 2'b00; #10;
    sel = 2'b01; #10;
    sel = 2'b10; #10;
    sel = 2'b11; #10;
    
    in = 0; sel = 2'b00; #10;
    sel = 2'b01; #10;
    sel = 2'b10; #10;
    sel = 2'b11; #10;
    $finish;
end

endmodule