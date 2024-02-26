`timescale 1ns / 1ps

// TestBench for 2 bit muxer
module muxer2_TB();

logic [1:0] in;
logic       sel;
logic       q;

muxe2 DUT(.in(in), .sel(sel), .q(q));

initial begin
    sel = 0; in = 2'b01; #10;
    sel = 0; in = 2'b10; #10;
    sel = 1; in = 2'b10; #10;
    sel = 1; in = 2'b01; #10;
$finish;
end

endmodule

// TestBench for 4 bit muxer
module muxer4_TB();
logic [3:0] in;
logic [1:0] sel;
logic       q;
    
muxer4 DUT(.in(in), .sel(sel), .q(q));

initial begin
    sel = 2'b00; in = 4'h1; #10;
    repeat (3) begin
        sel = sel + 1; in = in << 1; #10;
    end;
    
    sel = 2'b00; in = 4'he; #10;
    repeat (3) begin
        sel = sel + 1; in = (in << 1) + 1; #10;
    end;
$finish;
end

endmodule
