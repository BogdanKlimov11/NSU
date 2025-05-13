`timescale 1ns / 1ps

// TestBench for 8 bit demuxer
module demuxer8_TB();

logic in;
logic [2:0] sel;
logic [7:0] out;

demuxer8 DUT(.in(in), .sel(sel), .out(out));

initial begin
    in = 1; sel = 3'b000; #10;
    repeat (7) begin
        sel = sel + 1; #10;
    end
    in = 0; sel = 3'b000; #10;
    repeat (7) begin
        sel = sel + 1; #10;
    end
    $finish;
end

endmodule

// TestBench for 64 bit demuxer
module demuxer64_TB();

logic in;
logic [5:0] sel;
logic [63:0] out;

demuxer64 DUT(.in(in), .sel(sel), .out(out));

initial begin
    in = 1; sel = 6'b000000; #10;
    repeat (63) begin
        sel = sel + 1; #10;
    end
    in = 0; sel = 6'b000000; #10;
    repeat (63) begin
        sel = sel + 1; #10;
    end
    $finish;
end

endmodule
