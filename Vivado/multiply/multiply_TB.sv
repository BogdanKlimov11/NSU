`timescale 1ns / 1ps

module multiply_TB();

logic clk;
logic [3:0] a, b;
logic [7:0] m;

multiply DUT(.a(a), .b(b), .clk(clk), .m(m));

always begin
    clk = 0; #5;
    clk = 1; #5;
end

initial begin
    {a, b} = 8'b0000_0000; #10;
    repeat (255) begin
        {a, b} = {a, b} + 1; #10;
    end

$finish;
end

endmodule


module multiplys_TB();

logic clk, start, ready;
logic [7:0] a, b;
logic [15:0] m;

multiplys DUT(.a(a), .b(b), .clk(clk), .start(start), .ready(ready), .m(m));

always begin
    clk = 0; #5;
    clk = 1; #5;
end

initial begin
    {a, b} = 16'b00000000_00000000; start = 1; #10;
                                    start = 0; #70;
    repeat (65535) begin
        {a, b} = {a, b} + 1; start = 1; #10;
                             start = 0; #70;
    end
    #10

$finish;
end

endmodule
