`timescale 1ns / 1ps

// TestBench for 2 bit compare
module compare2_TB();

logic [1:0] a, b;
logic more, less;

compare2 DUT(.a(a), .b(b), .more(more), .less(less));

initial begin
    {b, a} = 4'b0000; #10;
    repeat (15) begin
        {b, a} = {b, a} + 1; #10;
    end;
$finish;
end

endmodule

// TestBench for 4 bit copmare
module compare4_TB();

logic [3:0] a, b;
logic MORE, LESS;

compare4 DUT(.a(a), .b(b), .MORE(MORE), .LESS(LESS));

initial begin
    {b, a} = 8'b0; #10;
    repeat (255) begin
        {b, a} = {b, a} + 1; #10;
    end;
$finish;
end

endmodule
