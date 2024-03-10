`timescale 1ns / 1ps

// TestBench for 4 bit adder
module adder4_TB();

logic [3:0] a, b, s;
logic cin, cout;

//adder4_pr DUT(.a(a), .b(b), .cin(cin), .s(s), .cout(cout));
adder4_cn DUT(.a(a), .b(b), .cin(cin), .s(s), .cout(cout));

initial begin
    {cin, b, a} =9'b0; #10;
    repeat (511) begin
        {cin, b, a} = {cin, b, a} +1; #10;
    end
$finish;
end

endmodule

// TestBench for 16 bit adder
module adder16_TB();

logic [15:0] a, b, s;
logic cin;
logic cout;

//adder16_pr DUT(.a(a), .b(b), .cin(cin), .s(s), .cout(cout));
adder16_cn DUT(.a(a), .b(b), .cin(cin), .s(s), .cout(cout));

initial begin
    cin = 0;
    {a, b} = 32'h0; #10;
    {a, b} = 32'h0000ffff; #10;
    {a, b} = 32'hffff0000; #10;
    {a, b} = 32'hffffffff; #10;
    cin = 1;
    {a, b} = 32'h0; #10;
    {a, b} = 32'h0000ffff; #10;
    {a, b} = 32'hffff0000; #10;
    {a, b} = 32'hffffffff; #10;
$finish;
end

endmodule
