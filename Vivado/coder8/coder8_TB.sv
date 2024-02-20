`timescale 1ns / 1ps

module coder8_TB();

logic [7:0] in;
logic [2:0] code;

coder8 DUT(.in(in), .code(code));

initial begin
    in = 8'b00000001; #10;
    in = 8'b00000010; #10;
    in = 8'b00000100; #10;
    in = 8'b00001000; #10;
    in = 8'b00010000; #10;
    in = 8'b00100000; #10;
    in = 8'b01000000; #10;
    in = 8'b10000000; #10;
$finish;
end

endmodule
