`timescale 1ns / 1ps

module coder4_TB();

logic [3:0] in;
logic [1:0] code;

coder DUT(.in(in), .code(code));

initial begin
    in = 4'b0001; #10;
    in = 4'b0010; #10;
    in = 4'b0100; #10;
    in = 4'b1000; #10;
$finish;
end

endmodule
