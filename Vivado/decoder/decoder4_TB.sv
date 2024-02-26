`timescale 1ns / 1ps

module decoder4_TB();

logic [1:0] code;
logic [3:0] q;

decoder DUT(.code(code), .q(q));

initial begin
    code = 2'b00; #10;
    code = 2'b01; #10;
    code = 2'b10; #10;
    code = 2'b11; #10;
$finish;
end;

endmodule
