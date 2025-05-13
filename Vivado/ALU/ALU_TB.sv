`timescale 1ns / 1ps

module ALU_TB();

logic [7:0] a, b, result;
logic [2:0] op;
logic zero;

ALU DUT(.a(a), .b(b), .op(op), .result(result), .zero(zero));

initial begin
    a = 8'h0A; b = 8'h05; op = 3'b000; #10;
    op = 3'b001; #10;
    op = 3'b010; #10;
    op = 3'b011; #10;
    op = 3'b100; #10;
    op = 3'b101; #10;
    op = 3'b110; #10;
    a = 8'h00; b = 8'h00; op = 3'b000; #10;
    $finish;
end

endmodule