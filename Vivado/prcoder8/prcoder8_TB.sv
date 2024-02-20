`timescale 1ns / 1ps

module prcoder8_TB();

logic [7:0] in;
logic [2:0] code;

prcoder8 DUT(.in(in), .code(code));

initial begin
    in = 8'b00000000; #10;
    repeat (255) begin
        in = in + 1; #10;
    end
$finish;
end

endmodule
