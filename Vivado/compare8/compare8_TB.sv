`timescale 1ns / 1ps

module compare8_TB();

logic [7:0] a, b;
logic MORE, LESS;

compare8 DUT(.a(a), .b(b), .MORE(MORE), .LESS(LESS));

initial begin
    {b, a} = 16'b0; #10;
    repeat (65535) begin
        {b, a} = {b, a} + 1; #10;
    end;
$finish;
end

endmodule
