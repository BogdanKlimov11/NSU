`timescale 1ns / 1ps

module prcoder4_TB();
logic [3:0] in;
logic [1:0] code;
prcoder4 DUT(.in(in), .code(code));

initial begin
    in = 4'b0000; #10;
    repeat (15) begin
        in = in + 1; #10;
    end
$finish;
end

endmodule