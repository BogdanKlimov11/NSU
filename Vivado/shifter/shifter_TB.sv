`timescale 1ns / 1ps

module shifter_TB();

logic [7:0] in;
logic [2:0] sh;
logic [7:0] q;

shifter DUT(.in(in), .sh(sh), .q(q));

initial begin
    in = 8'b00000001;
    sh = 3'b111;
    repeat (8) begin
        repeat (8) begin
            sh = sh + 1; #10;
        end;
        in = in << 1;
    end
    $finish;
end

endmodule
