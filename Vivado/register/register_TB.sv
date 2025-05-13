`timescale 1ns / 1ps

module register_TB();

logic clk, reset, en;
logic [7:0] d, q;

register DUT(.clk(clk), .reset(reset), .en(en), .d(d), .q(q));

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    reset = 1; en = 0; d = 8'h00; #10;
    reset = 0; en = 1; d = 8'hAA; #10;
    en = 0; d = 8'hFF; #10;
    en = 1; d = 8'h55; #10;
    reset = 1; #10;
    $finish;
end

endmodule