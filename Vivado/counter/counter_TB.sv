`timescale 1ns / 1ps

module counter_TB();

logic clk, reset, en;
logic [7:0] count;

counter DUT(.clk(clk), .reset(reset), .en(en), .count(count));

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    reset = 1; en = 0; #10;
    reset = 0; en = 1; #50;
    en = 0; #20;
    en = 1; #30;
    reset = 1; #10;
    $finish;
end

endmodule