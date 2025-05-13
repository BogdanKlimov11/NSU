`timescale 1ns / 1ps

module LIFO_TB();

logic clk, reset, push, pop;
logic [7:0] din, dout;
logic full, empty;

LIFO DUT(.clk(clk), .reset(reset), .push(push), .pop(pop), .din(din), .dout(dout), .full(full), .empty(empty));

always begin
    clk = 0; #5;
    clk = 1; #5;
end

initial begin
    reset = 1; push = 0; pop = 0; din = 8'h00; #10;
    reset = 0; #10;
    push = 1; din = 8'h01; #10;
    din = 8'h02; #10;
    din = 8'h03; #10;
    din = 8'h04; #10;
    push = 0; #10;
    pop = 1; #40;
    pop = 0; #10;
    push = 1; din = 8'h05; #10;
    din = 8'h06; #10;
    din = 8'h07; #10;
    din = 8'h08; #10;
    din = 8'h09; #10;
    push = 0; pop = 1; #10;
    pop = 0; #10;
    $finish;
end

endmodule