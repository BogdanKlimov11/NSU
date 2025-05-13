`timescale 1ns / 1ps

module DRAM_TB();

logic clk, we, refresh;
logic [2:0] addr, refresh_addr;
logic [7:0] din, dout;

DRAM DUT(.clk(clk), .we(we), .refresh(refresh), .addr(addr), .din(din), .dout(dout));

always begin
    clk = 0; #5;
    clk = 1; #5;
end

initial begin
    we = 0; refresh = 0; addr = 3'd0; din = 8'h00; refresh_addr = 3'd0; #10;
    we = 1; addr = 3'd0; din = 8'hAA; #10;
    we = 1; addr = 3'd1; din = 8'h55; #10;
    we = 0; addr = 3'd0; #10;
    refresh = 1; refresh_addr = 3'd0; #10;
    refresh = 0; addr = 3'd1; #10;
    $finish;
end

endmodule