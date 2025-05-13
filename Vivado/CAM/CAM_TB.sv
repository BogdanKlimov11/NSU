`timescale 1ns / 1ps

module CAM_TB();

logic clk, we;
logic [7:0] key;
logic [2:0] waddr, raddr;
logic hit;

CAM DUT(.clk(clk), .we(we), .key(key), .waddr(waddr), .raddr(raddr), .hit(hit));

always begin
    clk = 0; #5;
    clk = 1; #5;
end

initial begin
    we = 0; key = 8'h00; waddr = 3'd0; #10;
    we = 1; waddr = 3'd0; key = 8'hAA; #10;
    we = 1; waddr = 3'd1; key = 8'h55; #10;
    we = 0; key = 8'hAA; #10;
    key = 8'h55; #10;
    key = 8'hFF; #10;
    $finish;
end

endmodule