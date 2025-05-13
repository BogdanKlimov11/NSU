`timescale 1ns / 1ps

module ROM_TB();

logic [2:0] addr;
logic [7:0] dout;

ROM DUT(.addr(addr), .dout(dout));

initial begin
    addr = 3'd0; #10;
    addr = 3'd1; #10;
    addr = 3'd2; #10;
    addr = 3'd3; #10;
    addr = 3'd4; #10;
    addr = 3'd5; #10;
    addr = 3'd6; #10;
    addr = 3'd7; #10;
    $finish;
end

endmodule