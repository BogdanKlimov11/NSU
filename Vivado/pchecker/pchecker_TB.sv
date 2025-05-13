`timescale 1ns / 1ps

module pchecker_TB();

logic [7:0] data;
logic parity;

pchecker DUT(.data(data), .parity(parity));

initial begin
    data = 8'h00; #10;
    data = 8'h01; #10;
    data = 8'h03; #10;
    data = 8'hFF; #10;
    $finish;
end

endmodule