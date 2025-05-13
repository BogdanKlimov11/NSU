`timescale 1ns / 1ps

module divider_TB();

logic [7:0] a, b;
logic [7:0] q, r;

divider DUT(.a(a), .b(b), .q(q), .r(r));

initial begin
    a = 8'd20; b = 8'd5; #10;
    a = 8'd15; b = 8'd3; #10;
    a = 8'd17; b = 8'd4; #10;
    a = 8'd10; b = 8'd0; #10;
    $finish;
end

endmodule