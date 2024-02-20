`timescale 1ns / 1ps

module adder_TB();

logic a, b, cin, s, cout;

adder DUT(.a(a), .b(b), .s(s), .cin(cin), .cout(cout));

initial begin
    a = 0; b = 0; cin = 0; #10
    a = 0; b = 0; cin = 1; #10
    a = 0; b = 1; cin = 0; #10
    a = 0; b = 1; cin = 1; #10
    a = 1; b = 0; cin = 0; #10
    a = 1; b = 0; cin = 1; #10
    a = 1; b = 1; cin = 0; #10
    a = 1; b = 1; cin = 1; #10
$finish;
end

endmodule
