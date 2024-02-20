`timescale 1ns / 1ps

module subtractor_TB();

logic a, b, cin, m, cout;

subtractor DUT(.a(a), .b(b), .cin(cin), .m(m), .cout(cout));

initial begin
    a = 0; b = 0; cin = 0; #10;
    a = 1; b = 0; cin = 0; #10;
    a = 0; b = 1; cin = 0; #10;
    a = 1; b = 1; cin = 0; #10;
    a = 0; b = 0; cin = 1; #10;
    a = 1; b = 0; cin = 1; #10;
    a = 0; b = 1; cin = 1; #10;
    a = 1; b = 1; cin = 1; #10;
$finish;
end

endmodule
