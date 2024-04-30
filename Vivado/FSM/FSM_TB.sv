`timescale 1ns / 1ps

module FSM_TB();

logic clk, clear, reset, ud;
logic [3:0] q;

FSM DUT(.clk(clk), .clear(clear), .reset(reset), .ud(ud), .q(q));

always begin
    clk = 0; #5;
    clk = 1; #5;
end    

initial begin
    clear = 0; reset = 0; ud = 0; #10;
    clear = 1; reset = 0; ud = 0; #10;
    clear = 0; reset = 0; ud = 1; #50;
               reset = 1; ud = 1; #10;
               reset = 0; ud = 0; #50;
$finish;
end

endmodule


module CNT_TB();

logic clk, clear, q;
logic [2:0] div;

CNT DUT(.clk(clk), .clear(clear), .div(div), .q(q));

always begin
    clk = 0; #5;
    clk = 1; #5;
end    

initial begin
    clear = 1; div = 3'o2; #10;
    clear = 0; #200;
    repeat (5)
    begin
        clear = 1; div = div + 1; #10;
        clear = 0; #200;
    end

$finish;
end

endmodule
