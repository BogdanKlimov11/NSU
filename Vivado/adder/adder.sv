`timescale 1ns / 1ps

module adder(
    input a,
    input b,
    input cin,
    output s,
    output cout
    );
    
    assign s = !a & !b & cin | 
               !a & b & !cin |
               a & !b & !cin |
               a & b & cin;

    assign cout = !a & b & cin |
                  a & !b & cin |
                  a & b & !cin |
                  a & b & cin; 
    
endmodule
