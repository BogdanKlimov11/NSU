`timescale 1ns / 1ps

module subtractor(
    input a,
    input b,
    input cin,
    output m,
    output cout
    );

assign m = !(!a & b & cin 
	    | !a & !b & !cin 
	    | a & b & !cin 
	    | a & !b & cin);

assign cout = !a & b & !cin 
	    | !a & !b & cin 
	    | !a & b & cin 
	    | a & b & cin;

endmodule
