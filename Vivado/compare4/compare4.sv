`timescale 1ns / 1ps

// 2 bit compare
module compare2(
    input [1:0] a,
    input [1:0] b,
    output more,
    output less
    );
    
assign more = a[1] & !b[1]
              | !a[1] & !b[1] & a[0] & !b[0]
              | a[1] &  b[1] & a[0] & !b[0];

assign less = !a[1] &  b[1]
              | a[1] &  b[1] & !a[0] & b[0]
              | !a[1] & !b[1] & !a[0] & b[0];

endmodule

// 4 bit compare
module compare4(
    input [3:0] a,
    input [3:0] b,
    output MORE,
    output LESS
    );
    
logic [1:0] l, m;

genvar i;
generate
    for (i = 0; i < 2; i = i + 1)
    begin: label
    	compare2 COMPARE (.a(a[2*i+1 : 2*i+0]), .b(b[2*i+1 : 2*i+0]), .more(m[i]), .less(l[i]));
    end
endgenerate

//compare2 COMPARE_LOW (.a(a[1:0]), .b(b[1:0]), .more(m[0]), .less(l[0]));
//compare2 COMPARE_HIGH (.a(a[3:2]), .b(b[3:2]), .more(m[1]), .less(l[1]));
compare2 COMPARE (.a(m[1:0]), .b(l[1:0]), .more(MORE), .less(LESS));

//compare2 COMPARE_1 (.a(a[3:2]), .b(b[3:2]), .more(m[0]), .less(l[0]));
//compare2 COMPARE_2 (.a({m[0], a[1]}), .b({l[0], b[1]}), .more(m[1]), .less(l[1]));
//compare2 COMPARE_3 (.a({m[1], a[0]}), .b({l[1], b[0]}), .more(MORE), .less(LESS));

endmodule
