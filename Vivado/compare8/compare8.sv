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
              |  a[1] &  b[1] & a[0] & !b[0];

assign less = !a[1] &  b[1]
              |  a[1] &  b[1] & !a[0] & b[0]
              | !a[1] & !b[1] & !a[0] & b[0];

endmodule

// 8 bit compare
module compare8(
    input [7:0] a,
    input [7:0] b,
    output MORE,
    output LESS
    );
    
//logic [7:0] l, m;

//compare2 COMPARE_1 (.a(a[7:6]), .b(b[7:6]), .more(m[0]), .less(l[0]));
//compare2 COMPARE_2 (.a({m[0], a[5]}), .b({l[0], b[5]}), .more(m[1]), .less(l[1]));
//compare2 COMPARE_3 (.a({m[1], a[4]}), .b({l[1], b[4]}), .more(m[2]), .less(l[2]));
//compare2 COMPARE_4 (.a({m[2], a[3]}), .b({l[2], b[3]}), .more(m[3]), .less(l[3]));
//compare2 COMPARE_5 (.a({m[3], a[2]}), .b({l[3], b[2]}), .more(m[4]), .less(l[4]));
//compare2 COMPARE_6 (.a({m[4], a[1]}), .b({l[4], b[1]}), .more(m[5]), .less(l[5]));
//compare2 COMPARE_7 (.a({m[5], a[0]}), .b({l[5], b[0]}), .more(MORE), .less(LESS));

//logic [3:0] l, m;
//logic [1:0] le, mo;

//compare2 COMPARE_0 (.a(a[1:0]), .b(b[1:0]), .more(m[0]), .less(l[0]));
//compare2 COMPARE_1 (.a(a[3:2]), .b(b[3:2]), .more(m[1]), .less(l[1]));
//compare2 COMPARE_2 (.a(a[5:4]), .b(b[5:4]), .more(m[2]), .less(l[2]));
//compare2 COMPARE_3 (.a(a[7:6]), .b(b[7:6]), .more(m[3]), .less(l[3]));
//compare2 COMPARE_LOW (.a(m[1:0]), .b(l[1:0]), .more(mo[0]), .less(le[0]));
//compare2 COMPARE_HIGH (.a(m[3:2]), .b(l[3:2]), .more(mo[1]), .less(le[1]));
//compare2 COMPARE (.a(mo[1:0]), .b(le[1:0]), .more(MORE), .less(LESS));

logic [3:0] l, m;
logic [1:0] le, mo;

genvar i;
generate
    for (i = 0; i < 4; i = i + 1)
    begin: label1
    	compare2 COMPARE (.a(a[2*i+1 : 2*i+0]), .b(b[2*i+1 : 2*i+0]), .more(m[i]), .less(l[i]));
    end
    
    for (i = 0; i < 2; i = i + 1)
    begin: label2
    	compare2 COMPARE_NEW(.a(m[2*i+1 : 2*i+0]), .b(l[2*i+1 : 2*i+0]), .more(mo[i]), .less(le[i]));
    end
endgenerate

    compare2  COMPARE(.a(mo[1:0]), .b(le[1:0]), .more(MORE), .less(LESS));

endmodule
