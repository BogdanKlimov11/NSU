`timescale 1ns / 1ps

// 4 bit Carry Look Ahead (CLA)
module CLA(
    input cin,
    // propagate group
    input [3:0] p,
    // generate group
    input [3:0] g,
    // carry
    output [3:0] c
);

assign c[0] = g[0]
            | p[0] & cin;
            
assign c[1] = g[1]
            | p[1] & g[0]
            | p[1] & p[0] & cin;
            
assign c[2] = g[2]
            | p[2] & g[1]
            | p[2] & p[1] & g[0]
            | p[2] & p[1] & p[0] & cin;

assign c[3] = g[3]
            | p[3] & g[2]
            | p[3] & p[2] & g[1]
            | p[3] & p[2] & p[1] & g[0]
            | p[3] & p[2] & p[1] & p[0] & cin;

endmodule

// 4 bit adder
module adder4_cn(
    input [3:0] a,
    input [3:0] b,
    input cin,
    output [3:0] s,
    // propagate group
    output p_group,
    // generate group
    output g_group
//    output cout
    );

logic [3:0] p, g;
logic [4:0] c;
assign c[0] = cin;

genvar i;
generate
   for (i = 0; i < 4; i = i + 1)
       begin: label
         assign s[i] = a[i] ^ b[i] ^ c[i];
         assign p[i] = a[i] ^ b[i];
         assign g[i] = a[i] & b[i];
       end
endgenerate

CLA CARRY(.cin(cin), .p(p), .g(g), .c(c[4:1]));

//assign cout = c[4];

assign p_group = p[3] & p[2] & p[1] & p[0] & cin;

assign g_group = g[3]
               | p[3] & g[2]
               | p[3] & p[2] & g[1]
               | p[3] & p[2] & p[1] & g[0];

endmodule

// 16 bit adder
module adder16_cn(
    input [15:0] a,
    input [15:0] b,
    input cin,
    output [15:0] s,
    output cout
);

logic [4:0] c;
assign c[0] = cin;
logic [4:0] p_group, g_group;

genvar i;
generate
   for (i = 0; i < 4; i = i + 1)
       begin: label
           adder4_cn ADDER(.a(a[4*(i+1)-1:4*i]),
           .b(b[4*(i+1)-1:4*i]),
           .cin(c[i]),
           .s(s[4*(i+1)-1:4*i]), 
           .p_group(p_group[i]),
           .g_group(g_group[i]));
       end
endgenerate

CLA CARRY(.cin(cin), .p(p_group), .g(g_group), .c(c[4:1]));

assign cout = c[4];

endmodule