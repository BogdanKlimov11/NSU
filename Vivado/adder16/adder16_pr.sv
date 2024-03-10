`timescale 1ns / 1ps

// 4 bit adder
module adder4_pr(
    input [3:0] a,
    input [3:0] b,
    input cin,
    output [3:0] s,
    output cout
    );

logic [4:0] c;
logic [3:0] p, g;
assign c[0] = cin;
    
genvar i;
generate
    for (i=0; i < 4; i=i+1)
        begin: label
            assign s[i] = a[i] ^ b[i] ^ c[i];
            assign g[i] = a[i] & b[i];
            assign p[i] = a[i] ^ b[i];
//            assign c[i+1] = (a[i] & b[i]) | ((a[i] ^ b[i]) & c[i]);
        end
endgenerate
        
//assign c[1] = (g[0]) | ((p[0]) & c[0]));
//assign c[2] = (g[1]) | ((p[1]) & c[1]));
//assign c[3] = (g[2]) | ((p[2]) & c[2]));
//assign c[4] = (g[3]) | ((p[3]) & c[3]));

assign c[1] = g[0] 
            | p[0] & c[0];
assign c[2] = g[1] 
            | p[1] & g[0] 
            | p[1] & p[0] & c[0];
assign c[3] = g[2] 
            | p[2] & g[1] 
            | p[2] & p[1] & g[0] 
            | p[2] & p[1] & p[0] & c[0];
assign c[4] = g[3] 
            | p[3] & g[2] 
            | p[3] & p[2] & g[1] 
            | p[3] & p[2] & p[1] & g[0] 
            | p[3] & p[2] & p[1] & p[0] & c[0];
    
//assign p_group = p[3] & p[2] & p[1] & p[0];
//assign g_group = g[3] 
//                | p[3] & g[2] 
//                | p[3] & p[2] & g[1] 
//                | p[3] & p[2] & p[1] & g[0] ;
        
assign cout = c[4];

endmodule

// 16 bit adder
module adder16_pr(
    input [15:0] a,
    input [15:0] b,
    input cin,
    output [15:0] s,
    output cout
    );

logic [3:0] coutn;

adder4_pr ADDER(.a(a[3:0]), .b(b[3:0]), .cin(cin), .s(s[3:0]), .cout(coutn[0]));

genvar i;
generate
    for (i = 1; i < 4; i = i + 1)
    begin: label
        adder4_pr ADDER(.a(a[4*i+3:4*i]), .b(b[4*i+3:4*i]), .cin(coutn[i-1]), .s(s[4*i+3:4*i]), .cout(coutn[i]));
    end
endgenerate

assign cout = coutn[3];

endmodule
