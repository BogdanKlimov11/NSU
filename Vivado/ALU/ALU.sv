`timescale 1ns / 1ps

module ALU(
    input logic [7:0] a, b, // inputs
    input logic [2:0] op, // operation
    output logic [7:0] result, // result
    output logic zero // zero flag
    );

logic [7:0] sum, sub, and, or, xor, sh_left, sh_right;
logic [7:0] carry, b_comp;
logic cout;

// addition
always_comb begin
    {carry[0], sum[0]} = a[0] + b[0];
    {carry[1], sum[1]} = a[1] + b[1] + carry[0];
    {carry[2], sum[2]} = a[2] + b[2] + carry[1];
    {carry[3], sum[3]} = a[3] + b[3] + carry[2];
    {carry[4], sum[4]} = a[4] + b[4] + carry[3];
    {carry[5], sum[5]} = a[5] + b[5] + carry[4];
    {carry[6], sum[6]} = a[6] + b[6] + carry[5];
    {cout, sum[7]} = a[7] + b[7] + carry[6];
end

// subtraction
always_comb begin
    b_comp = ~b;
    {carry[0], diff[0]} = a[0] + b_comp[0] + 1'b1;
    {carry[1], diff[1]} = a[1] + b_comp[1] + carry[0];
    {carry[2], diff[2]} = a[2] + b_comp[2] + carry[1];
    {carry[3], diff[3]} = a[3] + b_comp[3] + carry[2];
    {carry[4], diff[4]} = a[4] + b_comp[4] + carry[3];
    {carry[5], diff[5]} = a[5] + b_comp[5] + carry[4];
    {carry[6], diff[6]} = a[6] + b_comp[6] + carry[5];
    {cout, sub[7]} = a[7] + b_comp[7] + carry[6];
end

// bitwise operations
always_comb begin
    and = {8{a[7]&b[7], a[6]&b[6], a[5]&b[5], a[4]&b[4], 
           a[3]&b[3], a[2]&b[2], a[1]&b[1], a[0]&b[0]}};
    or = {8{a[7]|b[7], a[6]|b[6], a[5]|b[5], a[4]|b[4], 
           a[3]|b[3], a[2]|b[2], a[1]|b[1], a[0]|b[0]}};
    xor = {8{a[7]^b[7], a[6]^b[6], a[5]^b[5], a[4]^b[4], 
           a[3]^b[3], a[2]^b[2], a[1]^b[1], a[0]^b[0]}};
end

// shifts
always_comb begin
    sh_left = {a[6:0], 1'b0}; // shift left
    sh_right = {1'b0, a[7:1]}; // shift right
end

// operation selection
always_comb begin
    case (op)
        3'b000: result = sum;
        3'b001: result = sub;
        3'b010: result = and;
        3'b011: result = or;
        3'b100: result = xor;
        3'b101: result = sh_left;
        3'b110: result = sh_right;
        default: result = 8'b0;
    endcase
    zero = ~|result;
end

//always_comb begin
//    carry[0] = 1'b0;
//    for (int i = 0; i < 8; i++) begin
//        {carry[i+1], sum[i]} = a[i] + b[i] + carry[i];
//    end
//    b_comp = ~b;
//    carry[0] = 1'b1;
//    for (int i = 0; i < 8; i++) begin
//        {carry[i+1], sub[i]} = a[i] + b_comp[i] + carry[i];
//    end
//    for (int i = 0; i < 8; i++) begin
//        and[i] = a[i] & b[i];
//        or[i] = a[i] | b[i];
//        xor[i] = a[i] ^ b[i];
//    end
//    sh_left = {a[6:0], 1'b0};
//    sh_right = {1'b0, a[7:1]};
//end
//
//always_comb begin
//    case (op)
//        3'b000: result = sum;
//        3'b001: result = sub;
//        3'b010: result = and;
//        3'b011: result = or;
//        3'b100: result = xor;
//        3'b101: result = sh_left;
//        3'b110: result = sh_right;
//        default: result = 8'b0;
//    endcase
//    zero = ~|result;
//end

endmodule