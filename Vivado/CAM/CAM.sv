`timescale 1ns / 1ps

// Content-Addressable Memory
module CAM(
    input logic clk,
    input logic we,
    input logic [7:0] key,
    input logic [2:0] waddr,
    output logic [2:0] raddr,
    output logic hit
    );

logic [7:0] mem [0:7];

always_ff @(posedge clk) begin
    if (we)
        mem[waddr] <= key;
end

always_comb begin
    hit = 1'b0;
    raddr = 3'b0;
    for (int i = 0; i < 8; i++) begin
        if (mem[i] == key) begin
            hit = 1'b1;
            raddr = i;
        end
    end
end

//logic [7:0] match;
//
//always_ff @(posedge clk) begin
//    if (we)
//        mem[waddr] <= key;
//end
//
//always_comb begin
//    for (int i = 0; i < 8; i++)
//        match[i] = (mem[i] == key);
//    hit = |match;
//    raddr = match[0] ? 3'd0 :
//            match[1] ? 3'd1 :
//            match[2] ? 3'd2 :
//            match[3] ? 3'd3 :
//            match[4] ? 3'd4 :
//            match[5] ? 3'd5 :
//            match[6] ? 3'd6 :
//            match[7] ? 3'd7 : 3'd0;
//end

endmodule