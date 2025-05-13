`timescale 1ns / 1ps

module counter(
    input logic clk,
    input logic reset,
    input logic en,
    output logic [7:0] count
    );

always_ff @(posedge clk) begin
    if (reset)
        count <= 8'b0;
    else if (en)
        count <= count + 1;
end

//always_ff @(posedge clk) begin
//    case ({reset, en})
//        2'b1x: count <= 8'b0;
//        2'b01: count <= count + 1;
//        default: count <= count;
//    endcase
//end

endmodule