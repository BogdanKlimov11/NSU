`timescale 1ns / 1ps

module register(
    input logic clk,
    input logic reset,
    input logic en, // enable
    input logic [7:0] d, // data
    output logic [7:0] q // quit
    );

always_ff @(posedge clk) begin
    if (reset)
        q <= 8'b0;
    else if (en)
        q <= d;
end

//always_ff @(posedge clk) begin
//    case ({reset, en})
//        2'b1x: q <= 8'b0;
//        2'b01: q <= d;
//        default: q <= q;
//    endcase
//end

endmodule