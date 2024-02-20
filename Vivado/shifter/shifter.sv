`timescale 1ns / 1ps

module shifter (
    input [7:0] in,
    input [2:0] sh,
    output logic [7:0] q
);

always_comb begin
    case(sh)
        3'b000: q = in;
        3'b001: q = {in[6:0], in[7]};
        3'b010: q = {in[5:0], in[7:6]};
        3'b011: q = {in[4:0], in[7:5]};
        3'b100: q = {in[3:0], in[7:4]};
        3'b101: q = {in[2:0], in[7:3]};
        3'b110: q = {in[1:0], in[7:2]};
        3'b111: q = {in[0], in[7:1]};
    endcase
end

endmodule
