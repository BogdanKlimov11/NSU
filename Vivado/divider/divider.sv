`timescale 1ns / 1ps

module divider(
    input logic [7:0] a, // dividend
    input logic [7:0] b, // divisor
    output logic [7:0] q, // quotient
    output logic [7:0] r // remainder
    );

always_comb begin
    if (b == 8'b0) begin
        q = 8'b0;
        r = 8'b0;
    end else begin
        q = a / b;
        r = a % b;
    end
end

//always_comb begin
//    q = 8'b0;
//    r = a;
//    case (b)
//        8'd1: q = a;
//        8'd2: begin q = a >> 1; r = a[0]; end
//        8'd4: begin q = a >> 2; r = a[1:0]; end
//        default: begin q = a / b; r = a % b; end
//    endcase;
//end

//logic [7:0] temp_a, temp_q;
//always_comb begin
//    temp_a = a;
//    temp_q = 8'b0;
//    r = 8'b0;
//    if (b != 8'b0) begin
//        while (temp_a >= b) begin
//            temp_a = temp_a - b;
//            temp_q = temp_q + 1;
//        end
//        q = temp_q;
//        r = temp_a;
//    end else begin
//        q = 8'b0;
//        r = 8'b0;
//    end
//end

endmodule