`timescale 1ns / 1ps

module pchecker(
    input logic [7:0] data,
    output logic parity
    );

always_comb begin
    parity = ^data;
end

//always_comb begin
//    parity = 1'b0;
//    for (int i = 0; i < 8; i++)
//        parity = parity ^ data[i];
//end

//assign parity = data[0] ^ data[1] ^ data[2] ^ data[3] ^
//                data[4] ^ data[5] ^ data[6] ^ data[7];

endmodule