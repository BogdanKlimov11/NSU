`timescale 1ns / 1ps

// Read-Only Memory
module ROM(
    input logic [2:0] addr,
    output logic [7:0] dout
    );

logic [7:0] mem [0:7];

initial begin
    mem[0] = 8'h01;
    mem[1] = 8'h02;
    mem[2] = 8'h03;
    mem[3] = 8'h04;
    mem[4] = 8'h05;
    mem[5] = 8'h06;
    mem[6] = 8'h07;
    mem[7] = 8'h08;
end

always_comb begin
    dout = mem[addr];
end

//always_comb begin
//    case (addr)
//        3'd0: dout = 8'h01;
//        3'd1: dout = 8'h02;
//        3'd2: dout = 8'h03;
//        3'd3: dout = 8'h04;
//        3'd4: dout = 8'h05;
//        3'd5: dout = 8'h06;
//        3'd6: dout = 8'h07;
//        3'd7: dout = 8'h08;
//        default: dout = 8'h00;
//    endcase
//end

endmodule