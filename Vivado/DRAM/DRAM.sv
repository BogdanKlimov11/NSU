`timescale 1ns / 1ps

// Dynamic Random-Access Memory
module DRAM(
    input logic clk,
    input logic we,
    input logic refresh,
    input logic [2:0] addr,
    input logic [7:0] din,
    output logic [7:0] dout
    );

logic [7:0] mem [0:7];
logic [7:0] valid [0:7];
logic [2:0] refresh_addr;

always_ff @(posedge clk) begin
    if (we) begin
        mem[addr] <= din;
        valid[addr] <= 8'hFF;
    end
    if (refresh) begin
        valid[refresh_addr] <= 8'hFF;
        refresh_addr <= refresh_addr + 1;
    end
    dout <= valid[addr] ? mem[addr] : 8'h00;
end

//logic [7:0] timers [0:7];
//
//always_ff @(posedge clk) begin
//    if (we) begin
//        mem[addr] <= din;
//        timers[addr] <= 8'd10;
//    end
//    for (int i = 0; i < 8; i++) begin
//        if (timers[i] > 0)
//            timers[i] <= timers[i] - 1;
//    end
//    if (refresh) begin
//        timers[refresh_addr] <= 8'd10;
//        refresh_addr <= refresh_addr + 1;
//    end
//    dout <= (timers[addr] > 0) ? mem[addr] : 8'h00;
//end

endmodule