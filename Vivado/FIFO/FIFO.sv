`timescale 1ns / 1ps

// First-In First-Out
module FIFO(
    input logic clk,
    input logic reset,
    input logic push,
    input logic pop,
    input logic [7:0] din,
    output logic [7:0] dout,
    output logic full,
    output logic empty
    );

logic [7:0] mem [0:7];
logic [2:0] wr_ptr, rd_ptr;
logic [3:0] count;

always_ff @(posedge clk) begin
    if (reset) begin
        wr_ptr <= 3'b0;
        rd_ptr <= 3'b0;
        count <= 4'b0;
        dout <= 8'b0;
    end else begin
        if (push && !full) begin
            mem[wr_ptr] <= din;
            wr_ptr <= wr_ptr + 1;
            count <= count + 1;
        end
        if (pop && !empty) begin
            dout <= mem[rd_ptr];
            rd_ptr <= rd_ptr + 1;
            count <= count - 1;
        end
    end
end

always_comb begin
    full = (count == 4'd8);
    empty = (count == 4'd0);
end

//enum logic [1:0] {IDLE, PUSH, POP} state;
//
//always_ff @(posedge clk) begin
//    if (reset) begin
//        wr_ptr <= 3'b0;
//        rd_ptr <= 3'b0;
//        count <= 4'b0;
//        dout <= 8'b0;
//        state <= IDLE;
//    end else begin
//        case (state)
//            IDLE: begin
//                if (push && !full) begin
//                    mem[wr_ptr] <= din;
//                    wr_ptr <= wr_ptr + 1;
//                    count <= count + 1;
//                    state <= PUSH;
//                end else if (pop && !empty) begin
//                    dout <= mem[rd_ptr];
//                    rd_ptr <= rd_ptr + 1;
//                    count <= count - 1;
//                    state <= POP;
//                end
//            end
//            PUSH: if (!push) state <= IDLE;
//            POP: if (!pop) state <= IDLE;
//        endcase
//    end
//end
//
//always_comb begin
//    full = (count == 4'd8);
//    empty = (count == 4'd0);
//end

endmodule