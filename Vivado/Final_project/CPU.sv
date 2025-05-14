`timescale 1ns/1ps

module CPU(
    input logic clk,
    input logic rst,
    output logic [7:0] pc,
    output logic [7:0] data_addr,
    inout logic [7:0] data_bus,
    output logic [7:0] reg_debug
);

logic [7:0] reg_file [0:7];
logic [7:0] instr;
logic [2:0] opcode;
logic [1:0] rs1, rs2, rd;
logic [7:0] imm;
logic we_reg, we_mem;

logic [7:0] instr_mem [0:255];
logic [7:0] data_mem [0:255];

assign opcode = instr[7:5];
assign rd = instr[4:3];
assign rs1 = instr[2:1];
assign rs2 = instr[0];
assign imm = (opcode == 3'b010) ? instr_mem[pc + 1] : 0;

assign data_bus = we_mem ? reg_file[rs2] : 'z;

function [7:0] add(input [7:0] a, b);
    reg [8:0] res;
    begin
        res = a + b;
        add = res[7:0];
    end
endfunction

function [7:0] sub(input [7:0] a, b);
    reg [8:0] res;
    begin
        res = a - b;
        sub = res[7:0];
    end
endfunction

function [7:0] mul(input [7:0] a, b);
    reg [15:0] res = 0;
    integer i;
    begin
        for (i = 0; i < 8; i = i + 1)
            if (b[i]) res = res + (a << i);
        mul = res[7:0];
    end
endfunction

function [7:0] div(input [7:0] a, b);
    reg [7:0] q = 0, r = 0;
    integer i;
    begin
        if (b == 0) div = 8'hFF;
        else begin
            for (i = 7; i >= 0; i = i - 1) begin
                r = {r[6:0], a[i]};
                if (r >= b) begin
                    r = r - b;
                    q[i] = 1;
                end
            end
            div = q;
        end
    end
endfunction

always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
        pc <= 0;
        reg_file <= '{default: 0};
        data_mem <= '{default: 0};
    end
    else begin
        instr <= instr_mem[pc];
        
        case (opcode)
            3'b000: reg_file[rd] <= add(reg_file[rs1], reg_file[rs2]);
            3'b001: reg_file[rd] <= sub(reg_file[rs1], reg_file[rs2]);
            3'b010: begin
                reg_file[rd] <= imm;
                pc <= pc + 1;
            end
            3'b011: reg_file[rd] <= mul(reg_file[rs1], reg_file[rs2]);
            3'b100: reg_file[rd] <= div(reg_file[rs1], reg_file[rs2]);
            3'b101: begin
                data_mem[reg_file[rs1]] <= reg_file[rs2];
                we_mem <= 1;
            end
            3'b110: reg_file[rd] <= data_mem[reg_file[rs1]];
            default: ;
        endcase
        
        pc <= pc + 1;
        we_mem <= 0;
    end
end

assign reg_debug = reg_file[0];

initial begin
    instr_mem[0] = 8'b010_00_00_00;
    instr_mem[1] = 8'b00000101;
    instr_mem[2] = 8'b010_00_01_00;
    instr_mem[3] = 8'b00000011;
    instr_mem[4] = 8'b011_00_00_01;
    instr_mem[5] = 8'b100_00_00_01;
    instr_mem[6] = 8'b111_00_00_00;
end

endmodule