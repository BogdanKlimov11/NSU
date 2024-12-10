// ALU (Arithmetic Logic Unit) module
module alu (
    input logic [31:0] a,        // First operand
    input logic [31:0] b,        // Second operand
    input logic [1:0] alu_op,    // ALU operation signal
    output logic [31:0] result,  // Operation result
    output logic carry_out,      // Carry-out flag
    output logic zero_flag,      // Zero flag
    output logic error_flag      // Error flag (e.g., division by zero)
);

    always_comb begin
        error_flag = 0;
        case (alu_op)
            2'b00: begin  // Addition
                {carry_out, result} = a + b;
                if (result < a) error_flag = 1; // Overflow
            end
            2'b01: begin  // Subtraction
                {carry_out, result} = a - b;
                if (result > a) error_flag = 1; // Overflow
            end
            2'b10: begin  // Multiplication
                result = a * b;
            end
            2'b11: begin  // Division
                if (b == 0) begin
                    error_flag = 1; // Division by zero error
                    result = 32'b0;
                end else begin
                    result = a / b;
                end
            end
            default: begin
                result = 32'b0;
                carry_out = 0;
            end
        endcase

        // Set the zero flag
        zero_flag = (result == 0);
    end
endmodule

// Memory module
module memory (
    input logic clk,
    input logic reset,
    input logic mem_read,          // Read signal
    input logic mem_write,         // Write signal
    input logic [31:0] addr,       // Memory address
    input logic [31:0] write_data, // Data to write
    output logic [31:0] read_data  // Data read from memory
);

    logic [31:0] mem [0:1023];  // Memory array with 1024 locations of 32 bits each

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all memory locations
            foreach (mem[i]) mem[i] = 32'b0;
        end else if (mem_write) begin
            // Write data to memory at the specified address
            mem[addr] = write_data;
        end
    end

    assign read_data = (mem_read) ? mem[addr] : 32'b0;  // Read data from memory
endmodule

// Registers module
module registers (
    input logic clk,                 // Clock signal
    input logic reset,               // Reset signal
    input logic [4:0] reg_addr,      // Register address (5 bits for 32 registers)
    input logic [31:0] data_in,      // Input data
    input logic write_en,            // Write enable signal
    output logic [31:0] data_out     // Output data
);

    logic [31:0] reg_file [31:0];  // Array of 32 registers, each 32 bits wide

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            // Reset all registers
            foreach (reg_file[i]) reg_file[i] = 32'b0;
        end else if (write_en) begin
            // Write data to the register at the specified address
            reg_file[reg_addr] = data_in;
        end
    end

    assign data_out = reg_file[reg_addr];  // Read data from the register
endmodule

// Control unit module
module control_unit (
    input logic [5:0] opcode,        // Operation code (6 bits)
    output logic [1:0] alu_op,       // ALU operation signal
    output logic reg_write,          // Register write signal
    output logic [4:0] reg_addr,     // Register address for writing
    output logic [31:0] reg_data,    // Data to write to register
    output logic mem_read,           // Memory read signal
    output logic mem_write           // Memory write signal
);

    always_comb begin
        case (opcode)
            6'b000000: begin  // Addition
                alu_op = 2'b00; // ALU operation signal (e.g., 00 for addition)
                reg_write = 1;
                reg_addr = 5'b00001;
                reg_data = 32'h00000010;
                mem_read = 0;
                mem_write = 0;
            end
            6'b000001: begin  // Subtraction
                alu_op = 2'b01; // ALU operation signal (e.g., 01 for subtraction)
                reg_write = 1;
                reg_addr = 5'b00010;
                reg_data = 32'h00000020;
                mem_read = 0;
                mem_write = 0;
            end
            6'b000010: begin  // Multiplication
                alu_op = 2'b10; // ALU operation signal (e.g., 10 for multiplication)
                reg_write = 1;
                reg_addr = 5'b00011;
                reg_data = 32'h00000030;
                mem_read = 0;
                mem_write = 0;
            end
            6'b000011: begin  // Division
                alu_op = 2'b11; // ALU operation signal (e.g., 11 for division)
                reg_write = 1;
                reg_addr = 5'b00100;
                reg_data = 32'h00000040;
                mem_read = 0;
                mem_write = 0;
            end
            6'b000100: begin  // Logical AND
                alu_op = 2'b01;
                reg_write = 1;
                reg_addr = 5'b00101;
                reg_data = 32'h00000050;
                mem_read = 0;
                mem_write = 0;
            end
            6'b000101: begin  // Logical OR
                alu_op = 2'b10;
                reg_write = 1;
                reg_addr = 5'b00110;
                reg_data = 32'h00000060;
                mem_read = 0;
                mem_write = 0;
            end
            6'b000110: begin  // Memory read
                alu_op = 2'b00;
                reg_write = 0;
                reg_addr = 5'b00000;
                reg_data = 32'b0;
                mem_read = 1;
                mem_write = 0;
            end
            6'b000111: begin  // Memory write
                alu_op = 2'b00;
                reg_write = 0;
                reg_addr = 5'b00000;
                reg_data = 32'b0;
                mem_read = 0;
                mem_write = 1;
            end
            default: begin
                alu_op = 2'b00;
                reg_write = 0;
                reg_addr = 5'b00000;
                reg_data = 32'b0;
                mem_read = 0;
                mem_write = 0;
            end
        endcase
    end
endmodule

// Main processor module
module processor (
    input logic clk,                   // Clock signal
    input logic reset,                 // Reset signal
    input logic [5:0] opcode,          // Operation code (6 bits)
    input logic [31:0] addr,           // Memory address
    input logic [31:0] write_data,     // Data to write to memory
    output logic [31:0] result,        // Operation result
    output logic [31:0] memory_data    // Data from memory (for reading)
);

    // ---------------- Registers ----------------
    registers reg_unit (
        .clk(clk),
        .reset(reset),
        .reg_addr(reg_addr),
        .data_in(data_in),
        .write_en(reg_write),
        .data_out(data_out)
    );

    // ---------------- Memory module ----------------
    memory mem_unit (
        .clk(clk),
        .reset(reset),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .addr(addr),
        .write_data(write_data),
        .read_data(memory_data)
    );

    // ---------------- Control unit ----------------
    control_unit ctrl_unit (
        .opcode(opcode),
        .alu_op(alu_op),
        .reg_write(reg_write),
        .reg_addr(reg_addr),
        .reg_data(data_in),
        .mem_read(mem_read),
        .mem_write(mem_write)
    );

    // ---------------- ALU ----------------
    alu al_unit (
        .a(data_out),                  // First operand from register
        .b(data_in),                   // Second operand from register
        .alu_op(alu_op),                // ALU operation
        .result(result),                // Operation result
        .carry_out(carry_out),         // Carry-out flag
        .zero_flag(zero_flag),         // Zero flag
        .error_flag(error_flag)        // Error flag
    );
endmodule
