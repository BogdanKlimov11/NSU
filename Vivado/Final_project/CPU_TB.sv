`timescale 1ns/1ps

module CPU_TB();

logic clk;
logic rst;
logic [7:0] pc;
logic [7:0] data_addr;
logic [7:0] data_bus;
logic [7:0] reg_debug;

CPU DUT(
    .clk(clk),
    .rst(rst),
    .pc(pc),
    .data_addr(data_addr),
    .data_bus(data_bus),
    .reg_debug(reg_debug)
);

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    rst = 1; #10;
    
    rst = 0;
    
    #140;
    
    assert(DUT.reg_file[0] == 5);
    assert(DUT.pc == 7);
    assert(DUT.instr == 8'b111_00_00_00);
    
    assert(DUT.data_mem[0] == 0);
    
    assert(DUT.we_reg == 0);
    assert(DUT.we_mem == 0);
    
$finish;
end

always @(posedge clk) begin
    if (!rst) begin
        assert(pc <= 7);
        
        if (DUT.we_mem)
            assert(data_addr < 256);
    end
end

endmodule