`timescale 1ns / 1ps

module multiply(
    input clk,
    input [3:0] a,
    input [3:0] b,
    output logic [7:0] m
    );

logic [3:0] pp [3:0]; // partial production
logic [7:0] ps [1:0]; // partial sum

assign pp[0] = b & {4{a[0]}};
assign pp[1] = b & {4{a[1]}};
assign pp[2] = b & {4{a[2]}};
assign pp[3] = b & {4{a[3]}};
    
always_ff @(posedge clk) begin
    // первый такт
    ps[0] <= {4'b0000, pp[0]} + {3'b000, pp[1], 1'b0};
    ps[1] <= {2'b00, pp[2], 2'b00} + {1'b0, pp[3], 3'b000};
    // Второй такт
    m <= ps[1] + ps[0];
end

endmodule

/* схема на одном сумматоре для 8-битных операндов
   с использованием сдвиговых регистров */
module multiplys(
    input clk,
    input start,
    input [7:0] a,
    input [7:0] b,
    output logic ready,
    output logic [15:0] m
    );

logic [8:0] s;
// сдвиговые регисты; старшие и младшие биты (high and low)
logic [7:0] ar, br, mh, ml;

enum logic [3:0] {idle, st0, st1, st2, st3, st4, st5, st6, st7} state = idle;

always_ff @(posedge clk)
    case (state)
    idle: if (start == 1'b0)
            state <= idle;
          else
            state <= st0;
    st0: state <= st1;
    st1: state <= st2;
    st2: state <= st3;
    st3: state <= st4;
    st4: state <= st5;
    st5: state <= st6;
    st6: state <= st7;
    st7: if (start == 1'b1)
            state <= st0;
         else
            state <= idle;
    endcase

always_ff @(posedge clk) begin
    if (start) begin
        ar <= a;
        br <= b;
    end else
        ar <= {1'b0, ar[7:1]};
end
        
assign s = ((state == st0) ? 9'b000000000 : {1'b0, mh}) + ({1'b0, (br & {8{ar[0]}})});

always_ff @(posedge clk) begin
    mh <= s[8:1];
    ml <= {s[0], ml[7:1]};
end

assign m = {mh, ml};

always_ff @(posedge clk)
    ready <= (state == st7);

endmodule
