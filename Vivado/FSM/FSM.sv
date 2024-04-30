`timescale 1ns / 1ps

module FSM(
    input clk,
    input clear,
    input reset,
    input ud,
    output logic [3:0] q
    );
    
//typedef enum logic [1:0] {st0, st1, st2, st3} statetype;
//statetype state;

enum logic [3:0] {st0 = 4'b0001,
                  st1 = 4'b0010,
                  st2 = 4'b0100,
                  st3 = 4'b1000} state;
    
always_ff @(posedge clear or posedge clk)
    if (clear)
        state <= st0;
        
    else 
        case(state)
        st0: if (reset)
                state <= st0;
                 else if (ud)
                    state <= st1;
                 else
                    state <= st3;
        st1: if (reset)
                state <= st0;
                 else if (ud)
                    state <= st2;
                 else
                    state <= st0;
        st2: if (reset)
                state <= st0;
                 else if (ud)
                    state <= st3;
                 else
                    state <= st1;
        st3: if (reset)
                state <= st0;
                 else if (ud)
                    state <= st0;
                 else
                    state <= st2;
        endcase
    
assign q[0] = (state == st0);
assign q[2] = (state == st2);
assign q[3] = (state == st3);

always_comb
    if (state == st1)
        q[1] = 1'b1;
    else 
        q[1] = 1'b0;
// Synonym of assign q[1] = (state == st1);

endmodule


// счетчик по клоку с делителем частоты
module CNT(
    input clk,
    input clear,
    input [2:0] div,
    output logic q
    );
    
enum logic [2:0] {st0, st1, st2, st3, st4, st5, st6} state;
    
always_ff @(posedge clear or posedge clk)
    if (clear)
        state <= st0;

    else
        case (state)
        st0: state <= st1;
        st1: if (div == 3'o2)
                state <= st0;
             else
                state <= st2;
        st2: if (div == 3'o3)
                state <= st0;
             else
                state <= st3;
        st3: if (div == 3'o4)
                state <= st0;
             else
                state <= st4;
        st4: if (div == 3'o5)
                state <= st0;
             else
                state <= st5;
        st5: if (div == 3'o6)
                state <= st0;
             else
                state <= st6;
        st6: state <= st0;
    endcase
    
// для простого делителя частоты
//assign q = (state == st0);

// для меандра
always_comb
    begin
    if (div == 3'o2 | div == 3'o3)
        q = (state == st0);
    else if (div == 3'o4 | div == 3'o5)
        q = (state == st0) | (state == st1);
    else
        q = (state == st0) | (state == st1) | (state == st2);
    end
    
endmodule
