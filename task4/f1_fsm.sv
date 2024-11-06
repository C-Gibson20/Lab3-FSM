module f1_fsm (
    input   logic       rst,
    input   logic       en,
    input   logic       clk,
    input   logic       trigger,
    output  logic [7:0] fsm_out
);

typedef enum {S_0, S_1, S_2, S_3, S_4, S_5, S_6, S_7, S_8} my_states;
my_states current_state, next_state;

always_ff @ (posedge clk)
        if (rst) current_state <= S_0;
        else current_state <= next_state;

always_comb
    case (current_state)
        S_0:    if (trigger) next_state = S_1;
                else    next_state = current_state;
        S_1:    if (en) next_state = S_2;
                else    next_state = current_state;
        S_2:    if (en) next_state = S_3;
                else    next_state = current_state;
        S_3:    if (en) next_state = S_4;
                else    next_state = current_state;
        S_4:    if (en) next_state = S_5;
                else    next_state = current_state;
        S_5:    if (en) next_state = S_6;
                else    next_state = current_state;
        S_6:    if (en) next_state = S_7;
                else    next_state = current_state;
        S_7:    if (en) next_state = S_8;
                else    next_state = current_state;
        S_8:    if (en) next_state = S_0;
                else    next_state = current_state;
        default: next_state = S_0;
    endcase

    always_comb
        case (current_state)
            S_0: fsm_out = 8'b00000000;
            S_1: fsm_out = 8'b00000001;
            S_2: fsm_out = 8'b00000011;
            S_3: fsm_out = 8'b00000111;
            S_4: fsm_out = 8'b00001111;
            S_5: fsm_out = 8'b00011111;
            S_6: fsm_out = 8'b00111111;
            S_7: fsm_out = 8'b01111111;
            S_8: fsm_out = 8'b11111111;
            default: fsm_out = 8'b0;
        endcase
        
endmodule
