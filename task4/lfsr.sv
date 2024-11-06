module lfsr (
    input   logic       clk,
    output  logic [6:0] lfsr_out
);

logic [6:0] sreg;

always_ff @ (posedge clk)
    sreg <= {sreg[5:0], sreg[6] ^ sreg[2]};

assign lfsr_out = sreg;

endmodule
