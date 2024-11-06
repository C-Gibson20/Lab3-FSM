module mux(
    input   logic       input0,
    input   logic       input1,
    input   logic       select,
    output  logic       mux_out
);

assign mux_out = select ? input1 : input0;

endmodule
