module top (
  input  logic clk,
  input  logic rst,
  input  logic en,
  input  logic [15:0] N,
  output logic [7:0] data_out
);

logic [7:0] tick;

clktick myClktick (
  .clk (clk),
  .rst (rst),
  .en (en),
  .N (N),
  .tick (tick)
);

f1_fsm myF1_fsm (
  .rst (rst),
  .en (tick),
  .clk(clk),
  .data_out (data_out)
);

endmodule
