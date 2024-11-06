// Contains mux and fsm
module f1 (
    input logic rst,
    input logic clk,
    input logic [15:0] N,
    input logic trigger,
    input logic cmd_seq,
    input logic cmd_delay,
    input logic [7:0] react_time,
    output logic [7:0] fsm_out,
    output logic [11:0] bcd
);

logic tick;
logic delay;
logic mux_out;
logic [6:0] K;

lfsr myLfsr(
    .clk (clk),
    .lfsr_out (K)
);

clktick myClktick(
    .N (N),
    .en (cmd_seq),
    .rst (rst),
    .clk (clk),
    .tick (tick)
);

delay myDelay(
    .K (K),
    .trigger (cmd_delay),
    .rst (rst),
    .clk (clk),
    .time_out (delay)
);

mux myMux (
  .input0 (delay),
  .input1 (tick),
  .select (cmd_seq),
  .mux_out (mux_out)
);

f1_fsm myF1_fsm (
  .rst (rst),
  .en (mux_out),
  .trigger(trigger),
  .clk(clk),
  .fsm_out (fsm_out)
);

bin2bcd myBin2bcd (
  .x (react_time),
  .BCD (bcd)
);

endmodule
