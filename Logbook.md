
# GTest

---
### Verification

GTest is an industry standard verification framework.
#### main.cpp

```cpp
#include "gtest/gtest.h"

int add(int a, int b) { return a + b; }

class TestAdd : public ::testing::Test
{
    void SetUp() override
    {
        // Runs before each test
    }

    void TearDown() override
    {
        // Runs after each test
    }
};

// This is how to add a test case
TEST_F(TestAdd, AddTest)
{
    // This should pass, 2 + 4 = 6
    EXPECT_EQ(add(2, 4), 6);
}

TEST_F(TestAdd, AddTest2)
{
    // This should not pass, 2 + 4 != 6
    EXPECT_EQ(add(2, 4), 7);
}

int main(int argc, char **argv)
{
    // Standard Google Test main function
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    return res;
}```

This implements a simple test which behaves as intended.

#### Output

![[]](/images/1.png)

---
# LFSR and Pseudo Random Binary Sequence Generator

---
#### lsfr.sv

```verilog
module lfsr(
    input   logic       clk,
    input   logic       rst,
    input   logic       en,
    output  logic [3:0] data_out
);

logic [3:0] sreg;

always_ff @ (posedge clk, posedge rst)
    if (rst)
        sreg <= 4'b1;
    else if(en)
        sreg <= {sreg[2:0], sreg[3] ^ sreg[2]};

assign data_out = sreg;
endmodule```

This implements the primitive polynomial $X^4+X^3+1$. On each clock cycle the $0^{th}$ bit is replaced by the $3^{rd}$ bit XORed with the $4^{th}$ bit.

#### verify.cpp

```cpp
#include "gtest/gtest.h"
#include "Vdut.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

Vdut *top;
VerilatedVcdC *tfp;
unsigned int ticks = 0;

class TestDut : public ::testing::Test
{
public:
    void SetUp() override
    {
        initializeInputs();
        runReset();
    }

    void TearDown() override
    {
    }

    void initializeInputs()
    {
        top->clk = 0;
        top->rst = 0;
        top->en = 1;
    }

    void runReset()
    {
        top->rst = 1;
        runSimulation();
        top->rst = 0;
    }

    // Runs the simulation for a clock cycle, evaluates the DUT, dumps    waveform.
    void runSimulation()
    {
        for (int clk = 0; clk < 2; clk++)
        {
            top->eval();
            tfp->dump(2 * ticks + clk);
            top->clk = !top->clk;
        }
        ticks++;

        if (Verilated::gotFinish())
        {
            exit(0);
        }
    }
};

TEST_F(TestDut, InitialStateTest)
{
    top->rst = 1;
    runSimulation();
    EXPECT_EQ(top->data_out, 0b0001);
}

TEST_F(TestDut, SequenceTestMini)
{
    runSimulation();
    EXPECT_EQ(top->data_out, 0b0010);
}

TEST_F(TestDut, SequenceTest)
{
    std::vector<int> expected = {
        0b0001,
        0b0010,
        0b0100,
        0b1001,
        0b0011,
        0b0110,
        0b1101,
        0b1010,
        0b0101,
        0b1011,
        0b0111,
        0b1111,
        0b1110,
        0b1100,
        0b1000,
        0b0001};

    for (int exp : expected)
    {
        EXPECT_EQ(top->data_out, exp);
        runSimulation();
    }
}

int main(int argc, char **argv)
{
    top = new Vdut;
    tfp = new VerilatedVcdC;

    Verilated::traceEverOn(true);
    top->trace(tfp, 99);
    tfp->open("waveform.vcd");

    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    top->final();
    tfp->close();

    delete top;
    delete tfp;

    return res;
}
```

The `lsfr.sv` module implements the correct behaviour and passed all of the test cases.

#### lsfr_7.sv

```verilog
module lfsr_7 (
    input   logic       clk,
    input   logic       rst,
    input   logic       en,
    output  logic [6:0] data_out
);

logic [6:0] sreg;
  
always_ff @ (posedge clk, posedge rst)
    if (rst)
        sreg <= 6'b1;
    else if(en)
        sreg <= {sreg[5:0], sreg[6] ^ sreg[2]};
        
assign data_out = sreg;

endmodule
```

This LSFR implements another primitive polynomial of the form $X^7+X^3+1$. The conversion from the primitive polynomial to the shift registers is exactly the same as the 4-bit LSFR.
The `verify_7.cpp` file implements checks in the same way as `verify.cpp` but scaled for the 7-bit LSFR. This LSFR again passes all of the test cases. 

#### Verification

![[]](/images/2.png)

---
# F1 Light Sequence

---

#### FSM

![[]](/images/state_diag.jpg)

#### f1_fsm.sv

```verilog
module f1_fsm (
    input   logic       rst,
    input   logic       en,
    input   logic       clk,
    output  logic [7:0] data_out
);

typedef enum {S_0, S_1, S_2, S_3, S_4, S_5, S_6, S_7, S_8} my_states;
my_states current_state, next_state;

always_ff @ (posedge clk)
        if (rst) current_state <= S_0;
        else current_state <= next_state;

always_comb
    case (current_state)
        S_0:    if (en) next_state = S_1;
                else    next_state = current_state;
        S_1:    if (en) next_state = S_2;
                else    next_state = current_state;
        S_2:    if (en) next_state = S_3;
                else    next_state = current_state;
        S_3:    if (en) next_state = S_4;
                else    next_state = current_state;
        S_4:    if (en) next_state = S_5;
                else    next_state = current_state;
        S_5:    if (en) next_state = S_6;
                else    next_state = current_state;
        S_6:    if (en) next_state = S_7;
                else    next_state = current_state;
        S_7:    if (en) next_state = S_8;
                else    next_state = current_state;
        S_8:    if (en) next_state = S_0;
                else    next_state = current_state;
        default: next_state = S_0;
    endcase

    always_comb
        case (current_state)
            S_0: data_out = 8'b00000000;
            S_1: data_out = 8'b00000001;
            S_2: data_out = 8'b00000011;
            S_3: data_out = 8'b00000111;
            S_4: data_out = 8'b00001111;
            S_5: data_out = 8'b00011111;
            S_6: data_out = 8'b00111111;
            S_7: data_out = 8'b01111111;
            S_8: data_out = 8'b11111111;
            default: data_out = 8'b0;
        endcase
endmodule
```

This `sv` module implements the state machine shown using state enumeration and cases. The state enumeration gives all of the states a state name and declares the current state and the next state. The case statements specify all of the transition logic and the output logic. Since this is a Moore machine, the outputs only depend on the current state as implemented.

#### Verification
![[]](/images/3.png)

#### f1_fsm_tb.cpp

```cpp
#include "Vf1_fsm.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include "vbuddy.cpp"

int main(int argc, char **argv, char **env) {
    int edge;
    int clk;
    
    Verilated::commandArgs(argc, argv);
    Vf1_fsm* fsm = new Vf1_fsm;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    fsm->trace (tfp, 99);
    tfp->open ("f1_fsm.vcd");

    if (vbdOpen()!=1) return(-1);
    vbdHeader("Lab 3 F1 FSM");
    fsm->clk = 1;
    fsm->rst = 0;
    fsm->en = 1;

    for (edge=0; edge<400; edge++){

        for (clk=0; clk<2; clk++) {
            tfp->dump (2*edge+clk);
            fsm->clk = !fsm->clk;
            fsm->eval ();
        }

        vbdBar(fsm->data_out & 0xFF);
        vbdCycle(edge+1);

        fsm->rst = (edge > 395);
        fsm->en = vbdFlag();
        if (Verilated::gotFinish()) exit(0);
    }

    vbdClose();
    tfp->close();
    exit(0);
}
```

This testbench cycles through the specified number of clock cycles and updates the light display based on the output of the designed FSM. Each bit of the FSM output corresponds to one of the lights on the Vbuddy light strip. 
In practice the light display sequence behaves according to the FSM as expected.

---
# Clock Prescaler and Delay Modules

---
### Clock Tick Frequency Prescaler

#### clktick.sv

```verilog
module clktick #(
    parameter WIDTH = 16
)(
  // interface signals
  input  logic             clk,      // clock
  input  logic             rst,      // reset
  input  logic             en,       // enable signal
  input  logic [WIDTH-1:0] N,        // clock divided by N+1
  output logic             tick      // tick output
);

logic [WIDTH-1:0] count;

always_ff @ (posedge clk)
    if (rst) begin
        tick <= 1'b0;
        count <= N;  
        end
    else if (en) begin
        if (count == 0) begin
            tick <= 1'b1;
            count <= N;
            end
        else begin
            tick <= 1'b0;
            count <= count - 1'b1;
            end
        end
endmodule
```

This `sv` module implements the frequency prescaler. When the count has reached $0$ from $N$ then there is a high output tick. The frequency of the clock is reduced by the factor $N$ which is desirable to set a new clock signal.


#### Verification
![[]](/images/4.png)

#### clktick_tb.cpp

```cpp
#include "Vclktick.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include "../vbuddy.cpp" // include vbuddy code
#define MAX_SIM_CYC 100000

int main(int argc, char **argv, char **env)
{
    int simcyc;     // simulation clock count
    int tick;       // each clk cycle has two ticks for two edges
    int lights = 0; // state to toggle LED lights

    Verilated::commandArgs(argc, argv);
    // init top verilog instance
    Vclktick *top = new Vclktick;
    // init trace dump
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("clktick.vcd");

    // init Vbuddy
    if (vbdOpen() != 1)
        return (-1);
    vbdHeader("L3T2:Clktick");
    vbdSetMode(1); // Flag mode set to one-shot

    // initialize simulation inputs
    top->clk = 1;
    top->rst = 0;
    top->en = 0;
    top->N = vbdValue();

    // run simulation for MAX_SIM_CYC clock cycles
    for (simcyc = 0; simcyc < MAX_SIM_CYC; simcyc++)
    {
        // dump variables into VCD file and toggle clock
        for (tick = 0; tick < 2; tick++)
        {
            tfp->dump(2 * simcyc + tick);
            top->clk = !top->clk;
            top->eval();
        }

        // Display toggle neopixel
        if (top->tick)
        {
            vbdBar(lights);
            lights = lights ^ 0xFF;
        }
        // set up input signals of testbench
        top->rst = (simcyc < 2); // assert reset for 1st cycle
        top->en = (simcyc > 2);
        top->N = vbdValue();
        vbdCycle(simcyc);

        if (Verilated::gotFinish() || vbdGetkey() == 'q')
            exit(0);
    }

    vbdClose(); // ++++
    tfp->close();
    exit(0);
}
```

This testbench cycles through the specified number of clock cycles and flashes the Vbuddy light strip when the clktick frequency prescaler outputs a high signal. The Vbuddy value sets the value of $N$ for the clktick module.
For my computer the Vbuddy value required for 60bpm synchronisation was V = 49.

---
### Clock Tick FSM

#### top.sv

```verilog
module top (
  input  logic clk,
  input  logic rst,
  input  logic en,
  input  logic [15:0] N,
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
```

This is the top level `sv` module which implements the FSM with frequency set by the frequency prescaler. This is achieved by combining the `clktick.sv` module and the `f1_fsm.sv` from the previous tasks.

#### clktick_fsm_tb.cpp

```cpp
#include "Vtop.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include "../vbuddy.cpp"
#define MAX_SIM_CYC 100000

int main(int argc, char **argv, char **env)
{
    int simcyc;
    int tick;  

    Verilated::commandArgs(argc, argv);
    Vtop *top = new Vtop;
    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("clktick_fsm.vcd");

    if (vbdOpen() != 1)
        return (-1);
    vbdHeader("L3T2: Clktick FSM");
    vbdSetMode(1);

    top->clk = 1;
    top->rst = 0;
    top->en = 0;
    top->N = vbdValue();

    for (simcyc = 0; simcyc < MAX_SIM_CYC; simcyc++)
    {
        for (tick = 0; tick < 2; tick++)
        {
            tfp->dump(2 * simcyc + tick);
            top->clk = !top->clk;
            top->eval();
        }

        vbdBar(top->data_out & 0xFF);
        top->rst = (simcyc < 2);
        top->en = (simcyc > 2);
        top->N = vbdValue();
        vbdCycle(simcyc);

        if (Verilated::gotFinish() || vbdGetkey() == 'q')
            exit(0);
    }

    vbdClose();
    tfp->close();
    exit(0);
}
```

This testbench implemented the desired behaviour, cycling through the FSM light sequency with a 1 second delay per transition. No new techniques were used for this test bench.

---
# Full F1 Starting Light Implementation

#### f1.sv

```verilog
// Contains mux and fsm
module f1 (
    input logic rst,
    input logic clk,
    input logic [15:0] N,
    input logic trigger,
    input logic [7:0] react_time,
    logic cmd_seq;
	logic cmd_delay;
    output logic [7:0] fsm_out,
    output logic [11:0] bcd
);

logic cmd_seq;
logic cmd_delay;
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
  .cmd_seq(cmd_seq),
  .cmd_delay(cmd_seq),
  .fsm_out (fsm_out)
);

bin2bcd myBin2bcd (
  .x (react_time),
  .BCD (bcd)
);

endmodule
```

This is the top level `sv` file which co-ordinates the signals between the modules that make up the F1 light sequence circuit. The `delay` and `clktick` modules are completely unchanged, and the `lsfr` module has had the enable signal removed. The `f1_fsm` module required more modification and I created an additional module to implement the multiplexer so that all of modules could be cleanly organised. The signal naming in this top level file was designed to be informative according to the circuit schematic notation. 
I also chose to display a decimal output for the time elapsed so I included the `bin2ncd` module from Lab 1.

#### f1_fsm.sv

```verilog
module f1_fsm (
    input   logic       rst,
    input   logic       en,
    input   logic       clk,
    input   logic       trigger,
    output  logic       cmd_seq,
    output  logic       cmd_delay,
    output  logic [7:0] fsm_out
);

typedef enum {S_0, S_1, S_2, S_3, S_4, S_5, S_6, S_7, S_8} my_states;
my_states current_state, next_state;

always_ff @ (posedge clk)
begin
        if (rst) current_state <= S_0;
        else current_state <= next_state;

        if (trigger)
        begin
            cmd_seq <= 1'b1;
            cmd_delay <= 1'b0;
        end
        else
        begin
            cmd_seq <= 1'b0;
            cmd_delay <= 1'b1;
        end
end

always_comb
    case (current_state)
        S_0:    if (trigger) next_state = S_1;
                else    next_state = current_state;
        S_1:    if (en) next_state = S_2;
                else    next_state = current_state;
        S_2:    if (en) next_state = S_3;
                else    next_state = current_state;
        S_3:    if (en) next_state = S_4;
                else    next_state = current_state;
        S_4:    if (en) next_state = S_5;
                else    next_state = current_state;
        S_5:    if (en) next_state = S_6;
                else    next_state = current_state;
        S_6:    if (en) next_state = S_7;
                else    next_state = current_state;
        S_7:    if (en) next_state = S_8;
                else    next_state = current_state;
        S_8:    if (en) next_state = S_0;
                else    next_state = current_state;
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
```

This module has been modified so that now a trigger is required for the first state transition, essentially initialising the FSM sequence. In addition the command sequence and delay outputs have been added to be used as input signal to the other blocks.

#### f1_tb.cpp

```cpp
#include "Vf1.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include "../vbuddy.cpp"
#define MAX_SIM_CYC 100000

int main(int argc, char **argv, char **env)
{
    int simcyc;    
    int tick;
    int last_state;  

    Verilated::commandArgs(argc, argv);
    Vf1 *top = new Vf1;

    Verilated::traceEverOn(true);
    VerilatedVcdC *tfp = new VerilatedVcdC;
    top->trace(tfp, 99);
    tfp->open("f1.vcd");

    if (vbdOpen() != 1)
        return (-1);

    vbdHeader("Lab 3 F1");
    vbdSetMode(1);

    top->clk = 1;
    top->rst = 0;
    top->N = vbdValue();
    top->trigger = 0;

    for (simcyc = 0; simcyc < MAX_SIM_CYC; simcyc++)
    {
        for (tick = 0; tick < 2; tick++)
        {
            tfp->dump(2 * simcyc + tick);
            top->clk = !top->clk;
            top->eval();
        }

        top->N = vbdValue();
        top->rst = (simcyc < 2);
        vbdBar(top->fsm_out & 0xFF);

        if (top->fsm_out == 0xFF) {
            last_state = 1;
        }
        else{
            last_state = 0;
        }

        if ((top->fsm_out == 0) && (last_state)) {
            vbdInitWatch();
        }

        if (vbdFlag()){
            top->trigger = 1;
            top->react_time = vbdElapsed();

            vbdHex(4, (int(top->bcd) >> 16) & 0xF);
            vbdHex(3, (int(top->bcd) >> 8) & 0xF);
            vbdHex(2, (int(top->bcd) >> 4) & 0xF);
            vbdHex(1, int(top->bcd) & 0xF);
        } else {
            top->trigger = 0;
        }

        vbdCycle(simcyc+1);

        if (Verilated::gotFinish() || vbdGetkey() == 'q')
            exit(0);
    }

    vbdClose();
    tfp->close();
    exit(0);
}
```

This testbench uses the state of the lights to start the time and the flag to stop the timer. This implementation works very successfully.