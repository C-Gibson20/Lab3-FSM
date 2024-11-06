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