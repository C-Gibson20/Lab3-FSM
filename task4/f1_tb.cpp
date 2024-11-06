#include "Vf1.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

#include "../vbuddy.cpp"
#define MAX_SIM_CYC 100000

int main(int argc, char **argv, char **env)
{
    int simcyc;     
    int tick;  

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
    top->cmd_seq = 0;
    top->cmd_delay = 0;
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
        
        top->cmd_seq = (top->fsm_out);
        vbdBar(top->fsm_out & 0xFF);
        
        if (top->fsm_out == 0xFF) { 
            top->cmd_delay = 1;  
            vbdInitWatch(); 
        } else {
            top->cmd_delay = 0;
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
