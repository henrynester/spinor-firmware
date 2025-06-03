#pragma once
#include "constants.h"

#include <stdint.h>

void setup(void);
void start_timers(void);

// for loop: 3 instructions + 3 nop instructions 
// = 6 instr/iter * 8clock/instr = 48clocks/iter = 1us/iter at 48MHz
#define DELAY_US(x) for(uint32_t i=0; i<x; i++) { __asm__("nop;nop;nop"); }
