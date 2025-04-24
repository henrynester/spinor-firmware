#include "isr.h"

#include "libopencm3/cm3/nvic.h"

void sys_tick_handler() {
	g_uptime_ms++;
}

