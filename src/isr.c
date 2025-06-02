#include "isr.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/dma.h"
#include "adc.h"
#include "setup.h"
#include "foc.h"
#include "encoder.h"
#include "pins.h"
#include "constants.h"
#include "math.h"
#include "ControllerStateMachine.h"
#include "flash.h"


volatile uint32_t g_uptime_ms;
void sys_tick_handler(void) {
	g_uptime_ms++;
}

volatile controller_t controller;
void dma1_channel1_isr(void) {
	gpio_clear(LED_PORT, LED_B);
	controller_update((controller_t*)&controller);
	//clear interrupt flag at end of routine - avoids retriggering
	//before the ISR completes execution
	dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
	gpio_set(LED_PORT, LED_B);
}
