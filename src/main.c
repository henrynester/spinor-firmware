#include "dronecan.h"
#include "svm.h"
#include "constants.h"
#include "pins.h"
#include "setup.h"
#include "trig.h"
#include "isr.h"

#include "libcanard/canard.h"
#include "libcanard/drivers/stm32/canard_stm32.h"
#include "dsdl/dsdl_generated/include/dronecan_msgs.h"

#include "libopencm3/stm32/rcc.h" 
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"

volatile uint32_t g_uptime_ms;
volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];


#define DLY(x) for(uint32_t i=0; i<x; i++) { __asm__("nop"); }


int main(void) {
	setup();
	timer_enable_break_main_output(TIM1);
	uint16_t angle = 0; //PWM_TIMER_MAX_SAFE;
	uint32_t t0 = 0;
	uint16_t out_pwm_a, out_pwm_b, out_pwm_c;
	while(1) {
		if((g_uptime_ms - t0) > 1) {
			t0 = g_uptime_ms;
			volatile uint32_t t = systick_get_value();
			angle+=64;
			if(angle > U14_MAX) {
			       angle = 0;
			}
			svm_calc(angle, 1500, 0, 
					&out_pwm_a, &out_pwm_b, &out_pwm_c);
			timer_set_oc_value(TIM1, TIM_OC1, out_pwm_a);
			timer_set_oc_value(TIM1, TIM_OC2, out_pwm_b);
			timer_set_oc_value(TIM1, TIM_OC3, out_pwm_c);
			volatile uint32_t t1 = systick_get_value();
			__asm__("nop");
		}
		if((g_uptime_ms % 1000) < 500) {
			gpio_clear(LED_PORT, LED_R);
		} else {
			gpio_set(LED_PORT, LED_R);
		}
	}





	//uavcanInit();
	/*while(1) {
		sendCanard();
		receiveCanard();
		spinCanard();
		publishCanard();

		//gpio_toggle(GPIOA, GPIO3);
		//for (i = 0; i < 1000000; i++) {
		//	__asm__("nop");
		//}
	}*/
}

