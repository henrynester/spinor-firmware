#include "flash.h"
#include "isr.h"
#include "dronecan.h"
#include "constants.h"
#include "config.h"
#include "pins.h"
#include "setup.h"
#include "math.h"
#include "ControllerStateMachine.h"

#include "libcanard/canard.h"

#include "libopencm3/stm32/rcc.h" 
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"

#include "local.SPINORStatus.h"
					
CSM_t csm;

int main(void) {
	setup();
	DELAY_US(100000);
	if(config_load() != FLASH_ERROR_OK) {
		config_defaults();
	}
	controller_init((controller_t *)&controller);
	dronecan_init((controller_in_t *)&controller.in);
	
	CSM_init(&csm, (controller_in_t *)&controller.in, (controller_out_t *)&controller.out);
	//ISR will start running at 8kHz once timer starts
	start_timers();

	//main loop
	while(1) {
		//send all queued messages
		dronecan_transmit();
		//receive just one message, handler runs
		dronecan_receive();
		//wait for next ISR run to complete, then copy input/output port structs
		controller_rendezvous_sync_inout(&controller);
		//send periodic messages. rate-limiting in fn
		dronecan_publish_NodeStatus();
		dronecan_publish_SPINORStatus((controller_out_t *)&controller.out, &csm);
		dronecan_publish_SPINORFeedback((controller_out_t*)&controller.out);

		//200Hz loop
		static uint32_t t_loop=0;
		if(g_uptime_ms > t_loop+5) {
			t_loop = g_uptime_ms;
			//send some debug data 
			static uint8_t id000 = 0;
			dronecan_publish_debug_KeyValue("iq", controller.out.iq, &id000);
			static uint8_t id001 = 0;
			dronecan_publish_debug_KeyValue("ir", controller.out.iq_ref, &id001);
			static uint8_t id002 = 0;
			dronecan_publish_debug_KeyValue("ac", csm.offset_accumulate, &id002);
			static uint8_t id003 = 0;
			dronecan_publish_debug_KeyValue("ti", controller.in.theta_e_ref, &id003);
			static uint8_t id004 = 0;
			dronecan_publish_debug_KeyValue("to", controller.out.theta_e, &id004);
			//
			//see if any received message would trigger a FSM event
			ControllerEventType_t dronecan_event = dronecan_event_pop();
			if(dronecan_event != EVENT_DEFAULT) {
				//gpio_toggle(LED_PORT, LED_B);
				ControllerEvent_t event = {
					.type = dronecan_event,
					.t = g_uptime_ms
				};
				CSM_dispatch_event(&csm, event);
			}
			//safety checks
			uint8_t err = 
			   controller_slow_safety_checks((controller_t*)&controller);
			if(err != LOCAL_SPINORSTATUS_ERROR_OK) {
				//fire an error event in the FSM if there is a safety error
				csm.error = err;
				ControllerEvent_t err_event = {
					.type = EVENT_ERROR,
					.t = g_uptime_ms
				};
				CSM_dispatch_event(&csm, err_event);
			}
			//periodic FSM event, used for timeout-triggered transitions
			ControllerEvent_t event = {
				.type = EVENT_DEFAULT,
				.t = t_loop
			};
			CSM_dispatch_event(&csm, event);

			CSM_led_indicator(&csm, t_loop);
		}
	}
}
