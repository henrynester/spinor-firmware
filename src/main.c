#include "dronecan.h"
#include "constants.h"
#include "config.h"
#include "pins.h"
#include "setup.h"
#include "math.h"
#include "isr.h"
#include "ControllerStateMachine.h"
#include "flash.h"

#include "libcanard/canard.h"
#include "dsdl/dsdl_generated/include/dronecan_msgs.h"
#include "uavcan.protocol.debug.KeyValue.h"

#include "libopencm3/stm32/rcc.h" 
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
					

//main's copy			  
isr_out_t isr_out;
isr_in_t isr_in;
CSM_t controller;

int main(void) {
	setup();
	dronecan_init();
	if(config_load()) {
		config_defaults();
	}
	//config_apply(NULL, NULL); //very impotant
	
	CSM_init(&controller, &isr_in, &isr_out);
	//ISR will start running at 8kHz once timer starts
	start_timers();
	while(1) {
		static uint32_t t_loop=0;
		if(g_uptime_ms > t_loop+5) {
			t_loop = g_uptime_ms;

			sync_isr_out(&isr_out);
			ControllerEvent_t event = {
				.type = EVENT_DEFAULT,
				.t = t_loop
			};
			if(isr_out.error != ISR_ERROR_OK) {
				event.type = EVENT_ERROR;
			}
			CSM_dispatch_event(&controller, event);
			if(controller.state == CONTROLLERSTATE_ARMED) {
			isr_in.iq_ref = parameter_storage.iq_ref;
			isr_in.omega_m_ref = parameter_storage.omega_m_ref;
			isr_in.theta_m_ref = parameter_storage.theta_m_ref;
			}
			sync_isr_in(&isr_in);

			static uint8_t temp001=0;
			dronecan_publish_debug_KeyValue("id", isr_out.id, &temp001);
			static uint8_t temp002=0;
			dronecan_publish_debug_KeyValue("iq", isr_out.iq, &temp002);
			static uint8_t temp011=0;
			dronecan_publish_debug_KeyValue("vd", isr_out.vd, &temp011);
			static uint8_t temp012=0;
			dronecan_publish_debug_KeyValue("vq", isr_out.vq, &temp012);
			static uint8_t temp003=0;
			dronecan_publish_debug_KeyValue("ia", isr_out.ia, &temp003);
			static uint8_t temp004=0;
			dronecan_publish_debug_KeyValue("ib", isr_out.ib, &temp004);
			static uint8_t temp005=0;
			dronecan_publish_debug_KeyValue("ic", isr_out.ic, &temp005);
			static uint8_t temp013=0;
			dronecan_publish_debug_KeyValue("va", isr_out.va, &temp013);
			static uint8_t temp014=0;
			dronecan_publish_debug_KeyValue("vb", isr_out.vb, &temp014);
			static uint8_t temp015=0;
			dronecan_publish_debug_KeyValue("vc", isr_out.vc, &temp015);
			static uint8_t temp020=0;
			dronecan_publish_debug_KeyValue("tm", isr_out.theta_m, &temp020);
			static uint8_t temp021=0;
			dronecan_publish_debug_KeyValue("om", isr_out.omega_m, &temp021);
			static uint8_t temp022=0;
			dronecan_publish_debug_KeyValue("ir", isr_out.iq_ref_combined, &temp022);
		}
		dronecan_publish_NodeStatus();
		dronecan_transmit();
		dronecan_receive();
	}
}
