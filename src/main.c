#include "dronecan.h"
#include "svm.h"
#include "constants.h"
#include "config.h"
#include "pins.h"
#include "setup.h"
#include "trig.h"
#include "isr.h"
//#include "parameters.h"

#include "libcanard/canard.h"
//#include "libcanard/drivers/stm32/canard_stm32.h"
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
					
config_t config;

parameter_t parameters[] = {
	{"iq_ref", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &config.iq_ref, 0, -250, 250},
	{"vd_ref", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &config.vd_ref, 0, INT16_MIN, INT16_MAX },
	{"vq_ref", UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &config.vq_ref, 0, INT16_MIN, INT16_MAX },
	{"foc_kp", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &config.foc_kp, 90000, 0, INT32_MAX },
	{"foc_ki", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &config.foc_ki, 29000, 0, INT32_MAX },
	{"foc_k_afc", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &config.foc_k_afc, 0, 0, INT32_MAX }
};

volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

//data moving ISR->main
volatile ISR_tx_t _ISR_tx; //ISR's copy
ISR_tx_t ISR_tx; //main's copy
//data moving main->ISR
volatile ISR_rx_t _ISR_rx; //ISR's copy
ISR_rx_t ISR_rx; //main's copy

volatile uint32_t g_uptime_ms;

#define DLY(x) for(uint32_t i=0; i<x; i++) { __asm__("nop"); }

typedef enum {
	INIT,
	WAIT_MOVE,
	MEASURE_THETA_M_PUBLISH,
	INCREMENT_THETA_M_REF,
	DONE
} calibration_state_t;
calibration_state_t calibration_state = INIT;

#define CALIBRATION_WAIT_MOVE_COUNT 25
#define CALIBRATION_DELTA_THETA_M_REF 32 

int main(void) {
	ISR_rx.pwm_src=LOW;
	setup();
	dronecan_init();
	uint32_t t0 = 0;
	uint32_t t1 = 0;
	uint16_t theta_m_ref;
	uint8_t wait_count;
	uint8_t fwd;
	uint32_t error_acc=0;
	uint16_t count=0;
	while(1) {
		/*if((g_uptime_ms - t0) > 5) {
			t0=g_uptime_ms;
			switch(calibration_state) {
				case INIT:
					theta_m_ref=0;
					ISR_rx.pwm_src = FORCE_THETA_E;
					calibration_state = WAIT_MOVE;
					fwd=1;
					break;
				case WAIT_MOVE:
					wait_count++;
					if(wait_count >= CALIBRATION_WAIT_MOVE_COUNT) {
						calibration_state = MEASURE_THETA_M_PUBLISH;
					}
					break;
				case MEASURE_THETA_M_PUBLISH:
					//publishCalibration(theta_m_ref, ISR_tx.theta_m);
					error_acc+=(ISR_tx.theta_m-theta_m_ref)&0x3FFF;
					calibration_state = INCREMENT_THETA_M_REF;
					count++;
					break;
				case INCREMENT_THETA_M_REF:
					if(fwd) {
						theta_m_ref += CALIBRATION_DELTA_THETA_M_REF;
						if(theta_m_ref > U14_MAX) {
							fwd=0;
							theta_m_ref=U14_MAX;
						}
					} else {
						theta_m_ref -= CALIBRATION_DELTA_THETA_M_REF;
						if(theta_m_ref > U14_MAX) { //underflow
							theta_m_ref = 0;
							calibration_state=DONE;
							break;
						}
					}
					calibration_state = WAIT_MOVE;
					wait_count=0;
					break;
				case DONE:
					ISR_rx.pwm_src=LOW;
					volatile uint16_t error_mean = ((error_acc / count)*14)&0x3FFF;
					break;
			}
			ISR_rx.theta_e_cmd = (theta_m_ref*14)&0x3FFF;		
			ISR_rx.va_cmd = CALIBRATION_VD;
			nvic_disable_irq(NVIC_TIM1_CC_IRQ);
			ISR_tx = _ISR_tx;
			_ISR_rx=ISR_rx;
			nvic_enable_irq(NVIC_TIM1_CC_IRQ);
		}*/
		dronecan_publish_NodeStatus();
		static uint32_t t_last_tx=0;
		if(g_uptime_ms > t_last_tx+5) {
			t_last_tx=g_uptime_ms;

			ISR_rx.id_cmd = 0;
			ISR_rx.iq_cmd = (int16_t)config.iq_ref;
			ISR_rx.pwm_src=FOC;

			nvic_disable_irq(NVIC_TIM1_CC_IRQ);
			ISR_tx = _ISR_tx;
			_ISR_rx=ISR_rx;
			nvic_enable_irq(NVIC_TIM1_CC_IRQ);

			//static uint8_t temp001=0;
			//dronecan_publish_debug_KeyValue("ia", ISR_tx.ia, &temp001);
			//static uint8_t temp002=0;
			//dronecan_publish_debug_KeyValue("ib", ISR_tx.ib, &temp002);
			//static uint8_t temp003=0;
			//dronecan_publish_debug_KeyValue("ic", ISR_tx.ic, &temp003);
			static uint8_t temp004=0;
			dronecan_publish_debug_KeyValue("id", ISR_tx.iq_ref_clamped, &temp004);
			static uint8_t temp005=0;
			dronecan_publish_debug_KeyValue("iq", ISR_tx.iq, &temp005);
			static uint8_t temp006=0;
			dronecan_publish_debug_KeyValue("vd", ISR_tx.vd, &temp006);
			static uint8_t temp007=0;
			dronecan_publish_debug_KeyValue("vq", ISR_tx.vq, &temp007);
			static uint8_t temp008=0;
			dronecan_publish_debug_KeyValue("tm", ISR_tx.theta_m&0x3FFF, &temp008);
			static uint8_t temp009=0;
			dronecan_publish_debug_KeyValue("om", ISR_tx.omega_m, &temp009);
		}
		dronecan_transmit();
		dronecan_receive();
	}
}
