#include "isr.h"
#include "setup.h"
#include "svm.h"
#include "encoder.h"
#include "pins.h"
#include "constants.h"
#include "trig.h"
#include "config.h"

#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/adc.h"

void sys_tick_handler(void) {
	g_uptime_ms++;
}

void dma1_channel1_isr(void) {
	gpio_clear(LED_PORT, LED_B);
	uint32_t t_exec_start = systick_get_value();

	//switch to the requested PWM source
	//if overcurrent has tripped, we can only
	//switch to the LOW state.
	if(_ISR_tx.pwm_src != _ISR_rx.pwm_src) {
		if(_ISR_tx.pwm_src != LOW_OC_TRIP) {
			_ISR_tx.pwm_src = _ISR_rx.pwm_src;
		} else {
			if(_ISR_rx.pwm_src == LOW) {
				_ISR_tx.pwm_src = LOW;
			}
		}
	} 

	//Some tasks take up lots of ISR time, so do them rarely
	static uint16_t counter_slow = 0;
	counter_slow++;
	if(counter_slow > PWM_FREQ/4) {
		counter_slow = 0;
	}
	//always read 3 current channels + bus voltage channel
	_ISR_tx.ia = adc_buffer[0];
	_ISR_tx.ib = adc_buffer[1];
	_ISR_tx.ic = adc_buffer[2];

	//swap out voltage channel for temp channels 
	//also release sense line so current flows
	//in thermistors while measuring
	if(counter_slow==0*PWM_FREQ/12) {
		ADC1_CHSELR = ADC_CHSELR_IABC_TMTR;
		gpio_set(TEMP_PORT, TEMP_PIN);
	} else if(counter_slow==1*PWM_FREQ/12) {
		ADC1_CHSELR = ADC_CHSELR_IABC_TFET;
		gpio_set(TEMP_PORT, TEMP_PIN);
	}
	//swap back to voltage channel at next loop	
	//also record temperature conversion results
	//After temperature conversion, tie thermistors'
	//sense lines low to avoid steady overcurrent in thermistors
	if(counter_slow==0*PWM_FREQ/12+1) {
		_ISR_tx.T_mtr = ((_ISR_tx.T_mtr)*3 + adc_buffer[3]) / 4;
		ADC1_CHSELR = ADC_CHSELR_IABC_VBUS;
		gpio_clear(TEMP_PORT, TEMP_PIN);
	} else if(counter_slow==1*PWM_FREQ/12+1) {
		_ISR_tx.T_fet = ((_ISR_tx.T_fet)*3 + adc_buffer[3]) / 4;
		ADC1_CHSELR = ADC_CHSELR_IABC_VBUS;
		gpio_clear(TEMP_PORT, TEMP_PIN);
	} else {
		_ISR_tx.v_bus = ((_ISR_tx.v_bus)*15 + adc_buffer[3]) / 16;
	}
	//set up the ADC to do another conversion sequence	
	ADC1_CR |= ADC_CR_ADSTP;
	dma_disable_channel(DMA1, DMA_CHANNEL1);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUFFER_SIZE);
	dma_enable_channel(DMA1, DMA_CHANNEL1);
	while(ADC1_CR & ADC_CR_ADSTART);
	ADC1_CR |= ADC_CR_ADSTART;

	//overcurrent is the only fault that needs to be handled
	//in the 20kHz FOC loop. Other faults can be detected in the 1kHz 
	//dynamics control loop.
	if(OUTSIDE(_ISR_tx.ia, I_MIN, I_MAX) 
		|| OUTSIDE(_ISR_tx.ib, I_MIN, I_MAX)
		|| OUTSIDE(_ISR_tx.ic, I_MIN, I_MAX)) {
		_ISR_tx.pwm_src = LOW_OC_TRIP;
	}

	uint16_t theta_m_sensor;
	//read electrical angle, noting if SPI errors occur
	if(encoder_read_angle(&theta_m_sensor) && _ISR_tx.n_encoder_err < UINT16_MAX) {
		_ISR_tx.n_encoder_err++;
	}
#define PLL_BANDWIDTH 1000.0
#define DELTA_T (3.0/(float)PWM_FREQ)
#define KP_PLL (2.0*PLL_BANDWIDTH)
#define KP_PLL_TIMES_DT_SCALED (int32_t)((float)0x8000*(KP_PLL*DELTA_T))
#define KI_PLL_TIMES_DT_SCALED (int32_t)((float)0x8000*(0.25*KP_PLL*KP_PLL*DELTA_T*DELTA_T))
	_ISR_tx.theta_m = _ISR_tx.theta_m_next;
	_ISR_tx.omega_m = _ISR_tx.omega_m_next;
	_ISR_tx.theta_m_sensor = theta_m_sensor;
	int16_t theta_m_err = wrap_u14_signed(theta_m_sensor - (uint16_t)(_ISR_tx.theta_m&0x3FFF));
	_ISR_tx.theta_m_next = _ISR_tx.theta_m + _ISR_tx.omega_m / 0x100 
		+ (KP_PLL_TIMES_DT_SCALED * (int32_t)theta_m_err) / 0x8000;
	_ISR_tx.omega_m_next = _ISR_tx.omega_m
		+ (KI_PLL_TIMES_DT_SCALED * (int32_t)theta_m_err) / 0x80;
	
	//every so often, read status registers of encoder
	if(counter_slow==2*PWM_FREQ/3) {
		_ISR_tx.encoder_err = encoder_read_status();
	}

	//convert to electrical angle
	_ISR_tx.theta_e = ((((uint16_t)(_ISR_tx.theta_m&0x3FFF) * N_POLE_PAIRS) & 0x3FFF)-11618)&0x3FFF;
	//save sin and cos for repeated use later
	int32_t cos_theta_e = (int32_t)fcos(_ISR_tx.theta_e);
	int32_t sin_theta_e = (int32_t)fsin(_ISR_tx.theta_e);

	//Clarke-Park transform of measured current to dq frame
	iabc_to_idq(cos_theta_e, sin_theta_e, 
		_ISR_tx.ia, _ISR_tx.ib, _ISR_tx.ic,
		&_ISR_tx.id, &_ISR_tx.iq);
			
	//Generate different PWM outputs based on selected PWM source
#define IQ_REF_MAX 250
#define OMEGA_M_MAX 5000
#define SLOPE (int32_t)(2*(float)IQ_REF_MAX/(float)OMEGA_M_MAX*(float)0x100)
	//torque-velocity domain restriction
	int16_t iq_ref_upper = CONSTRAIN(((OMEGA_M_MAX-_ISR_tx.omega_m)*SLOPE)/0x100,
			-IQ_REF_MAX, IQ_REF_MAX); 
	int16_t iq_ref_lower = CONSTRAIN(((-OMEGA_M_MAX-_ISR_tx.omega_m)*SLOPE)/0x100,
			-IQ_REF_MAX, IQ_REF_MAX);
	int16_t iq_ref = CONSTRAIN(_ISR_rx.iq_cmd, iq_ref_lower, iq_ref_upper); 
#define IQ_REF_AVG_LEN 16
	static int32_t iq_ref_avg[IQ_REF_AVG_LEN];
	static int32_t acc;
	static uint16_t iq_ref_avg_idx;
	acc-=iq_ref_avg[iq_ref_avg_idx];
	acc+=iq_ref;
	iq_ref_avg[iq_ref_avg_idx] = iq_ref;
	iq_ref_avg_idx++;
	if(iq_ref_avg_idx >= IQ_REF_AVG_LEN) {
		iq_ref_avg_idx=0;
	}
	_ISR_tx.iq_ref_clamped = acc/IQ_REF_AVG_LEN; //(_ISR_tx.iq_ref_clamped*31 + iq_ref)/32;
	
	switch(_ISR_tx.pwm_src) {
		case FOC:
			//PI control on id, iq 
			pi_control_idq(_ISR_tx.id, _ISR_tx.iq, _ISR_rx.id_cmd, _ISR_tx.iq_ref_clamped, 
					&_ISR_tx.vd, &_ISR_tx.vq); 
			if(config.vd_ref != 0) { _ISR_tx.vd = config.vd_ref; }
			if(config.vq_ref != 0) { _ISR_tx.vq = config.vq_ref; } 
			//Inverse Park-Clarke transform of dq voltage to phase voltages 
			vdq_to_vabc(cos_theta_e, sin_theta_e, _ISR_tx.vd, _ISR_tx.vq,
					&_ISR_tx.va, &_ISR_tx.vb, &_ISR_tx.vc);
			break;
		//apply direct-axis voltage specified in force_va at angle specified
		//in force_theta_e. Used only for encoder offset calibration.
		case FORCE_THETA_E:
			cos_theta_e = (int32_t)fcos(_ISR_rx.theta_e_cmd);
			sin_theta_e = (int32_t)fsin(_ISR_rx.theta_e_cmd); 
			_ISR_tx.vd = _ISR_rx.va_cmd; _ISR_tx.vq = 0; 
			vdq_to_vabc(cos_theta_e, sin_theta_e, _ISR_tx.vd, _ISR_tx.vq, 
					&_ISR_tx.va, &_ISR_tx.vb, &_ISR_tx.vc); 
			break; 
		//apply phase voltages specified in force_va,b,c
		case FORCE_V_ABC:
			_ISR_tx.va=_ISR_rx.va_cmd; 
			_ISR_tx.vb=_ISR_rx.vb_cmd;
			_ISR_tx.vc=_ISR_rx.vc_cmd;
			break;
		//tie all phases low. Used for disarmed state
		case LOW:
		case LOW_OC_TRIP:
			_ISR_tx.va=0;	
			_ISR_tx.vb=0;	
			_ISR_tx.vc=0;	
			break;
	}
	if(_ISR_tx.pwm_src==LOW_OC_TRIP) {
		gpio_clear(LED_PORT, LED_R);
	}

	//Write out PWM values to TIM1. They will be latched in when the TIM1 counter
	//either over- or underflows
	pwm_write(_ISR_tx.va, _ISR_tx.vb, _ISR_tx.vc);
	
	//Keep track of execution time of this ISR
	uint32_t t_exec_end = systick_get_value();
	//uint32_t t_exec_end = systick_get_value();	
	//Handle overflow. systick counts downward
	if(t_exec_end < t_exec_start) {
		_ISR_tx.t_exec = t_exec_start - t_exec_end;
	} else {
		_ISR_tx.t_exec = (t_exec_start + SYSTICK_RELOAD) - t_exec_end;
	}
	//note the longest FOC ISR execution time
	if(_ISR_tx.t_exec > _ISR_tx.t_exec_max) {
		_ISR_tx.t_exec_max = _ISR_tx.t_exec;
	}
	gpio_set(LED_PORT, LED_B);
	//clear interrupt flag at end of routine - avoids retriggering
	//before the ISR completes execution
	dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
}
