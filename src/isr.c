#include "isr.h"
#include "adc.h"
#include "setup.h"
#include "foc.h"
#include "encoder.h"
#include "pins.h"
#include "constants.h"
#include "math.h"
#include "ControllerStateMachine.h"
#include "flash.h"

#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/adc.h"

volatile uint32_t g_uptime_ms;
void sys_tick_handler(void) {
	g_uptime_ms++;
}

//data moving ISR->main
volatile isr_out_t _isr_out; //ISR's copy
//data moving main->ISR
volatile isr_in_t _isr_in; //ISR's copy
//config coming into ISR DO NOT EDIT HERE
volatile config_t *isr_config = NULL;

void dma1_channel1_isr(void) {
	//BEGIN TIME CRITICAL. t=0 at ADC conversion start, t=1.2us at conversion end
	volatile uint32_t t_exec_start = systick_get_value();
	gpio_clear(LED_PORT, LED_B);

	//get pointer to config at beginning
	if(isr_config == NULL) {
		isr_config = config_get();
	}

	static adc_results_t adc_results;
	static encoder_t encoder;
	static foc_t foc;

	//first copy data from ADC buffer before next conversion causes DMA to overwrite
	//the present data
	adc_copy_results(&adc_results); //TIME CRITICAL: must complete before next ADC conversion start, t=1/24kHz=42us
	
	static uint16_t counter_slow = 0;
	counter_slow++;
	if(counter_slow > CONTROL_FREQ/4) { counter_slow = 0; }
	//TIME CRITICAL: must complete before next ADC conv start, t=42us	
	//here we request thermistor conversions in place of vbus conversions
	//at intervals tied to the slow counter
	if(counter_slow==0*CONTROL_FREQ/12) {
		adc_set_measure_mode(ADC_MODE_IA_IB_IC_TFET);
	}
	if(counter_slow==1*CONTROL_FREQ/12) {
		adc_set_measure_mode(ADC_MODE_IA_IB_IC_TMTR);
	} else {
		adc_set_measure_mode(ADC_MODE_IA_IB_IC_VBUS);
	}

	//use volatile to force register to actually be read at this point, not later
	//DMA counter is reset to max right before this interrupt because last ADC 
	//If we are too slow and an ADC trigger occurs before this point, we will have
	//inconcsistent ADC mode (e.g. one vbus and two temp conversions or something).
	volatile uint8_t adc_late_channel_switch = (DMA1_CNDTR1 < ADC_BUFFER_LEN);

	encoder_pll_load_next(&encoder); //call before any foc or dynamics control laws...
	//TIME CRITICAL: must complete before next PWM DMA request, t~1/24kHz*(75% mod depth)=31us 
	foc_update(&foc, &adc_results, &encoder); 

	//immediately report an error if timing deadline missed,
	//plus tie all phases low
	//check that the FOC read->process->write to PWM completed within
	//90% of a single PWM period.
	_isr_out.t_exec_foc = SYSTICK_TO_DELTA_US(t_exec_start, systick_get_value()); 

	//TIME CRITICAL: must occur close to next PWM DMA request 
	//(within 1/2 1/24kHz period is OK) and with low jitter
	//code expects angle was read at time of first PWM DMA request
	encoder_read_angle(); //takes 10us for 2x spi trans
	//END TIME CRITICAL
	//(still need to finish before next interrupt with some time for main-loop tasks, ofc)
	//every so often, read status registers of encoder
	//this takes a while, 15 us for 3x spi trans
	if(counter_slow==2*CONTROL_FREQ/12) {
		encoder_read_status(&encoder);
	}
	//compute next PLL encoder angle estimate based on current estimate, measured
	//angle. Also precompute sin, cos for next loop's electrical angles
	encoder_pll_compute_next(&encoder);	

	//collect inputs to this control loop
	//handle FOC control mode switches
	static int32_t omega_m_err_integral = 0;
	if(_isr_in.foc_control_mode != foc.control_mode) {
		//reset integrators when first activating FOC
		foc.control_mode = _isr_in.foc_control_mode;
		if(foc.control_mode == FOC_CONTROL_MODE_IDQ) {
			foc_reset();
			omega_m_err_integral = 0;
		}
	}
	if(foc.control_mode == FOC_CONTROL_MODE_VDQ 
		|| foc.control_mode == FOC_CONTROL_MODE_VDQ_THETA_E) {
		foc.vd_ref = _isr_in.vd_ref;
		foc.vq_ref = _isr_in.vq_ref;
		foc.theta_e_ref = _isr_in.theta_e_ref;
	}
	if(encoder.theta_e_offset != isr_config->theta_e_offset) {
		encoder.theta_e_offset = isr_config->theta_e_offset;
	}
	if(encoder.theta_m_homing_offset != _isr_in.theta_m_homing_offset) {
		encoder.theta_m_homing_offset = 0x10*_isr_in.theta_m_homing_offset;
	}

	//averaging of velocity estimate yields a smoother result
	//however, we do need high frequencies in omega_m for the PLL
#define OMEGA_M_AVG_LEN 16 
	static uint8_t omega_m_avgidx = 0;
	static int32_t omega_m_avgarr[OMEGA_M_AVG_LEN];
	static int32_t omega_m_avg = 0;
	omega_m_avg -= omega_m_avgarr[omega_m_avgidx];
	omega_m_avgarr[omega_m_avgidx] = encoder.omega_m;
	omega_m_avg += omega_m_avgarr[omega_m_avgidx];
	omega_m_avgidx++;
	if(omega_m_avgidx >= OMEGA_M_AVG_LEN) {
		omega_m_avgidx = 0;
	}
	int32_t omega_m_ref = NONE;
	int32_t iq_ref = NONE;
	//position control law
	if(_isr_in.theta_m_ref != NONE) {
		int32_t theta_m_ref = CONSTRAIN(_isr_in.theta_m_ref*0x10,
			THETA_M_MIN_INT, THETA_M_MAX_INT);
		int32_t theta_m_error = encoder.theta_m_homed - theta_m_ref; 
		theta_m_error = CONSTRAIN(theta_m_error, -INT16_MAX, INT16_MAX); //avoid overflow
		omega_m_ref = FMUL(0.1*-POS_KP*THETA_M_LSB/OMEGA_M_LSB, theta_m_error);
	} 
	//or use direct velocity control
	else if(_isr_in.omega_m_ref != NONE) {
		omega_m_ref = _isr_in.omega_m_ref; 
	} 
	//torque control, or just disabled
	else {
		omega_m_err_integral = 0;
	}
		
	//velocity control law
	if(omega_m_ref != NONE) {
		omega_m_ref = CONSTRAIN(omega_m_ref,
			-VEL_LIMIT_INT, VEL_LIMIT_INT);
		int32_t omega_m_err = (omega_m_avg) - omega_m_ref*OMEGA_M_AVG_LEN;
		//reduce effective gain at higher commanded speeds
		//to avoid oscillations
		int32_t factor = 4*omega_m_ref;
		if(factor > 0x7800) {
			factor = 0x7800;
		}
		//omega_m_err -= (omega_m_err * factor) / 0x8000;
		int32_t omega_m_tau_ref = -FMUL(VEL_KP*OMEGA_M_LSB/TORQUE_IDQ_LSB, omega_m_err) - omega_m_err_integral;
		uint8_t saturated = false;
		if(omega_m_tau_ref <= TORQUE_MIN_INT) {
			omega_m_tau_ref = TORQUE_MIN_INT;
			saturated = true;
		}
		if(omega_m_tau_ref >= TORQUE_MAX_INT) {
			omega_m_tau_ref = TORQUE_MAX_INT;
			saturated = true;
		}
		if(saturated) {
			omega_m_err_integral -= omega_m_err_integral / 64;
		} else {
			omega_m_err_integral += FMUL(VEL_KI*OMEGA_M_LSB/TORQUE_IDQ_LSB*CONTROL_DT, omega_m_err);
		}
		iq_ref = omega_m_tau_ref;
	} 
	//or use direct torque control
	else if(_isr_in.iq_ref != NONE) {
		omega_m_err_integral = 0;
		iq_ref = _isr_in.iq_ref; 
	} 
	//torque, vel, pos inputs are all NONE. stop
	else {
		iq_ref = 0;
	}
	//average torque setpoint
#define IDQ_REF_AVG_LEN 1 
	static uint8_t idq_ref_avgidx = 0;
	static int32_t idq_ref_avgarr[IDQ_REF_AVG_LEN];
	static int32_t idq_ref_avg = 0;
	idq_ref_avg -= idq_ref_avgarr[idq_ref_avgidx];
	idq_ref_avgarr[idq_ref_avgidx] = iq_ref;
	idq_ref_avg += idq_ref_avgarr[idq_ref_avgidx];
	idq_ref_avgidx++;
	if(idq_ref_avgidx >= IDQ_REF_AVG_LEN) {
		idq_ref_avgidx = 0;
	}
	iq_ref = idq_ref_avg/IDQ_REF_AVG_LEN;
	//limit torque setpoint based on velocity
	int32_t upper_limit = FMUL(
		(1.0/TORQUE_LIMIT_SLOPE_FRACTION*TORQUE_LIMIT/VEL_LIMIT/TORQUE_IDQ_LSB*OMEGA_M_LSB), 
		(VEL_LIMIT_INT - (omega_m_avg/OMEGA_M_AVG_LEN)));	
	int32_t lower_limit = FMUL(
		(1.0/TORQUE_LIMIT_SLOPE_FRACTION*TORQUE_LIMIT/VEL_LIMIT/TORQUE_IDQ_LSB*OMEGA_M_LSB), 
		(-VEL_LIMIT_INT - (omega_m_avg/OMEGA_M_AVG_LEN)));
	iq_ref = CONSTRAIN(iq_ref, lower_limit, upper_limit);
	//torque limiting even at low velocities, too
	iq_ref = CONSTRAIN(iq_ref, TORQUE_MIN_INT, TORQUE_MAX_INT);

	foc.iq_ref = iq_ref;
	foc.id_ref = 0;
	
	//outputs reported from this control loop
	_isr_out.ia = adc_results.ia; 
	_isr_out.ib = adc_results.ib; 
	_isr_out.ic = adc_results.ic; 
	_isr_out.id = foc.id;
	_isr_out.iq = foc.iq;
	_isr_out.iq_ref_combined = foc.iq_ref;
	_isr_out.va = foc.pwm_a;
	_isr_out.vb = foc.pwm_b;
	_isr_out.vc = foc.pwm_c;
	_isr_out.vd = foc.vd;
	_isr_out.vq = foc.vq;
	_isr_out.theta_m = encoder.theta_m_homed / 0x10;
	_isr_out.omega_m = encoder.omega_m;
	_isr_out.theta_e = encoder.theta_e;
	_isr_out.n_encoder_err = encoder.num_spi_errors;
	_isr_out.encoder_err = encoder.status;
	_isr_out.v_bus = adc_results.vbus;
	_isr_out.T_mtr = adc_results.Tmtr;
	_isr_out.T_fet = adc_results.Tfet;

	//ISR fast safety checks
	//overcurrent check. runs in ISR due to high time sensitivity
	if(OUTSIDE(adc_results.ia, SAFETY_IABC_MIN_INT, SAFETY_IABC_MAX_INT)
	|| OUTSIDE(adc_results.ib, SAFETY_IABC_MIN_INT, SAFETY_IABC_MAX_INT)	
	|| OUTSIDE(adc_results.ic, SAFETY_IABC_MIN_INT, SAFETY_IABC_MAX_INT)) {
		_isr_out.error = ISR_ERROR_OVERCURRENT;
		timer_disable_break_main_output(TIM1);
	}
	//Keep track of execution time of this ISR
	uint16_t t_exec_end = systick_get_value();
	_isr_out.t_exec_foc_vel_pos = SYSTICK_TO_DELTA_US(t_exec_start, t_exec_end);
	if(_isr_out.t_exec_foc_vel_pos > T_EXEC_FOC_POS_VEL_MAX) {
		_isr_out.error = ISR_ERROR_POS_VEL_DEADLINE_MISSED;
		timer_disable_break_main_output(TIM1);
		gpio_clear(LED_PORT, LED_R);
	}
	if(_isr_out.t_exec_foc > T_EXEC_FOC_MAX || adc_late_channel_switch) {
		_isr_out.error = ISR_ERROR_FOC_DEADLINE_MISSED;
		timer_disable_break_main_output(TIM1);
	}

	gpio_set(LED_PORT, LED_B);
	//clear interrupt flag at end of routine - avoids retriggering
	//before the ISR completes execution
	dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
}

void sync_isr_in(isr_in_t *isr_in) {
	nvic_disable_irq(NVIC_TIM1_CC_IRQ);
	_isr_in = *isr_in;
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
}

void sync_isr_out(isr_out_t *isr_out) {
	nvic_disable_irq(NVIC_TIM1_CC_IRQ);
	*isr_out = _isr_out;
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
}
