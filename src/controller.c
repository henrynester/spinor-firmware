#include "controller.h"

#include "pins.h"
#include "constants.h"
#include "math.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/adc.h"
#include "local.SPINORStatus.h"

void controller_init(controller_t *self) {
	self->config = config_get();
	//default to a low temperature before readings have occured
	//to avoid erroring out
	self->adc_results.Tfet = 0x0FFF*3;
	self->adc_results.Tmtr = 0x0FFF*3;
	encoder_init(&self->encoder);
}

void controller_reset(controller_t *self) {
	nvic_disable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	foc_reset(&self->foc);
	self->vel.omega_m_err_integral = 0;
	self->vel.omega_m_avg = 0;
	self->out.fast_error = FAST_ERROR_OK;
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}

void controller_update(controller_t *self) {
	//BEGIN TIME CRITICAL. t=0 at ADC conversion start, t=1.2us at conversion end
	volatile uint32_t t_exec_start = systick_get_value();

	//first copy data from ADC buffer before next conversion causes DMA to overwrite
	//the present data
	adc_copy_results(&self->adc_results); //TIME CRITICAL: must complete before next ADC conversion start, t=1/24kHz=42us
	
	self->counter_slow++;
	if(self->counter_slow > CONTROL_FREQ/4) { self->counter_slow = 0; }
	//TIME CRITICAL: must complete before next ADC conv start, t=42us	
	//here we request thermistor conversions in place of vbus conversions
	//at intervals tied to the slow counter
	if(self->counter_slow==0*CONTROL_FREQ/12) {
		adc_set_measure_mode(ADC_MODE_IA_IB_IC_TFET);
	} else if(self->counter_slow==1*CONTROL_FREQ/12) {
		adc_set_measure_mode(ADC_MODE_IA_IB_IC_TMTR);
	} else {
		adc_set_measure_mode(ADC_MODE_IA_IB_IC_VBUS);
	}

	//use volatile to force register to actually be read at this point, not later
	//DMA counter is reset to max right before this interrupt because last ADC 
	//If we are too slow and an ADC trigger occurs before this point, we will have
	//inconcsistent ADC mode (e.g. one vbus and two temp conversions or something).
	volatile uint8_t adc_late_channel_switch = (DMA1_CNDTR1 < ADC_BUFFER_LEN);

	encoder_pll_load_next(&self->encoder); //call before any foc or dynamics control laws...
	//TIME CRITICAL: must complete before next PWM DMA request, t~1/24kHz*(75% mod depth)=31us 
	foc_update(&self->foc, &self->adc_results, &self->encoder);

	self->_out.t_exec_foc = SYSTICK_TO_DELTA_US(t_exec_start, systick_get_value()); 

	//TIME CRITICAL: must occur close to next PWM DMA request 
	//(within 1/2 1/24kHz period is OK) and with low jitter
	//code expects angle was read at time of first PWM DMA request
	encoder_read_angle(); //takes 10us for 2x spi trans
	//END TIME CRITICAL
	//(still need to finish before next interrupt with some time for main-loop tasks, ofc)
	//every so often, read status registers of encoder
	//this takes a while, 15 us for 3x spi trans
	if(self->counter_slow==2*CONTROL_FREQ/12) {
		encoder_read_status(&self->encoder);
	}
	//compute next PLL encoder angle estimate based on current estimate, measured
	//angle. Also precompute sin, cos for next loop's electrical angles
	encoder_pll_compute_next(&self->encoder);	

	//collect inputs to this control loop
	//handle FOC control mode switches
	if(self->foc.control_mode != self->_in.foc_control_mode) {
		self->foc.control_mode = self->_in.foc_control_mode;
		//reset integrators when first activating FOC
		if(self->foc.control_mode == FOC_CONTROL_MODE_IDQ) {
			foc_reset(&self->foc);
		}
	}
	if(self->foc.control_mode == FOC_CONTROL_MODE_VDQ 
		|| self->foc.control_mode == FOC_CONTROL_MODE_VDQ_THETA_E) {
		self->foc.vd_ref = self->_in.vd_ref;
		self->foc.vq_ref = self->_in.vq_ref;
		self->foc.theta_e_ref = self->_in.theta_e_ref;
	}
	//update encoder offset
	if(self->encoder.theta_e_offset != self->config->theta_e_offset) {
		self->encoder.theta_e_offset = self->config->theta_e_offset;
	}
	//update homing pos
	if(self->encoder.theta_m_homing_offset != self->_in.theta_m_homing_offset) {
		self->encoder.theta_m_homing_offset = self->_in.theta_m_homing_offset;
	}

	//averaging of velocity estimate yields a smoother result
	//however, we do need high frequencies in omega_m for the PLL
	self->vel.omega_m_avg -= self->vel.omega_m_avgarr[self->vel.omega_m_avgidx];
	self->vel.omega_m_avgarr[self->vel.omega_m_avgidx] = self->encoder.omega_m;
	self->vel.omega_m_avg += self->vel.omega_m_avgarr[self->vel.omega_m_avgidx];
	if(++self->vel.omega_m_avgidx >= OMEGA_M_AVG_LEN) {
		self->vel.omega_m_avgidx = 0;
	}
	int32_t omega_m_ref = 0;
	int32_t iq_ref = 0;
	//position control law
	if(self->_in.control_mode == CONTROL_MODE_POS) {
		int32_t theta_m_ref = CONSTRAIN(self->_in.theta_m_ref,
			0, THETA_M_MAX_INT);
		if(self->config->invert_direction) {
			theta_m_ref = -theta_m_ref;
		}
		int32_t theta_m_error =
			self->encoder.theta_m_homed - theta_m_ref; 
		theta_m_error /= 256;
		theta_m_error = CONSTRAIN(theta_m_error, -INT16_MAX, INT16_MAX); 
		/*if(theta_m_error < 0x10000 && theta_m_error > -0x10000) {
			theta_m_error = 0;
		}*/
		omega_m_ref = -4*FMUL(
			(float)0x100*POS_BANDWIDTH*THETA_M_LSB/OMEGA_M_LSB/4.0,
			theta_m_error
			);	
		//omega_m_ref = -4*theta_m_error; //CONSTRAIN(omega_m_ref, -5000, 5000);
	} 
	//or use direct velocity control
	else if(self->_in.control_mode == CONTROL_MODE_VEL) {
		omega_m_ref = self->_in.omega_m_ref; 
		if(self->config->invert_direction) {
			omega_m_ref = -omega_m_ref;
		}
	} 
		
	//velocity control law
	if(self->_in.control_mode >= CONTROL_MODE_VEL) {
		omega_m_ref = CONSTRAIN(omega_m_ref,
			-VEL_LIMIT_INT, VEL_LIMIT_INT);
		int32_t omega_m_err = (self->vel.omega_m_avg)
			- omega_m_ref*OMEGA_M_AVG_LEN;
		//reduce effective gain at higher commanded speeds
		//to avoid oscillations
		//int32_t factor = 16*omega_m_ref;
		//if(factor > 0x7800) {
		//	factor = 0x7800;
		//}
		//omega_m_err -= (omega_m_err * factor) / 0x8000;
		int32_t omega_m_tau_ref = 0;
	        omega_m_tau_ref += -FMUL(
			VEL_KP*OMEGA_M_LSB/OMEGA_M_AVG_LEN/TORQUE_IDQ_LSB,
			omega_m_err);
		omega_m_tau_ref += -self->vel.omega_m_err_integral / 0x1000;
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
			self->vel.omega_m_err_integral -= 
				self->vel.omega_m_err_integral / 64;
		} else {
			self->vel.omega_m_err_integral +=
				FMUL(
					(float)0x1000*VEL_KI*OMEGA_M_LSB/OMEGA_M_AVG_LEN/TORQUE_IDQ_LSB*CONTROL_DT, 
					omega_m_err
				);
		}
		iq_ref = omega_m_tau_ref;
	} 
	//or use direct torque control
	else {
		self->vel.omega_m_err_integral = 0;
		iq_ref = self->_in.iq_ref; 
		if(self->config->invert_direction) {
			iq_ref = -iq_ref;
		}
	} 

	//limit torque setpoint based on velocity
	int32_t upper_limit = FMUL(
		(1.0/TORQUE_LIMIT_SLOPE_FRACTION*TORQUE_LIMIT/VEL_LIMIT
			 /TORQUE_IDQ_LSB*OMEGA_M_LSB), 
		(VEL_LIMIT_INT - (self->vel.omega_m_avg/OMEGA_M_AVG_LEN)));	
	int32_t lower_limit = FMUL(
		(1.0/TORQUE_LIMIT_SLOPE_FRACTION*TORQUE_LIMIT/VEL_LIMIT/
			 TORQUE_IDQ_LSB*OMEGA_M_LSB), 
		(-VEL_LIMIT_INT - (self->vel.omega_m_avg/OMEGA_M_AVG_LEN)));
	iq_ref = CONSTRAIN(iq_ref, lower_limit, upper_limit);
	//torque limiting even at low velocities, too
	iq_ref = CONSTRAIN(iq_ref, TORQUE_MIN_INT, TORQUE_MAX_INT);

	//send next loop's idq setpoints to FOC
	self->foc.iq_ref = iq_ref;
	self->foc.id_ref = 0;
	
	//outputs reported from this control loop
	self->_out.ia = self->adc_results.ia; 
	self->_out.ib = self->adc_results.ib; 
	self->_out.ic = self->adc_results.ic; 
	self->_out.id = self->foc.id;
	self->_out.iq = self->foc.iq;
	self->_out.iq_ref = self->foc.iq_ref;
	self->_out.va = self->foc.pwm_a;
	self->_out.vb = self->foc.pwm_b;
	self->_out.vc = self->foc.pwm_c;
	self->_out.vd = self->foc.vd;
	self->_out.vq = self->foc.vq;
	self->_out.theta_m = self->encoder.theta_m_homed;
	self->_out.omega_m = self->vel.omega_m_avg / OMEGA_M_AVG_LEN;
	self->_out.theta_e = self->encoder.theta_e;
	self->_out.n_encoder_err = self->encoder.num_spi_errors;
	self->_out.encoder_err = self->encoder.status;
	self->_out.encoder_agc = self->encoder.agc;
	self->_out.v_bus = self->adc_results.vbus;
	self->_out.T_mtr = self->adc_results.Tmtr;
	self->_out.T_fet = self->adc_results.Tfet;
	if(self->config->invert_direction) {
		self->_out.theta_m *= -1;
		self->_out.omega_m *= -1;
		self->_out.iq_ref *= -1;
		self->_out.iq *= -1;
		self->_out.vq *= -1;
	}

	//clear errors if requested
	if(self->_in.clear_errors_flag) {
		self->_out.fast_error = FAST_ERROR_OK;		
		self->_out.encoder_err = ENCODER_ERROR_OK;
		self->_out.n_encoder_err = 0;
		encoder_clear_error(&self->encoder);
	}
	//ISR fast safety checks, errors are sticky
	//overcurrent check. runs in ISR for fastest response
	if(OUTSIDE(self->adc_results.ia, SAFETY_IABC_MIN_INT, SAFETY_IABC_MAX_INT)
	|| OUTSIDE(self->adc_results.ib, SAFETY_IABC_MIN_INT, SAFETY_IABC_MAX_INT)	
	|| OUTSIDE(self->adc_results.ic, SAFETY_IABC_MIN_INT, SAFETY_IABC_MAX_INT)) {
		self->_out.fast_error = FAST_ERROR_IABC_OVER;
	}
	//Keep track of execution time of this ISR
	uint16_t t_exec_end = systick_get_value();
	self->_out.t_exec_controller = SYSTICK_TO_DELTA_US(t_exec_start, t_exec_end);
	if(self->_out.t_exec_controller > T_EXEC_FOC_POS_VEL_MAX) {
		self->_out.fast_error = FAST_ERROR_POS_VEL_DEADLINE_MISSED;
	}
	//check that the FOC read->process->write to PWM completed within
	//90% of a single PWM period, and that the next ADC read has not occured
	//too soon, before ADC reconfiguration step.
	if(self->_out.t_exec_foc > T_EXEC_FOC_MAX || adc_late_channel_switch) {
		self->_out.fast_error = FAST_ERROR_FOC_DEADLINE_MISSED;
	}
	//Disable PWM outputs immediately if there is a fast error
	if(self->_out.fast_error != FAST_ERROR_OK) {
		timer_disable_break_main_output(TIM1);
	} else {
		timer_enable_break_main_output(TIM1);
	}
	
	//set rendezvous flag, used for syncing i/o ports of this routine
	self->_rendezvous_flag = true;
}

void controller_rendezvous_sync_inout(volatile controller_t *self) {
	//flag is set within update() called in ISR
	//clear the flag and wait for ISR to set it then return control
	self->_rendezvous_flag = false;
	while(!self->_rendezvous_flag);
	//now we have up until the next 8kHz ISR call.
	//disable interrupts even though the copies should complete way before then 
	nvic_disable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	//sync i/o ports
	self->_in = self->in; //public input->private
	self->out = self->_out; //private output->public
	//and reenable
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}

uint8_t controller_slow_safety_checks(controller_t *self) {
	static uint8_t error_counter = 0;
	controller_out_t out = self->out;	
	//adc decreases with temp, so use adc<threshold
	if(out.T_fet < SAFETY_TFET_MAX_INT) {
		return LOCAL_SPINORSTATUS_ERROR_TFET_HIGH;
	}
	if(out.T_mtr < SAFETY_TMTR_MAX_INT) {
		//thermistor not soldered yet, uncomment later
		//return LOCAL_SPINORSTATUS_ERROR_TMTR_HIGH;
	}
	if(out.encoder_err != ENCODER_ERROR_OK) {
		if(out.encoder_err == ENCODER_ERROR_BFIELD_OVER ||
				out.encoder_err == ENCODER_ERROR_BFIELD_OVER) {
			return LOCAL_SPINORSTATUS_ERROR_ENCODER_B;
		} else {
			return LOCAL_SPINORSTATUS_ERROR_ENCODER_SPI;
		}
	}
	if(out.omega_m > SAFETY_VEL_MAX_INT
			|| out.omega_m < -SAFETY_VEL_MAX_INT) {
		return LOCAL_SPINORSTATUS_ERROR_VEL_HIGH;
	}
	if(out.v_bus > SAFETY_VBUS_MAX_INT) {
		return LOCAL_SPINORSTATUS_ERROR_VBUS_HIGH;
	}
	if(out.v_bus < SAFETY_VBUS_MIN_INT) {
		if(error_counter >= 4) {
			return LOCAL_SPINORSTATUS_ERROR_VBUS_LOW;
		}
	}
	if(out.fast_error != FAST_ERROR_OK) {
		if(out.fast_error == FAST_ERROR_IABC_OVER) {
		       return LOCAL_SPINORSTATUS_ERROR_IABC_HIGH;
		}
		if(out.fast_error == FAST_ERROR_FOC_DEADLINE_MISSED
			|| out.fast_error == FAST_ERROR_POS_VEL_DEADLINE_MISSED) {
			return LOCAL_SPINORSTATUS_ERROR_CONTROLLER_DEADLINE_MISSED;
		}
	}

	error_counter = 0;
	return LOCAL_SPINORSTATUS_ERROR_OK;
}
