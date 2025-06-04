#include "ControllerStateMachine.h"
#include "pins.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "constants.h"
#include "flash.h"
#include "local.SPINORStatus.h"

static void CSM_enter_CALIBRATE_ENCODER(CSM_t *self) {
	self->isr_in->foc_control_mode=FOC_CONTROL_MODE_VDQ_THETA_E;
	self->isr_in->vd_ref = CAL_OFFSET_VD;
	self->isr_in->vq_ref = 0;

	self->encoder_offset_valid = false;
	self->num_samples = 0;
	self->offset_accumulate = 0;
}
static void CSM_enter_MOVE_A(CSM_t *self) {
	self->isr_in->theta_e_ref = 0;
}
static void CSM_enter_MOVE_B(CSM_t *self) {
	self->isr_in->theta_e_ref = 0x4000/3;
}
static void CSM_enter_MOVE(CSM_t *self) {
	if(self->num_samples < CAL_OFFSET_NUM_SAMPLES/2) {
		self->isr_in->theta_e_ref += CAL_OFFSET_STEP;
	} else {
		self->isr_in->theta_e_ref -= CAL_OFFSET_STEP;
	}
	self->isr_in->theta_e_ref &= 0x3FFF;
}
static void CSM_enter_MEASURE(CSM_t *self) {
	self->offset_accumulate += (int32_t)_delta_u14(self->isr_in->theta_e_ref, self->isr_out->theta_e);
	self->num_samples++;
}
static void CSM_exit_CALIBRATE_ENCODER(CSM_t *self) {
	self->config->theta_e_offset = (int32_t)self->offset_accumulate / (int32_t)self->num_samples;
	self->encoder_offset_valid = true;
}
static void CSM_enter_HOMING(CSM_t *self) {
	self->homing_valid = false;
	self->isr_in->theta_m_homing_offset = 0; //handles invert_direction=true case
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_IDQ;
	self->isr_in->control_mode = CONTROL_MODE_VEL;
	//home from positive command angles toward negative, regardless of
	//config.invert_direction, and set zero-angle to the stall position
	self->isr_in->omega_m_ref = (int16_t)(-CAL_HOME_VEL/OMEGA_M_LSB);
}

static void CSM_exit_HOMING(CSM_t *self) {
	self->homing_valid = true;
	self->isr_in->theta_m_homing_offset = self->isr_out->theta_m;
	if(self->config->invert_direction) {
		self->isr_in->theta_m_homing_offset *= -1;
	}
}
static void CSM_enter_ARMED(CSM_t *self) {
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_IDQ;
	self->isr_in->control_mode = CONTROL_MODE_TORQUE;
	self->isr_in->iq_ref = 0;
	self->isr_in->omega_m_ref = 0;
	self->isr_in->theta_m_ref = 0;
}
static void CSM_enter_DISARMED(CSM_t *self) {
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_STOP;
	self->isr_in->clear_errors_flag = false; //allow ISR to report errors once more
}
static void CSM_enter_DISARMED_ERROR(CSM_t *self) {
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_STOP;
	if(self->error == LOCAL_SPINORSTATUS_ERROR_ENCODER_SPI ||
			self->error == LOCAL_SPINORSTATUS_ERROR_ENCODER_B) {
		self->homing_valid = false; //encoder may have lost tracking, must home again
	}
	self->isr_in->clear_errors_flag = true; //clear sticky errors in ISR, they'll remain in the csm.error sticky
}
void CSM_dispatch_event(CSM_t *self, ControllerEvent_t event) {
	switch(event.type) {
		case EVENT_DEFAULT:
			//default event is sent at every mainloop iteration to this state machine
			//offset calibration sm
			if(self->state == CONTROLLERSTATE_CALIBRATE_ENCODER) {
				switch(self->substate) {
					case CONTROLLERSUBSTATE_NULL:
						break;
					case CONTROLLERSUBSTATE_MOVE_B:
						if(event.t > self->t_start+CAL_OFFSET_INITIAL_DELAY) {
							CSM_enter_MOVE_A(self);
							self->substate = CONTROLLERSUBSTATE_MOVE_A;
							self->t_start = event.t;
						}
						break;
					case CONTROLLERSUBSTATE_MOVE_A:
						if(event.t > self->t_start+CAL_OFFSET_INITIAL_DELAY) {
							CSM_enter_MEASURE(self);
							self->substate = CONTROLLERSUBSTATE_MEASURE;
						}
						break;
					case CONTROLLERSUBSTATE_MEASURE:
						if(self->num_samples < CAL_OFFSET_NUM_SAMPLES) {
							CSM_enter_MOVE(self);
							self->substate = CONTROLLERSUBSTATE_MOVE;
							self->t_start = event.t;
						} else {
							CSM_exit_CALIBRATE_ENCODER(self);
							CSM_enter_DISARMED(self);
							self->state = CONTROLLERSTATE_DISARMED;
							self->substate = CONTROLLERSUBSTATE_NULL;
						}
						break;
					case CONTROLLERSUBSTATE_MOVE:
						if(event.t > self->t_start+CAL_OFFSET_DELAY) {
							CSM_enter_MEASURE(self);
							self->substate = CONTROLLERSUBSTATE_MEASURE;
						}
						break;
				}
			} 
			//stall-based homing sm
			else if(self->state == CONTROLLERSTATE_HOMING) {
				//use high abs(torque) to indicate stall at home pos
				//this works for invert_direction=true
				if(self->isr_out->iq < 200 
					&& self->isr_out->iq > -200) {
					self->t_start = event.t;
				}
				//must maintain stall torque threshold for a time to avoid noise ending the home sequence early 
				if(event.t > self->t_start+500) {
					CSM_exit_HOMING(self);
					CSM_enter_DISARMED(self);
					self->state = CONTROLLERSTATE_DISARMED;
				}
			} else if(self->state == CONTROLLERSTATE_ARMED) {
				//arming message timeout causes an error 
				if(event.t > self->t_start+1000) {
					self->error = LOCAL_SPINORSTATUS_ERROR_COMMAND_TIMEOUT;
					CSM_enter_DISARMED_ERROR(self);
					self->state = CONTROLLERSTATE_DISARMED_ERROR;
				}
			}
			//used at startup
			else if(self->state == CONTROLLERSTATE_NULL) {
				CSM_enter_DISARMED(self);
				self->state = CONTROLLERSTATE_DISARMED;
				self->substate = CONTROLLERSUBSTATE_NULL;

				/*if(self->config->theta_e_offset == NONE) {
					CSM_enter_CALIBRATE_ENCODER(self);
					CSM_enter_MOVE_B(self);
					self->state = CONTROLLERSTATE_CALIBRATE_ENCODER;
					self->substate = CONTROLLERSUBSTATE_MOVE_B;
					self->t_start=event.t;
				} else {
					CSM_enter_HOMING(self);
					self->state = CONTROLLERSTATE_HOMING;
				}*/
			}
			break;
		case EVENT_ERROR:
			if(self->state != CONTROLLERSTATE_DISARMED_ERROR) {
				CSM_enter_DISARMED_ERROR(self);
				self->state = CONTROLLERSTATE_DISARMED_ERROR;
			}
			break;
		case EVENT_DISARM:
			if(self->state == CONTROLLERSTATE_ARMED) {
				//CSM_exit_ARMED(self);
				CSM_enter_DISARMED(self);
				self->state = CONTROLLERSTATE_DISARMED;
			}
			else if(self->state == CONTROLLERSTATE_DISARMED_ERROR) {
				//CSM_exit_DISARMED_ERROR(self);
				CSM_enter_DISARMED(self);
				self->state = CONTROLLERSTATE_DISARMED;
				//disarm rx will clear errors
				self->error = LOCAL_SPINORSTATUS_ERROR_OK;
				timer_disable_break_main_output(TIM1);
			}
			break;
		case EVENT_ARM:
			//can only arm from disarmed state
			if(self->state == CONTROLLERSTATE_DISARMED) {
				//and only if already homed
				if(true || self->homing_valid) {
					CSM_enter_ARMED(self);
					self->state = CONTROLLERSTATE_ARMED;
					self->t_start = event.t; //fix race condition
				}
				//if not homed yet, do it now
				else {
					CSM_enter_HOMING(self);
					self->state = CONTROLLERSTATE_HOMING;
					self->t_start = event.t;
				}
			}
			//update time of most recent arm rx for timeout handling
			else if(self->state == CONTROLLERSTATE_ARMED) {
				self->t_start = event.t;
			}
			break;
		case EVENT_CALIBRATE_ENCODER:
			if(self->state == CONTROLLERSTATE_DISARMED) {
				CSM_enter_CALIBRATE_ENCODER(self);
				self->state = CONTROLLERSTATE_CALIBRATE_ENCODER;
				self->substate = CONTROLLERSUBSTATE_MOVE_B;
				self->t_start = event.t;
			}
			break;
		case EVENT_HOMING:
			if(self->state == CONTROLLERSTATE_DISARMED) {
				CSM_enter_HOMING(self);
				self->state = CONTROLLERSTATE_HOMING;
				self->t_start = event.t;
			}
			break;
	}
}

void CSM_init(CSM_t *self, controller_in_t *isr_in, controller_out_t *isr_out) {
	self->isr_in = isr_in;
	self->isr_out = isr_out;
	self->state = CONTROLLERSTATE_NULL;
	self->substate = CONTROLLERSUBSTATE_NULL;
	self->config = config_get();
}

void CSM_led_indicator(CSM_t *self, uint32_t t) {
	uint8_t r = false;
	uint8_t b = false;
	switch(self->state) {
		case CONTROLLERSTATE_DISARMED_ERROR:
			//red slow flash
			r = (t % 1024) < 256; 
			break;
		case CONTROLLERSTATE_DISARMED:
			//blue slow flash
			b = (t % 1024) < 256; 
			break;
		case CONTROLLERSTATE_ARMED:
			//blue fast flash
			b = (t % 256) < 128;
			break;
		case CONTROLLERSTATE_HOMING:
		case CONTROLLERSTATE_CALIBRATE_ENCODER:
			//LICE CAR ;)
			r = (t % 256) < 128;
			b = !r;
			break;
		default:
			break;
	}
	//active low LEDs
	if(r) {
		gpio_clear(LED_PORT, LED_R);
	} else {
		gpio_set(LED_PORT, LED_R);
	}
	if(b) {
		gpio_clear(LED_PORT, LED_B);
	} else {
		gpio_set(LED_PORT, LED_B);
	}
}
