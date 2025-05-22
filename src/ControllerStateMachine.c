#include "ControllerStateMachine.h"
#include "constants.h"
#include "flash.h"

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
	if(config_save()) {
		//while(1);
	}
}
static void CSM_enter_HOMING(CSM_t *self) {
	self->homing_valid = false;
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_IDQ;
	self->isr_in->iq_ref = NONE;
	self->isr_in->omega_m_ref = (int16_t)(CAL_HOME_VEL/OMEGA_M_LSB);
	self->isr_in->theta_m_ref = NONE;
}

static void CSM_exit_HOMING(CSM_t *self) {
	self->homing_valid = true;
	self->isr_in->theta_m_homing_offset = self->isr_out->theta_m;
}
static void CSM_enter_ARMED(CSM_t *self) {
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_IDQ;
	self->isr_in->iq_ref = NONE;
}
static void CSM_enter_DISARMED(CSM_t *self) {
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_STOP;
}
static void CSM_enter_DISARMED_ERROR(CSM_t *self) {
	self->isr_in->foc_control_mode = FOC_CONTROL_MODE_STOP;
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
							CSM_enter_ARMED(self);
							self->state = CONTROLLERSTATE_ARMED;
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
				if(self->isr_out->iq < (int32_t)(CAL_HOME_THRESHOLD_TORQUE/TORQUE_IDQ_LSB)) {
					self->t_start = event.t;
				}
				if(event.t > self->t_start+CAL_HOME_THRESHOLD_TIME) {
					CSM_exit_HOMING(self);
					CSM_enter_ARMED(self);
					self->state = CONTROLLERSTATE_ARMED;
				}
			} else if(self->state == CONTROLLERSTATE_ARMED) {
			       if(false) {
				       CSM_enter_DISARMED(self);
				       self->state = CONTROLLERSTATE_DISARMED;
			       }
			}
			//used at startup
			else if(self->state == CONTROLLERSTATE_NULL) {
				if(self->config->theta_e_offset == NONE) {
					CSM_enter_CALIBRATE_ENCODER(self);
					CSM_enter_MOVE_B(self);
					self->state = CONTROLLERSTATE_CALIBRATE_ENCODER;
					self->substate = CONTROLLERSUBSTATE_MOVE_B;
					self->t_start=event.t;
				} else {
					CSM_enter_HOMING(self);
					self->state = CONTROLLERSTATE_HOMING;
				}
			}
			break;
		case EVENT_ERROR:
			if(self->state != CONTROLLERSTATE_DISARMED_ERROR) {
				CSM_enter_DISARMED_ERROR(self);
				self->state = CONTROLLERSTATE_DISARMED_ERROR;
			}
			break;
		case EVENT_DISARM:
			if(self->state != CONTROLLERSTATE_DISARMED) {
				CSM_enter_DISARMED(self);
				self->state = CONTROLLERSTATE_DISARMED;
			}
			break;
		case EVENT_ARM:
			if(self->state == CONTROLLERSTATE_DISARMED) {
				CSM_enter_ARMED(self);
				self->state = CONTROLLERSTATE_ARMED;
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

void CSM_init(CSM_t *self, isr_in_t *isr_in, isr_out_t *isr_out) {
	self->isr_in = isr_in;
	self->isr_out = isr_out;
	self->state = CONTROLLERSTATE_NULL;
	self->substate = CONTROLLERSUBSTATE_NULL;
	self->config = config_get();
}
