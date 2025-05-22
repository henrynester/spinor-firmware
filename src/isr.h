#pragma once

#include "foc.h"

typedef enum {
	CONTROL_MODE_NONE,
	CONTROL_MODE_TORQUE,
	CONTROL_MODE_VEL,
	CONTROL_MODE_POS
} control_mode_t;

typedef enum {
	ISR_ERROR_OK,
	ISR_ERROR_FOC_DEADLINE_MISSED,
	ISR_ERROR_POS_VEL_DEADLINE_MISSED,
	ISR_ERROR_OVERCURRENT
} isr_error_t;

typedef struct {
	foc_control_mode_t foc_control_mode;
	control_mode_t control_mode;
	int16_t id_ref;
	int16_t iq_ref;
	int16_t omega_m_ref;
	int16_t theta_m_ref;
	int16_t vd_ref;
	int16_t vq_ref;
	uint16_t theta_e_ref;
	int32_t theta_m_homing_offset;
} isr_in_t;

typedef struct {
	uint16_t ia;
	uint16_t ib;
	uint16_t ic;
	
	int16_t id;
	int16_t iq;
	int16_t iq_ref_combined;

	uint16_t va;
	uint16_t vb;
	uint16_t vc;

	int16_t vd;
	int16_t vq;

	int32_t theta_m;
	int32_t omega_m;
	uint16_t theta_e;

	uint16_t n_encoder_err;
	uint8_t encoder_err;

	uint16_t v_bus;
	uint16_t T_mtr;
	uint16_t T_fet;

	uint16_t t_exec_foc;
	uint16_t t_exec_foc_vel_pos;

	isr_error_t error;
} isr_out_t; 

void sync_isr_out(isr_out_t *isr_out);
void sync_isr_in(isr_in_t *isr_in);

extern volatile uint32_t g_uptime_ms;
