#pragma once

#include <stdint.h>

typedef enum {
	FOC,
	FORCE_THETA_E,
	FORCE_V_ABC,
	LOW,
	LOW_OC_TRIP
} ISR_pwm_src_t; 

typedef struct {
	ISR_pwm_src_t pwm_src;	
	int16_t id_cmd;
	int16_t iq_cmd;
	uint16_t theta_e_cmd;
	int16_t va_cmd;
	int16_t vb_cmd;
	int16_t vc_cmd;
} ISR_rx_t;

typedef struct {
	uint16_t ia;
	uint16_t ib;
	uint16_t ic;
	
	int16_t id;
	int16_t iq;
	int32_t iq_ref_clamped;

	uint16_t va;
	uint16_t vb;
	uint16_t vc;

	int16_t vd;
	int16_t vq;

	int32_t theta_m_next;
	int32_t omega_m_next;
	int32_t theta_m;
	int32_t omega_m;
	uint16_t theta_m_sensor;
	uint16_t theta_e;
	uint16_t n_encoder_err;
	uint8_t encoder_err;

	uint16_t v_bus;
	uint16_t T_mtr;
	uint16_t T_fet;

	uint32_t t_exec;
	uint32_t t_exec_max;

	ISR_pwm_src_t pwm_src;
} ISR_tx_t; 

extern volatile uint32_t g_uptime_ms;
extern volatile ISR_tx_t _ISR_tx;
extern volatile ISR_rx_t _ISR_rx; //does not need to be volatile since interrupt will not touch it
