#pragma once

#include "flash.h"
#include "encoder.h"
#include "foc.h"
#include "adc.h"

typedef enum {
	CONTROL_MODE_TORQUE,
	CONTROL_MODE_VEL,
	CONTROL_MODE_POS
} control_mode_t;

typedef enum {
	FAST_ERROR_OK,
	FAST_ERROR_FOC_DEADLINE_MISSED,
	FAST_ERROR_POS_VEL_DEADLINE_MISSED,
	FAST_ERROR_IABC_OVER
} fast_error_t;

typedef struct {
	foc_control_mode_t foc_control_mode;
	control_mode_t control_mode;
	int16_t id_ref;
	int16_t iq_ref;
	int32_t omega_m_ref;
	int32_t theta_m_ref;
	int16_t vd_ref;
	int16_t vq_ref;
	uint16_t theta_e_ref;
	int32_t theta_m_homing_offset;
	uint8_t clear_errors_flag;
} controller_in_t;

typedef struct {
	uint16_t ia;
	uint16_t ib;
	uint16_t ic;
	
	int16_t id;
	int16_t iq;
	int16_t iq_ref;

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
	uint8_t encoder_agc;

	uint16_t v_bus;
	uint16_t T_mtr;
	uint16_t T_fet;

	uint16_t t_exec_foc;
	uint16_t t_exec_controller;

	fast_error_t fast_error;
} controller_out_t; 

typedef struct {
	controller_in_t in; //public input
	controller_in_t _in; //private
	controller_out_t out; //public output
	controller_out_t _out; //private
	uint8_t _rendezvous_flag;

	foc_t foc;
	encoder_t encoder;
	adc_results_t adc_results;
	config_t *config;

	uint16_t counter_slow;

	struct {
		int32_t omega_m_err_integral;
#define OMEGA_M_AVG_LEN 1 
		int32_t omega_m_avgarr[OMEGA_M_AVG_LEN];
		int32_t omega_m_avg;
		uint8_t omega_m_avgidx;
	} vel;
} controller_t;

void controller_init(controller_t *self);
void controller_reset(controller_t *self);
void controller_update(controller_t *self);
void controller_rendezvous_sync_inout(volatile controller_t *self);
uint8_t controller_slow_safety_checks(controller_t *self);
