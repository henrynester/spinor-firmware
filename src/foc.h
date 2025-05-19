#pragma once
#include "adc.h"
#include "encoder.h"
#include <stdint.h>

typedef enum {
	FOC_CONTROL_MODE_STOP,
	FOC_CONTROL_MODE_IDQ,
	FOC_CONTROL_MODE_VDQ,
	FOC_CONTROL_MODE_VDQ_THETA_E
} foc_control_mode_t;

typedef struct {
	int16_t id;
	int16_t iq;
	int16_t id_ref;
	int16_t iq_ref;
	int16_t vd_ref;
	int16_t vq_ref;
	uint16_t theta_e_ref;
	int16_t vd;
	int16_t vq;
	uint16_t pwm_a;
	uint16_t pwm_b;
	uint16_t pwm_c;
	foc_control_mode_t control_mode;
} foc_t;

void foc_update(foc_t *foc, adc_results_t *adc_results, encoder_t *encoder);
