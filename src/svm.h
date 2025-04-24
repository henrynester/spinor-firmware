#pragma once

#include <stdint.h>

void svm_calc(uint16_t theta_e_u14, int16_t pwm_d, int16_t pwm_q, 
		uint16_t *out_pwm_a, uint16_t *out_pwm_b, uint16_t *out_pwm_c);
