#include "svm.h"

#include "trig.h"
#include "constants.h"

void svm_calc(uint16_t theta_e_u14, int16_t pwm_d, int16_t pwm_q, 
		uint16_t *out_pwm_a, uint16_t *out_pwm_b, uint16_t *out_pwm_c) {
	int32_t cos_theta_e = (int32_t)fcos(theta_e_u14);
	int32_t sin_theta_e = (int32_t)fsin(theta_e_u14);
	//Inverse Park transform
	int32_t pwm_alpha = ((cos_theta_e * (int32_t)pwm_d) >> 16) 
		+ ((-sin_theta_e * (int32_t)pwm_q) >> 16);
	int32_t pwm_beta = ((sin_theta_e * (int32_t)pwm_d) >> 16)
	       	+ ((cos_theta_e * (int32_t)pwm_q) >> 16);
	//Inverse Clarke transform
	int16_t pwm_a = pwm_alpha >> 1;
	int16_t pwm_b = ((-HALF_TIMES_I16_MAX*pwm_alpha) >> 16) 
		+ ((SQRT3_OV_2_TIMES_I16_MAX*pwm_beta) >> 16);
	int16_t pwm_c = ((-HALF_TIMES_I16_MAX*pwm_alpha) >> 16)
		+ ((-SQRT3_OV_2_TIMES_I16_MAX*pwm_beta) >> 16);
	//scale down from int16 to int10 size sent to PWM timer
	pwm_a >>= 4;
	pwm_b >>= 4;
	pwm_c >>= 4;
	//convert from sinusoidal to SVM voltages using third harmonic injection
	int16_t pwm_min = pwm_a;
	if(pwm_min > pwm_b) {
		pwm_min = pwm_b;
	}
	if(pwm_min > pwm_c) {
		pwm_min = pwm_c;
	}
	int16_t pwm_max = pwm_a;
	if(pwm_max < pwm_b) {
		pwm_max = pwm_b;
	}
	if(pwm_max < pwm_c) {
		pwm_max = pwm_c;
	}
	uint16_t pwm_center = PWM_CENTER - (pwm_min>>1) - (pwm_max>>1); 
	*out_pwm_a = pwm_center + pwm_a;
	*out_pwm_b = pwm_center + pwm_b;
	*out_pwm_c = pwm_center + pwm_c;
	CONSTRAIN(*out_pwm_a, 0, PWM_MAX);
	CONSTRAIN(*out_pwm_b, 0, PWM_MAX);
	CONSTRAIN(*out_pwm_c, 0, PWM_MAX);
}
