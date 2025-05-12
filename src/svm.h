#pragma once

#include <stdint.h>

uint8_t calibration(void);
void adc_read_averaging_iabc_vbus(uint16_t *ia, uint16_t *ib, uint16_t *ic,
		uint16_t *vbus, uint16_t *Tmot, uint16_t *Tfet, 
		uint8_t measure_temperature);
void iabc_to_idq(int32_t cos_theta_e, int32_t sin_theta_e, 
		uint16_t ia, uint16_t ib, uint16_t ic, 
		int16_t *out_id, int16_t *out_iq);
void pi_control_idq(int16_t id, int16_t iq, int16_t id_ref, int16_t iq_ref,
		int16_t *out_vd, int16_t *out_vq);
void vdq_to_vabc(int32_t cos_theta_e, int32_t sin_theta_e, 
		int16_t vd, int16_t vq,
		uint16_t *out_va, uint16_t *out_vb, uint16_t *out_vc);
void pwm_write(uint16_t pwm_a, uint16_t pwm_b, uint16_t pwm_c);
