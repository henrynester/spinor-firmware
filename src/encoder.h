#pragma once

#include <stdint.h>

typedef enum {
	ENCODER_ERROR_OK,
	ENCODER_ERROR_BAD_PARITY_AT_MASTER,
	ENCODER_ERROR_ERROR_FLAG_AT_MASTER,
	ENCODER_ERROR_BAD_PARITY_AT_SLAVE,
	ENCODER_ERROR_INVALID_COMMAND_AT_SLAVE,
	ENCODER_ERROR_BAD_FRAMING_AT_SLAVE,
	ENCODER_ERROR_ONBOARD_DSP,
	ENCODER_ERROR_BFIELD_UNDER,
	ENCODER_ERROR_BFIELD_OVER
} encoder_error_t;

typedef struct {
	uint16_t theta_m_sensor;
	int32_t theta_m;
	int32_t theta_m_homed;
	int32_t omega_m;
	uint16_t theta_e;
	int32_t cos_theta_e;
	int32_t sin_theta_e;
	encoder_error_t status;
	uint16_t num_spi_errors;
	int32_t theta_m_homing_offset;
	int16_t theta_e_offset;
} encoder_t;

void encoder_pll_load_next(encoder_t *encoder);
void encoder_read_angle(void);
void encoder_read_status(encoder_t *encoder);
void encoder_clear_error(encoder_t *encoder);
void encoder_pll_compute_next(encoder_t *encoder);
int16_t _delta_u14(uint16_t initial, uint16_t final);
