#pragma once

#include <stdint.h>
#include "libcanard/canard.h"
#include "isr.h"

typedef struct {
	int16_t theta_e_offset;
	
	int16_t safety_vbus_min;
	int16_t safety_vbus_max;
	int16_t safety_iabc_max;
	int16_t safety_vel_max;
	
	int16_t homing_velocity;
	int16_t homing_threshold_torque;
	int16_t homing_threshold_time;

	int32_t pos_limit_min;
	int32_t pos_limit_max;
	int16_t vel_limit;
	int16_t torque_limit;

	uint8_t invert_direction;

	uint8_t node_id;
	uint8_t actuator_index; 
} config_t;

typedef enum {
	FLASH_ERROR_OK,
	FLASH_ERROR_ERASE_FAIL,
	FLASH_ERROR_WRITE_FAIL,
	FLASH_ERROR_WRITE_READBACK_FAIL,
	FLASH_ERROR_READ_BAD_MAGIC,
	FLASH_ERROR_READ_BAD_CHECKSUM
} flash_error_t;

typedef struct {
    char *name;
    float* value;
    float min_value;
    float default_value;
    float max_value;
} parameter_t;

config_t* config_get(void);
uint8_t config_load(void);
uint8_t config_save(void);
void config_apply(CanardInstance* canard, isr_in_t *isr_in); 
void config_defaults(void);
parameter_t *config_get_param_by_idx(uint8_t idx);
parameter_t *config_get_param_by_name(char* name, uint8_t len);
void config_from_params(void);
void config_to_params(void);
