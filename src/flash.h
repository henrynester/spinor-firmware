#pragma once

#include <stdint.h>
#include "libcanard/canard.h"

typedef struct {
	int16_t theta_e_offset;
	
	int32_t pos_limit_max;
	uint8_t invert_direction;

	uint8_t node_id;
	uint8_t actuator_index; 

	int32_t pos_kp;
	int32_t vel_kp;
	int32_t vel_ki;
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
//void config_apply(CanardInstance* canard, controller_in_t *controller_in); 
void config_defaults(void);
parameter_t *config_get_param_by_idx(uint8_t idx);
parameter_t *config_get_param_by_name(char* name, uint8_t len);
void config_from_params(void);
void config_to_params(void);
