#pragma once

#include <stdint.h>
#include "libcanard/canard.h"
#include "isr.h"

typedef struct {
	uint8_t node_id;
	int16_t theta_e_offset;
} config_t;

typedef enum {
	FLASH_ERROR_OK,
	FLASH_ERROR_ERASE_FAIL,
	FLASH_ERROR_WRITE_FAIL,
	FLASH_ERROR_WRITE_READBACK_FAIL,
	FLASH_ERROR_READ_BAD_MAGIC,
	FLASH_ERROR_READ_BAD_CHECKSUM
} flash_error_t;

config_t* config_get(void);
uint8_t config_load(void);
uint8_t config_save(void);
void config_apply(CanardInstance* canard, isr_in_t *isr_in); 
void config_defaults(void);
