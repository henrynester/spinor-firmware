#pragma once

#include <stdint.h>
#include "config.h"
#include "isr.h"
#include "flash.h"

typedef enum {
	CONTROLLERSTATE_NULL,
	CONTROLLERSTATE_DISARMED,
	CONTROLLERSTATE_DISARMED_ERROR,
	CONTROLLERSTATE_ARMED,
	CONTROLLERSTATE_CALIBRATE_ENCODER,
	CONTROLLERSTATE_HOMING
} ControllerState_t; 

typedef enum {
	CONTROLLERSUBSTATE_NULL,
	CONTROLLERSUBSTATE_MOVE_B,
	CONTROLLERSUBSTATE_MOVE_A,
	CONTROLLERSUBSTATE_MEASURE,
	CONTROLLERSUBSTATE_MOVE
} ControllerSubstate_t;

typedef enum {
	EVENT_DEFAULT,
	EVENT_ERROR,
	EVENT_DISARM,
	EVENT_ARM,
	EVENT_CALIBRATE_ENCODER,
	EVENT_HOMING
} ControllerEventType_t;

typedef struct {
	ControllerEventType_t type;
	uint32_t t;
	union data_t {
		uint8_t potato;
		int16_t soup;
	} data;
} ControllerEvent_t;

typedef struct {
	ControllerState_t state;
	ControllerSubstate_t substate;
	isr_in_t *isr_in;
	isr_out_t *isr_out;
	config_t *config;
	uint32_t t_start;
	int32_t offset_accumulate;
	uint16_t num_samples;
	uint8_t encoder_offset_valid;
	uint8_t homing_valid;
} CSM_t;

typedef enum {
	ERROR_OK,
	ERROR_RECEIVE_TIMEOUT,
	ERROR_FOC_DEADLINE_MISSED,
	ERROR_POS_VEL_DEADLINE_MISSED,
	ERROR_IABC_HIGH,
	ERROR_VBUS_LOW,
	ERROR_VBUS_HIGH,
	ERROR_TMTR_HIGH,
	ERROR_TFET_HIGH,
	ERROR_VEL_HIGH,
	ERROR_ENCODER,
} error_t;

void CSM_init(CSM_t *self, isr_in_t *isr_in, isr_out_t *isr_out);
void CSM_dispatch_event(CSM_t *self, ControllerEvent_t event);
