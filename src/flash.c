#include "flash.h"
#include "config.h"
#include "constants.h"
#include <string.h>

#include "libopencm3/stm32/flash.h"

#define CONFIG_DB_PAGE_ADDRESS 0x0801F800 //last 2K page of 128KB flash
#define CONFIG_DB_MAGIC 0xFFC0FFEE 

typedef struct {
	float safety_vbus_min;
	float safety_vbus_max;
	float safety_iabc_max;
	float safety_vel_max;
	
	float homing_velocity;
	float homing_threshold_torque;
	float homing_threshold_time;

	float pos_limit_min;
	float pos_limit_max;
	float vel_limit;
	float torque_limit;

	float invert_direction;

	float node_id;
	float actuator_index; 

	float theta_m_ref;
	float omega_m_ref;
	float iq_ref;
} parameter_storage_t;
parameter_storage_t param_db;

parameter_t param_reg[] = {
	{"safety_vbus_min", &param_db.safety_vbus_min, 0.0, SAFETY_VBUS_MIN, 35.0},
	{"safety_vbus_max", &param_db.safety_vbus_max, 0.0, SAFETY_VBUS_MAX, 35.0},
	{"safety_iabc_max", &param_db.safety_iabc_max, 0.0, SAFETY_IABC_MAX, 20.0},
	{"safety_vel_max", &param_db.safety_vel_max, 0.0, SAFETY_VEL_MAX, 10.0*SAFETY_VEL_MAX},

	{"homing_velocity", &param_db.homing_velocity, 0.0, CAL_HOME_VEL, SAFETY_VEL_MAX},
	{"homing_threshold_torque", &param_db.homing_threshold_torque, 0.0, CAL_HOME_THRESHOLD_TORQUE, SAFETY_VEL_MAX},
	{"homing_threshold_time", &param_db.homing_threshold_time, 0.0, CAL_HOME_THRESHOLD_TIME, 2.0},
	
	{"pos_limit_min", &param_db.pos_limit_min, -100.0*POS_LIMIT_MAX, -POS_LIMIT_MAX, 100.0*POS_LIMIT_MAX},
	{"pos_limit_max", &param_db.pos_limit_max, -100.0*POS_LIMIT_MAX, +POS_LIMIT_MAX, 100.0*POS_LIMIT_MAX},
	{"vel_limit", &param_db.vel_limit, 0.0, VEL_LIMIT, 10.0*VEL_LIMIT},
	{"torque_limit", &param_db.torque_limit, 0.0, TORQUE_LIMIT, 10.0*TORQUE_LIMIT},

	{"invert_direction", &param_db.invert_direction, 0.0, 0.0, 1.0},

	{"node_id", &param_db.node_id, 0.0, 42.0, 127.0},
	{"actuator_index", &param_db.actuator_index, 0.0, 0.0, 15.0},
};


typedef struct {
	uint32_t magic;
	config_t config;
	uint32_t magic2;
} config_db_t;

config_db_t config_db;

config_t* config_get(void) {
	return &config_db.config;
}

uint8_t config_load(void) {
	for(uint32_t idx=0; idx<sizeof(config_db_t); idx+=4) {
		//read address in config page
		uint32_t word = *(uint32_t*)( CONFIG_DB_PAGE_ADDRESS+idx );
		//save into struct
		*(uint32_t*)((void*)(&config_db) + idx) = word;
	}
	//config data integrity checks
	if(config_db.magic != CONFIG_DB_MAGIC || config_db.magic2 != CONFIG_DB_MAGIC) {
		return FLASH_ERROR_READ_BAD_MAGIC;
	}
	return FLASH_ERROR_OK;
}

uint8_t config_save(void) {
	//make sure we have the magic number and the latest checksum
	config_db.magic = CONFIG_DB_MAGIC;
	config_db.magic2 = CONFIG_DB_MAGIC;

	//unlock flash, erase the page
	flash_unlock();
	flash_erase_page(CONFIG_DB_PAGE_ADDRESS);
	uint32_t flash_status = flash_get_status_flags();
	if(flash_status != FLASH_SR_EOP) {
		return FLASH_ERROR_ERASE_FAIL;
	}
	//write one uint32_t at a time
	for(uint32_t idx=0; idx<sizeof(config_db_t); idx+=4) {
		volatile uint32_t word = *(uint32_t*)( ((void *)&config_db) + idx );
		flash_program_word(CONFIG_DB_PAGE_ADDRESS+idx, word); 
		flash_status = flash_get_status_flags();
		if(flash_status != FLASH_SR_EOP) {
			return FLASH_ERROR_WRITE_FAIL;
		}

		//readback check
		if(*((uint32_t*)(CONFIG_DB_PAGE_ADDRESS+idx)) != word) {
			return FLASH_ERROR_WRITE_READBACK_FAIL;
		}
	}
	return FLASH_ERROR_OK;
}

void config_apply(CanardInstance* canard, isr_in_t *isr_in) {
	//canardSetLocalNodeID(canard, config_db.config.node_id);
	//isr_in->theta_e_offset = config_db.config.theta_e_offset;
}

void config_defaults(void) {
	//set all outward facing float parameters to their default values
	for(uint8_t i = 0; i < sizeof(param_reg)/sizeof(parameter_t); i++) {
		*(param_reg[i].value) = param_reg[i].default_value;
	}
	//update internal int config values from parameters
	config_from_params();
	config_to_params();
	config_db.config.theta_e_offset = NONE; //clear this bad boi
}

parameter_t *config_get_param_by_idx(uint8_t idx) {
	if(idx < sizeof(param_reg)/sizeof(parameter_t)) {
		return &param_reg[idx];
	} else {
		return NULL;
	}
}

parameter_t *config_get_param_by_name(char* name, uint8_t len) {
        for (uint8_t i=0; i<sizeof(param_reg)/sizeof(parameter_t); i++) {
            if (len == strlen(param_reg[i].name) &&
                strncmp((const char *)name, param_reg[i].name, len) == 0) {
		    return &param_reg[i];
            }
        }
	return NULL;
}

void config_from_params(void) {
	config_t *c = &config_db.config;
	c->safety_vbus_min = param_db.safety_vbus_min / VBUS_LSB;
	c->safety_vbus_max = param_db.safety_vbus_max / VBUS_LSB;
	c->safety_iabc_max = param_db.safety_iabc_max / IABC_LSB;
	c->safety_vel_max = param_db.safety_vel_max / OMEGA_M_LSB;

	c->homing_velocity = param_db.homing_velocity / OMEGA_M_LSB;
	c->homing_threshold_torque = param_db.homing_threshold_torque / TORQUE_IDQ_LSB;
	c->homing_threshold_time = param_db.homing_threshold_time / 1000.0;

	c->pos_limit_min = param_db.pos_limit_min / (THETA_M_LSB/(float)0x100);
	c->pos_limit_max = param_db.pos_limit_min / (THETA_M_LSB/(float)0x100);
	c->vel_limit = param_db.vel_limit / OMEGA_M_LSB;
	c->torque_limit = param_db.torque_limit / TORQUE_IDQ_LSB;

	c->invert_direction = (param_db.invert_direction != 0) ? true : false;

	c->node_id = param_db.node_id;
	c->actuator_index = param_db.actuator_index;
}

void config_to_params(void) {
	config_t *c = &config_db.config;
	param_db.safety_vbus_min = c->safety_vbus_min * VBUS_LSB;
	param_db.safety_vbus_max = c->safety_vbus_max * VBUS_LSB;
	param_db.safety_iabc_max = c->safety_iabc_max * IABC_LSB;
	param_db.safety_vel_max = c->safety_vel_max * OMEGA_M_LSB;

	param_db.homing_velocity = c->homing_velocity * OMEGA_M_LSB;
	param_db.homing_threshold_torque = c->homing_threshold_torque * TORQUE_IDQ_LSB;
	param_db.homing_threshold_time = c->homing_threshold_time * 1000.0;

	param_db.pos_limit_min = c->pos_limit_min * (THETA_M_LSB/(float)0x100);
	param_db.pos_limit_max = c->pos_limit_min * (THETA_M_LSB/(float)0x100);
	param_db.vel_limit = c->vel_limit * OMEGA_M_LSB;
	param_db.torque_limit = c->torque_limit * TORQUE_IDQ_LSB;

	param_db.invert_direction = (c->invert_direction) ? 1.0 : 0.0;

	param_db.node_id = c->node_id;
	param_db.actuator_index = c->actuator_index;
}
