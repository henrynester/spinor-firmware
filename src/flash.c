#include "flash.h"
#include "pins.h"
#include "setup.h"
#include "config.h"
#include "constants.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/scb.h"
#include "libopencm3/cm3/nvic.h"
#include <string.h>

#include "libopencm3/stm32/flash.h"

#define CONFIG_DB_PAGE_ADDRESS 0x0801F800 //last 2K page of 128KB flash
#define CONFIG_DB_MAGIC 0xFFC0FFEE 

typedef struct {
	float pos_limit_max; //radians
	float invert_direction; //0 or 1

	float node_id; //0...127 (0=anon)
	float actuator_index; //0...15

	float pos_kp; //rad/s / rad
	float vel_kp; //Nm / (rad/s)
	float vel_ki; //Nm / (rad/s)s
} parameter_storage_t;
parameter_storage_t param_db;

parameter_t param_reg[] = {
	{"pos_limit_max", &param_db.pos_limit_max,
		0.0, +POS_LIMIT_MAX, 100.0*POS_LIMIT_MAX},
	{"invert_direction", &param_db.invert_direction, 
		0.0, 0.0, 1.0},

	{"node_id", &param_db.node_id, 
		0.0, 42.0, 127.0},
	{"actuator_index", &param_db.actuator_index, 
		0.0, 0.0, 15.0},

	//max values chosen such that integer representation of the pid
	//coefficients stays inside an int16 
	//(which is what we can multiply efficiently)
	{"pos_kp", &param_db.pos_kp, 
		0, POS_KP, INT16_MAX*POS_KP_LSB},
	{"vel_kp", &param_db.vel_kp, 
		0, VEL_KP, INT16_MAX*VEL_KP_LSB},
	{"vel_ki", &param_db.vel_ki, 
		0, VEL_KI, INT16_MAX*VEL_KI_LSB}
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
	config_to_params();
	return FLASH_ERROR_OK;
}

uint8_t config_save(void) {
	gpio_clear(LED_PORT, LED_R);
	gpio_clear(LED_PORT, LED_B);
	timer_disable_break_main_output(TIM1); //stop the motor here
	nvic_disable_irq(NVIC_DMA1_CHANNEL1_IRQ); //disable ISR
	DELAY_US(1000000); //let everything come to a stop
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

	DELAY_US(1000000); //let it sink in to flash just in case :)
	//and... reboot
	scb_reset_system();
	while(1);

	//never runs :P
	return FLASH_ERROR_OK;
}

//void config_apply(CanardInstance* canard, isr_in_t *isr_in) {
	//canardSetLocalNodeID(canard, config_db.config.node_id);
	//isr_in->theta_e_offset = config_db.config.theta_e_offset;
//}

void config_defaults(void) {
	//set all outward facing float parameters to their default values
	for(uint8_t i = 0; i < sizeof(param_reg)/sizeof(param_reg[0]); i++) {
		*(param_reg[i].value) = param_reg[i].default_value;
	}
	//update internal int config values from parameters
	config_from_params();
	//convert back to reflect fixed-point errors
	config_to_params();
}

parameter_t *config_get_param_by_idx(uint8_t idx) {
	if(idx < sizeof(param_reg)/sizeof(param_reg[0])) {
		return &param_reg[idx];
	} else {
		return NULL;
	}
}

parameter_t *config_get_param_by_name(char* name, uint8_t len) {
        for (uint8_t i=0; i<sizeof(param_reg)/sizeof(param_reg[0]); i++) {
            if (len == strlen(param_reg[i].name) &&
                strncmp((const char *)name, param_reg[i].name, len) == 0) {
		    return &param_reg[i];
            }
        }
	return NULL;
}

void config_from_params(void) {
	config_t *c = &config_db.config;
	c->pos_limit_max = param_db.pos_limit_max / THETA_M_LSB;
	c->invert_direction = (param_db.invert_direction != 0) ? true : false;

	c->node_id = param_db.node_id;
	c->actuator_index = param_db.actuator_index;

	c->pos_kp = CONSTRAIN((int32_t)(param_db.pos_kp / POS_KP_LSB)
		, -INT16_MAX, INT16_MAX);
	c->vel_kp = CONSTRAIN((int32_t)(param_db.vel_kp / VEL_KP_LSB)
		, -INT16_MAX, INT16_MAX);
	c->vel_ki = CONSTRAIN((int32_t)(param_db.vel_ki / VEL_KI_LSB)
		, -INT16_MAX, INT16_MAX);
}

void config_to_params(void) {
	config_t *c = &config_db.config;
	param_db.pos_limit_max = (float)c->pos_limit_max * THETA_M_LSB;
	param_db.invert_direction = (c->invert_direction) ? 1.0 : 0.0;

	param_db.node_id = c->node_id;
	param_db.actuator_index = c->actuator_index;

	param_db.pos_kp = (float)c->pos_kp * POS_KP_LSB;
	param_db.vel_kp = (float)c->vel_kp * VEL_KP_LSB;
	param_db.vel_ki = (float)c->vel_ki * VEL_KI_LSB;
}
