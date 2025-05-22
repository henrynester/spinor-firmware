#include "flash.h"

#include "libopencm3/stm32/flash.h"

#define CONFIG_DB_PAGE_ADDRESS 0x0801F800 //last 2K page of 128KB flash
#define CONFIG_DB_MAGIC 0xEEC0FFEE 

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
	config_db.config.node_id = 0x42;
	config_db.config.theta_e_offset = INT16_MIN; //this 14-bit number is not found in the wild
}
