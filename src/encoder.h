#pragma once

#include <stdint.h>

#define ENCODER_ERR_OK 0
#define ENCODER_ERR_SPIMASTER 1 //must =1 since parity errors return 1
#define ENCODER_ERR_SPISLAVE 2
#define ENCODER_ERR_DSP 3
#define ENCODER_ERR_MAGL 4
#define ENCODER_ERR_MAGH 5

int16_t wrap_u14_signed(int16_t delta_angle);
uint8_t encoder_read_angle(uint16_t* angle);
uint8_t encoder_read_status(void);
