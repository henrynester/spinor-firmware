#pragma once
#include "constants.h"

#include <stdint.h>

extern volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];

void setup(void);
