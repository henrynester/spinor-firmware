#pragma once
#include <stdint.h>

typedef enum {
	ADC_MODE_IA_IB_IC_VBUS,
	ADC_MODE_IA_IB_IC_TMTR,
	ADC_MODE_IA_IB_IC_TFET
} adc_measure_mode_t;

#define ADC_BUFFER_LEN 3*4
extern volatile uint16_t adc_buffer[ADC_BUFFER_LEN];

typedef struct {
	uint16_t ia;
	uint16_t ib;
	uint16_t ic;
	uint16_t vbus;
	uint16_t Tmtr;
	uint16_t Tfet;
} adc_results_t;

void adc_copy_results(adc_results_t *adc_results);
void adc_set_measure_mode(adc_measure_mode_t mode);
