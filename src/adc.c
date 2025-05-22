#include "adc.h"
#include "libopencm3/stm32/adc.h"
#include "pins.h"

volatile uint16_t adc_buffer[ADC_BUFFER_LEN];
adc_measure_mode_t _mode = ADC_MODE_IA_IB_IC_VBUS;

void adc_copy_results(adc_results_t *adc_results) {
	uint16_t misc = 0;
	adc_results->ia=0;
	adc_results->ib=0;
	adc_results->ic=0;
	for(uint8_t i = 0; i < ADC_BUFFER_LEN; i += 4) {
		adc_results->ia += adc_buffer[i];
		adc_results->ib += adc_buffer[i+1];
		adc_results->ic += adc_buffer[i+2];
		misc += adc_buffer[i+3];
	}

	if(_mode == ADC_MODE_IA_IB_IC_VBUS) {
		adc_results->vbus = misc;
	} else if(_mode == ADC_MODE_IA_IB_IC_TMTR) {
		adc_results->Tmtr = misc;
	} else if(_mode == ADC_MODE_IA_IB_IC_TFET) {
		adc_results->Tfet = misc;
	}
}

void adc_set_measure_mode(adc_measure_mode_t mode) {
	if(_mode != mode) {
		_mode = mode;
		//stop ADC so we can adjust its registers
		ADC1_CR |= ADC_CR_ADSTP;
		while(ADC1_CR & ADC_CR_ADSTART);
		//select new channels
		if(_mode == ADC_MODE_IA_IB_IC_VBUS) {
			ADC1_CHSELR = ADC_CHSELR_IA_IB_IC_VBUS;
			//After temperature conversion, tie thermistors'
			//sense lines low to avoid steady overcurrent in thermistors
			gpio_clear(TEMP_PORT, TEMP_PIN);
		} else if(_mode == ADC_MODE_IA_IB_IC_TMTR) {
			ADC1_CHSELR = ADC_CHSELR_IA_IB_IC_TMTR;
			gpio_set(TEMP_PORT, TEMP_PIN);
		} else if(_mode == ADC_MODE_IA_IB_IC_TFET) {
			ADC1_CHSELR = ADC_CHSELR_IA_IB_IC_TFET;
			gpio_set(TEMP_PORT, TEMP_PIN);
		}
		//restart ADC
		ADC1_CR |= ADC_CR_ADSTART;
	}
}
