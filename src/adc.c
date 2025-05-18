#include "adc.h"
#include "libopencm3/stm32/adc.h"
#include "pins.h"

volatile uint16_t adc_buffer[ADC_BUFFER_LEN];
adc_measure_mode_t _mode = ADC_MODE_IA_IB_IC_VBUS;

void adc_copy_results(adc_results_t *adc_results) {
	adc_results->ia[0] = adc_buffer[0]; 
	adc_results->ib[0] = adc_buffer[1]; 
	adc_results->ic[0] = adc_buffer[2]; 
	adc_results->ia[1] = adc_buffer[4]; 
	adc_results->ib[1] = adc_buffer[5]; 
	adc_results->ic[1] = adc_buffer[6]; 
	adc_results->ia[2] = adc_buffer[8]; 
	adc_results->ib[2] = adc_buffer[9]; 
	adc_results->ic[2] = adc_buffer[10]; 

	uint16_t acc = (adc_buffer[3] + adc_buffer[7] + adc_buffer[11]);
	if(_mode == ADC_MODE_IA_IB_IC_VBUS) {
		adc_results->vbus = acc;
	} else if(_mode == ADC_MODE_IA_IB_IC_TMTR) {
		adc_results->Tmtr = acc;
	} else if(_mode == ADC_MODE_IA_IB_IC_TFET) {
		adc_results->Tfet = acc;
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
