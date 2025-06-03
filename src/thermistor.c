#include "thermistor.h"

//ADC readings for 10k NTC with B=3900K
//at 0, 10, ...100degC
#define NTC_LUT {\
  3587*3,  3316*3,  2976*3,  2587*3,  2178*3,\
  1785*3,  1432*3,  1132*3,   888*3,   694*3,\
   543*3}
#define NTC_LUT_LEN 11

const uint16_t ntc_lut[] = NTC_LUT;

float thermistor_temperature_from_adc(uint16_t adc) {
	//find index of first time table value is smaller than our reading
	uint8_t i;
	for(i=0; i<NTC_LUT_LEN; i++) {
		if(adc > ntc_lut[i]) {
			break;
		}
	}
	//handle being past both ends of table
	if(i==0) return 0;
	if(i==NTC_LUT_LEN) return 100;
	//inside the table. interpolate
	return 10.0*((float)i -  
		((float)(adc-ntc_lut[i]) / (float)(ntc_lut[i-1]-ntc_lut[i])));
}
