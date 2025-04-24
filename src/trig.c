#include "trig.h"

const int16_t sin_lut[] = SIN_LUT;

// arg: uint16_t ranging [0,2**14) ~ [0,2pi)
// returns sine of represented angle as int16_t ranging [-2**15, 2**15) ~ [-1,1]
int16_t fsin(uint16_t angle_u14) {
	//clamp input
	if(angle_u14 > U14_MAX) {
	       angle_u14 = U14_MAX;
	}
	//downsample the sine by factor 64 (14-bit -> 8-bit)
	uint8_t idx = angle_u14 >> 6; 
	//which quadrant?
	//[0...pi/2) sin(theta) = sin(theta) 
	if(idx < N_SIN_LUT) {
		return sin_lut[idx];
	}
	//[pi/2, pi) sin(theta) = sin(pi - theta) 
	else if(idx < 2*N_SIN_LUT) {	
		return sin_lut[2*N_SIN_LUT - idx];
	}
	//[pi, 3pi/2) sin(theta)=-sin(theta-pi)
	else if(idx < 3*N_SIN_LUT) {
		return -sin_lut[idx - 2*N_SIN_LUT];
	}
	//[3pi/2, 2pi) sin(theta)=-sin(2pi-theta)
	else {
		return -sin_lut[4*N_SIN_LUT - idx];
	}
}

int16_t fcos(uint16_t angle_u14) {
	//clamp input
	if(angle_u14 > U14_MAX) {
	       angle_u14 = U14_MAX;
	}
	//downsample by factor 64 (14-bit -> 8-bit)
	uint8_t idx = angle_u14 >> 6; 
	//which quadrant?
	//[0...pi/2) cos(theta) = sin(pi/2-theta) 
	if(idx < N_SIN_LUT) {
		return sin_lut[N_SIN_LUT - idx];
	}
	//[pi/2, pi) cos(theta) = -sin(theta - pi/2) 
	else if(idx < 2*N_SIN_LUT) {	
		return -sin_lut[idx - N_SIN_LUT];
	}
	//[pi, 3pi/2) cos(theta) = -sin(3pi/2-theta)
	else if(idx < 3*N_SIN_LUT) {
		return -sin_lut[3*N_SIN_LUT - idx];
	}
	//[3pi/2, 2pi) sin(theta)=sin(theta-3pi/2)
	else {
		return sin_lut[idx-3*N_SIN_LUT];
	}
}

