#include "trig.h"

#define N_SIN_LUT 64 
#define SIN_LUT {\
     0,   804,  1607,  2410,  3211,  4011,  4807,  5601,  6392,  7179,\
  7961,  8739,  9511, 10278, 11038, 11792, 12539, 13278, 14009, 14732,\
 15446, 16150, 16845, 17530, 18204, 18867, 19519, 20159, 20787, 21402,\
 22004, 22594, 23169, 23731, 24278, 24811, 25329, 25831, 26318, 26789,\
 27244, 27683, 28105, 28510, 28897, 29268, 29621, 29955, 30272, 30571,\
 30851, 31113, 31356, 31580, 31785, 31970, 32137, 32284, 32412, 32520,\
 32609, 32678, 32727, 32757, 32767}

const int16_t sin_lut[] = SIN_LUT;

// arg: uint16_t ranging [0,2**14) ~ [0,2pi)
// returns sine of represented angle as int16_t ranging [-2**15, 2**15) ~ [-1,1]
int16_t fsin(uint16_t angle_u14) {
	//clamp input
	if(angle_u14 > U14_MAX) {
	       angle_u14 = U14_MAX;
	}
	//downsample the sine by factor 64 (14-bit -> 8-bit)
	uint8_t idx = angle_u14 / 64; 
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
	uint8_t idx = angle_u14 / 64; 
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

