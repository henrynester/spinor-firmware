#pragma once

#include <stdint.h>
#include <arm_fp16.h>

#define M_PI 3.141593 
#define M_SQRT3 1.732051 
#define M_SQRT2 1.414214 

//Multiply a fixed-point by a floating-point number
//Converts the float to an integer with 1.0=0x8000
//single-instruction multiply on M0 processor makes this efficient
#define FMUL(floatingpoint, fixedpoint) (( (int32_t)((float)0x8000*floatingpoint) * (int32_t)(fixedpoint) )/0x8000)

#define CONSTRAIN(x, min, max) (((x)<(min))?(min) : (((x)>(max)) ? (max) : (x)))
#define OUTSIDE(x, min, max) ((x)<(min) || (x)>(max))

#define U14_MAX 0x3FFF //14-bit

int16_t fsin(uint16_t angle_u14);
int16_t fcos(uint16_t angle_u14);
