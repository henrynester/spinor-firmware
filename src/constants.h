#pragma once
#include "config.h"

#define CLOCK_FREQ 48000000
#define SYSTICK_PER_MS (CLOCK_FREQ/1000)
#define SYSTICK_RELOAD (SYSTICK_PER_MS-1)
#define SYSTICK_PER_US (CLOCK_FREQ/1000000)

#define PWM_FREQ 24000
#define CONTROL_FREQ (PWM_FREQ/3) 
#define PWM_DT (1.0/(float)PWM_FREQ)
#define CONTROL_DT (1.0/(float)(CONTROL_FREQ))
#define PWM_TIMER_RELOAD (CLOCK_FREQ/PWM_FREQ/2)
//120 counts=2.5us (5us current sampling time)
#define PWM_MAX 0x300 //#(PWM_TIMER_MAX-120) 
#define PWM_CENTER (PWM_MAX/2) 
//600ns wait after FETs go low before starting current sampling
#define PWM_TIMER_SAMPLE_CURRENT (PWM_MAX+25)

#define SYSTICK_TO_DELTA_US(start, end) (end < start) ? ((start - end)/SYSTICK_PER_US) : (((start + SYSTICK_PER_MS) - end)/SYSTICK_PER_US)
#define T_EXEC_FOC_MAX (uint16_t)(1.0e6/(float)PWM_FREQ *0.90)
#define T_EXEC_FOC_POS_VEL_MAX (uint16_t)(1.0e6/(float)CONTROL_FREQ *0.95) 


//engineering unit->internal representation conversion factors
//100x amplified 1mOhm shunt -> 100mV/amp. 12bit, 3.3V max. 3x summation for averaging
#define IABC_LSB ((3.3/0.1)/(float)(0x1000)/3.0) //A
//Park transform preserves amplitudes
#define IDQ_LSB (IABC_LSB) //A
#define TORQUE_IDQ_LSB (TORQUE_CONSTANT * IDQ_LSB) 
#define TORQUE_MIN_INT -(int32_t)(TORQUE_LIMIT/TORQUE_IDQ_LSB)
#define TORQUE_MAX_INT (int32_t)(TORQUE_LIMIT/TORQUE_IDQ_LSB)
//int16 gets converted to max SVM range, which is +/- Vbus/sqrt(3)
#define VDQ_LSB ((VBUS_NOMINAL*(float)PWM_MAX/(float)PWM_TIMER_RELOAD/M_SQRT3)/(float)(0x8000)) //V
#define VDQ_MAX (int32_t)((float)0x8000/M_SQRT2)
//0-PWM_MAX is the PWM value range, 0-PWM_TIMER_RELOAD is the full vbus range 
#define PWM_ABC_LSB ((VBUS_NOMINAL*(float)PWM_MAX/(float)PWM_TIMER_RELOAD)
//14-bit encoder count
#define ENCODER_CPR 0x4000
#define THETA_M_LSB (2.0*M_PI/OUTPUT_GEARING/(float)ENCODER_CPR/(float)0x100) //rad output
#define THETA_E_LSB (2.0*M_PI/(float)ENCODER_CPR) //rad electrical phase
#define THETA_M_MIN_INT -(int32_t)((2.0*M_PI)/THETA_M_LSB)						  
#define THETA_M_MAX_INT (int32_t)((2.0*M_PI)/THETA_M_LSB)						  
#define THETA_M_INPUT_LSB (THETA_M_LSB*(float)0x10)
//internal angular velocity repr is scaled up by 0x100 to give more resolution
#define OMEGA_M_LSB (2.0*M_PI/OUTPUT_GEARING/(float)ENCODER_CPR/(float)0x100 / CONTROL_DT) //rad/s output
#define VEL_LIMIT_INT (int32_t)(VEL_LIMIT/OMEGA_M_LSB)
//100k-10k voltage divider, 3x samples summed for averaging
#define VBUS_LSB (3.3*11.0/(float)0x1000/3.0) //V

//LSB of PID parameter integer representations
#define POS_KP_LSB (1.0/(float)0x1000/(THETA_M_LSB*(float)0x100/OMEGA_M_LSB))
#define VEL_KP_LSB (1.0/(float)0x1000/(OMEGA_M_LSB/TORQUE_IDQ_LSB))
#define VEL_KI_LSB (1.0/(float)0x100000/(OMEGA_M_LSB*CONTROL_DT/TORQUE_IDQ_LSB))

//convert safety limits
#define SAFETY_IABC_MAX_INT (uint16_t)( (uint16_t)(SAFETY_IABC_MAX/IABC_LSB) + 0x3000/2)
#define SAFETY_IABC_MIN_INT (uint16_t)(-(uint16_t)(SAFETY_IABC_MAX/IABC_LSB) + 0x3000/2)
#define SAFETY_VBUS_MIN_INT (uint16_t)(SAFETY_VBUS_MIN / VBUS_LSB)
#define SAFETY_VBUS_MAX_INT (uint16_t)(SAFETY_VBUS_MAX / VBUS_LSB)
#define SAFETY_VEL_MAX_INT (uint16_t)(SAFETY_VEL_MAX / OMEGA_M_LSB)
#define SAFETY_TFET_MAX_INT (uint16_t)2178*3 //see thermistor LUT, ~40C
#define SAFETY_TMTR_MAX_INT (uint16_t)1432*3 //see thermistor LUT, ~70C
