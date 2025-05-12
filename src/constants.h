#pragma once

#define CLOCK_FREQ 48000000
#define SYSTICK_RELOAD ((CLOCK_FREQ/1000)-1)
#define SYSTICK_PER_US (CLOCK_FREQ/1000000)

#define PWM_FREQ 22500
#define SYSTICK_ISR_PERIOD (CLOCK_FREQ/PWM_FREQ)
#define SYSTICK_ISR_MAX_SAFE ((SYSTICK_ISR_PERIOD*3)/4)
#define PWM_TIMER_RELOAD ((CLOCK_FREQ/2)/PWM_FREQ)
//120 counts=2.5us (5us current sampling time)
#define PWM_MAX 1024 //#(PWM_TIMER_MAX-120) 
#define PWM_CENTER (PWM_MAX/2) 
//600ns wait after FETs go low before starting current sampling
#define PWM_TIMER_SAMPLE_CURRENT 1040 
#define PID_FREQ 1000

#define N_POLE_PAIRS 14

#define CALIBRATION_VD 3000

#define VBUS_MIN 0 
#define VBUS_MAX 0xFFF
#define I_MIN 0x100 
#define I_MAX 0xF00

#define ADC_BUFFER_SIZE 4
