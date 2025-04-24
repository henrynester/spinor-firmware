#pragma once

#define CLOCK_FREQ 48000000
#define PWM_FREQ 21500
#define PWM_TIMER_RELOAD ((CLOCK_FREQ/2)/PWM_FREQ)
//120 counts=2.5us (5us current sampling time)
#define PWM_MAX 1024 //#(PWM_TIMER_MAX-120) 
#define PWM_CENTER 512
//600ns wait after FETs go low before starting current sampling
#define PWM_TIMER_SAMPLE_CURRENT (PWM_MAX+PWM_TIMER_RELOAD)/4
#define PID_FREQ 1000
#define ADC_BUFFER_SIZE (PWM_FREQ*4)/PID_FREQ
