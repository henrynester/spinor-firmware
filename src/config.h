#pragma once

#include <stdint.h>
#include "uavcan.protocol.param.GetSet.h"

#include "math.h"
//Motor properties
#define NUM_POLE_PAIRS 14
#define PHASE_RESISTANCE 210e-3 //Ohm
#define PHASE_INDUCTANCE 120e-6 //H
#define OUTPUT_GEARING 8.0
#define TORQUE_CONSTANT 0.47 //Nm/A
//Nm2. scale up value given for rotor by gear ratio to get inertia felt at output
#define ROTOR_INERTIA (28e-7*OUTPUT_GEARING) 

//Encoder PLL properties
#define PLL_BANDWIDTH 1000.0 //rad/s
#define PLL_KP 2.0*PLL_BANDWIDTH //(rad/s)/rad
#define PLL_KI 0.25*PLL_KP*PLL_KP //(rad/s2)/rad

//FOC properties
#define FOC_BANDWIDTH 750.0 //rad/s
#define FOC_KP FOC_BANDWIDTH*PHASE_INDUCTANCE //Ohm
#define FOC_KI FOC_BANDWIDTH*PHASE_RESISTANCE //Ohm/s

//Velocity control properties
#define VEL_BANDWIDTH 9000.0 //rad/s
#define VEL_LIMIT ((360.0)/60.0*2*M_PI) //rad/s at output
#define VEL_KP (2.0*ROTOR_INERTIA*VEL_BANDWIDTH) //Nm/(rad/s)
#define VEL_KI (0.25*VEL_KP*VEL_KP/ROTOR_INERTIA/350.0) //Nm/(rad) 
#define TORQUE_LIMIT 1.0 //Nm
//fraction of VEL_LIMIT at which torque limit starts to fall off linearly with speed
#define TORQUE_LIMIT_SLOPE_FRACTION 0.75 

//Position control properties
#define POS_LIMIT_MIN 0
#define POS_LIMIT_MAX 2*M_PI
#define POS_BANDWIDTH 75.0
#define POS_KP POS_BANDWIDTH

//Power supply properties
#define VBUS_NOMINAL 20.0

//Calibration properties
#define CAL_OFFSET_VD 2000 
#define CAL_OFFSET_STEP (0x4000/0x08) //theta e counts
#define CAL_OFFSET_NUM_SAMPLES 2*((NUM_POLE_PAIRS * 0x4000) / CAL_OFFSET_STEP)
#define CAL_OFFSET_DELAY 200 //ms
#define CAL_OFFSET_INITIAL_DELAY 1000 //ms
#define CAL_HOME_VEL (10.0/60.0*2*M_PI) //rad/s output
#define CAL_HOME_THRESHOLD_TORQUE 0.5 //N-m
#define CAL_HOME_THRESHOLD_TIME 0.5
				  
//Safety limits
#define SAFETY_VBUS_MIN 16.5
#define SAFETY_VBUS_MAX 22.5
#define SAFETY_IBUS_MAX 1.0
#define SAFETY_IBUS_REGEN_MAX 1.0
#define SAFETY_IABC_MAX 10.0
#define SAFETY_VEL_MAX (300.0)/60.0*2*M_PI

//dronecan can r/w this struct, which has pointers
//to the main config struct above. Default, min, and max
//values are also set up here. Defaults are applied at power up
