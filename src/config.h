#pragma once

#include <stdint.h>
#include "uavcan.protocol.param.GetSet.h"

//configuration parameters must all be floats
//parameters are read by rest of the code out of
//this struct
typedef struct {
	float iq_ref,
	      vd_ref,
	      vq_ref,
	      foc_kp,
	      foc_ki,
	      foc_k_afc;
} config_t;
extern config_t config;

//dronecan can r/w this struct, which has pointers
//to the main config struct above. Default, min, and max
//values are also set up here. Defaults are applied at power up
typedef struct {
    char *name;
    enum uavcan_protocol_param_Value_type_t type;
    float* value;
    float default_value;
    float min_value;
    float max_value;
} parameter_t;

extern parameter_t parameters[sizeof(config)/sizeof(float)];
