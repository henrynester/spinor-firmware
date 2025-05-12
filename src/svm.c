#include "svm.h"

#include "trig.h"
#include "config.h"
#include "constants.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/nvic.h"

uint16_t ia_zero, ib_zero, ic_zero;
int16_t ia, ib, ic;

#define CALIBRATION_IABC_ZERO_NAVG 128
#define CALIBRATION_IABC_ZERO_MIN 1900 
#define CALIBRATION_IABC_ZERO_MAX 2200
#define CALIBRATION_OK 0 
#define CALIBRATION_ERR_IABC_ZERO 1
uint8_t calibration() {
	return CALIBRATION_OK;
}


void iabc_to_idq(const int32_t cos_theta_e, const int32_t sin_theta_e, 
		const uint16_t ia, const uint16_t ib, const uint16_t ic, 
		int16_t *out_id, int16_t *out_iq) {
	//force igamma (common-mode current) to be zero as any
	//igamma in our case comes from sensor error
	uint16_t igamma = (ia+ib+ic) / 3;
	// Clarke transform
	// ( ialpha ) = ( 2/3  -1/3      ) * ( ia )
	// ( ibeta  )   ( 0    1/sqrt(3) )   ( ib )
	//sign convention for BLDC modeling has current entering
	//motor as positive, so we need to reverse the sign of the ADC
	//results ia,ib,ic here
	int32_t ialpha = (int32_t)(igamma-ia); 
	int32_t ibeta = (( ONE_OV_SQRT3_TIMES_I16_MAX * (int32_t)(igamma-ia)) / 0x8000)
		      + (( TWO_OV_SQRT3_TIMES_I16_MAX * (int32_t)(igamma-ib)) / 0x8000);

	//Park transform
	int32_t id = ((cos_theta_e * ialpha) / 0x8000) 
		+ ((sin_theta_e * ibeta) / 0x8000);
	int32_t iq = ((-sin_theta_e * ialpha) / 0x8000)
	       	+ ((cos_theta_e * ibeta) / 0x8000);
#define IDQ_AVG_LEN 4 
	static int32_t iq_avg[IDQ_AVG_LEN];
	static int32_t acc;
	static uint16_t iq_avg_idx;
	acc-=iq_avg[iq_avg_idx];
	acc+=iq;
	iq_avg[iq_avg_idx] = iq;
	iq_avg_idx++;
	if(iq_avg_idx >= IDQ_AVG_LEN) {
		iq_avg_idx=0;
	}
	*out_iq = acc / IDQ_AVG_LEN;
		
	static int32_t id_avg[IDQ_AVG_LEN];
	static int32_t acc2;
	static uint16_t id_avg_idx;
	acc2-=id_avg[id_avg_idx];
	acc2+=id;
	id_avg[id_avg_idx] = id;
	id_avg_idx++;
	if(id_avg_idx >= IDQ_AVG_LEN) {
		id_avg_idx=0;
	}
	*out_id = acc2/IDQ_AVG_LEN; //id; //((int32_t)(*out_iq) + iq)/128;
	//*out_iq = (((int32_t)(*out_iq))*31 + iq)/32;
}

void pi_control_idq(int16_t id, int16_t iq, int16_t id_ref, int16_t iq_ref,
		int16_t *out_vd, int16_t *out_vq) {
	int32_t err_d = id - id_ref;
	static int32_t err_int_d = 0;
	int32_t pwm_d = ((-(int32_t)config.foc_kp * err_d) / 0x8000) - err_int_d;
	if(pwm_d < INT16_MAX/2 && pwm_d > INT16_MIN/2) {
		err_int_d = CONSTRAIN(err_int_d + (((int32_t)config.foc_ki * err_d)/0x8000),
				INT16_MIN/2, INT16_MAX/2);
	} else {
		err_int_d -= err_int_d / 128;
	}
	*out_vd = CONSTRAIN(pwm_d, INT16_MIN/2, INT16_MAX/2);

	int16_t err_q = iq - iq_ref;
	static int32_t err_int_q = 0;
	int32_t pwm_q = ((-(int32_t)config.foc_kp * err_q) / 0x8000) - err_int_q;;
	if(pwm_q < INT16_MAX/2 && pwm_q > INT16_MIN/2) {
		err_int_q = CONSTRAIN(err_int_q + (((int32_t)config.foc_ki * err_q)/0x8000),
				INT16_MIN/2, INT16_MAX/2);
	} else {
		err_int_q -= err_int_q / 128;
	}
	*out_vq = CONSTRAIN(pwm_q, INT16_MIN/2, INT16_MAX/2);
}

void vdq_to_vabc(int32_t cos_theta_e, int32_t sin_theta_e, 
		int16_t vd, int16_t vq,
		uint16_t *out_va, uint16_t *out_vb, uint16_t *out_vc) {
	//Inverse Park transform
	int32_t v_alpha = ((cos_theta_e * (int32_t)vd) / 0x8000) 
		+ ((-sin_theta_e * (int32_t)vq) / 0x8000);
	int32_t v_beta = ((sin_theta_e * (int32_t)vd) / 0x8000)
	       	+ ((cos_theta_e * (int32_t)vq) / 0x8000);
	//Inverse Clarke transform
	int32_t va = v_alpha;
	int32_t vb = ((-ONE_OV_SQRT3_TIMES_I16_MAX*v_alpha) / 0x8000) 
		+ v_beta;
	int32_t vc = ((-ONE_OV_SQRT3_TIMES_I16_MAX*v_alpha) / 0x8000)
		+ (-v_beta);
	va = CONSTRAIN(va, INT16_MIN, INT16_MAX);
	vb = CONSTRAIN(vb, INT16_MIN, INT16_MAX);
	vc = CONSTRAIN(vc, INT16_MIN, INT16_MAX);
	//scale down from int16 to int10 size sent to PWM timer
	va /= 0x40;
	vb /= 0x40;
	vc /= 0x40;
	//convert from sinusoidal to SVM voltages using third harmonic injection
	int16_t pwm_min = va;
	if(pwm_min > vb) {
		pwm_min = vb;
	}
	if(pwm_min > vc) {
		pwm_min = vc;
	}
	int16_t pwm_max = va;
	if(pwm_max < vb) {
		pwm_max = vb;
	}
	if(pwm_max < vc) {
		pwm_max = vc;
	}
	uint16_t pwm_center = PWM_CENTER - (pwm_min/2) - (pwm_max/2); 
	*out_va = CONSTRAIN(pwm_center+va, 0, 1024); //PWM_CENTER-400, PWM_CENTER+400);
	*out_vb = CONSTRAIN(pwm_center+vb, 0, 1024); //PWM_CENTER-400, PWM_CENTER+400);
	*out_vc = CONSTRAIN(pwm_center+vc, 0, 1024); //PWM_CENTER-400, PWM_CENTER+400);
}

void pwm_write(uint16_t pwm_a, uint16_t pwm_b, uint16_t pwm_c) {
	TIM1_CCR1 = pwm_a;	
	TIM1_CCR2 = pwm_b;	
	TIM1_CCR3 = pwm_c;	
}
