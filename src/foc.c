#include "foc.h"

#include "math.h"
#include "pins.h"
#include "config.h"
#include "constants.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/nvic.h"

int32_t _id_integral;
int32_t _iq_integral;

static void _park(int32_t cos_theta_e, int32_t sin_theta_e, uint16_t ia, uint16_t ib, uint16_t ic, 
		int16_t *out_id, int16_t *out_iq) {
	//force igamma (common-mode current) to be zero as any
	//igamma in our case comes from sensor error
	int16_t igamma = (ia+ib+ic) / 3;
	// Clarke transform for igamma=0 case
	// ( ialpha ) = ( 1         0         ) * ( ia )
	// ( ibeta  )   ( 1/sqrt(3) 2/sqrt(3) )   ( ib )
	//sign convention for BLDC modeling has current entering
	//motor as positive, so we need to reverse the sign of the ADC
	//results ia,ib,ic here, which are positive for current leaving
	//motor and flowing to ground
	int32_t ialpha = (igamma-ia); 
	int32_t ibeta = FMUL(1.0/M_SQRT3, (igamma-ia)) + FMUL(2.0/M_SQRT3, (igamma-ib));
	
	//Park transform
	int32_t id = ((cos_theta_e * ialpha) / 0x8000) 
		+ ((sin_theta_e * ibeta) / 0x8000);
	int32_t iq = ((-sin_theta_e * ialpha) / 0x8000)
	       	+ ((cos_theta_e * ibeta) / 0x8000);

	//should not overflow an int16 since input is only int14, but just in case
	id = CONSTRAIN(id, INT16_MIN, INT16_MAX);
	iq = CONSTRAIN(iq, INT16_MIN, INT16_MAX);

	*out_id = id;
	*out_iq = iq;
}

static void _pi_control(int16_t i, int16_t i_ref, int32_t *integral, int16_t *out_v) {
	int32_t err = i - i_ref;
	int32_t v = /*FMUL(-FOC_KP/VDQ_LSB*IDQ_LSB, err)*/ - *integral;
	uint8_t saturated = false;
	if(v <= -VDQ_MAX) {
		v = -VDQ_MAX;
		saturated = true;
	}
	if(v >= VDQ_MAX) {
		v = VDQ_MAX;
		saturated = true;
	}
	*out_v = (int16_t)v;
	//if the output saturates, lock the integral term and, in fact, start its decay
	if(saturated) {
		*integral -= *integral / 64;
	}
       	//once the output desaturates, unlock the integral term again	
	//integral term is limited to output range to avoid windup
	else {
		*integral += FMUL(FOC_KI*CONTROL_DT/VDQ_LSB*IDQ_LSB, err);
		*integral = CONSTRAIN(*integral, -VDQ_MAX, VDQ_MAX);
	}
}

static void _inverse_park_inject_harmonic_three(
		int32_t cos_theta_e,
		int32_t sin_theta_e,
		int16_t vd, int16_t vq,
		volatile uint16_t *out_pwma, 
		volatile uint16_t *out_pwmb,
		volatile uint16_t *out_pwmc) {
	//Inverse Park transform
	int32_t v_alpha = ((cos_theta_e * (int32_t)vd) / 0x8000) 
		+ ((-sin_theta_e * (int32_t)vq) / 0x8000);
	int32_t v_beta = ((sin_theta_e * (int32_t)vd) / 0x8000)
	       	+ ((cos_theta_e * (int32_t)vq) / 0x8000);

	//Inverse Clarke transform
	//introduce factor 3/4 because PWM range is 0...0x300
	//Then when we divide down to PWM values at the end, we can do a fast
	//power-of-two division
	//this is the original transform, which preserves the amplitudes of valpha,beta->va,b,c
	/*int32_t va = (v_alpha * (int32_t)0x6000) / 0x8000;
	int32_t vb = ( (int32_t)((float)0x6000*(-1.0/2.0)) *    v_alpha ) / 0x8000 
		   + ( (int32_t)((float)0x6000*(M_SQRT3/2.0)) * v_beta ) / 0x8000;
	int32_t vc = ( (int32_t)((float)0x6000*(-1.0/2.0)) *    v_alpha ) / 0x8000 
		   + ( (int32_t)((float)0x6000*(-M_SQRT3/2.0)) * v_beta ) / 0x8000;*/
	//This is the new transform, which is scaled up by 2/sqrt(3). Then a 
	//valpha,beta input with norm 0x8000 will scale to a vabc output
	//which (after svm has been applied) exactly fits within the PWM range
	int32_t va = ( (int32_t)((float)0x6000*2.0/M_SQRT3) *    v_alpha ) / 0x8000;
	int32_t vb = ( (int32_t)((float)0x6000*-1.0/M_SQRT3) *    v_alpha ) / 0x8000 
		   + ( (int32_t)((float)0x6000*1.0) * v_beta ) / 0x8000;
	int32_t vc = ( (int32_t)((float)0x6000*-1.0/M_SQRT3) *    v_alpha ) / 0x8000 
		   + ( (int32_t)((float)0x6000*-1.0) * v_beta ) / 0x8000;

	//Third Harmonic Injection (increases maximum voltage amplitude)
	//find smallest and largest of three phase voltages
	int32_t v_min = va;
	if(v_min > vb) {
		v_min = vb;
	}
	if(v_min > vc) {
		v_min = vc;
	}
	int32_t v_max = va;
	if(v_max < vb) {
		v_max = vb;
	}
	if(v_max < vc) {
		v_max = vc;
	}
	//force largest and smallest phase voltage to be centered around Vbus/2
	//this doesn't affect the phase-phase voltages, but gives us more headroom
	int32_t v_center = 0x6000 - ((v_min+v_max) / 2); 
	*out_pwma = CONSTRAIN((v_center+va)/0x40, 0, PWM_MAX);
	*out_pwmb = CONSTRAIN((v_center+vb)/0x40, 0, PWM_MAX);
	*out_pwmc = CONSTRAIN((v_center+vc)/0x40, 0, PWM_MAX);
}

void foc_reset(void) {
	_id_integral = 0;
	_iq_integral = 0;
}

void foc_update(foc_t *foc, adc_results_t *adc_results, encoder_t *encoder) {
	//take Park transform of three previous phase current samples
	//using three differently retarded ;) electrical angles
	gpio_set(LED_PORT, LED_B);
	gpio_clear(LED_PORT, LED_B);
#define FOC_AVG_LEN 8
	static int32_t id_avg_arr[FOC_AVG_LEN];
	static int32_t iq_avg_arr[FOC_AVG_LEN];
	static int32_t id_avg=0, iq_avg=0;
	static uint16_t idq_avg_idx=0;

	id_avg -= id_avg_arr[idq_avg_idx];
	iq_avg -= iq_avg_arr[idq_avg_idx];
	int16_t id, iq;
	_park(
		encoder->cos_theta_e,
		encoder->sin_theta_e,
		adc_results->ia,
		adc_results->ib,
		adc_results->ic,
		&id,
		&iq
	);
	id_avg_arr[idq_avg_idx] = id;
	iq_avg_arr[idq_avg_idx] = iq;

	id_avg += id_avg_arr[idq_avg_idx];
	iq_avg += iq_avg_arr[idq_avg_idx];
	idq_avg_idx++;
	if(idq_avg_idx >= FOC_AVG_LEN) {
		idq_avg_idx = 0;
	}
	foc->id=id_avg/FOC_AVG_LEN;
	foc->iq=iq_avg/FOC_AVG_LEN;

	gpio_set(LED_PORT, LED_B);
	gpio_clear(LED_PORT, LED_B);

	if(foc->control_mode == FOC_CONTROL_MODE_IDQ) {
		//PI control of id and iq -> vd and vq
		_pi_control(foc->id, foc->id_ref, &_id_integral, &foc->vd);
		_pi_control(foc->iq, foc->iq_ref, &_iq_integral, &foc->vq);
	} else if(foc->control_mode == FOC_CONTROL_MODE_VDQ || 
		foc->control_mode == FOC_CONTROL_MODE_VDQ_THETA_E) {
		//pass commanded vd,q on to PWM generator after limiting range
		foc->vd = CONSTRAIN(foc->vd_ref, -VDQ_MAX, VDQ_MAX);
		foc->vq = CONSTRAIN(foc->vq_ref, -VDQ_MAX, VDQ_MAX);
	}
	gpio_set(LED_PORT, LED_B);
	gpio_clear(LED_PORT, LED_B);

	//inverse Park transform at three differently advanced electric angles,
	//composed with third harmonic injection/SVM
	//we store this in arrays to get DMA'd to the PWM timer
	if(foc->control_mode == FOC_CONTROL_MODE_VDQ_THETA_E) {
		//use commanded electrical angle to commutate (calibration only) 
		int32_t cos_theta_e = fcos(foc->theta_e_ref);
		int32_t sin_theta_e = fsin(foc->theta_e_ref);
		_inverse_park_inject_harmonic_three(
				cos_theta_e, 
				sin_theta_e, 
				foc->vd,
				foc->vq,
				&foc->pwm_a,
				&foc->pwm_b,
				&foc->pwm_c
			);
	} else if(foc->control_mode == FOC_CONTROL_MODE_STOP) {
		//force all outputs LOW
		foc->pwm_a = 0;
		foc->pwm_b = 0;
		foc->pwm_c = 0;
	}
	else {
		//use measured e-angle to commutate
		_inverse_park_inject_harmonic_three(
				encoder->cos_theta_e, 
				encoder->sin_theta_e, 
				foc->vd,
				foc->vq,
				&foc->pwm_a,
				&foc->pwm_b,
				&foc->pwm_c
			);
	}
	TIM1_CCR1 = foc->pwm_a;
	TIM1_CCR2 = foc->pwm_b;
	TIM1_CCR3 = foc->pwm_c;

	gpio_set(LED_PORT, LED_B);
	gpio_clear(LED_PORT, LED_B);
}
