#include "encoder.h"
#include "setup.h"
#include "foc.h"
#include "config.h"
#include "constants.h"
#include "pins.h"
#include "isr.h"

#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/spi.h"


#define ENCODER_AGC_MIN 0x10
#define ENCODER_AGC_MAX 0xF0

//AS5047 command is 16 bits:
//15: even parity
//14: r=1, w=0
//13:0: register address to r/w
#define ENCODER_CMD_PARITY (1<<15)
#define ENCODER_CMD_R (1<<14)
#define ENCODER_CMD W 0
#define ENCODER_CMD_R_NOP ENCODER_CMD_PARITY | ENCODER_CMD_R | 0x0000
#define ENCODER_CMD_R_ERRFL ENCODER_CMD_PARITY | ENCODER_CMD_R | 0x0000
#define ENCODER_CMD_R_DIAAGC ENCODER_CMD_PARITY | ENCODER_CMD_R | 0x3FFC
#define ENCODER_CMD_R_ANGLECOM ENCODER_CMD_PARITY | ENCODER_CMD_R | 0x3FFF
#define ENCODER_RESPONSE_EF (1<<14)

#define ENCODER_ERRFL_MASK 0x003
#define ENCODER_ERRFL_PARERR (1<<2) //parity error
#define ENCODER_ERRFL_INVCOMM (1<<1) //invalid command
#define ENCODER_ERRFL_FRERR (1<<0) //SPI frame err
#define ENCODER_DIAAGC_MASK 0x0FFF
#define ENCODER_DIAAGC_MAGL (1<<11) //B too weak
#define ENCODER_DIAAGC_MAGH (1<<10) // B too strong
#define ENCODER_DIAAGC_COF (1<<9) // cordic overflow
#define ENCODER_DIAAGC_LF (1<<8) //offset loop cal. finished
#define ENCODER_DIAAGC_FLAGS_MASK 0xF00
#define ENCODER_DIAAGC_AGC 0xFF //automatic gain control (B too strong=0, too weak=0xFF)
#define ENCODER_ANGLECOM_MASK 0x3FFF //low 14 bits contain angle

#define PARITY16(x) x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1; x & 1

int32_t _theta_m_next;
int32_t _omega_m_next;
uint16_t _theta_m_sensor;
uint16_t _theta_m_modular;
uint16_t _status;
uint16_t _num_spi_errors;

uint8_t _is_odd_parity(uint16_t x);

int16_t _delta_u14(uint16_t initial, uint16_t final) {
	int16_t delta = (int16_t)final - (int16_t)initial;
	if(delta > 0x2000) {
		//rollover while decreasing: 0x3FFF-0x0000=0x3FFF, or nearer 0
		return delta - 0x4000;
	}
	if(delta < -0x2000) {
		//rollover while increasing: 0-0x3FFF=-0x3FFF, or nearer 0
		return delta + 0x4000;
	}
	return delta;
}

uint8_t _is_odd_parity(uint16_t x) {
	x ^= x>>8;
	x ^= x>>4;
	x ^= x>>2;
	x ^= x>>1;
	return x & 1;
}

//transmit 16 bits while receiving 16 bits simultaneously.
//nCS pulses low for duration of transaction
static uint8_t _encoder_transfer(uint16_t tx_data, uint16_t *rx_data) {
	//bring NCS low
	gpio_clear(SPI_NCS_PORT, SPI_NCS_PIN);
	//datasheet specifies 350ns of nCS=0 prior to clocking data in
	DELAY_US(2);
	//place 16 bits into SPI's TXFIFO. This will clock out.
	SPI1_DR = tx_data;
	// while loop should never hang because we're just waiting
	// for SPI peripheral to clock 16x
	while(!(SPI1_SR & SPI_SR_RXNE));
	//bring NCS high again. For the AS5047D, this has to happen after
	//each 16-bit transfer in order to actually execute the requested
	//register op
	gpio_set(GPIOA, GPIO15);
	DELAY_US(2); //datasheet specifies 350ns to latch register data into SPI shift register
	//SPI's RXFIFO should contain 16 bits from the AS5047D now.
	*rx_data = SPI_DR(SPI1);

	if(*rx_data & ENCODER_RESPONSE_EF) {
		_status = ENCODER_ERROR_ERROR_FLAG_AT_MASTER;
		_num_spi_errors++;
		return false;
	}
	if(_is_odd_parity(*rx_data)) {
		_status = ENCODER_ERROR_BAD_PARITY_AT_MASTER;
		_num_spi_errors++;
		return false;
	}
	return true;
}

//AS5047D register read requires two transactions 
//in the format (even parity) (r=1, w=0) (14 bit register address)
// #1: tx address of register to read. rx is discarded.
// pulse nCS high to latch register contents into SPI slave circuit
// #2: any address, I usually use NOP. rx contains register contents 

void encoder_read_angle(void) {
	uint16_t _;
	_encoder_transfer(ENCODER_CMD_R_ANGLECOM, &_);
	_encoder_transfer(ENCODER_CMD_R_NOP, &_theta_m_sensor);
	//keep lower 14 bits containing angle value
	_theta_m_sensor &= ENCODER_ANGLECOM_MASK;
}

void encoder_read_status(encoder_t *encoder) {
	//check onboard status regs for errors
	uint16_t errfl_contents, diaagc_contents, _;
	_encoder_transfer(ENCODER_CMD_R_ERRFL, &_);
	_encoder_transfer(ENCODER_CMD_R_DIAAGC, &errfl_contents);
	_encoder_transfer(ENCODER_CMD_R_NOP, &diaagc_contents);
	uint8_t agc = diaagc_contents & ENCODER_DIAAGC_AGC; 
	encoder->agc = agc;
	if(errfl_contents & ENCODER_ERRFL_FRERR) {
		_status = ENCODER_ERROR_BAD_FRAMING_AT_SLAVE;
		_num_spi_errors++;
	} else if(errfl_contents & ENCODER_ERRFL_PARERR) {
		_status = ENCODER_ERROR_BAD_PARITY_AT_SLAVE;
		_num_spi_errors++;
	} else if(errfl_contents & ENCODER_ERROR_INVALID_COMMAND_AT_SLAVE) {
		_status = ENCODER_ERROR_BAD_PARITY_AT_SLAVE;
		_num_spi_errors++;
	} else if((diaagc_contents & ENCODER_DIAAGC_COF) 
		|| (~diaagc_contents & ENCODER_DIAAGC_LF)) {
		_status = ENCODER_ERROR_ONBOARD_DSP;
		_num_spi_errors++;
	} else if(agc < ENCODER_AGC_MIN) {
		_status = ENCODER_ERROR_BFIELD_OVER;
		_num_spi_errors++;
	} else if(agc > ENCODER_AGC_MAX) {
		_status = ENCODER_ERROR_BFIELD_UNDER;
		_num_spi_errors++;
	}
	//if error counter has reached a threshold,
	//report errors and reset error counter
	if(_status!=ENCODER_ERROR_OK && _num_spi_errors > 256) {
		encoder->status=_status;
		encoder->num_spi_errors = _num_spi_errors;
		_status = ENCODER_ERROR_OK;
		_num_spi_errors = 0;
	}
}

void encoder_clear_error(encoder_t *encoder) {
	encoder->status = ENCODER_ERROR_OK;
	encoder->num_spi_errors = 0;
}

void encoder_pll_load_next(encoder_t *encoder) {
	encoder->theta_m = _theta_m_next;
	encoder->theta_m_homed = _theta_m_next - encoder->theta_m_homing_offset;
	encoder->omega_m = _omega_m_next;
}

void encoder_pll_compute_next(encoder_t *encoder) {
	encoder->theta_m_sensor = _theta_m_sensor;
	//PLL tracks encoder sensor readings. Call after reading SPI encoder
	int32_t pll_error = _delta_u14((encoder->theta_m&0x3FFF), _theta_m_sensor);
	_theta_m_next = encoder->theta_m + encoder->omega_m / 0x100 +
	       FMUL(PLL_KP*CONTROL_DT, pll_error);	
	//integration is done at factor 0x100 larger so that small integrand 
	//does not vanish
	_omega_m_next = encoder->omega_m + 
		FMUL(PLL_KI*THETA_M_LSB*CONTROL_DT/OMEGA_M_LSB, pll_error);
	
	//calculate cos, sin of theta_e at -3, -2, -1, 0, and +2 PWM periods
	//away from the (next) control loop time. sin, cos are somewhat costly even
	//using the lookup table, so we are cacheing the results for the next FOC iteration
	//after the current FOC iteration completes
	//This doesn't introduce any lag because the PLL position estimate is
	//always for the upcoming control loop anyway
	uint16_t theta_e = ((_theta_m_next & 0x3FFF) * NUM_POLE_PAIRS) & 0x3FFF;
	theta_e -= encoder->theta_e_offset;
	theta_e &= 0x3FFF;
	encoder->theta_e = theta_e; 
	encoder->cos_theta_e = fcos(theta_e); 
	encoder->sin_theta_e = fsin(theta_e);
}

void encoder_init(encoder_t *self) {
	encoder_clear_error(self);
}
