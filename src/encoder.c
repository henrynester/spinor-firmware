#include "pins.h"

#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/spi.h"

#define ENCODER_ERR_OK 0
#define ENCODER_ERR_SPIMASTER 1 //must =1 since parity errors return 1
#define ENCODER_ERR_SPISLAVE 2
#define ENCODER_ERR_DSP 3
#define ENCODER_ERR_MAGL 4
#define ENCODER_ERR_MAGH 5

#define ENCODER_AGC_MIN 0x20
#define ENCODER_AGC_MAX 0xD0

//AS5047 command is 16 bits:
//15: even parity
//14: r=1, w=0
//13:0: register address to r/w
#define ENCODER_CMD_PARITY (1<<15)
#define ENCODER_CMD_R (1<<14)
#define ENCODER_CMD W 0
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

uint8_t _encoder_transfer(uint16_t tx_data, uint16_t *rx_data);

uint8_t _encoder_transfer(uint16_t tx_data, uint16_t *rx_data) {
	//bring NCS low
	gpio_clear(SPI_NCS_PORT, SPI_NCS_PIN);
	//place 16 bits into SPI's TXFIFO. This will clock out.
	SPI1_DR = tx_data;
	// while loop should never hang because we're just waiting
	// for SPI peripheral to clock 16x
	while(!(SPI1_SR & SPI_SR_RXNE));
	//bring NCS high again. For the AS5047D, this has to happen after
	//each 16-bit transfer in order to actually execute the requested
	//register op
	gpio_set(GPIOA, GPIO15);
	//SPI's RXFIFO should contain 16 bits from the AS5047D now.
	*rx_data = SPI_DR(SPI1);

	uint16_t rx_data_copy = *rx_data;

	if(!(rx_data_copy & ENCODER_RESPONSE_EF)) {
		return PARITY16(rx_data_copy);
	} else {
		return ENCODER_ERR_SPISLAVE;
	}
}

uint8_t encoder_read_angle(uint16_t *angle) {
      uint8_t err = _encoder_transfer(ENCODER_CMD_R_ANGLECOM, angle);
      *angle &= ENCODER_ANGLECOM_MASK;
      return err;
}

uint8_t encoder_read_status() {
	uint16_t regval;
	uint8_t err = _encoder_transfer(ENCODER_CMD_R_ERRFL, &regval);
	if(err) return ENCODER_ERR_SPIMASTER;
	if(regval & ENCODER_ERRFL_MASK) return ENCODER_ERR_SPISLAVE;
	err = encoder_transfer(ENCODER_CMD_R_DIAAGC, &regval);
	if(err) return ENCODER_ERR_SPIMASTER;
	if((regval & ENCODER_DIAAGC_COF) || (~regval & ENCODER_DIAAGC_LF)) {
	       return ENCODER_ERR_DSP;
	}
	regval &= ENCODER_DIAAGC_AGC;
	if(regval < ENCODER_AGC_MIN) {
		return ENCODER_ERR_MAGL;
	}
	if(regval > ENCODER_AGC_MAX) {
		return ENCODER_ERR_MAGH;
	}
	return ENCODER_ERR_OK;
}
