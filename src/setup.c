#include "setup.h"
#include "pins.h"

#include "libopencm3/stm32/rcc.h" 
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/stm32/adc.h"
#include "libopencm3/stm32/dma.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"

void adc_configure_fast_channels(void);
uint16_t adc_slow_sensor(uint8_t);
void nvic_setup(void);
void clocks_setup(void);
void gpio_setup(void);
void adc_setup(void);
void pwm_timer_setup(void);
void spi_setup(void);

void nvic_setup() {
	//nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	//nvic_enable_irq(NVIC_ADC_COMP_IRQ);
}

void gpio_setup(void) {	
	//turn off LEDs to start 
	gpio_set(LED_PORT, LED_R | LED_B);
	//LEDs set as outputs
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_R | LED_B);

	//current meas and voltage sense set as inputs
	gpio_mode_setup(IABC_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, IABC_PIN); 
	gpio_mode_setup(TEMP_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, TEMP_PIN); 
	gpio_mode_setup(VBUS_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VBUS_PIN); 

	//3-phase complementary PWM outputs connected to TIM1
	gpio_mode_setup(PWMH_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWMH_PIN);
	gpio_mode_setup(PWML_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, PWML_PIN);
	gpio_set_af(PWMH_PORT, GPIO_AF2, PWMH_PIN);
	gpio_set_af(PWML_PORT, GPIO_AF2, PWML_PIN);

	//setup CANRX,TX alternate function pins
	gpio_mode_setup(CAN_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CAN_PIN);
	gpio_set_af(CAN_PORT, GPIO_AF4, CAN_PIN); 

	//setup SPI pins
	//might need to call set_af after mode_setup??
	//if this works it's preferable as it avoids muxing hot output pins around between periphs
	gpio_set(SPI_NCS_PORT, SPI_NCS_PIN);
	gpio_mode_setup(SPI_NCS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_NCS_PIN);
	gpio_set_af(SPI_CLK_MISO_MOSI_PORT, GPIO_AF0, SPI_CLK_MISO_MOSI_PIN);
	gpio_mode_setup(SPI_CLK_MISO_MOSI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4 | SPI_CLK_MISO_MOSI_PIN);
}

void clocks_setup() {
	//start 8MHz ext crystal and PLL up to 48MHz for system and peripheral clocks.
	rcc_clock_setup_in_hse_8mhz_out_48mhz();

	//must enable clocking to each peripheral before we can rw registers	
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_DMA);
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_CAN);
	rcc_periph_clock_enable(RCC_SPI1);
	
	//setup SysTick 1ms interrupt for timekeeping
	systick_set_frequency(1000, rcc_ahb_frequency);
	systick_clear();
	systick_interrupt_enable();
	systick_counter_enable();
}

void adc_setup() {
	// max ADC clock speed is 14MHz. We could use the 14MHz internal
	// oscillator intended to clock the ADC, but then we would have some
	// jitter in the timing of the ADC samples. Here we use the
	// 48MHz APB clock / 4 = 12 MHz ADC clock
	adc_set_clk_source(ADC, ADC_CLKSOURCE_PCLK_DIV4);
	//ADC must be powered down to run calibration. It is powered off at reset
	adc_calibrate(ADC);
	adc_power_on(ADC); // power up
	//when triggered, convert all channels in sequence once, then stop.
	adc_set_operation_mode(ADC, ADC_MODE_SCAN); 
	//analog watchdog settings (used for phase overcurrent interrupt)
	adc_enable_analog_watchdog_on_all_channels(ADC);
	adc_set_watchdog_low_threshold(ADC, 1500);
	adc_set_watchdog_high_threshold(ADC, 2500);
	//set ADC to trigger from TIM1 OCREF4
	adc_enable_external_trigger_regular(ADC, ADC_CFGR1_EXTSEL_TIM1_TRGO, ADC_CFGR1_EXTEN_RISING_EDGE);
	//setup ADC DMA
	adc_enable_dma_circular_mode(ADC);
	dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)(&ADC_DR(ADC)));
	dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
	dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)(adc_buffer));
	dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
	dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
	dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
	//dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
	//dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL1);
	dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUFFER_SIZE);
	dma_enable_channel(DMA1, DMA_CHANNEL1);

	adc_configure_fast_channels();
}

void pwm_timer_setup() {
	//3-phase PWM timer setup
	//timer upper limit sets PWM period
	timer_set_period(TIM1, PWM_TIMER_RELOAD);
	//down-up counting (CMS_CENTER_3)
	timer_set_mode(TIM1, 
			TIM_CR1_CKD_CK_INT, 
			TIM_CR1_CMS_CENTER_3, 
			TIM_CR1_DIR_UP);
	//PWM mode 1 (meaning phase A,B,C output compares are ACTIVE when counter < compare register)
	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);

	//\/ allows forcing outputs to nonzero state when MOE cleared
	//timer_set_enabled_off_state_in_idle_mode(TIM1);
	//enable all 6 outputs of timer
	timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_oc_output(TIM1, TIM_OC1N);
	timer_enable_oc_output(TIM1, TIM_OC2);
	timer_enable_oc_output(TIM1, TIM_OC2N);
	timer_enable_oc_output(TIM1, TIM_OC3);
	timer_enable_oc_output(TIM1, TIM_OC3N);
	
	//deadtime-gen clock runs at 48MHz. Here we configure 16/48MHz=320ns deadtime
	timer_set_deadtime(TIM1, (0x7F & 16));

	//set default output compare register values
	timer_set_oc_value(TIM1, TIM_OC1, 0); 
	timer_set_oc_value(TIM1, TIM_OC2, 0);
	timer_set_oc_value(TIM1, TIM_OC3, 0);
	
	//this will create a one-cycle (of the TIM1 clock) pulse LOW
	//on OCREF4 signal at the peak of the counter triangle wave
	//(center of interval when low-side FETs will be on) for adc triggering
	timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM2); // different polarity for OCREF4.
	timer_set_oc_value(TIM1, TIM_OC4, PWM_TIMER_SAMPLE_CURRENT); //no pulse generated w/o the -1
	timer_enable_oc_output(TIM1, TIM_OC4);
	//direct OC4REF signal to TRGO output of TIM1
	//we will use this to trigger the ADC
	timer_set_master_mode(TIM1, TIM_CR2_MMS_COMPARE_OC4REF);

	//set the timer running. However, outputs are not enabled yet because MOE bit is cleared 
	//call timer_enable_break_main_output() to set it
	timer_enable_counter(TIM1);

}

uint16_t adc_slow_sensor(uint8_t channel) {
	//CONFIGURE TO READ SLOW SENSOR
	//make sure we are out of the current sense conversion time
	if(timer_get_counter(TIM1) > PWM_MAX) {
		return 0xFFFF;
	}
	volatile uint32_t t1 = systick_get_value();
	//stop any ongoing conversions (there should be none) using ADSTP bit
	ADC_CR(ADC) |= ADC_CR_ADSTP;
	//hardware clears ADSTART (and ADSTP) when stop complete 
	while(ADC_CR(ADC) & ADC_CR_ADSTART);
	//use a slower sampling time because these sensors have ~10k output impedance
	//minimum is 10.2 12MHz cycles, but we can go higher because we have
	//plenty of time before we need to read the current sensors again (20kHz pwm)
	//(28.5 sample+12.5 convert cycles) at 12MHz -> 3.5us
	adc_set_sample_time_on_all_channels(ADC, ADC_SMPR_SMP_028DOT5);
	//disable analog watchdog interrupt
	adc_disable_watchdog_interrupt(ADC);
	//select conversion channel
	adc_set_regular_sequence(ADC, 1, &channel);
	//disable DMA
	adc_disable_dma(ADC);

	//READ SLOW SENSOR
	//clear sequence complete interrupt flag
	ADC_ISR(ADC) |= ADC_ISR_EOC;
	//ADSTART set, will trigger at next timer peak
	ADC_CR(ADC) |= ADC_CR_ADSTART;
	//loop until conversion completes
	//good place to yield control :)
	volatile uint32_t t2 = systick_get_value();
	while(!adc_eoc(ADC));
	uint16_t result = ADC_DR(ADC); 
	volatile uint32_t t3=systick_get_value();

	//RESTORE CONFIGURATION FOR CURRENT SENSORS
	adc_configure_fast_channels();
	volatile uint32_t t4=systick_get_value();

	if(t4+t3+t2+t1) {
		__asm__("nop");
	}
	
	return result;
}

void adc_configure_fast_channels() {
	//stop any ongoing conversions (there should be none) using ADSTP bit
	ADC_CR(ADC) |= ADC_CR_ADSTP;
	//hardware clears ADSTART (and ADSTP) when stop complete 
	while(ADC_CR(ADC) & ADC_CR_ADSTART);
	//current sense amp outputs go into 100R shunt 10nF filter
	//before entering ADC. We can calculate LSB/4 sampling time as
	//Tsample,min = (Rin(100)+Rsw(1k))*(Cadc(8pF))*ln(2**(12bit+2))
	// = 0.085us, 1.02 cycles at 12MHz adc clock
	// use the lowest, 1.5-cycle sampling time
	// The sampling time required for the 10k NTC thermistors is an order of
	// magnitude higher, but we can just average that
	adc_set_sample_time_on_all_channels(ADC, ADC_SMPR_SMP_001DOT5);
	//enable analog watchdog
	adc_clear_watchdog_flag(ADC); //clear first. vbus,temp channels may have set AWD flag 
	adc_enable_watchdog_interrupt(ADC);
	//select current sensors + vbus as conversion channels
	adc_set_regular_sequence(ADC, sizeof(IABC_CHANNEL), IABC_CHANNEL);  
	//enable DMA
	adc_enable_dma(ADC);
	//re-enable ADC triggering
	ADC_CR(ADC) |= ADC_CR_ADSTART;
}

void spi_setup() {
	spi_set_master_mode(SPI1);
	//48MHz div 8 = 6MHz < 10MHz max for the encoder chip
	spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_8);
	//SPI mode=1 (CPOL=0, CPHA=1)
	spi_set_standard_mode(SPI1, 1);
	spi_set_data_size(SPI1, SPI_CR2_DS_16BIT); 
	spi_enable_ss_output(SPI1);
	spi_enable(SPI1);
}

void setup(void) {
	nvic_setup(); //call first in case anything else uses interrupts or exceptions
	clocks_setup(); //call before all periph reg accesses
	spi_setup();
	adc_setup();
	pwm_timer_setup();
	gpio_setup(); //call last so that peripheral outputs are steady when linked to GPIO pins
}
