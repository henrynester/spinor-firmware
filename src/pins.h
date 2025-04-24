#pragma once

#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/adc.h"

#define SPI_NCS_PORT GPIOA
#define SPI_NCS_PIN GPIO15
#define SPI_CLK_MISO_MOSI_PORT GPIOB
#define SPI_CLK_MISO_MOSI_PIN GPIO3 | GPIO4 | GPIO5

#define LED_PORT GPIOA
#define LED_R GPIO4
#define LED_B GPIO3

#define IABC_PORT GPIOA
#define IABC_PIN GPIO0 | GPIO1 | GPIO2
#define IABC_CHANNEL (uint8_t[]){0, 1, 2}

#define TEMP_PORT GPIOA
#define TEMP_PIN GPIO6 | GPIO7
#define TEMP_MOTOR_CHANNEL 6
#define TEMP_FET_CHANNEL 7

#define VBUS_PORT GPIOB
#define VBUS_PIN GPIO0
#define VBUS_CHANNEL 8

#define CAN_PORT GPIOB
#define CAN_PIN GPIO8 | GPIO9

#define PWML_PORT GPIOB
#define PWML_PIN GPIO13 | GPIO14 | GPIO15
#define PWMH_PORT GPIOA
#define PWMH_PIN GPIO8 | GPIO9 | GPIO10
