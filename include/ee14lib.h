/* Header file for all EE14lib functions. */

#ifndef EE14LIB_H
#define EE14LIB_H

#include "stm32l432xx.h"
#include <stdbool.h>

// Pin names on the Nucleo silkscreen, copied from the Arduino Nano form factor
typedef enum {A0,A1,A2,A3,A4,A5,A6,A7,
	  D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13 } EE14Lib_Pin ;

typedef int EE14Lib_Err;

#define EE14Lib_Err_OK 0
#define EE14Lib_Err_INEXPLICABLE_FAILURE -1
#define EE14Lib_Err_NOT_IMPLEMENTED -2
#define EE14Lib_ERR_INVALID_CONFIG -3

// GPIO modes
#define INPUT 0b00
#define OUTPUT 0b01
// Normally shouldn't need to use ALTERNATE_FUNCTION directly; the peripheral modes will set this up
#define ALTERNATE_FUNCTION 0b10
#define ANALOG 0b11

// GPIO pullup modes
#define PULL_OFF 0b00
#define PULL_UP 0b01
#define PULL_DOWN 0b10
// Both on is an error

// defining pins associated with joystick x/y
// #define VRX (EE14Lib_Pin)D3
// #define VRY (EE14Lib_Pin)D6

// defining LED pins 
#define RED (EE14Lib_Pin)D9
#define GREEN (EE14Lib_Pin)D11
#define BLUE (EE14Lib_Pin)D10

EE14Lib_Err gpio_config_mode(EE14Lib_Pin pin, unsigned int mode);
EE14Lib_Err gpio_config_pullup(EE14Lib_Pin pin, unsigned int mode);
EE14Lib_Err gpio_config_alternate_function(EE14Lib_Pin pin, unsigned int function);
void gpio_write(EE14Lib_Pin pin, bool value);
bool gpio_read(EE14Lib_Pin pin);

EE14Lib_Err timer_config_freerun(TIM_TypeDef* const timer, const unsigned int prescalar);
uint32_t timer_get_count(TIM_TypeDef* const timer);

EE14Lib_Err timer_config_pwm(TIM_TypeDef* const timer, const unsigned int freq_hz);
EE14Lib_Err timer_config_channel_pwm(TIM_TypeDef* const timer, const EE14Lib_Pin pin, const unsigned int duty);

EE14Lib_Err adc_config_single(const EE14Lib_Pin pin);
unsigned int adc_read_single(void);

// Initialize the serial port
void host_serial_init();

// Very basic function: send a character string to the UART, one byte at a time.
// Spin wait after each byte until the UART is ready for the next byte.
void serial_write(USART_TypeDef *USARTx, const char *buffer, int len);

// Spin wait until we have a byte.
char serial_read(USART_TypeDef *USARTx);

// motor pins
#define BPHASE D2
#define BENABLE D3
#define APHASE D1 
#define AENABLE D1

// joystick pins
#define VRX D6
#define VRY D3

// calculating speeds
#define CENTER 123
#define DEADZONE 10
#define MAX_PWM 1023


#endif