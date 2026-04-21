#include <stm32l432xx.h>
#include "ee14lib.h"
#include "spi.h"
#include "nrf.h"
#include "delay.h"
#include <stdio.h>

int _write(int file, char *data, int len) {
    serial_write(USART2, data, len);
    return len;
}

void move_wheels(int x, int y) {
    // 1. normalize inputs 
    int drive = CENTER - y;  // positive = Forward
    int steer = x - CENTER;  // positive = Right

    // 2. apply Deadzone
    if (drive < DEADZONE && drive > -DEADZONE) drive = 0;
    if (steer < DEADZONE && steer > -DEADZONE) steer = 0;

    // 3. differential steering logic
    int left_val  = drive;
    int right_val = drive - steer;
    
    // 4. scale to pwm range
    int left_pwm  = (left_val * MAX_PWM) / 127;
    int right_pwm = (right_val * MAX_PWM) / 127;

    // 5. constrain values to max PWM limit
    if (left_pwm > 1023)   left_pwm = 1023;
    if (left_pwm < -1023)  left_pwm = -1023;
    if (right_pwm > 1023)  right_pwm = 1023;
    if (right_pwm < -1023) right_pwm = -1023;

    // 6. Output to wheels
    // Motor B (Left)
    if (left_pwm > 0) { // go forwards
        gpio_write(BPHASE, 1);
        timer_config_channel_pwm(TIM1, BENABLE, (unsigned int)left_pwm);    

    } else if (left_pwm == 0) {
        // set to 1 bc 0 signifies some speed in hardware
        timer_config_channel_pwm(TIM1, BENABLE, 1);
        
    } else { // go backwards
        gpio_write(BPHASE, 0);
        timer_config_channel_pwm(TIM1, BENABLE, (unsigned int)(-left_pwm));
    }

    // Motor A (Right) 
    if (right_pwm > 0) {
        gpio_write(APHASE, 1);
        timer_config_channel_pwm(TIM1, AENABLE, (unsigned int)right_pwm);
    } else if (right_pwm == 0) {
        timer_config_channel_pwm(TIM1, AENABLE, 1);
    } else {
        gpio_write(APHASE, 0);
        timer_config_channel_pwm(TIM1, AENABLE, (unsigned int)(-right_pwm));
    }
}

uint8_t TxAddress[] = {0xEE,0xDD,0xCC,0xBB,0xAA};
uint8_t TxData[2];

// transmitter code 
int main() {
    // initial configurations
    host_serial_init();
    delay_init(1600000);
	NRF24_Init();
	NRF24_TxMode(TxAddress, 10);
    
    while (1) {
        adc_config_single(VRX);
        uint8_t raw_x = adc_read_single();

        adc_config_single(VRY);
        uint8_t raw_y = adc_read_single();

        TxData[0] = raw_x;
        TxData[1] = raw_y;

        if (NRF24_Transmit(TxData) == 1) {
            printf("Transmitted data (%d, %d)\n", raw_x, raw_y);
        }
        delay(1000);

    }
    return 0;
}
