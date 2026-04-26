#include <stm32l432xx.h>
#include <stdio.h>
#include "ee14lib.h"
#include "spi.h"
#include "nrf.h"
#include "delay.h"

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
    // left = motor B, right = motor A
    int left_val  = drive + steer;
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

uint8_t RxAddress[] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32];

// receiver code 
int main() {
    // initial configurations
    host_serial_init();
    delay_init(16000000);
	NRF24_Init();
	NRF24_RxMode(RxAddress, 10);

    gpio_config_mode(BPHASE, OUTPUT);
    gpio_config_mode(BENABLE, OUTPUT);
    gpio_config_mode(APHASE, OUTPUT);
    gpio_config_mode(AENABLE, OUTPUT);

    timer_config_pwm(TIM1, 50);


    while(1) {
		if (isDataAvailable(2) == 1) {
            NRF24_Receive(RxData);
            printf("Received data: %d %d\n", RxData[0], RxData[1]);
            move_wheels(RxData[0], RxData[1]);
		}
	}

    return 0;
}





// for motors
// int main() {
//     // initial configurations
//     gpio_config_mode(BPHASE, OUTPUT);
//     gpio_config_mode(BENABLE, OUTPUT);
//     host_serial_init();
//     timer_config_pwm(TIM1, 50);

//     while (1) {
//         /* Read XY position of joystick. */
//         /* Ranges from 0 to 1023. */
//         adc_config_single(VRX);
//         uint8_t raw_x = adc_read_single();

//         adc_config_single(VRY);
//         uint8_t raw_y = adc_read_single();

//         /* Move wheels based on joystick position. */
//         move_wheels(raw_x, raw_y);

//         for (volatile int i = 0; i < 1000; i++) {}
        
//     }
//     return 0;
// }