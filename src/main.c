#include <stm32l432xx.h>
#include "ee14lib.h"

// motor pins
#define BPHASE D11
#define BENABLE D10
#define APHASE D1 
#define AENABLE D1

// joystick pins
#define VRX D6
#define VRY D3

// calculating speeds
#define CENTER 123
#define DEADZONE 10
#define MAX_PWM 1023


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

// transmitter code 
int main() {
    // initial configurations
    host_serial_init();
    timer_config_pwm(TIM1, 50);
    SPI1_Init();
    EXTI4_Init();
    nRF24_TX_Init();

    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};

    while (1) {
        nRF24_SendPacket(payload, 4);

        uint8_t status = nRF24_ReadRegister(0x07); 
        printf("TX Status: %d\n", status);

        if (status & (1 << 4)) { // If MAX_RT (Max retransmits reached)
            printf("Failed to get ACK!\n");
            // Flush TX FIFO to clear the clog
            GPIOA->BSRR = GPIO_BSRR_BR6; 
            SPI1_Transfer(0xE1); // Flush TX command
            GPIOA->BSRR = GPIO_BSRR_BS6;
        }

        nRF24_WriteRegister(0x07, 0x70); // Clear all flags

        for (volatile int i = 0; i < 100000; i++) {}
    }
    return 0;
}
