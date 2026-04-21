#include "spi.h"
#include <stm32l432xx.h>

void spi_init()
{
    // 1. Enable clocks for GPIOA (SCK), GPIOB (MOSI/MISO), and SPI1
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // 2. Configure PA5 for SPI1_SCK (AF05)
    GPIOA->MODER &= ~(GPIO_MODER_MODE5);
    GPIOA->MODER |= GPIO_MODER_MODE5_1;
    GPIOA->AFR[0] |= (5 << 20); // PA5 = AF5

    // 3. Configure PB4 (MISO) and PB5 (MOSI) for SPI1 (AF05)
    GPIOB->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOB->MODER |= (GPIO_MODER_MODE4_1 | GPIO_MODER_MODE5_1);
    GPIOB->AFR[0] |= (5 << 16) | (5 << 20); // PB4=AF5, PB5=AF5

    // 4. Configure SPI1 Control Register 1
    // Master mode, Software Slave Mgmt, Baud Rate div 16
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1 | SPI_CR1_BR_0;

    // 5. Configure SPI1 Control Register 2 (CRITICAL FOR L4)
    // Set DS[3:0] to 0111 for 8-bit data
    SPI1->CR2 &= ~(SPI_CR2_DS); // Clear bits
    SPI1->CR2 |= (7 << SPI_CR2_DS_Pos); // Set to 8-bit (Value 7)
    
    // Set FRXTH=1: RXNE fires when FIFO has at least 8 bits
    SPI1->CR2 |= SPI_CR2_FRXTH;

    // 6. Enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi1_transmit(uint8_t *data, uint32_t size)
{
    for(uint32_t i = 0; i < size; i++)
    {
        while(!(SPI1->SR & SPI_SR_TXE));
        // Force 8-bit access to the data register
        *((__IO uint8_t *)&SPI1->DR) = data[i];
    }
    while(SPI1->SR & SPI_SR_BSY);
    
    // Drain RX FIFO — nRF24L01 clocks in STATUS during every TX byte;
    // these stale bytes must be discarded so spi1_receive gets the real data
    while(SPI1->SR & SPI_SR_RXNE) {
        uint8_t temp = *((__IO uint8_t *)&SPI1->DR);
        (void)temp;
    }
}

void spi1_receive(uint8_t *data, uint32_t size)
{
    while(size)
    {
        while(!(SPI1->SR & SPI_SR_TXE));
        *((__IO uint8_t *)&SPI1->DR) = 0x00; // Send dummy byte

        while(!(SPI1->SR & SPI_SR_RXNE));
        *data++ = *((__IO uint8_t *)&SPI1->DR); // Read 8-bit
        size--;
    }
}