#include "spi.h"
#include <stm32l432xx.h>

void spi_init()
	{

	#define AF05  (0x05)
	//enable clock for GPIOA
	RCC->AHB1ENR|=RCC_AHB2ENR_GPIOAEN;

	//set PA5, PA6 and PA7 to alternate function mode
	GPIOA->MODER|=GPIO_MODER_MODE5_1|GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1;
	//set which type of alternate function is
	GPIOA->AFR[0]|=(AF05<<20)|(AF05<<24)|(AF05<<28);
	//enable clock access to SPI1
	RCC->APB2ENR|=RCC_APB2ENR_SPI1EN;
	//set software slave managment
	SPI1->CR1|=SPI_CR1_SSM|SPI_CR1_SSI;
	//set SPI in master mode
	SPI1->CR1|=SPI_CR1_MSTR;
	//SPI1->CR1|=SPI_CR1_BR_0;
	SPI1->CR1|=SPI_CR1_SPE;
	}

void spi1_transmit(uint8_t *data,uint32_t size)
{
	uint32_t i=0;
	uint8_t temp;

	while(i<size)
	{
		/*Wait until TXE is set*/
		while(!(SPI1->SR & (SPI_SR_TXE))){}

		/*Write the data to the data register*/
		SPI1->DR = data[i];
		i++;
	}
	/*Wait until TXE is set*/
	while(!(SPI1->SR & (SPI_SR_TXE))){}

	/*Wait for BUSY flag to reset*/
	while((SPI1->SR & (SPI_SR_BSY))){}

	/*Clear OVR flag*/
	temp = SPI1->DR;
	temp = SPI1->SR;
}

void spi1_receive(uint8_t *data,uint32_t size)
{
	while(size)
	{
		/*Send dummy data*/
		SPI1->DR =0;

		/*Wait for RXNE flag to be set*/
		while(!(SPI1->SR & (SPI_SR_RXNE))){}

		/*Read data from data register*/
		*data++ = (SPI1->DR);
		size--;
	}
}