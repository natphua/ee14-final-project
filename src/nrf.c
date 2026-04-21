#include "nrf.h"
#include "delay.h"
#include <stm32l432xx.h>

//CS = chip select, listening to SPI when selected (pulls line low)
void CS_Select (void)
{
    GPIOA->BSRR=GPIO_BSRR_BR1;
}

void CS_UnSelect (void)
{
    GPIOA->BSRR=GPIO_BSRR_BS1;
}

//CE = chip enable, will actually starting receiving/transmitting
void CE_Enable (void)
{
	GPIOA->BSRR=GPIO_BSRR_BS0;

}

void CE_Disable (void)
{
	GPIOA->BSRR=GPIO_BSRR_BR0;

}


// write a single byte to the particular register
void nrf24_WriteReg (uint8_t Reg, uint8_t Data)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	spi1_transmit(buf, 2);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}

//write multiple bytes starting from a particular register
void nrf24_WriteRegMulti (uint8_t Reg, uint8_t *data, int size)
{
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
    //	buf[1] = Data;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	spi1_transmit(buf,1);
	spi1_transmit(data, size);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


uint8_t nrf24_ReadReg (uint8_t Reg)
{
	uint8_t data=0;

	// Pull the CS Pin LOW to select the device
	CS_Select();

	spi1_transmit(&Reg, 1);
	spi1_receive(&data,1);

	// Pull the CS HIGH to release the device
	CS_UnSelect();

	return data;
}


/* Read multiple bytes from the register */
void nrf24_ReadReg_Multi (uint8_t Reg, uint8_t *data, int size)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	spi1_transmit(&Reg, 1);
	spi1_receive(data, size);
	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


// send the command to the NRF
void nrfsendCmd (uint8_t cmd)
{
	// Pull the CS Pin LOW to select the device
	CS_Select();

	spi1_transmit( &cmd, 1);

	// Pull the CS HIGH to release the device
	CS_UnSelect();
}


void ncs_cs_init()
{
    /*enable clock access ti GPI)A*/
    RCC->AHB1ENR|=RCC_AHB2ENR_GPIOAEN;
    /*Set PA0 and PA1 as output*/
    GPIOA->MODER|=GPIO_MODER_MODER0_0|GPIO_MODER_MODER1_0;
    GPIOA->MODER&=~(GPIO_MODER_MODER0_1|GPIO_MODER_MODER1_1);
}

void NRF24_Init (void)
{
	spi_init();
	ncs_cs_init();
	// disable the chip before configuring the device
	CE_Disable();


	// reset everything
	nrf24_reset (0);

	nrf24_WriteReg(CONFIG, 0);  // will be configured later

	nrf24_WriteReg(EN_AA, 0);  // No Auto ACK

	nrf24_WriteReg (EN_RXADDR, 0);  // Not Enabling any data pipe right now

	nrf24_WriteReg (SETUP_AW, 0x03);  // 5 Bytes for the TX/RX address

	nrf24_WriteReg (SETUP_RETR, 0);   // No retransmission

	nrf24_WriteReg (RF_CH, 0);  // will be setup during Tx or RX

	nrf24_WriteReg (RF_SETUP, 0x0E);   // Power= 0db, data rate = 2Mbps

	// Enable the chip after configuring the device
	CE_Enable();

}

void NRF24_TxMode (uint8_t *Address, uint8_t channel) {
    CE_Disable();
    nrf24_WriteReg(RF_CH, channel);  // select the channel
    nrf24_WriteRegMulti(TX_ADDR, Address, 5);  // Write the TX address

    uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1);   // write 1 in the PWR_UP bit
    //	config = config & (0xF2);    // write 0 in the PRIM_RX, and 1 in the PWR_UP, and all other bits are masked
	nrf24_WriteReg (CONFIG, config);

    // Enable the chip after configuring the device
	CE_Enable();
}

uint8_t NRF24_Transmit(uint8_t *data) {
    uint8_t cmdtosend = 0;
    CS_Select();

    // payload command
    cmdtosend = W_TX_PAYLOAD;
    spi1_transmit( &cmdtosend, 1);

    // send the payload
    spi1_transmit(data, 32);

    // Unselect the device
	CS_UnSelect();
    delay(1);

    uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);

	// check the fourth bit of FIFO_STATUS to know if the TX fifo is empty
	if ((fifostatus&(1<<4)) && (!(fifostatus&(1<<3))))
	{
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);

		// reset FIFO_STATUS
		nrf24_reset (FIFO_STATUS);

		return 1;
	}

	return 0;
    
}

void NRF24_RxMode(uint8_t *Address, uint8_t channel) {
    CE_Disable();
    nrf24_reset(STATUS);
    nrf24_WriteReg (RF_CH, channel);  // select the channel

    // select data pipe 2
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<2);
	nrf24_WriteReg (EN_RXADDR, en_rxaddr);

    nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);  // Write the Pipe1 address
    nrf24_WriteReg(RX_ADDR_P2, 0xEE);  // Write the Pipe2 LSB address
    nrf24_WriteReg (RX_PW_P2, 32);   // 32 bit payload size for pipe 2

    // power up the device in Rx mode
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config | (1<<1) | (1<<0);
	nrf24_WriteReg (CONFIG, config);

    // Enable the chip after configuring the device
	CE_Enable();
}

uint8_t isDataAvailable (int pipenum) {
    uint8_t status = nrf24_ReadReg(STATUS);

	if ((status&(1<<6))&&(status&(pipenum<<1)))
	{

		nrf24_WriteReg(STATUS, (1<<6));

		return 1;
	}

	return 0;
}

void NRF24_Receive (uint8_t *data) {
    uint8_t cmdtosend = 0;

	// select the device
	CS_Select();

	// payload command
	cmdtosend = R_RX_PAYLOAD;
	spi1_transmit( &cmdtosend, 1);

	// Receive the payload
	spi1_receive(data, 32);

	// Unselect the device
	CS_UnSelect();

	delay(1);

	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);
}