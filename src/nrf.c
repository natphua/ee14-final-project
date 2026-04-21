#include "nrf.h"
#include "delay.h"
#include <stm32l432xx.h>

//CS = chip select, listening to SPI when selected (pulls line low)
void CS_Select (void) {
    GPIOA->BSRR = GPIO_BSRR_BR6; // Pull PA6 LOW
}

void CS_UnSelect (void) {
    GPIOA->BSRR = GPIO_BSRR_BS6; // Pull PA6 HIGH
}

// CE is PA7
void CE_Enable (void) {
    GPIOA->BSRR = GPIO_BSRR_BS7; // Pull PA7 HIGH
}

void CE_Disable (void) {
    GPIOA->BSRR = GPIO_BSRR_BR7; // Pull PA7 LOW
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
    // Enable clock for GPIOA (for CE/CSN) and GPIOB (for SPI)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Set PA6 and PA7 as Output (01 in MODER)
    GPIOA->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER |= (GPIO_MODER_MODE6_0 | GPIO_MODER_MODE7_0);

    // Initial State
    CS_UnSelect(); // PA6 HIGH
    CE_Disable();  // PA7 LOW
}

void nrf24_reset(uint8_t REG) 
{
    if (REG == STATUS) 
    {
        nrf24_WriteReg(STATUS, 0x70);
    }
    else if (REG == FIFO_STATUS)
    {
        nrfsendCmd(FLUSH_TX);
        nrfsendCmd(FLUSH_RX);
    }
    else if (REG == 0)
    {
        /* Complete Software Reset - Restore Power-On Reset (POR) Defaults */
        nrf24_WriteReg(CONFIG,      0x08); // 1-byte CRC, Power Down
        nrf24_WriteReg(EN_AA,       0x3F); // Auto-ACK on all pipes
        nrf24_WriteReg(EN_RXADDR,   0x03); // Enable Pipe 0 and 1
        nrf24_WriteReg(SETUP_AW,    0x03); // 5-byte address width
        nrf24_WriteReg(SETUP_RETR,  0x03); // 250us delay, 3 retries
        nrf24_WriteReg(RF_CH,       0x02); // Channel 2
        nrf24_WriteReg(RF_SETUP,    0x0E); // 2Mbps, 0dBm
        nrf24_WriteReg(STATUS,      0x70); // Clear all interrupts
        
        // Clear FIFOs
        nrfsendCmd(FLUSH_TX);
        nrfsendCmd(FLUSH_RX);
    }
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
    nrf24_WriteReg(RF_CH, channel);  
    nrf24_WriteRegMulti(TX_ADDR, Address, 5);  
    
    nrf24_WriteRegMulti(RX_ADDR_P0, Address, 5); 

    // 4. Configure the CONFIG register
    // We want: PWR_UP = 1, PRIM_RX = 0 (for TX mode), and EN_CRC = 1
    // A value of 0x0A (0000 1010) or 0x0E (0000 1110) is standard.
    nrf24_WriteReg(CONFIG, 0x0A); 

    // Enable the chip after configuring
    CE_Enable();
}

uint8_t NRF24_Transmit(uint8_t *data) {
    // 1. Load the payload
    CS_Select();
    uint8_t cmd = W_TX_PAYLOAD;
    spi1_transmit(&cmd, 1);
    spi1_transmit(data, 32);
    CS_UnSelect();

    // 2. Pulse CE to start the radio transmission
    CE_Enable();
    delay(1); // Wait for radio to blast
    CE_Disable();

    // 3. Wait for success or failure
    uint8_t status;
    uint32_t timeout = 10000; 
    while(timeout--) {
        status = nrf24_ReadReg(STATUS);
        if (status & (1 << 5)) { // TX_DS: Success
            nrf24_WriteReg(STATUS, (1 << 5)); // Clear flag
            return 1;
        }
        if (status & (1 << 4)) { // MAX_RT: Failed
            nrf24_WriteReg(STATUS, (1 << 4)); // Clear flag
            nrfsendCmd(FLUSH_TX);
            return 0;
        }
    }
    return 0; // Timeout
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

    // Wait >=1.5ms for nRF24L01 to transition from power-down to standby-I
    delay(2);

    // Enable the chip after configuring the device
	CE_Enable();
}

uint8_t isDataAvailable (int pipenum) {
    uint8_t status = nrf24_ReadReg(STATUS);

	if ((status & (1<<6)) && (((status >> 1) & 0x07) == (uint8_t)pipenum))
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