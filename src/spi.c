#include "stm32l432xx.h"
#include "ee14lib.h"

// Pinout:
// PA5 = SCK  (A4, AF5, SPI1)
// PB4 = MISO (D12, AF5, SPI1)
// PB5 = MOSI (D11, AF5, SPI1)
// PA6 = CSN  (A5, GPIO output)
// PA7 = CE   (A6, GPIO output)
// PA4 = IRQ  (A3, EXTI4 falling edge)

// ============================================================
// FORWARD DECLARATIONS
// ============================================================

uint8_t SPI1_Transfer(uint8_t data);
void    nRF24_WriteRegister(uint8_t reg, uint8_t value);
uint8_t nRF24_ReadRegister(uint8_t reg);
void    nRF24_ReadPayload(uint8_t *buf, uint8_t len);
void    nRF24_SendPacket(uint8_t *data, uint8_t len);

// ============================================================
// SPI + GPIO INIT
// ============================================================

void SPI1_Init(void) {
    // 1. Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;            // SPI1 peripheral clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;           // GPIOA (SCK, CSN, CE, IRQ)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;           // GPIOB (MISO, MOSI)

    // 2. PA5 → Alternate Function mode (SCK)
    GPIOA->MODER &= ~GPIO_MODER_MODE5;
    GPIOA->MODER |=  (0b10 << GPIO_MODER_MODE5_Pos);
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5;
    GPIOA->AFR[0] |=  (5 << GPIO_AFRL_AFSEL5_Pos);         // AF5 = SPI1
    GPIOA->OSPEEDR |= (0b11 << GPIO_OSPEEDR_OSPEED5_Pos);  // high speed

    // 3. PB4, PB5 → Alternate Function mode (MISO, MOSI)
    GPIOB->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOB->MODER |=  (0b10 << GPIO_MODER_MODE4_Pos) |
                     (0b10 << GPIO_MODER_MODE5_Pos);
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
    GPIOB->AFR[0] |=  (5 << GPIO_AFRL_AFSEL4_Pos) |        // AF5 = SPI1
                      (5 << GPIO_AFRL_AFSEL5_Pos);
    GPIOB->OSPEEDR |= (0b11 << GPIO_OSPEEDR_OSPEED5_Pos);  // high speed on MOSI

    // 4. PA6 = CSN, PA7 = CE → GPIO outputs
    GPIOA->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOA->MODER |=  (0b01 << GPIO_MODER_MODE6_Pos) |      // CSN = output
                     (0b01 << GPIO_MODER_MODE7_Pos);        // CE  = output

    // CSN high (deselect nRF24), CE low (idle)
    GPIOA->BSRR = GPIO_BSRR_BS6;
    GPIOA->BSRR = GPIO_BSRR_BR7;

    // 5. PA4 = IRQ → input, no pull (nRF24 drives it low)
    GPIOA->MODER &= ~GPIO_MODER_MODE4;                      // 0b00 = input
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD4;                      // no pull

    // 6. Configure SPI1
    SPI1->CR1  =  0;
    SPI1->CR1 |=  SPI_CR1_MSTR;                            // master mode
    SPI1->CR1 |=  SPI_CR1_SSM | SPI_CR1_SSI;              // software NSS management
    SPI1->CR1 |=  (0b011 << SPI_CR1_BR_Pos);              // baud = PCLK/16
    // CPOL=0, CPHA=0 = SPI Mode 0 (reset default, matches nRF24)

    SPI1->CR2  =  0;
    SPI1->CR2 |=  (0b0111 << SPI_CR2_DS_Pos);             // 8-bit data size
    SPI1->CR2 |=  SPI_CR2_FRXTH;                          // RXNE fires at 8-bit threshold

    SPI1->CR1 |=  SPI_CR1_SPE;                            // enable SPI
}

// ============================================================
// EXTI (IRQ PIN) INIT
// ============================================================

void EXTI4_Init(void) {
    // 1. Enable SYSCFG clock (needed to route EXTI lines)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // 2. Route EXTI4 to PA4
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |=  SYSCFG_EXTICR2_EXTI4_PA;

    // 3. Trigger on falling edge (IRQ pin is active low)
    EXTI->RTSR1 &= ~EXTI_RTSR1_RT4;                       // disable rising edge
    EXTI->FTSR1 |=  EXTI_FTSR1_FT4;                       // enable falling edge

    // 4. Unmask EXTI4 so it can fire
    EXTI->IMR1 |= EXTI_IMR1_IM4;

    // 5. Enable in NVIC
    NVIC_SetPriority(EXTI4_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_IRQn);
}

// ============================================================
// IRQ HANDLER
// ============================================================

void EXTI4_IRQHandler(void) {
    if (EXTI->PR1 & EXTI_PR1_PIF4) {
        EXTI->PR1 = EXTI_PR1_PIF4;                        // clear flag by writing 1

        uint8_t status = nRF24_ReadRegister(0x07);

        if (status & (1 << 6)) {                           // RX_DR: payload received
            nRF24_WriteRegister(0x07, (1 << 6));           // clear RX_DR
        }
        if (status & (1 << 5)) {                           // TX_DS: packet sent + ACK received
            nRF24_WriteRegister(0x07, (1 << 5));           // clear TX_DS
        }
        if (status & (1 << 4)) {                           // MAX_RT: all retries failed
            nRF24_WriteRegister(0x07, (1 << 4));           // clear MAX_RT flag

            // flush TX FIFO — without this the stuck packet blocks all future sends
            GPIOA->BSRR = GPIO_BSRR_BR6;                  // CSN low
            SPI1_Transfer(0xE1);                           // FLUSH_TX command
            GPIOA->BSRR = GPIO_BSRR_BS6;                  // CSN high
        }
    }
}

// ============================================================
// SPI PRIMITIVES
// ============================================================

uint8_t SPI1_Transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    *(__IO uint8_t *)&SPI1->DR = data;                    // 8-bit write (prevents 16-bit frame)
    while (!(SPI1->SR & SPI_SR_RXNE));
    return *(__IO uint8_t *)&SPI1->DR;                    // 8-bit read
}

void nRF24_WriteRegister(uint8_t reg, uint8_t value) {
    GPIOA->BSRR = GPIO_BSRR_BR6;                         // CSN low (select)
    SPI1_Transfer(0x20 | reg);                            // W_REGISTER command (001AAAAA)
    SPI1_Transfer(value);
    GPIOA->BSRR = GPIO_BSRR_BS6;                         // CSN high (deselect)
}

uint8_t nRF24_ReadRegister(uint8_t reg) {
    uint8_t result;
    GPIOA->BSRR = GPIO_BSRR_BR6;                         // CSN low (select)
    SPI1_Transfer(0x00 | reg);                            // R_REGISTER command (000AAAAA)
    result = SPI1_Transfer(0xFF);                         // clock out the data
    GPIOA->BSRR = GPIO_BSRR_BS6;                         // CSN high (deselect)
    return result;
}

// ============================================================
// nRF24 TX
// ============================================================

void nRF24_TX_Init(void) {
    // 1. Power down before configuring
    nRF24_WriteRegister(0x00, 0x00);
    for (volatile int i = 0; i < 10000; i++);

    // 2. Flush both FIFOs and clear all status flags before configuring
    GPIOA->BSRR = GPIO_BSRR_BR6;
    SPI1_Transfer(0xE1);                                  // FLUSH_TX
    GPIOA->BSRR = GPIO_BSRR_BS6;

    GPIOA->BSRR = GPIO_BSRR_BR6;
    SPI1_Transfer(0xE2);                                  // FLUSH_RX
    GPIOA->BSRR = GPIO_BSRR_BS6;

    nRF24_WriteRegister(0x07, 0x70);                      // clear RX_DR, TX_DS, MAX_RT

    // 3. Set TX address — must match RX pipe 0 address
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    GPIOA->BSRR = GPIO_BSRR_BR6;
    SPI1_Transfer(0x20 | 0x10);                           // W_REGISTER TX_ADDR (0x10)
    for (int i = 0; i < 5; i++) SPI1_Transfer(addr[i]);
    GPIOA->BSRR = GPIO_BSRR_BS6;

    // 4. Set RX pipe 0 address = TX address (required to receive ACK packets)
    GPIOA->BSRR = GPIO_BSRR_BR6;
    SPI1_Transfer(0x20 | 0x0A);                           // W_REGISTER RX_ADDR_P0 (0x0A)
    for (int i = 0; i < 5; i++) SPI1_Transfer(addr[i]);
    GPIOA->BSRR = GPIO_BSRR_BS6;

    nRF24_WriteRegister(0x01, 0x01);                      // EN_AA: auto-ACK on pipe 0
    nRF24_WriteRegister(0x02, 0x01);                      // EN_RXADDR: enable pipe 0
    nRF24_WriteRegister(0x03, 0x03);                      // SETUP_AW: 5 byte address width
    nRF24_WriteRegister(0x04, 0x1A);                      // SETUP_RETR: 500us delay, 10 retries
    nRF24_WriteRegister(0x05, 0x4C);                      // RF_CH: channel 76 (2476 MHz)
    nRF24_WriteRegister(0x06, 0x0E);                      // RF_SETUP: 2Mbps, 0dBm, full LNA gain
    nRF24_WriteRegister(0x00, 0x0E);                      // CONFIG: PWR_UP=1, PRIM_RX=0, 2-byte CRC

    // Wait 1.5ms for crystal oscillator startup
    for (volatile int i = 0; i < 20000; i++);
}

void nRF24_SendPacket(uint8_t *data, uint8_t len) {
    // 1. Load payload into TX FIFO
    GPIOA->BSRR = GPIO_BSRR_BR6;                         // CSN low
    SPI1_Transfer(0xA0);                                  // W_TX_PAYLOAD command
    for (int i = 0; i < len; i++) SPI1_Transfer(data[i]);
    GPIOA->BSRR = GPIO_BSRR_BS6;                         // CSN high

    // 2. Pulse CE >10us to trigger transmission
    GPIOA->BSRR = GPIO_BSRR_BS7;                         // CE high
    for (volatile int i = 0; i < 200; i++);              // ~10us @ 16MHz
    GPIOA->BSRR = GPIO_BSRR_BR7;                         // CE low
}

// ============================================================
// nRF24 RX
// ============================================================

void nRF24_RX_Init(void) {
    // 1. Power down before configuring
    nRF24_WriteRegister(0x00, 0x00);
    for (volatile int i = 0; i < 10000; i++);

    // 2. Flush RX FIFO and clear all status flags before configuring
    GPIOA->BSRR = GPIO_BSRR_BR6;
    SPI1_Transfer(0xE2);                                  // FLUSH_RX
    GPIOA->BSRR = GPIO_BSRR_BS6;

    nRF24_WriteRegister(0x07, 0x70);                      // clear RX_DR, TX_DS, MAX_RT

    // 3. Set RX pipe 0 address — must match TX's TX_ADDR
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    GPIOA->BSRR = GPIO_BSRR_BR6;
    SPI1_Transfer(0x20 | 0x0A);                           // W_REGISTER RX_ADDR_P0 (0x0A)
    for (int i = 0; i < 5; i++) SPI1_Transfer(addr[i]);
    GPIOA->BSRR = GPIO_BSRR_BS6;

    nRF24_WriteRegister(0x01, 0x01);                      // EN_AA: auto-ACK on pipe 0
    nRF24_WriteRegister(0x02, 0x01);                      // EN_RXADDR: enable pipe 0
    nRF24_WriteRegister(0x03, 0x03);                      // SETUP_AW: 5 byte address width
    nRF24_WriteRegister(0x04, 0x1A);                      // SETUP_RETR: 500us delay, 10 retries
    nRF24_WriteRegister(0x05, 0x4C);                      // RF_CH: channel 76 (must match TX)
    nRF24_WriteRegister(0x06, 0x0E);                      // RF_SETUP: 2Mbps, 0dBm, full LNA gain
    nRF24_WriteRegister(0x11, 0x04);                      // RX_PW_P0: 4 byte payload (must match TX)
    nRF24_WriteRegister(0x00, 0x0F);                      // CONFIG: PWR_UP=1, PRIM_RX=1, 2-byte CRC

    // Wait 1.5ms for crystal oscillator startup
    for (volatile int i = 0; i < 20000; i++);

    // CE held high permanently to keep nRF24 listening
    GPIOA->BSRR = GPIO_BSRR_BS7;
}

void nRF24_ReadPayload(uint8_t *buf, uint8_t len) {
    GPIOA->BSRR = GPIO_BSRR_BR6;                         // CSN low
    SPI1_Transfer(0x61);                                  // R_RX_PAYLOAD command
    for (int i = 0; i < len; i++)
        buf[i] = SPI1_Transfer(0xFF);                     // clock out each byte
    GPIOA->BSRR = GPIO_BSRR_BS6;                         // CSN high

    nRF24_WriteRegister(0x07, (1 << 6));                  // clear RX_DR flag
}