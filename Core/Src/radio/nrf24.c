// Functions to manage the nRF24L01+ transceiver

#include "../../Inc/radio/nrf24.h"

// Read a register
// input:
//   reg - number of register to read
// return: value of register
uint8_t nRF24_ReadReg(const NRF24_TypeDef *nrf, uint8_t reg) {
	uint8_t value;

	nRF24_CSN_L(nrf);
	nRF24_LL_RW(nrf, reg & nRF24_MASK_REG_MAP);
	value = nRF24_LL_RW(nrf, nRF24_CMD_NOP);
	nRF24_CSN_H(nrf);

	return value;
}

// Write a new value to register
// input:
//   reg - number of register to write
//   value - value to write
void nRF24_WriteReg(const NRF24_TypeDef *nrf, uint8_t reg, uint8_t value) {
	nRF24_CSN_L(nrf);
	if (reg < nRF24_CMD_W_REGISTER) {
		// This is a register access
		nRF24_LL_RW(nrf, nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP));
		nRF24_LL_RW(nrf, value);
	} else {
		// This is a single byte command or future command/register
		nRF24_LL_RW(nrf, reg);
		if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) && \
				(reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP)) {
			// Send register value
			nRF24_LL_RW(nrf, value);
		}
	}
	nRF24_CSN_H(nrf);
}

// Read a multi-byte register
// input:
//   reg - number of register to read
//   pBuf - pointer to the buffer for register data
//   count - number of bytes to read
static void nRF24_ReadMBReg(const NRF24_TypeDef *nrf, uint8_t reg, uint8_t *pBuf, uint8_t count) {
	nRF24_CSN_L(nrf);
	nRF24_LL_RW(nrf, reg);
	while (count--) {
		*pBuf++ = nRF24_LL_RW(nrf, nRF24_CMD_NOP);
	}
	nRF24_CSN_H(nrf);
}

// Write a multi-byte register
// input:
//   reg - number of register to write
//   pBuf - pointer to the buffer with data to write
//   count - number of bytes to write
static void nRF24_WriteMBReg(const NRF24_TypeDef *nrf, uint8_t reg, uint8_t *pBuf, uint8_t count) {
	nRF24_CSN_L(nrf);
	nRF24_LL_RW(nrf, reg);
	while (count--) {
		nRF24_LL_RW(nrf, *pBuf++);
	}
	nRF24_CSN_H(nrf);
}

// Set transceiver to it's initial state
// note: RX/TX pipe addresses remains untouched
void nRF24_Init(const NRF24_TypeDef *nrf) {
	// Write to registers their initial values
	nRF24_WriteReg(nrf, nRF24_REG_CONFIG, 0x0A);
	nRF24_WriteReg(nrf, nRF24_REG_EN_AA, 0x3F);
	nRF24_WriteReg(nrf, nRF24_REG_EN_RXADDR, 0x03);
	nRF24_WriteReg(nrf, nRF24_REG_SETUP_AW, 0x03);
	nRF24_WriteReg(nrf, nRF24_REG_SETUP_RETR, 0x03);
	nRF24_WriteReg(nrf, nRF24_REG_RF_CH, 0x03);
	nRF24_WriteReg(nrf, nRF24_REG_RF_SETUP, 0x0E);
	nRF24_WriteReg(nrf, nRF24_REG_STATUS, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_RX_PW_P0, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_RX_PW_P1, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_RX_PW_P2, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_RX_PW_P3, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_RX_PW_P4, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_RX_PW_P5, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_DYNPD, 0x00);
	nRF24_WriteReg(nrf, nRF24_REG_FEATURE, 0x00);

	// Clear the FIFO's
	nRF24_FlushRX(nrf);
	nRF24_FlushTX(nrf);

	// Clear any pending interrupt flags
	nRF24_ClearIRQFlags(nrf);

	// Deassert CSN pin (chip release)
	nRF24_CSN_H(nrf);
}

// Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t nRF24_Check(const NRF24_TypeDef *nrf) {
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;

	// Write test TX address and read TX_ADDR register
	nRF24_WriteMBReg(nrf, nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
	nRF24_ReadMBReg(nrf, nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);

	// Compare buffers, return error on first mismatch
	for (i = 0; i < 5; i++) {
		if (rxbuf[i] != *ptr++) return 0;
	}

	return 1;

	nRF24_WriteReg(nrf, nRF24_REG_RF_CH, 0x02);
	return nRF24_ReadReg(nrf, nRF24_REG_RF_CH) == 0x02;
}

void nRF24_SetTxAddr(const NRF24_TypeDef *nrf, uint8_t *addr, uint8_t size) {
    nRF24_WriteMBReg(nrf, nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, addr, size);
}

void nRF24_PrettyPrint(const NRF24_TypeDef *nrf) {
    // SEGGER_RTT_printf(0, "nRF24L01+ Configuration Details:\n");
    
    // // General Configurations
    // uint8_t config = nRF24_ReadReg(nrf, nRF24_REG_CONFIG);
    // SEGGER_RTT_printf(0, "  CONFIG: 0x%02X\n", config);
    // // SEGGER_RTT_printf(0, "    CRC: %s\n", (config & nRF24_CONFIG_EN_CRC) ? "Enabled" : "Disabled");
    // SEGGER_RTT_printf(0, "    Power Mode: %s\n", (config & nRF24_CONFIG_PWR_UP) ? "ON" : "OFF");
    // SEGGER_RTT_printf(0, "    Mode: %s\n", (config & nRF24_CONFIG_PRIM_RX) ? "RX" : "TX");

    // // RF Settings
    // uint8_t rf_ch = nRF24_GetRFChannel(nrf);
    // uint8_t rf_setup = nRF24_ReadReg(nrf, nRF24_REG_RF_SETUP);
    // SEGGER_RTT_printf(0, "  RF Channel: %d (Frequency: %d MHz)\n", rf_ch, 2400 + rf_ch);
    // SEGGER_RTT_printf(0, "  RF Setup: 0x%02X\n", rf_setup);
    // // SEGGER_RTT_printf(0, "    Data Rate: %s\n", (rf_setup & nRF24_RF_DR_HIGH) ? "2Mbps" : (rf_setup & nRF24_RF_DR_LOW) ? "250kbps" : "1Mbps");
    // SEGGER_RTT_printf(0, "    TX Power: %d dBm\n", (rf_setup & nRF24_MASK_RF_PWR) >> 1);

    // // Auto Retransmit Settings
    // uint8_t retr = nRF24_ReadReg(nrf, nRF24_REG_SETUP_RETR);
    // SEGGER_RTT_printf(0, "  Auto Retransmit:\n");
    // SEGGER_RTT_printf(0, "    Delay: %d us\n", ((retr >> 4) & 0x0F) * 250);
    // SEGGER_RTT_printf(0, "    Count: %d\n", retr & 0x0F);

    // uint8_t address_width = (nRF24_ReadReg(nrf, nRF24_REG_SETUP_AW) & 0x03) + 2;
    // SEGGER_RTT_printf(0, "  Address Width: %d bytes\n", address_width);

    // // TX Address (existing code)
    // uint8_t tx_address[5];
    // nRF24_ReadMBReg(nrf, nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, tx_address, address_width);
    // SEGGER_RTT_printf(0, "  TX Address: ");
    // for (int i = 0; i < address_width; i++) {
    //     SEGGER_RTT_printf(0, "%02X", tx_address[i]);
    // }
    // SEGGER_RTT_printf(0, "\n");

    // // Add RX Addresses for Pipes 0-5
    // SEGGER_RTT_printf(0, "  RX Addresses:\n");
    // for (int pipe = 0; pipe < 6; pipe++) {
    //     uint8_t en_rxaddr = nRF24_ReadReg(nrf, nRF24_REG_EN_RXADDR);
    //     if (en_rxaddr & (1 << pipe)) {
    //         uint8_t rx_address[5];
    //         nRF24_ReadMBReg(nrf, nRF24_CMD_R_REGISTER | nRF24_ADDR_REGS[pipe], rx_address, 
    //                         (pipe == 0 || pipe == 1) ? address_width : 1);
            
    //         SEGGER_RTT_printf(0, "    Pipe %d: ", pipe);
    //         if (pipe == 0 || pipe == 1) {
    //             for (int i = 0; i < address_width; i++) {
    //                 SEGGER_RTT_printf(0, "%02X", rx_address[i]);
    //             }
    //         } else {
    //             // For pipes 2-5, only the last byte changes
    //             // Retrieve the base address from Pipe 1
    //             uint8_t base_address[4];
    //             nRF24_ReadMBReg(nrf, nRF24_CMD_R_REGISTER | nRF24_ADDR_REGS[1], base_address, 4);
    //             for (int i = 0; i < 4; i++) {
    //                 SEGGER_RTT_printf(0, "%02X", base_address[i]);
    //             }
    //             SEGGER_RTT_printf(0, "%02X", rx_address[0]);
    //         }
    //         SEGGER_RTT_printf(0, "\n");
    //     }
    // }

    // // Enable Auto Acknowledgment and Enabled RX Pipes
    // uint8_t en_aa = nRF24_ReadReg(nrf, nRF24_REG_EN_AA);
    // uint8_t en_rxaddr = nRF24_ReadReg(nrf, nRF24_REG_EN_RXADDR);
    // SEGGER_RTT_printf(0, "  Auto Acknowledgment Enabled: 0x%02X\n", en_aa);
    // SEGGER_RTT_printf(0, "  RX Addresses Enabled: 0x%02X\n", en_rxaddr);

    // // Dynamic Payload Length
    // uint8_t dynpd = nRF24_ReadReg(nrf, nRF24_REG_DYNPD);
    // SEGGER_RTT_printf(0, "  Dynamic Payloads Enabled: 0x%02X\n", dynpd);

    // // Feature Register
    // uint8_t feature = nRF24_ReadReg(nrf, nRF24_REG_FEATURE);
    // SEGGER_RTT_printf(0, "  Features: 0x%02X\n", feature);
    // SEGGER_RTT_printf(0, "    Dynamic Payload Length: %s\n", (feature & nRF24_FEATURE_EN_DPL) ? "Enabled" : "Disabled");
    // SEGGER_RTT_printf(0, "    Payload with ACK: %s\n", (feature & nRF24_FEATURE_EN_ACK_PAY) ? "Enabled" : "Disabled");
    // SEGGER_RTT_printf(0, "    W_TX_PAYLOAD_NOACK Command: %s\n", (feature & nRF24_FEATURE_EN_DYN_ACK) ? "Enabled" : "Disabled");
}

// Control transceiver power mode
// input:
//   mode - new state of power mode, one of nRF24_PWR_xx values
void nRF24_SetPowerMode(const NRF24_TypeDef *nrf, uint8_t mode) {
	uint8_t reg;

	reg = nRF24_ReadReg(nrf, nRF24_REG_CONFIG);
	if (mode == nRF24_PWR_UP) {
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Stanby-I mode with consumption about 26uA
		reg |= nRF24_CONFIG_PWR_UP;
	} else {
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// into power down mode with consumption about 900nA
		reg &= ~nRF24_CONFIG_PWR_UP;
	}
	nRF24_WriteReg(nrf, nRF24_REG_CONFIG, reg);
}

// Set transceiver operational mode
// input:
//   mode - operational mode, one of nRF24_MODE_xx values
void nRF24_SetOperationalMode(const NRF24_TypeDef *nrf, uint8_t mode) {
	uint8_t reg;

	// Configure PRIM_RX bit of the CONFIG register
	reg  = nRF24_ReadReg(nrf, nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= (mode & nRF24_CONFIG_PRIM_RX);
	nRF24_WriteReg(nrf, nRF24_REG_CONFIG, reg);
}

// Set transceiver DynamicPayloadLength feature for all the pipes
// input:
//   mode - status, one of nRF24_DPL_xx values
void nRF24_SetDynamicPayloadLength(const NRF24_TypeDef *nrf, uint8_t mode) {
	uint8_t reg;
	reg  = nRF24_ReadReg(nrf, nRF24_REG_FEATURE);
	if(mode) {
		nRF24_WriteReg(nrf, nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_DPL);
		nRF24_WriteReg(nrf, nRF24_REG_DYNPD, 0x1F);
	} else {
		nRF24_WriteReg(nrf, nRF24_REG_FEATURE, reg &~ nRF24_FEATURE_EN_DPL);
		nRF24_WriteReg(nrf, nRF24_REG_DYNPD, 0x0);
	}
}

// Enables Payload With Ack. NB Refer to the datasheet for proper retransmit timing.
// input:
//   mode - status, 1 or 0
void nRF24_SetPayloadWithAck(const NRF24_TypeDef *nrf, uint8_t mode) {
	uint8_t reg;
	reg  = nRF24_ReadReg(nrf, nRF24_REG_FEATURE);
	if(mode) {
		nRF24_WriteReg(nrf, nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_ACK_PAY);
	} else {
		nRF24_WriteReg(nrf, nRF24_REG_FEATURE, reg &~ nRF24_FEATURE_EN_ACK_PAY);
	}
}

// Configure transceiver CRC scheme
// input:
//   scheme - CRC scheme, one of nRF24_CRC_xx values
// note: transceiver will forcibly turn on the CRC in case if auto acknowledgment
//       enabled for at least one RX pipe
void nRF24_SetCRCScheme(const NRF24_TypeDef *nrf, uint8_t scheme) {
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg  = nRF24_ReadReg(nrf, nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= (scheme & nRF24_MASK_CRC);
	nRF24_WriteReg(nrf, nRF24_REG_CONFIG, reg);
}

// Set frequency channel
// input:
//   channel - radio frequency channel, value from 0 to 127
// note: frequency will be (2400 + channel)MHz
// note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be reset
void nRF24_SetRFChannel(const NRF24_TypeDef *nrf, uint8_t channel) {
	nRF24_WriteReg(nrf, nRF24_REG_RF_CH, channel);
}

uint8_t nRF24_GetRFChannel(const NRF24_TypeDef *nrf) {
	return nRF24_ReadReg(nrf, nRF24_REG_RF_CH);
}

uint8_t nRF24_GetConfig(const NRF24_TypeDef *nrf) {
	return nRF24_ReadReg(nrf, nRF24_REG_CONFIG);
}

// Set automatic retransmission parameters
// input:
//   ard - auto retransmit delay, one of nRF24_ARD_xx values
//   arc - count of auto retransmits, value form 0 to 15
// note: zero arc value means that the automatic retransmission disabled
void nRF24_SetAutoRetr(const NRF24_TypeDef *nrf, uint8_t ard, uint8_t arc) {
	// Set auto retransmit settings (SETUP_RETR register)
	nRF24_WriteReg(nrf, nRF24_REG_SETUP_RETR, (uint8_t)((ard << 4) | (arc & nRF24_MASK_RETR_ARC)));
}

// Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
void nRF24_SetAddrWidth(const NRF24_TypeDef *nrf, uint8_t addr_width) {
	nRF24_WriteReg(nrf, nRF24_REG_SETUP_AW, addr_width - 2);
}

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
void nRF24_SetAddr(const NRF24_TypeDef *nrf, uint8_t pipe, const uint8_t *addr) {
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe) {
		case nRF24_PIPETX:
		case nRF24_PIPE0:
		case nRF24_PIPE1:
			// Get address width
			addr_width = nRF24_ReadReg(nrf, nRF24_REG_SETUP_AW) + 1;
			// Write address in reverse order (LSByte first)
			addr += addr_width;
			nRF24_CSN_L(nrf);
			nRF24_LL_RW(nrf, nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
			do {
				nRF24_LL_RW(nrf, *addr--);
			} while (addr_width--);
			nRF24_CSN_H(nrf);
			break;
		case nRF24_PIPE2:
		case nRF24_PIPE3:
		case nRF24_PIPE4:
		case nRF24_PIPE5:
			// Write address LSBbyte (only first byte from the addr buffer)
			nRF24_WriteReg(nrf, nRF24_ADDR_REGS[pipe], *addr);
			break;
		default:
			// Incorrect pipe number -> do nothing
			break;
	}
}

// Configure RF output power in TX mode
// input:
//   tx_pwr - RF output power, one of nRF24_TXPWR_xx values
void nRF24_SetTXPower(const NRF24_TypeDef *nrf, uint8_t tx_pwr) {
	uint8_t reg;

	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg  = nRF24_ReadReg(nrf, nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	nRF24_WriteReg(nrf, nRF24_REG_RF_SETUP, reg);
}

// Configure transceiver data rate
// input:
//   data_rate - data rate, one of nRF24_DR_xx values
void nRF24_SetDataRate(const NRF24_TypeDef *nrf, uint8_t data_rate) {
	uint8_t reg;

	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
	reg  = nRF24_ReadReg(nrf, nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= data_rate;
	nRF24_WriteReg(nrf, nRF24_REG_RF_SETUP, reg);
}

// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void nRF24_SetRXPipe(const NRF24_TypeDef *nrf, uint8_t pipe, uint8_t aa_state, uint8_t payload_len) {
	uint8_t reg;

	// Enable the specified pipe (EN_RXADDR register)
	reg = (nRF24_ReadReg(nrf, nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
	nRF24_WriteReg(nrf, nRF24_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	nRF24_WriteReg(nrf, nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = nRF24_ReadReg(nrf, nRF24_REG_EN_AA);
	if (aa_state == nRF24_AA_ON) {
		reg |=  (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	nRF24_WriteReg(nrf, nRF24_REG_EN_AA, reg);
}

// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
void nRF24_ClosePipe(const NRF24_TypeDef *nrf, uint8_t pipe) {
	uint8_t reg;

	reg  = nRF24_ReadReg(nrf, nRF24_REG_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= nRF24_MASK_EN_RX;
	nRF24_WriteReg(nrf, nRF24_REG_EN_RXADDR, reg);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
void nRF24_EnableAA(const NRF24_TypeDef *nrf, uint8_t pipe) {
	uint8_t reg;

	// Set bit in EN_AA register
	reg  = nRF24_ReadReg(nrf, nRF24_REG_EN_AA);
	reg |= (1 << pipe);
	nRF24_WriteReg(nrf, nRF24_REG_EN_AA, reg);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
void nRF24_DisableAA(const NRF24_TypeDef *nrf, uint8_t pipe) {
	uint8_t reg;

	if (pipe > 5) {
		// Disable Auto-ACK for ALL pipes
		nRF24_WriteReg(nrf, nRF24_REG_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		reg  = nRF24_ReadReg(nrf, nRF24_REG_EN_AA);
		reg &= ~(1 << pipe);
		nRF24_WriteReg(nrf, nRF24_REG_EN_AA, reg);
	}
}

// Get value of the STATUS register
// return: value of STATUS register
uint8_t nRF24_GetStatus(const NRF24_TypeDef *nrf) {
	return nRF24_ReadReg(nrf, nRF24_REG_STATUS);
}

// Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
uint8_t nRF24_GetIRQFlags(const NRF24_TypeDef *nrf) {
	return (nRF24_ReadReg(nrf, nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

// Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
uint8_t nRF24_GetStatus_RXFIFO(const NRF24_TypeDef *nrf) {
	return (nRF24_ReadReg(nrf, nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
}

// Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
uint8_t nRF24_GetStatus_TXFIFO(const NRF24_TypeDef *nrf) {
	return ((nRF24_ReadReg(nrf, nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t nRF24_GetRXSource(const NRF24_TypeDef *nrf) {
	return ((nRF24_ReadReg(nrf, nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}

// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
uint8_t nRF24_GetRetransmitCounters(const NRF24_TypeDef *nrf) {
	return (nRF24_ReadReg(nrf, nRF24_REG_OBSERVE_TX));
}

// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
void nRF24_ResetPLOS(const NRF24_TypeDef *nrf) {
	uint8_t reg;

	// The PLOS counter is reset after write to RF_CH register
	reg = nRF24_ReadReg(nrf, nRF24_REG_RF_CH);
	nRF24_WriteReg(nrf, nRF24_REG_RF_CH, reg);
}

// Flush the TX FIFO
void nRF24_FlushTX(const NRF24_TypeDef *nrf) {
	nRF24_WriteReg(nrf, nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

// Flush the RX FIFO
void nRF24_FlushRX(const NRF24_TypeDef *nrf) {
	nRF24_WriteReg(nrf, nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

// Clear any pending IRQ flags
void nRF24_ClearIRQFlags(const NRF24_TypeDef *nrf) {
	uint8_t reg;

	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg  = nRF24_ReadReg(nrf, nRF24_REG_STATUS);
	reg |= nRF24_MASK_STATUS_IRQ;
	nRF24_WriteReg(nrf, nRF24_REG_STATUS, reg);
}

// Write TX payload
// input:
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
void nRF24_WritePayload(const NRF24_TypeDef *nrf, uint8_t *pBuf, uint8_t length) {
	nRF24_WriteMBReg(nrf, nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}

static uint8_t nRF24_GetRxDplPayloadWidth(const NRF24_TypeDef *nrf) {
	uint8_t value;

	nRF24_CSN_L(nrf);
	nRF24_LL_RW(nrf, nRF24_CMD_R_RX_PL_WID);
	value = nRF24_LL_RW(nrf, nRF24_CMD_NOP);
	nRF24_CSN_H(nrf);

	return value;

}

static nRF24_RXResult nRF24_ReadPayloadGeneric(const NRF24_TypeDef *nrf, uint8_t *pBuf, uint8_t *length, uint8_t dpl) {
	uint8_t pipe;

	// Extract a payload pipe number from the STATUS register
	pipe = (nRF24_ReadReg(nrf, nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {
		// Get payload length
		if(dpl) {
			*length = nRF24_GetRxDplPayloadWidth(nrf);
			if(*length>32) { //broken packet
				*length = 0;
				nRF24_FlushRX(nrf);
			}
		} else {
			*length = nRF24_ReadReg(nrf, nRF24_RX_PW_PIPE[pipe]);
		}

		// Read a payload from the RX FIFO
		if (*length) {
			nRF24_ReadMBReg(nrf, nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
		}

		return ((nRF24_RXResult)pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return nRF24_RX_EMPTY;
}

// Read top level payload available in the RX FIFO
// input:
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
nRF24_RXResult nRF24_ReadPayload(const NRF24_TypeDef *nrf, uint8_t *pBuf, uint8_t *length) {
	return nRF24_ReadPayloadGeneric(nrf, pBuf, length,0);
}

nRF24_RXResult nRF24_ReadPayloadDpl(const NRF24_TypeDef *nrf, uint8_t *pBuf, uint8_t *length) {
	return nRF24_ReadPayloadGeneric(nrf, pBuf, length,1);
}

uint8_t nRF24_GetFeatures(const NRF24_TypeDef *nrf) {
    return nRF24_ReadReg(nrf, nRF24_REG_FEATURE);
}
void nRF24_ActivateFeatures(const NRF24_TypeDef *nrf) {
    nRF24_CSN_L(nrf);
    nRF24_LL_RW(nrf, nRF24_CMD_ACTIVATE);
    nRF24_LL_RW(nrf, 0x73);
    nRF24_CSN_H(nrf);
}
void nRF24_WriteAckPayload(const NRF24_TypeDef *nrf, nRF24_RXResult pipe, char *payload, uint8_t length) {
	nRF24_CSN_L(nrf);
	nRF24_LL_RW(nrf, nRF24_CMD_W_ACK_PAYLOAD | pipe);
	while (length--) {
		nRF24_LL_RW(nrf, (uint8_t) *payload++);
	}
	nRF24_CSN_H(nrf);

}