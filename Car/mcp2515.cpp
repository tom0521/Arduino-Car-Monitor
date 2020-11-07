#include <avr/io.h>
#include <util/delay.h>
#include "defauts.h"
#include "mcp2515.h"
#include "register.h"
#include "spi.h"

/*
 * Initialize MCP2515
 * 
 * First sets all the the input and output pins,
 * resets the chip, configures it, and confirms
 * communication works.
 * 
 */
bool mcp_init (uint8_t baud_prescaler) {
    // Set all chip selects HIGH because
    // I do not want to send data yet
    SET(MCP_CS);
    // Set the MCP2515 Chip Select pin as output
    SET_OUTPUT(MCP_CS);
    // Set up MCP2515 interrupt pin as input pin
    SET_INPUT(MCP_INT);
    // Enable the pull up resistor
    SET(MCP_INT);

    // Software RESET the MCP2515
    RESET(MCP_CS);
    spi_transmit(MCP_RESET);
    SET(MCP_CS);
    _delay_us(10);  // Give it some time

    // Initialize the CAN-BUS now that it is in Configuration Mode
    RESET(MCP_CS);
    spi_transmit(MCP_WRITE);        // WRITE instruction
    spi_transmit(CNF3);             // Start on CNF3 Register
    spi_transmit(1 << PHSEG21);     // PS2 length = (2 + 1) * Tq
                                    // Move to CNF2 Register
    // BTLMODE (PS2 len determined by CNF3) and PS1 length = (2 + 1) * Tq
    spi_transmit((1 << BTLMODE) | (1 << PHSEG11));
                                    // Move to CNF1 Register
    // Set the baud rate prescaler (Tq = 2 * (baud_rate + 1)/Fosc )
    spi_transmit(baud_prescaler);
                                    // Move to CANINTE Register
    spi_transmit((1 << RX0IE) | (1 << RX1IE));   // activate interrupts
    SET(MCP_CS);

    // On successful initialization, check the available PIDs
    if (mcp_read(CNF1) != 0x01) {
        return false;
    }

    // Deactivate the high Impedance State
    mcp_write(BFPCTRL, 0x0);

    // Set TXnRTS as inputs
    mcp_write(TXRTSCTRL, 0x0);

    // Turn off filters
    mcp_write(RXB0CTRL, (1 << RXM1) | (1 << RXM0));
    mcp_write(RXB1CTRL, (1 << RXM1) | (1 << RXM0));

    // Enter Normal Mode
    mcp_write(CANCTRL, 0x0);

    return true;
}

bool mcp_send_message (uint16_t id, uint8_t data_len, uint8_t * data) {
    uint8_t status = mcp_read_status();
    
    // Load TX Buffer address pointer bits
    uint8_t abc;
    if (!(status & 0x04)) { // TXB0CNTRL
        abc = 0x00;
    } else if (!(status & 0x10)) { // TXB1CNTRL
        abc = 0x02;
    } else if (!(status & 0x40)) { // TXB2CNTRL
        abc = 0x04;
    } else { // All buffers full
        return false;
    }

    RESET(MCP_CS);
    /* Send LOAD TX BUFFER instruction starting at
        register represented by abc bits */
    spi_transmit(MCP_LOAD_TX | abc);

    /* Write to TXBnSIDH register:
        - Identifier bits 10-3 */
    spi_transmit(id >> 3);

    /* Write to TXBnSIDL register:
        - Identifier bits 2-0 (bits 7-5)
        - Extended Identifier Enable Bit (bit 3)
        - Extended Identifier Bits 17-16 (bits 1-0) */
    spi_transmit(id << 5);

    /* Write to TXBnEID8 register:
        - Extended Identifer bits 15-8 */
    spi_transmit(0x00); // Not using extended identifier

    /* Write to TXBnEID0 register:
        - Extended Identifier bits 7-0 */
    spi_transmit(0x00); // Not using extended identifier

    /* Write to TXBnDLC register:
        - Remote Transmission Request Bit (bit 6)
        - Data Length Code (bits 3-0) */
    spi_transmit(data_len);

    /* Write to TXBnDm registers with data */
    for(uint8_t m = 0; m < data_len; ++m) {
        spi_transmit(data[m]);
    }

    SET(MCP_CS);

    /* Need to wait? */
    // _delay_us(1);

    /* Send the Request to send instruction */
    /* If message is in TXB0 registers, then
        the flag is different than the LOAD TX
        instruction. Otherwise, they're the same */
    uint8_t nnn = (abc == 0x00) ? 0x01 : abc;
    RESET(MCP_CS);
    spi_transmit(MCP_RTS | nnn);
    SET(MCP_CS);

    return true;
}

/*
 * MCP2515 Read Register Contents
 * 
 * Gets the contents of a given MCP2515
 * register
 */
uint8_t mcp_read (uint8_t addr) {
    uint8_t data;

    RESET(MCP_CS);
    spi_transmit(MCP_READ);
    spi_transmit(addr);
    data = spi_transmit(0x00);
    SET(MCP_CS);

    return data;
}

/*
 * MCP2515 Request-To-send
 * 
 * Sends the request-to-send instruction
 * with the given flags of buffers to
 * send.
 */
void mcp_rts (uint8_t txb_flags) {
    RESET(MCP_CS);
    spi_transmit(MCP_RTS | txb_flags);
    SET(MCP_CS);
}

/**
 * MCP2515 Read Status
 * 
 * Sends the read status instruction to
 * receive a byte of data containing
 * the status flags.
 */
uint8_t mcp_read_status () {
    uint8_t status;

    RESET(MCP_CS);
    spi_transmit(MCP_READ_STATUS);
    status = spi_transmit(0x00);
    SET(MCP_CS);

    return status;
}

/*
 * MCP2515 Write to Register
 * 
 * Puts the given data in the given MCP2515
 * register
 */
void mcp_write (uint8_t addr, uint8_t data) {
    RESET(MCP_CS);
    spi_transmit(MCP_WRITE);
    spi_transmit(addr);
    spi_transmit(data);
    SET(MCP_CS);
}

/*
 * MCP2515 Bit Modify Register
 * 
 * Modifies the given register by
 * using the supplied mask and data.
 * The mask contains 1 in all bit
 * positions that are to be changed to
 * the value in the respective position
 * of the data byte.
 */
void mcp_bit_modify (uint8_t addr, uint8_t mask, uint8_t data) {
    RESET(MCP_CS);
    spi_transmit(MCP_BIT_MODIFY);
    spi_transmit(addr);
    spi_transmit(mask);
    spi_transmit(data);
    SET(MCP_CS);
}