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
    if (mcp_read(CNF1) != baud_prescaler) {
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

/* TODO: Use interrupt to receive message */
/*
 * MCP2515 Check Message
 * 
 * Checks if the interrupt pins is set LOW
 */
bool mcp_check_message () {
    /* Interrupt pin LOW means a message is waiting */
    return (!IS_SET(MCP_INT));
}

/**
 * MCP2515 RX Message
 * 
 * If there is a buffer waiting to be received,
 * then move it into the provided data buffer
 * and set the buffer as available for use
 */
bool mcp_rx_message (mcp_can_frame * frame) {
    // Byte to store the recieved data
    uint8_t rx_byte;

    //Get the status of RX/TX buffers
    rx_byte = mcp_read_status();

    /* TODO: double check bit positions */
    // Read RX Buffer address flags
    uint8_t nm;
    if (!(rx_byte & 0x01)) { // RXB0
        nm = 0x00;
    } else if (!(rx_byte & 0x02)) { // RXB1
        nm = 0x02;
    } else { // There are no messages waiting
        return false;
    }

    RESET(MCP_CS);
    /* Send READ RX BUFFER instruction starting at
        register represented by nm flags */
    spi_transmit(MCP_READ_RX | nm);

    /* Read from RXBnSIDH register:
        - Identifier bits 10-3 */
    frame->sid = (spi_transmit(0x00) << 3);

    /* Read from RXBnSIDL register:
        - Identifier bits 2-0 (bits 7-5)
        - Standard Frame Remote Trasmit Request (bit 4)
        - Extended Identifier bit flag (bit 3)
        - Extended Identifier bits 17-16 (bits 1-0) */
    rx_byte = spi_transmit(0x00);
    frame->sid |= (rx_byte >> 5);
    frame->srr = ((rx_byte >> 4) & 0x01);
    frame->ide = ((rx_byte >> 3) & 0x01);
    frame->eid = (rx_byte << 16);

    /* Read from RXBnEID8 register:
        - Extended Identifier bits 15-8 (1 byte) */
    frame->eid |= (spi_transmit(0x00) << 8);

    /* Read from RXBnEID0 register:
        - Extended Identifier bits 7-0 (1 byte) */
    frame->eid |= spi_transmit(0x00);

    /* Read from RXBnDLC register:
        - Remote Transmission Request bit flag (bit 6)
        - Data Length Code (bits 3-0) */
    rx_byte = spi_transmit(0x00);
    frame->rtr = ((rx_byte >> 6) & 0x01);
    frame->dlc = (rx_byte & 0x07);

    /* Read from RXBnDm registers */
    for (uint8_t m = 0; m < frame->dlc; ++m) {
        frame->data[m] = spi_transmit(0x00);
    }

    SET(MCP_CS);

    // Clear the interrupt flag
    mcp_bit_modify(CANINTF, (1 << ((nm == 0x00) ? RX0IF : RX1IF)), 0);

    return true;
}

/*
 * MCP2515 TX Message
 * 
 * If there is an unused buffer in the MCP2515
 * memory, then store the message to send there
 * and request to send the message
 */
bool mcp_tx_message (uint16_t id, uint8_t data_len, uint8_t * data) {
    //Get the status of RX/TX buffers
    uint8_t status = mcp_read_status();
    
    /* TODO: double check bit positions */
    // Load TX Buffer address flags
    uint8_t abc;
    if (!(status & 0x04)) { // TXB0
        abc = 0x00;
    } else if (!(status & 0x10)) { // TXB1
        abc = 0x02;
    } else if (!(status & 0x40)) { // TXB2
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
    _delay_us(1);

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