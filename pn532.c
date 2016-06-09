/**
 * PN532 I2C Library
 * Christopher Bero <berocs@acedb.co>
 */

// Internal header files
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "pn532.h"
#include "i2c.h"
#include "usart.h"

const uint8_t pn532_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

#define PN532_BUFFER_SIZE	16
uint8_t pn532_sendBuffer[PN532_BUFFER_SIZE];
uint8_t pn532_recvBuffer[PN532_BUFFER_SIZE];

// State definitions
#define PN532_STATE_RESTING 	0x00
#define PN532_STATE_ACK_WAIT 	0x01
#define PN532_STATE_ACK_AVAIL 	0x02
#define PN532_STATE_ACK_READ 	0x03
#define PN532_STATE_CMD_WAIT 	0x03
#define PN532_STATE_CMD_AVAIL 	0x04
#define PN532_STATE_CMD_READ 	0x05

// Global variables
uint8_t state; // The global state for the async library
volatile uint8_t irqs; // The number of times INT1 is triggered

// Static (internal library use only) methods
static uint8_t recvAck();
static uint8_t recvResp();
static uint8_t writeCmdAck(uint8_t * cmd, uint8_t len);
static uint8_t writeCmd(uint8_t * cmd, uint8_t len);

/**
 * Set up registers for serial and interrupt
 */
void pn532_init(void)
{
	// Set initial state
	state = PN532_STATE_RESTING;

	cli(); // Disable interrupts

	// Setup INT1
	EICRA |= (1<<ISC11) | (0<<ISC10); // Trigger INT1 on falling edge
	EIMSK |= 2; 		// Enable Interrupt Mask for Int1
	DDRD &= ~(1<<PD3); 	// Set PD3 as input
	PORTD |= (1<<PD3); 	// Enable PD3 pull-up resistor

	sei(); // Enable Interrupts

	// Send dummy packet to synchronize on
//	pn532_sendBuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
//	writeCmdAck(pn532_sendBuffer, 1);
}

/**
 * Manage interrupt events
 * Controls state machine
 */
uint8_t pn532_poll()
{
	printf("Polling PN532. IRQ: %d\n", irqs);

	if (irqs < 0)
	{
		printf("pn532 variable irqs is bad: %d\n", irqs);
		while(1);
	}

	switch(state)
	{
	case PN532_STATE_RESTING:
		break;
	case PN532_STATE_ACK_WAIT:
		if (irqs > 0)
		{
			state = PN532_STATE_ACK_AVAIL;
			--irqs;
		}
		break;
	case PN532_STATE_ACK_AVAIL:
		if (recvAck())
		{
			printf("halting on failed ack\n");
			while(1);
		}
		state = PN532_STATE_CMD_WAIT;
		break;
	case PN532_STATE_CMD_WAIT:
		if (irqs > 0)
		{
			state = PN532_STATE_CMD_AVAIL;
			--irqs;
		}
		break;
	case PN532_STATE_CMD_AVAIL:
		recvResp();
		state = PN532_STATE_RESTING;
		break;
	default:
		printf("Default switch statement reached\n");
		break;
	}
	return(0);
}

/**
 * Reads and validates the Ack
 */
static uint8_t recvAck()
{
	printf("Receiving an ack.\n");

	i2c_start(PN532_I2C_ADDRESS | I2C_READ);
	pn532_recvBuffer[0] = i2c_read_ack();
	if (pn532_recvBuffer[0] != 0x1)
	{
		printf("\tAck failed, no 0x1 init byte\n");
		return(1); // ack failed
	}
	for (int c = 0; c < 6; c++)
	{
		if (c == 5)
			pn532_recvBuffer[c] = i2c_read_nack();
		else
			pn532_recvBuffer[c] = i2c_read_ack();

		printf("\tack: %#x\n", pn532_recvBuffer[c]);
		if (pn532_recvBuffer[c] != pn532_ack[c])
		{
			printf("\tAck failed\n");
			return(1); // ack failed
		}
	}
	i2c_stop();
	printf("Ack received successfully.\n");
	return(0);
}

/**
 * Receives the 0xD5 response from PN532
 * Stores into pn532_recvBuffer.
 * Length of the data (0xD5 + PDn...) in index 0.
 * Data from index 1 onward.
 */
static uint8_t recvResp()
{
	uint8_t checksum;
	printf("Receiving a response\n");
	i2c_start(PN532_I2C_ADDRESS | I2C_READ);

	// Gather preamble values
	for (int c = 0; c < 6; c++)
	{
		pn532_recvBuffer[c] = i2c_read_ack();
	}

	// List preamble values
	printf("\tReady bit: %#x\n", pn532_recvBuffer[0]);
	printf("\tPreamble: %#x\n", pn532_recvBuffer[1]);
	printf("\tStart Code: %#x %#x\n", pn532_recvBuffer[2], pn532_recvBuffer[3]);
	printf("\tLength: %#x %#x\n", pn532_recvBuffer[4], pn532_recvBuffer[5]);

	// Check preamble values
	if (pn532_recvBuffer[0] != 0x1)
	{
		printf("Recv failed, no 0x01 ready bit\n");
		return(1); // failed
	}
	if (pn532_recvBuffer[1] != 0x00)
	{
		printf("Recv failed, preamble wrong\n");
		return(1); // failed
	}
	if (pn532_recvBuffer[2] != 0x00 || pn532_recvBuffer[3] != 0xFF)
	{
		printf("Recv failed, Start code wrong\n");
		return(1); // failed
	}
	if (((pn532_recvBuffer[4] + pn532_recvBuffer[5]) & 0xFF) != 0x00)
	{
		printf("Recv failed, length checksum wrong\n");
		return(1); // failed
	}

	// Move length value to index 0
	pn532_recvBuffer[0] = pn532_recvBuffer[4];

	// Store main message in recv buffer
	for (int c = 1; c <= pn532_recvBuffer[0]; c++)
	{
		pn532_recvBuffer[c] = i2c_read_ack();
		printf("\trecv: %#x\n", pn532_recvBuffer[c]);
	}

	// Collect data checksum
	checksum = i2c_read_ack();
	printf("\tData Checksum: %#x\n", checksum);
	for (int c = 1; c <= pn532_recvBuffer[0]; c++)
	{
		checksum += pn532_recvBuffer[c];
	}
	printf("\tData Checksum Result: %#x\n", checksum);
	if (checksum != 0x00)
	{
		printf("Recv failed, bad data checksum\n");
		return(1); // failed
	}

	// Skip postamble and end transmission
	i2c_read_nack();
	i2c_stop();
	printf("Received successfully.\n");
	return(0);
}

/**
 * Writes a command, and kicks off state machine
 */
static uint8_t writeCmdAck(uint8_t * cmd, uint8_t len)
{
	// TODO: make this nonblocking somehow
	while (state != 0) // already waiting for an ack
	{
		_delay_ms(2);
	}

	// Write command(s)
	writeCmd(cmd, len);
	state = PN532_STATE_ACK_WAIT;

	return(1);
}

/**
 * Writes a command to I2C
 */
static uint8_t writeCmd(uint8_t * cmd, uint8_t len)
{
	uint8_t checksum;
	uint8_t output;

	if (state != 0) // system is busy
	{
		return(1);
	}

	output = 0;
	len = len + 1; // To account for PN532_HOSTTOPN532

	output = i2c_start(PN532_I2C_ADDRESS | I2C_WRITE);
	output = i2c_write(PN532_PREAMBLE);
	output = i2c_write(PN532_STARTCODE1);
	output = i2c_write(PN532_STARTCODE2);

	output = i2c_write(len);
	checksum = ~len + 1; // Length checksum
	output = i2c_write(checksum);

	output = i2c_write(PN532_HOSTTOPN532);
	checksum = 0xFF;
	checksum += PN532_HOSTTOPN532;

	for (uint8_t i=0; i < (len-1); i++) {
		output = i2c_write(cmd[i]);
		checksum += cmd[i];
	}
	checksum = ~checksum;

	output = i2c_write(checksum);
	output = i2c_write(PN532_POSTAMBLE);
	i2c_stop();
	return(0);
}

/**
 * IRQ interrupt vector
 */
ISR(INT1_vect)
{
	//PINB |= (1 << PB5);
	irqs++;
}














