/**
 * PN532 Asynchronous I2C Library
 * Christopher Bero <berocs@acedb.co>
 */

// Internal header files
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdarg.h>
#include "pn532.h"
#include "i2c.h"
#include "usart.h"

const uint8_t pn532_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};

#define PN532_BUFFER_SIZE	24
uint8_t pn532_sendBuffer[PN532_BUFFER_SIZE];
uint8_t pn532_recvBuffer[PN532_BUFFER_SIZE];
uint8_t pn532_recvLen;

// State definitions
#define PN532_STATE_RESTING 	0x00
#define PN532_STATE_ACK_WAIT 	0x01
#define PN532_STATE_ACK_AVAIL 	0x02
#define PN532_STATE_ACK_READ 	0x03
#define PN532_STATE_CMD_WAIT 	0x03
#define PN532_STATE_CMD_AVAIL 	0x04
#define PN532_STATE_CALLBACK 	0x05

// Global variables
uint8_t state; // The global state for the async library
volatile uint8_t irqs; // The number of times INT1 is triggered
uint8_t (*callback)(uint8_t *, uint8_t) = NULL; // Callback function pointer

// Static (internal library use only) methods
static uint8_t recvAck();
static uint8_t recvResp();
static uint8_t writeCmdAck(uint8_t * cmd, uint8_t len);
static uint8_t writeCmd(uint8_t * cmd, uint8_t len);
static uint8_t sendCmd(uint8_t cmd, uint8_t paramCount, ...);
static uint8_t sendCmdData(uint8_t cmd, uint8_t * data, uint8_t dataLen, uint8_t argCount, ...);

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
}

/**
 * Manage interrupt events
 * Controls state machine
 */
uint8_t pn532_poll()
{
	//printf("Polling PN532. IRQ: %d\n", irqs);

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
		state = PN532_STATE_CALLBACK;
		break;
	case PN532_STATE_CALLBACK:
		state = PN532_STATE_RESTING;
		callback(pn532_recvBuffer, pn532_recvLen);
		break;
	default:
		printf("Default switch statement reached\n");
		break;
	}
	return(state);
}

uint8_t pn532_blockForCallback()
{
	while(state != PN532_STATE_RESTING)
	{
		_delay_ms(1);
		pn532_poll();
	}
	return(0);
}

uint8_t pn532_getFirmwareVersion(uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_GETFIRMWAREVERSION, 0);
	return(0);
}

uint8_t pn532_getGeneralStatus(uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_GETGENERALSTATUS, 0);
	return(0);
}

uint8_t pn532_SAMConfiguration(uint8_t _mode, uint8_t _timeout, uint8_t _irq,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_SAMCONFIGURATION, 3, _mode, _timeout, _irq);
	return(0);
}

uint8_t pn532_powerDown(uint8_t _wakeUpEnable, uint8_t _generateIrq,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_POWERDOWN, 2, _wakeUpEnable, _generateIrq);
	return(0);
}

uint8_t pn532_RFConfiguration(uint8_t _cfgItem,
						uint8_t * _configurationData, uint8_t _configurationDataLen,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmdData(PN532_COMMAND_RFCONFIGURATION, _configurationData, _configurationDataLen,
				1, _cfgItem);
	return(0);
}

uint8_t pn532_inListPassiveTarget(uint8_t _maxTg, uint8_t _BrTy,
						uint8_t * _initiatorData, uint8_t _initiatorDataLen,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmdData(PN532_COMMAND_INLISTPASSIVETARGET, _initiatorData, _initiatorDataLen,
				2, _maxTg, _BrTy);
	return(0);
}

uint8_t pn532_inATR(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_INATR, 2, _tg, 0x00);
	return(0);
}

uint8_t pn532_inSelect(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_INSELECT, 1, _tg);
	return(0);
}

uint8_t pn532_inDeselect(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_INDESELECT, 1, _tg);
	return(0);
}

uint8_t pn532_inRelease(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_INRELEASE, 1, _tg);
	return(0);
}

uint8_t pn532_inDataExchange(uint8_t _tg,
						uint8_t * _dataOut, uint8_t _dataOutLen,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmdData(PN532_COMMAND_INDATAEXCHANGE, _dataOut, _dataOutLen, 1, _tg);
	return(0);
}

uint8_t pn532_inAutoPoll(uint8_t _pollNr, uint8_t _period, uint8_t _type1,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	sendCmd(PN532_COMMAND_INAUTOPOLL, 3, _pollNr, _period, _type1);
	return(0);
}

/**
 * Sends a command with arguments,
 * used for convenience.
 */
static uint8_t sendCmd(uint8_t cmd, uint8_t argCount, ...)
{
	pn532_sendBuffer[0] = cmd;
	va_list args;
	va_start(args, argCount);
	for (int c = 1; c <= argCount; c++)
	{
		uint8_t byte = va_arg(args, int);
		pn532_sendBuffer[c] = byte;
	}
	va_end(args);
	writeCmdAck(pn532_sendBuffer, argCount+1);
	return(0);
}

/**
 * Sends a command with arguments, and an arbitrary length data block.
 */
static uint8_t sendCmdData(uint8_t cmd, uint8_t * data, uint8_t dataLen, uint8_t argCount, ...)
{
	uint8_t len;
	pn532_sendBuffer[0] = cmd;

	va_list args;
	va_start(args, argCount);
	for (int c = 1; c <= argCount; c++)
	{
		uint8_t byte = va_arg(args, int);
		pn532_sendBuffer[c] = byte;
	}
	va_end(args);

	for (uint8_t i = 0; i < dataLen; i++)
	{
		pn532_sendBuffer[1+argCount+i] = data[i];
	}

	len = 1 + argCount + dataLen;
	writeCmdAck(pn532_sendBuffer, len);
	return(0);
}

/**
 * Reads and validates the Ack
 */
static uint8_t recvAck()
{
//	printf("Receiving an ack.\n");

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

//		printf("\tack: %#x\n", pn532_recvBuffer[c]);
		if (pn532_recvBuffer[c] != pn532_ack[c])
		{
			printf("\tAck failed\n");
			return(1); // ack failed
		}
	}
	i2c_stop();
//	printf("Ack received successfully.\n");
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
	//printf("Receiving a response\n");
	i2c_start(PN532_I2C_ADDRESS | I2C_READ);

	// Gather preamble values
	for (int c = 0; c < 6; c++)
	{
		pn532_recvBuffer[c] = i2c_read_ack();
	}

	// List preamble values
//	printf("\tReady bit: %#x\n", pn532_recvBuffer[0]);
//	printf("\tPreamble: %#x\n", pn532_recvBuffer[1]);
//	printf("\tStart Code: %#x %#x\n", pn532_recvBuffer[2], pn532_recvBuffer[3]);
//	printf("\tLength: %#x %#x\n", pn532_recvBuffer[4], pn532_recvBuffer[5]);

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

	// Move length value to variable
	pn532_recvLen = pn532_recvBuffer[4];

	// Store main message in recv buffer
	for (int c = 0; c < pn532_recvLen; c++)
	{
		pn532_recvBuffer[c] = i2c_read_ack();
//		printf("\trecv: %#x\n", pn532_recvBuffer[c]);
	}

	// Collect data checksum
	checksum = i2c_read_ack();
//	printf("\tData Checksum: %#x\n", checksum);
	for (int c = 0; c < pn532_recvLen; c++)
	{
		checksum += pn532_recvBuffer[c];
	}
//	printf("\tData Checksum Result: %#x\n", checksum);
	if (checksum != 0x00)
	{
		printf("Recv failed, bad data checksum\n");
		return(1); // failed
	}

	// Skip postamble and end transmission
	i2c_read_nack();
	i2c_stop();
//	printf("Received successfully.\n");
	return(0);
}

/**
 * Writes a command, and kicks off state machine
 */
static uint8_t writeCmdAck(uint8_t * cmd, uint8_t len)
{
	if (state != 0) // system is busy
	{
		return(1);
	}

	// Write command(s)
	writeCmd(cmd, len);
	state = PN532_STATE_ACK_WAIT;

	return(0);
}

/**
 * Writes a command to I2C
 */
static uint8_t writeCmd(uint8_t * cmd, uint8_t len)
{
	uint8_t checksum;
	uint8_t err;

	if (state != 0) // system is busy
	{
		return(1);
	}

//	printf("Writing command: ");
//	for (uint8_t i = 0; i < len; i++)
//	{
//		printf("%#x, ", cmd[i]);
//	}
//	printf("\n");

	err = 0;
	len = len + 1; // To account for PN532_HOSTTOPN532

	err += i2c_start(PN532_I2C_ADDRESS | I2C_WRITE);
	err += i2c_write(PN532_PREAMBLE);
	err += i2c_write(PN532_STARTCODE1);
	err += i2c_write(PN532_STARTCODE2);

	err += i2c_write(len);
	checksum = ~len + 1; // Length checksum
	err += i2c_write(checksum);

	err += i2c_write(PN532_HOSTTOPN532);
	checksum = 0xFF;
	checksum += PN532_HOSTTOPN532;

	for (uint8_t i=0; i < (len-1); i++) {
		err += i2c_write(cmd[i]);
		checksum += cmd[i];
	}
	checksum = ~checksum;

	err += i2c_write(checksum);
	err += i2c_write(PN532_POSTAMBLE);
	i2c_stop();
	return(err);
}

/**
 * IRQ interrupt vector
 */
ISR(INT1_vect)
{
	//PINB |= (1 << PB5);
	irqs++;
}














