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
#define PN532_STATE_RESTING 	(0x00)
#define PN532_STATE_ACK_WAIT 	(0x01)
#define PN532_STATE_ACK_AVAIL 	(0x02)
#define PN532_STATE_ACK_READ 	(0x03)
#define PN532_STATE_CMD_WAIT 	(0x03)
#define PN532_STATE_CMD_AVAIL 	(0x04)
#define PN532_STATE_CALLBACK 	(0x05)

// Global variables
static uint8_t state; // The global state for the async library
volatile uint8_t irqs; // The number of times INT1 is triggered
uint8_t (*callback)(uint8_t *, uint8_t) = NULL; // Callback function pointer
uint8_t (*ackCallback)() = NULL; // Ack callback function pointer

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
void pn532_init()
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

	switch(state)
	{
	case PN532_STATE_RESTING:
		break;
	case PN532_STATE_ACK_WAIT:
		if (irqs)
		{
			state++; // = PN532_STATE_ACK_AVAIL
			irqs--;
		}
		break;
	case PN532_STATE_ACK_AVAIL:
		if (recvAck())
		{
			printf("Ack failed.\n");
			return(1);
		}
		if (ackCallback) // TODO: re evaluate this.
		{
			ackCallback();
		}
		state++; // = PN532_STATE_CMD_WAIT
		break;
	case PN532_STATE_CMD_WAIT:
		if (irqs)
		{
			state++; // = PN532_STATE_CMD_AVAIL;
			irqs--;
		}
		break;
	case PN532_STATE_CMD_AVAIL:
		if (recvResp())
		{
			printf("Recv failed.\n");
			return(1);
		}
		state++; // = PN532_STATE_CALLBACK;
		break;
	case PN532_STATE_CALLBACK:
		state = PN532_STATE_RESTING;
		return(callback(pn532_recvBuffer, pn532_recvLen));
		break;
	default:
		printf("Default switch statement reached\n");
		break;
	}

	if (irqs > 2)
	{
		printf("pn532 variable irqs is weird: %d\n", irqs);
	}

	return(0);
}

uint8_t pn532_blockForCallback()
{
	uint8_t retval;
	OCR1A = 2;
	while(state != PN532_STATE_RESTING)
	{
		_delay_ms(1);
		if ((retval = pn532_poll()))
			break;
	}
	OCR1A = 0;
	return(retval);
}

uint8_t pn532_blockForAck()
{
	uint8_t retval;
	OCR1A = 2;
	while(state != PN532_STATE_CMD_WAIT)
	{
		_delay_ms(1);
		//printf("state: %#x\n", state);
		if ((retval = pn532_poll()))
			break;
	}
	OCR1A = 0;
	return(retval);
}

void pn532_setAckCallback(uint8_t (* _callback)())
{
	ackCallback = _callback;
}

uint8_t pn532_getFirmwareVersion(uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_GETFIRMWAREVERSION, 0));
}

uint8_t pn532_getGeneralStatus(uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_GETGENERALSTATUS, 0));
}

uint8_t pn532_SAMConfiguration(uint8_t _mode, uint8_t _timeout, uint8_t _irq,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_SAMCONFIGURATION, 3, _mode, _timeout, _irq));
}

uint8_t pn532_powerDown(uint8_t _wakeUpEnable, uint8_t _generateIrq,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_POWERDOWN, 2, _wakeUpEnable, _generateIrq));
}

uint8_t pn532_RFConfiguration(uint8_t _cfgItem,
						uint8_t * _configurationData, uint8_t _configurationDataLen,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmdData(PN532_COMMAND_RFCONFIGURATION, _configurationData,
						_configurationDataLen, 1, _cfgItem));
}

uint8_t pn532_inListPassiveTarget(uint8_t _maxTg, uint8_t _BrTy,
						uint8_t * _initiatorData, uint8_t _initiatorDataLen,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmdData(PN532_COMMAND_INLISTPASSIVETARGET, _initiatorData,
						_initiatorDataLen, 2, _maxTg, _BrTy));
}

uint8_t pn532_inATR(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_INATR, 2, _tg, 0x00));
}

uint8_t pn532_inSelect(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_INSELECT, 1, _tg));
}

uint8_t pn532_inDeselect(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_INDESELECT, 1, _tg));
}

uint8_t pn532_inRelease(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_INRELEASE, 1, _tg));
}

uint8_t pn532_inDataExchange(uint8_t _tg,
						uint8_t * _dataOut, uint8_t _dataOutLen,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmdData(PN532_COMMAND_INDATAEXCHANGE, _dataOut, _dataOutLen, 1, _tg));
}

uint8_t pn532_inAutoPoll(uint8_t _pollNr, uint8_t _period, uint8_t _type1,
						uint8_t (* _callback)(uint8_t *, uint8_t))
{
	callback = _callback;
	return(sendCmd(PN532_COMMAND_INAUTOPOLL, 3, _pollNr, _period, _type1));
}

/**
 * Sends a command with arguments,
 * used for convenience.
 */
static uint8_t sendCmd(uint8_t cmd, uint8_t argCount, ...)
{
	// Variables
	uint8_t index;
	va_list args;

	// Initialization
	pn532_sendBuffer[0] = cmd;

	va_start(args, argCount);
	for (index = 1; index <= argCount; index++)
	{
		uint8_t byte = va_arg(args, int);
		pn532_sendBuffer[index] = byte;
	}
	va_end(args);

	return(writeCmdAck(pn532_sendBuffer, argCount+1));
}

/**
 * Sends a command with arguments,
 * and an arbitrary length data block.
 */
static uint8_t sendCmdData(uint8_t cmd, uint8_t * data, uint8_t dataLen, uint8_t argCount, ...)
{
	// Variables
	uint8_t len;
	uint8_t index;
	va_list args;

	// Initialization
	len = 1 + argCount + dataLen;
	pn532_sendBuffer[0] = cmd;

	va_start(args, argCount);
	for (index = 1; index <= argCount; index++)
	{
		uint8_t byte = va_arg(args, int);
		pn532_sendBuffer[index] = byte;
	}
	va_end(args);

	for (index = 0; index < dataLen; index++)
	{
		pn532_sendBuffer[1+argCount+index] = data[index];
	}

	return(writeCmdAck(pn532_sendBuffer, len));
}

/**
 * Reads and validates the Ack
 */
static uint8_t recvAck()
{
	// Variables
	uint8_t index;

	i2c_start(PN532_I2C_ADDRESS | I2C_READ); // Begin i2c read

	// Check 0x01 init byte (only present for i2c connections)
	pn532_recvBuffer[0] = i2c_read_ack();
	if (pn532_recvBuffer[0] != 0x1)
	{
		printf("\tAck: no 0x1 init byte\n");
		return(1); // ack failed
	}

	// Collect Ack
	for (index = 0; index < 6; index++)
	{
		if (index == 5)
			pn532_recvBuffer[index] = i2c_read_nack();
		else
			pn532_recvBuffer[index] = i2c_read_ack();

		//printf("\tack: %#x\n", pn532_recvBuffer[index]);
		if (pn532_recvBuffer[index] != pn532_ack[index])
		{
			printf("\tAck: data mismatch.\n0x%02X != 0x%02X\n", pn532_recvBuffer[index], pn532_ack[index]);
			return(1); // ack failed
		}
	}

	i2c_stop(); // End i2c transfer
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
	// Variables
	uint8_t checksum;
	uint8_t index;

	i2c_start(PN532_I2C_ADDRESS | I2C_READ); // Begin i2c read

	// Gather preamble values
	for (index = 0; index < 6; index++)
	{
		pn532_recvBuffer[index] = i2c_read_ack();
	}

	// List preamble values
//	printf("\tReady bit: %#x\n", pn532_recvBuffer[0]);
//	printf("\tPreamble: %#x\n", pn532_recvBuffer[1]);
//	printf("\tStart Code: %#x %#x\n", pn532_recvBuffer[2], pn532_recvBuffer[3]);
//	printf("\tLength: %#x %#x\n", pn532_recvBuffer[4], pn532_recvBuffer[5]);

	// Check preamble values
	if (pn532_recvBuffer[0] != 0x1)
	{
		printf("Recv: no 0x01 ready bit.\n");
		return(1); // failed
	}
	if (pn532_recvBuffer[1] != 0x00)
	{
		printf("Recv: preamble wrong.\n");
		return(1); // failed
	}
	if (pn532_recvBuffer[2] != 0x00 || pn532_recvBuffer[3] != 0xFF)
	{
		printf("Recv: Start code wrong.\n");
		return(1); // failed
	}
	if (((pn532_recvBuffer[4] + pn532_recvBuffer[5]) & 0xFF) != 0x00)
	{
		printf("Recv: length checksum wrong.\n");
		return(1); // failed
	}

	// Move length value to variable
	pn532_recvLen = pn532_recvBuffer[4];

	// Store main message in recv buffer
	for (index = 0; index < pn532_recvLen; index++)
	{
		pn532_recvBuffer[index] = i2c_read_ack();
//		printf("Recv: %#x\n", pn532_recvBuffer[index]);
	}

	// Collect data checksum
	checksum = i2c_read_ack();
	for (index = 0; index < pn532_recvLen; index++)
	{
		checksum += pn532_recvBuffer[index];
	}
//	printf("\tData Checksum Result: %#x\n", checksum);
	if (checksum != 0x00)
	{
		printf("Recv: bad data checksum.\n");
		return(1); // failed
	}

	// Skip postamble and end transmission
	i2c_read_nack();
	i2c_stop();
	return(0);
}

/**
 * Writes a command, and kicks off state machine
 */
static uint8_t writeCmdAck(uint8_t * cmd, uint8_t len)
{
	if (state != PN532_STATE_RESTING) // system is busy
	{
		return(1);
	}

	state = PN532_STATE_ACK_WAIT;

	return(writeCmd(cmd, len));
}

/**
 * Writes a command to I2C
 */
static uint8_t writeCmd(uint8_t * cmd, uint8_t len)
{
	// Variables
	uint8_t checksum;
	uint8_t err;
	uint8_t index;

	if (state && state != PN532_STATE_ACK_WAIT) // system is busy
	{
		return(state);
	}

//	printf("Writing command: ");
//	for (uint8_t i = 0; i < len; i++)
//	{
//		printf("%#x, ", cmd[i]);
//	}
//	printf("\n");

	// Initialize
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

	for (index = 0; index < (len-1); index++) {
		err += i2c_write(cmd[index]);
		checksum += cmd[index];
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
	irqs++;
}














