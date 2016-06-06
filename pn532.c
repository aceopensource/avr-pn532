/**
 * PN532 I2C Library
 * Christopher Bero <berocs@acedb.co>
 */

#include "pn532.h"
#include "usart.h"

const uint8_t pn532_ack[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
const uint8_t pn532_firmware_version_response[] = {0x00, 0xFF, 0x06, 0xFA, 0xD5, 0x03};

#define PN532_BUFFER_SIZE	32
uint8_t pn532_sendBuffer[PN532_BUFFER_SIZE];
uint8_t pn532_recvBuffer[PN532_BUFFER_SIZE];

uint8_t state;
/* States
 * Waiting for something is an odd number
 * Ready to process is an even number
 * 0: not waiting on ack, default
 * 1: waiting on ack
 * 2: ack received, not processed
 */

ISR(INT1_vect)
{
	printf("Processing interrupt!\n\r");
	if (state == 1)
		state = 2;
}

void pn532_init(void)
{
	// Set initial state
	state = 0;

	cli(); // Disable interrupts

	// Setup INT1
	EICRA |= (1<<ISC11) | (0<<ISC10); // Trigger INT1 on falling edge
	EIMSK |= 2; 		// Enable Interrupt Mask for Int1
	DDRD &= ~(1<<PD3); 	// Set PD3 as input
	PORTD |= (1<<PD3); 	// Enable PD3 pull-up resistor

	sei(); // Enable Interrupts

	// Send dummy packet to synchronize on
	pn532_sendBuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;
	pn532_writeCmdAck(pn532_sendBuffer, 1);
}

// manage interrupt events
uint8_t pn532_poll()
{
	if (state%2 == 1)
	{
		return 0; // Waiting on an interrupt
	}

	switch(state)
	{
	case 2:
		pn532_recvAck();
		break;
	}
}

uint8_t pn532_recvAck()
{
	state = 0;
	i2c_start(PN532_I2C_ADDRESS | I2C_READ);
	pn532_recvBuffer[0] = i2c_read_ack();
	if (pn532_recvBuffer[0] != 0x1)
	{
		printf("Ack failed, no 0x1 init byte\n\r");
		return(1); // ack failed
	}
	for (int c = 0; c < 6; c++)
	{
		if (c == 5)
			pn532_recvBuffer[c] = i2c_read_nack();
		else
			pn532_recvBuffer[c] = i2c_read_ack();

		printf("ack: %#x\n\r", pn532_recvBuffer[c]);
		if (pn532_recvBuffer[c] != pn532_ack[c])
		{
			printf("Ack failed\n\r");
			return(1); // ack failed
		}
	}
	i2c_stop();
	printf("Ack received successfully.\n\r");
	return(0);
}

uint8_t pn532_writeCmdAck(uint8_t * cmd, uint8_t len)
{
	uint16_t timer = 0;

	// TODO: make this nonblocking somehow
	while (state == 1) // already waiting for an ack
	{
		_delay_ms(2);
	}

	// Write command(s)
	pn532_writeCmd(cmd, len);
	state = 1;

	return(1);
}

uint8_t pn532_writeCmd(uint8_t * cmd, uint8_t len)
{
	uint8_t checksum;
	uint8_t output;

	output = 0;
	len = len + 1; // To account for PN532_HOSTTOPN532

	// TODO: make this nonblocking somehow
	while (state == 1) // already waiting for an ack
	{
		_delay_ms(2);
	}

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
}






