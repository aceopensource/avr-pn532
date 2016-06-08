/**
 * PN532 I2C Library
 * Christopher Bero <berocs@acedb.co>
 */

#ifndef PN532_H
#define PN532_H

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "pn532_commands.h"
#include "i2c.h"

void pn532_init(void);
uint8_t pn532_poll(void);
uint8_t pn532_recvGetFirmwareVersion();

//uint8_t getField(void);
//uint8_t SAMConfig(void);
//uint32_t getFirmwareVersion(void);
//uint32_t readPassiveTargetID(uint8_t cardbaudrate);
//uint8_t readMemoryBlock(uint8_t cardnumber /* 1 or 2 */,
//					uint8_t blockaddress /* 0 to 63 */,
//					uint8_t* block);
//uint8_t writeMemoryBlock(uint8_t cardnumber /* 1 or 2 */,
//					uint8_t blockaddress /* 0 to 63 */,
//					uint8_t* block);

#endif // PN532_H
