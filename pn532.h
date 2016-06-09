/**
 * PN532 I2C Library
 * Christopher Bero <berocs@acedb.co>
 */

#ifndef PN532_H
#define PN532_H

// Required header files for public end-use
#include "pn532_commands.h"

// Public (external usage) functions
// Always prepended with pn532_
void pn532_init(void);
uint8_t pn532_poll(void);

#endif // PN532_H
