/**
 * PN532 Asynchronous I2C Library
 * Christopher Bero <berocs@acedb.co>
 */

#ifndef PN532_H
#define PN532_H 1

// Required header files for public end-use
#include "pn532_commands.h"
#include "printf.h"
//#include <stdio.h>

#ifndef NULL
#define NULL ((void *) 0)
#endif

#define true 1
#define false 0

// Emulation
#define NDEF_MAX_LENGTH 1282 // 1280
#define tagWriteable false

extern uint8_t pn532_async;

extern uint8_t ndef_file[];
extern const uint8_t ndef_file_base_len;

void pn532_tg_setPayloadLength(uint8_t _payload_len);
int pn532_tg_emulateTag();
void pn532_tg_setNdefFileLength(uint8_t _ndef_file_len);
uint8_t pn532_tg_getNdefFileLength();
void pn532_tg_setNdefBytesFunction(void (* _ndef_next_bytes_ptr)(uint8_t *, uint8_t, uint8_t));

uint8_t pn532_emulateTag(uint8_t _len_payload, void (* _ndef_next_bytes_ptr)(uint8_t *, uint8_t, uint8_t));

// Public (external usage) functions
// Always prepended with pn532_
void pn532_init(uint8_t async);
void pn532_recover();
uint8_t pn532_poll(void);
uint8_t pn532_blockForCallback();
uint8_t pn532_blockForCallback_timeout(uint16_t timeout);
uint8_t pn532_blockForAck();
void pn532_setAckCallback(uint8_t (* _callback)());
uint8_t pn532_getFirmwareVersion(uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_getGeneralStatus(uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_SAMConfiguration(uint8_t _mode, uint8_t _timeout, uint8_t _irq,
                               uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_powerDown(uint8_t _wakeUpEnable, uint8_t _generateIrq,
                        uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_RFConfiguration(uint8_t _cfgItem,
                              uint8_t * _configurationData, uint8_t _configurationDataLen,
                              uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_inListPassiveTarget(uint8_t _maxTg, uint8_t _BrTy,
                                  uint8_t * _initiatorData, uint8_t _initiatorDataLen,
                                  uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_inATR(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_inSelect(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_inDeselect(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_inRelease(uint8_t _tg, uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_inDataExchange(uint8_t _tg,
                             uint8_t * _dataOut, uint8_t _dataOutLen,
                             uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_inAutoPoll(uint8_t _pollNr, uint8_t _period, uint8_t _type1,
                         uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_tgInitAsTarget(uint8_t _mode,
                             uint8_t * _dataOut, uint8_t _dataOutLen,
                             uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_tgGetData(uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_tgSetData(uint8_t * _dataOut, uint8_t _dataOutLen,
                        uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_tgResponseToInitiator(uint8_t * _dataOut, uint8_t _dataOutLen,
                                    uint8_t (* _callback)(uint8_t *, uint8_t));
uint8_t pn532_tgGetInitiatorCommand(uint8_t (* _callback)(uint8_t *, uint8_t));

#endif // PN532_H
