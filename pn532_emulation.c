/*
Software License Agreement (BSD License)

Copyright (c) 2012, Adafruit Industries
Copyright (c) 2013, Seeed Technology Inc.
Copyright (c) 2016, ACE Creative Engagement
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <util/delay.h>
#include <string.h>
#include "pn532.h"

#ifndef NDEF_MAX_LENGTH
#define NDEF_MAX_LENGTH 32
#endif

#ifndef tagWriteable
#define tagWriteable false
#endif

// emulation variables
typedef enum {COMMAND_COMPLETE, TAG_NOT_FOUND, FUNCTION_NOT_SUPPORTED, MEMORY_FAILURE, END_OF_FILE_BEFORE_REACHED_LE_BYTES} responseCommand;
typedef enum { NONE, CC, NDEF } tag_file;   // CC -> Compatibility Container

const uint8_t command[] =
{
//	0x05, 				// MODE: PICC only, Passive only

	0x04, 0x00,         // SENS_RES
	0x21, 0x43, 0x65,   // NFCID1
	0x20,               // SEL_RES

	0x01, 0xFE,         // Parameters to build POL_RES
	0xA2, 0xA3, 0xA4,
	0xA5, 0xA6, 0xA7,
	0xC0, 0xC1, 0xC2,
	0xC3, 0xC4, 0xC5,
	0xC6, 0xC7, 0xFF,
	0xFF,
	0xAA, 0x99, 0x88, 	//NFCID3t (10 bytes)
	0x77, 0x66, 0x55, 0x44,
	0x33, 0x22, 0x11,

	0, // length of general bytes
	0  // length of historical bytes
};

uint8_t ndef_file[] =
{
	0x00, 0x00,
	0xD1,		// Header: b11010001 -> [MB] [ME] [CF] [SR] [IL] [..  TNF  ..]
	0x01,		// type length -> 1
	0x1F,		// payload length -> 0
	0x54,		// type -> URI Record (0x55) or Text File (0x54)
	0x02		// payload URI ID code (0x04 -> https://) or language code (0x02 -> "en")
};
const uint8_t ndef_file_base_len = 5;
uint8_t ndef_file_len; // First two bytes don't count.

const uint8_t compatibility_container[] =
{
	0x00, 0x0F, // CCLEN: Size of this CC. Values between 0x000F and 0xFFFE
	0x20, 		// Mapping version (???)
	0x00, 0x0F, // 0x54 MLe: Maximum data size that can be read using a single ReadBinary command. MLe = 000Fh-FFFFh
	0x00, 0x0F, // MLc: Maximum data size that can be sent using a single UpdateBinary command. MLc = 0001h-FFFFh
	// T & L of NDEF File Control TLV, followed by 6 bytes of V.
	0x04,       // T
	0x06,       // L
	0xE1, 0x04, // File Identifier of NDEF File
	((NDEF_MAX_LENGTH & 0xFF00) >> 8), (NDEF_MAX_LENGTH & 0xFF), // maximum NDEF file size
	0x00,       // read access 0x0 = granted | 0xFF = deny
	0xFF        // write access 0x0 = granted | 0xFF = deny
};
uint16_t cc_size = sizeof(compatibility_container);

uint8_t payload_len;
uint8_t rwbuf[18];
uint8_t sendlen;
int16_t status;
tag_file currentFile = NONE;
void (* ndef_next_bytes_ptr)(uint8_t *, uint8_t, uint8_t);

// Get and Set functions

void pn532_tg_setPayloadLength(uint8_t _payload_len)
{
	payload_len = _payload_len;
	ndef_file_len = ndef_file_base_len + payload_len - 1; // ndef type byte already counted.
//	printf("ndef file size: %d\n", ndef_file_len);
	ndef_file[0] = (ndef_file_len & 0xFF00) >> 8;
	ndef_file[1] = ndef_file_len & 0xFF;
	ndef_file[4] = payload_len;
}

void pn532_tg_setNdefFileLength(uint8_t _ndef_file_len)
{
	ndef_file_len = _ndef_file_len;
}

uint8_t pn532_tg_getNdefFileLength()
{
	return(ndef_file_len);
}

void pn532_tg_setNdefBytesFunction(void (* _ndef_next_bytes_ptr)(uint8_t *, uint8_t, uint8_t))
{
	ndef_next_bytes_ptr = _ndef_next_bytes_ptr;
}

uint8_t pn532_tg_cb_getData(uint8_t * response, uint8_t len)
{
    printf("callback: tgGetData.\n");

    status = response[2];
    printf("\tstatus: 0x%02X\n", status);

    printf("\t");
    for (int i = 3; i < len; i++)
    {
        rwbuf[i-3] = response[i];
        printf("0x%02X ", response[i]);
    }
    printf("\n");

    if (rwbuf[1] != 0xA4)
	{
		return 0;
	}

    printf("\tCLA: 0x%02X\n", rwbuf[0]);
    printf("\tINS: 0x%02X\n", rwbuf[1]);
    printf("\tP1:  0x%02X\n", rwbuf[2]);
    printf("\tP2:  0x%02X\n", rwbuf[3]);
    printf("\tLc:  0x%02X\n", rwbuf[4]);
    printf("\tData: ");
    for (int i = 5; i < (5+rwbuf[4]); i++)
    {
        printf("0x%02X ", rwbuf[i]);
    }
    printf("\n\tLe:  0x%02X\n", rwbuf[5+rwbuf[4]]);

    return 0;
}

uint8_t pn532_tg_cb_setData(uint8_t * response, uint8_t len)
{
    printf("callback: tgSetData.\n");

    if (len >= 2) // There, now len is used.
		status = response[2];

    printf("\tstatus: 0x%02X\n", status);

    return 0;
}

uint8_t pn532_tg_cb_initAsTarget(uint8_t * response, uint8_t len)
{
    printf("tgInitAsTarget. Length: %d\n", len);
    uint8_t mode = response[2];

    printf("\tMode: \n\tBaud: ");
    if (mode & (1 << 5))
    {
        printf("424kbps, ");
    }
    else if (mode & (1 << 4))
    {
        printf("212kbps, ");
    }
    else
    {
        printf("106kbps, ");
    }

    printf("14443-4: ");
    if (mode & (1 << 3))
    {
        printf("yes, ");
    }
    else
    {
        printf("no, ");
    }

    printf("DEP: ");
    if (mode & (1 << 2))
    {
        printf("yes, ");
    }
    else
    {
        printf("no, ");
    }

    printf("Framing type: ");
    if (mode & (1 << 1))
    {
        printf("FeliCa.\n");
    }
    else if (mode & (1))
    {
        printf("Active mode.\n");
    }
    else
    {
        printf("Mifare.\n");
    }

    printf("\n");
    printf("Initiator command: \n");

    for (uint8_t i = 3; i < len; i++)
    {
        printf("0x%02X\n", response[i]);
    }

    return(0);
}

void pn532_tg_setResponse(responseCommand cmd, uint8_t* buf, uint8_t* sendlen, uint8_t sendlenOffset)
{
    switch(cmd)
    {
    case COMMAND_COMPLETE:
        buf[0] = R_APDU_SW1_COMMAND_COMPLETE;
        buf[1] = R_APDU_SW2_COMMAND_COMPLETE;
        *sendlen = 2 + sendlenOffset;
        break;
    case TAG_NOT_FOUND:
        buf[0] = R_APDU_SW1_NDEF_TAG_NOT_FOUND;
        buf[1] = R_APDU_SW2_NDEF_TAG_NOT_FOUND;
        *sendlen = 2;
        break;
    case FUNCTION_NOT_SUPPORTED:
        buf[0] = R_APDU_SW1_FUNCTION_NOT_SUPPORTED;
        buf[1] = R_APDU_SW2_FUNCTION_NOT_SUPPORTED;
        *sendlen = 2;
        break;
    case MEMORY_FAILURE:
        buf[0] = R_APDU_SW1_MEMORY_FAILURE;
        buf[1] = R_APDU_SW2_MEMORY_FAILURE;
        *sendlen = 2;
        break;
    case END_OF_FILE_BEFORE_REACHED_LE_BYTES:
        buf[0] = R_APDU_SW1_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
        buf[1] = R_APDU_SW2_END_OF_FILE_BEFORE_REACHED_LE_BYTES;
        *sendlen= 2;
        break;
    }
}

int pn532_tg_emulateTag()
{
    printf("Initiating as target.\n");
    pn532_tgInitAsTarget(0x05, command, 36, pn532_tg_cb_initAsTarget);
    if (pn532_blockForCallback_timeout(20000))
	{
		return(1); // timeout reached or pn532_poll() returned failure.
	}

    //  tagWrittenByInitiator = false;

    printf("Entering tg loop.\n");
    while(1)
    {
        //    status = pn532.tgGetData(rwbuf, sizeof(rwbuf));
        //    if(status < 0){
        //      DMSG("tgGetData failed!\n");
        //      pn532.inRelease();
        //      return true;
        //    }
        pn532_tgGetData(pn532_tg_cb_getData);
        pn532_blockForCallback();

        if (status != 0)
		{
			_delay_ms(2000);
			break; // don't continue emulation.. return to main program.
			printf("Initiating as target.\n");
			pn532_tgInitAsTarget(0x05, command, 36, pn532_tg_cb_initAsTarget);
			pn532_blockForCallback();
			continue;
		}

        uint8_t p1 = rwbuf[C_APDU_P1];
        uint8_t p2 = rwbuf[C_APDU_P2];
        uint8_t lc = rwbuf[C_APDU_LC];
        uint16_t p1p2_length = ((int16_t) p1 << 8) + p2;

        printf("Operation: ");
        switch(rwbuf[C_APDU_INS])
        {
        case ISO7816_SELECT_FILE:
        	printf("select file: ");
            switch(p1)
            {
            case C_APDU_P1_SELECT_BY_ID:
            	printf("select by id\n");
                if(p2 != 0x0c)
                {
                    printf("C_APDU_P2 != 0x0c\n");
                    pn532_tg_setResponse(COMMAND_COMPLETE, rwbuf, &sendlen, 0);
                }
                else if(lc == 2 && rwbuf[C_APDU_DATA] == 0xE1 && (rwbuf[C_APDU_DATA+1] == 0x03 || rwbuf[C_APDU_DATA+1] == 0x04))
                {
                    pn532_tg_setResponse(COMMAND_COMPLETE, rwbuf, &sendlen, 0);
                    if(rwbuf[C_APDU_DATA+1] == 0x03)
                    {
                        currentFile = CC;
                    }
                    else if(rwbuf[C_APDU_DATA+1] == 0x04)
                    {
                        currentFile = NDEF;
                    }
                }
                else
                {
                    pn532_tg_setResponse(TAG_NOT_FOUND, rwbuf, &sendlen, 0);
                }
                break;
            case C_APDU_P1_SELECT_BY_NAME: ; // empty statement
				printf("select by name\n");
                const uint8_t ndef_tag_application_name_v2[] = {0, 0x7, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01 };
                if(0 == memcmp(ndef_tag_application_name_v2, rwbuf + C_APDU_P2, sizeof(ndef_tag_application_name_v2)))
                {
                    pn532_tg_setResponse(COMMAND_COMPLETE, rwbuf, &sendlen, 0);
                }
                else
                {
                    printf("function not supported\n");
                    pn532_tg_setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen, 0);
                }
                break;
            }
            break;
        case ISO7816_READ_BINARY:
        	printf("read binary: ");
            switch(currentFile)
            {
            case NONE:
            	printf("tag not found\n");
                pn532_tg_setResponse(TAG_NOT_FOUND, rwbuf, &sendlen, 0);
                break;
            case CC:
            	printf("cc\n");
                if( p1p2_length > NDEF_MAX_LENGTH)
                {
                    pn532_tg_setResponse(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rwbuf, &sendlen, 0);
                }
                else
                {
                    memcpy(rwbuf,compatibility_container + p1p2_length, lc);
                    pn532_tg_setResponse(COMMAND_COMPLETE, rwbuf + lc, &sendlen, lc);
                }
                break;
            case NDEF:
            	printf("ndef\n");
                if( p1p2_length > NDEF_MAX_LENGTH)
                {
                    pn532_tg_setResponse(END_OF_FILE_BEFORE_REACHED_LE_BYTES, rwbuf, &sendlen, 0);
                }
                else
                {
                	printf("memcpy: p1p2_length: %d, lc: %d\n", p1p2_length, lc);
//                    memcpy(rwbuf, ndef_file + p1p2_length, lc);
//					ndef_next_bytes(p1p2_length, lc);
					ndef_next_bytes_ptr(rwbuf, p1p2_length, lc);
                    pn532_tg_setResponse(COMMAND_COMPLETE, rwbuf + lc, &sendlen, lc);
//                    for (int i = 0; i < lc; i++)
//					{
//						printf("rwbuf[%d]: 0x%02X\n", i, rwbuf[i]);
//					}
                }
                break;
            }
            break;
        case ISO7816_UPDATE_BINARY:
        	printf("update binary: ");
            if(!tagWriteable)
            {
                pn532_tg_setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen, 0);
            }
            else
            {
                if( p1p2_length > NDEF_MAX_LENGTH)
                {
                    pn532_tg_setResponse(MEMORY_FAILURE, rwbuf, &sendlen, 0);
                }
                else
                {
                    memcpy(ndef_file + p1p2_length, rwbuf + C_APDU_DATA, lc);
                    pn532_tg_setResponse(COMMAND_COMPLETE, rwbuf, &sendlen, 0);
//					tagWrittenByInitiator = true;

//                    uint16_t ndef_length = (ndef_file[0] << 8) + ndef_file[1];
//					if ((ndef_length > 0) && (updateNdefCallback != 0)) {
//						updateNdefCallback(ndef_file + 2, ndef_length);
//					} // callback function
                }
            }
            break;
        default:
			  printf("Command not supported!");
			  printf("0x%02X\n", rwbuf[C_APDU_INS]);
            pn532_tg_setResponse(FUNCTION_NOT_SUPPORTED, rwbuf, &sendlen, 0);
        }
        //    status = pn532.tgSetData(rwbuf, sendlen);
        //    if(status < 0){
        //      DMSG("tgSetData failed\n!");
        //      pn532.inRelease();
        //      return true;
        //    }
        for (int i = 0; i < sendlen; i++)
		{
			printf("tgSetData[%d]: 0x%02X\n", i, rwbuf[i]);
		}
        pn532_tgSetData(rwbuf, sendlen, pn532_tg_cb_setData);
        pn532_blockForCallback();
    }
    //  pn532.inRelease();
    return 0;
}
