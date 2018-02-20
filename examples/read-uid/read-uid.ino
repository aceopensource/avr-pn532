/**
 * AVR-PN532 Read-UID example
 * Christopher Bero <berocs@acedb.co>
 *
 * This example project should read UIDs from
 * ISO14443A type NFC cards and print them as hex 
 * over UART at 38400 8N1
 * 
 * This example project should compile in Arduino IDE
 * with a little bit of configuration.
 * 
 * Requires: 
 * 
 * - avr-i2c to communicate with pn532
 * -- https://github.com/aceopensource/avr-i2c
 * 
 * - avr-printf to print
 * -- https://github.com/aceopensource/avr-printf
 * 
 * - avr-usart to print
 * -- https://github.com/aceopensource/avr-usart
 * 
 * - and - of course - avr-pn532
 * -- https://github.com/aceopensource/avr-pn532
 * 
 * To use these libraries with Arduino IDE,
 * copy/clone them into your sketchbook/libraries/ directory.
 * See: https://www.arduino.cc/en/Guide/Libraries#toc5
 * 
 * Note that this source file doesn't really look like a
 * normal Arduino project. That is because it tries to
 * use regular C as much as possible, making compiling
 * outside of Arduino IDE easier.
 * 
 * Hardware:
 * 
 * The PN532 needs to be in I2C mode,
 * and wired into the I2C lines of the atmega.
 * Additionally, the IRQ line from the PN532
 * is connected to pin 3 (aka INT1, aka PD3).
 * 
 * I2C uses the ATmega's
 * default I2C pins (27&28 on the DIP) to reach
 * the NFC board. Make sure these pins are wired.
 * Both of these pins also require a pull-up resistor
 * because neither the ATmega nor PN532 boards have
 * the hardware.
 * Put a 1.5K resistor to VCC on each line.
 */

#include "avr-printf.h"
#include "avr-pn532.h"
#include "avr-i2c.h"
#include "avr-usart.h"

// Application specific settings / macros
#define NFC_POLLING_NUM 100 // Number of polling periods before returning.
#define NFC_POLLING_LEN 1 // Polling period, in units of 150ms.
#define NFC_POLLING_TIMEOUT (NFC_POLLING_NUM*NFC_POLLING_LEN*150) // In milliseconds.

// State definitions
#define MAIN_STATE_INIT     (0x00)
#define MAIN_STATE_AUTOPOLL   (0x01)

static uint8_t state; // The global state for the main application

uint8_t nbTg; // Number of targets
//uint8_t type1 = response[3];
uint8_t nfcid_len;
uint8_t nfcid[7];

uint8_t cb_firmwareVersion(uint8_t * response, uint8_t len)
{
    if (len != 6)
    {
        return(1);
    }
    printf("Firmware: %d.%d\n", response[3], response[4]);
    return(0);
}

uint8_t cb_generalStatus(uint8_t * response, uint8_t len)
{
    printf("Gen Status:\n");
    printf("\tErr code: 0x%02X\n", response[2]);
    printf("\tField: 0x%02X\n", response[3]);
    printf("\t# tags: 0x%02X\n", response[4]);

    if (response[2] != 0)
    {
        printf("Error, restarting.\n");
        _delay_ms(250);
        pn532_getGeneralStatus(cb_generalStatus);
        pn532_blockForCallback();
    }
    return(0);
}

uint8_t cb_SAMConfiguration(uint8_t * response, uint8_t len)
{
    printf("SAM config done.\n");
    return(0);
}

uint8_t cb_inAutoPoll(uint8_t * response, uint8_t len)
{
    if (len != 17 && len != 14)
    {
//    led_set(10, 10, 0);
        nbTg = 0;
    printf("Len %d is not 17 or 14, no card available.\n", len);
//    _delay_ms(2);
    }
    nbTg = response[2];
    //type1 = response[3];
    nfcid_len = response[9];

    printf("%d cards available.\nUID: ", nbTg);
    for (uint8_t i = 0; i < nfcid_len; i++)
    {
        nfcid[i] = response[i+10];
        printf("0x%02X ", nfcid[i]);
    }
    printf("\n");
    return (0);
}

uint8_t procForCallback()
{
    uint8_t retval;
//    printf("\tProcessing pn532.\n");
  pn532_poll();
    retval = pn532_getState();
    return(retval);
}

/* Init
 * This function is kept over from the Arduino library for no reason.
 * I have arbitrarily decided that it will be used to initialize
 * low level hardware.
 */
void init ()
{
  // No special low-level hardware to configure in this example.
}

/**
 * setup() configures whatever isn't handled in init()
 */
void setup()
{
    // Printf setup
    usart_init();
    init_printf(NULL, usart_putchar);
    printf("usart init.\n");

    state = MAIN_STATE_INIT;

    i2c_init();
    printf("i2c init.\n");

    pn532_init(PN532_ASYNC);
    printf("pn532 init.\n");

    if (pn532_getFirmwareVersion(cb_firmwareVersion))
    {
        printf("init, pn532_getFirmwareVersion failed.\n");
    }
    if (pn532_blockForCallback())
    {
        printf("init, pn532_blockForCallback failed.\n");
    }

    pn532_SAMConfiguration(1, 0, 1, cb_SAMConfiguration);
    pn532_blockForCallback();

    pn532_getGeneralStatus(cb_generalStatus);
    pn532_blockForCallback();
}

/* Loop
 * Does the main task.
 */
void loop()
{
    // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
    // 'uid' will be populated with the UID, and uidLength will indicate
    // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
    // Variables
    uint8_t procStatus;

    // Instantiation
    procStatus = procForCallback();

//    printf("procstatus: %d, state: %d\n", procStatus, state);

    if (procStatus && (state == MAIN_STATE_AUTOPOLL))
    {
//      printf("Nothing to do in proc_pn532.\n");
        return;
    }
    else if (state == MAIN_STATE_AUTOPOLL)
    {
//      printf("Done processing for callback, moving along.\n");
      state = MAIN_STATE_INIT;
    }

    uint8_t retval;
    if((retval = pn532_inAutoPoll(NFC_POLLING_NUM, NFC_POLLING_LEN, 0x10, cb_inAutoPoll)))
    {
        printf("pn532_inAutoPoll failed: 0x%02X\n", retval);
        pn532_recover();
        state = MAIN_STATE_INIT;
    }
    
    state = MAIN_STATE_AUTOPOLL;
    printf("Waiting for card.\n");
}

/* Main
 * This function is used to override main() provided by the Arduino library.
 * It simply structures the other high-level functions.
 */
int main(void)
{
    init();

    setup();

    for (;;)
    {
        loop();
        _delay_ms(10);
    }

    return 0;
}



