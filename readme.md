## PN532 Asynchronous I2C Library

AVR-libc based, nonblocking command library for operating a PN532 based NFC terminal.

Uses polling and function pointer callbacks to operate.

Has a spinwait method to create blocking, synchronous operation.

### Examples

Check in [examples/](examples/) for a small proof-of-concept example made for atmega328p and Arduino IDE.

For a full example of how avr-pn532 can be used out in the wild, see [megadoor](https://github.com/makerslocal/megadoor/tree/testing).
