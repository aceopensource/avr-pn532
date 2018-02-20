#!/bin/bash

# This is the script I use to flash projects built with Arduino IDE,
# because my bootloader doesn't play nice.
# If you can flash from Arduino IDE, do that instead.

# Example usage:
# ./flash.sh /dev/ttyUSB3 38400 /tmp/arduino_build_235762/read-uid.ino.hex

avrdude -v -e -c arduino -P $1 -p atmega328p -b $2 -U flash:w:${3}.hex:i
