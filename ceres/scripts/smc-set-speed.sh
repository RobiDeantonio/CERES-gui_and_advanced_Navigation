#!/bin/bash
# Sets the speed of a Simple Motor Controller via its virtual serial port.
# Usage: smc-set-speed.sh DEVICE SPEED
# Linux example: bash smc-set-speed.sh /dev/ttyACM0 3200
# Mac OS X example: bash smc-set-speed.sh /dev/cu.usbmodemfa121 3200
# Windows example: bash smc-set-speed.sh '\\.\COM6' 3200
# DEVICE is the name of the virtual COM port device.
# SPEED is a number between -3200 and 3200
DEVICE=$1
SPEED=$2
byte() {
    printf "\\x$(printf "%x" $1)"
}
{
    byte 0x83 # exit safe-start
    if [ $SPEED -lt 0 ]; then
        byte 0x86 # motor reverse
        SPEED=$((-$SPEED))
    else
        byte 0x85 # motor forward
    fi
    byte $((SPEED & 0x1F))
    byte $((SPEED >> 5 & 0x7F))
} > $DEVICE