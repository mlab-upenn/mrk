#!/bin/bash

if [ ! -e ../../../../tools/EEPROM_mac_set/config-eeprom ]; then
    echo "config-eeprom not found!"
    echo "Please run 'make' in \$(ROOT_DIR)/tools/EEPROM_mac_set/"
    exit
fi

make clean
make program
../../../../tools/EEPROM_mac_set/config-eeprom /dev/ttyUSB1 00000002 15
