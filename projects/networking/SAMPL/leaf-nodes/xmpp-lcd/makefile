# Platform name  cc2420DK, firefly2_1, firefly2_2, micaZ
PLATFORM = firefly2_2


# Target file name (without extension).
TARGET = main

# Set the Port that you programmer is connected to 
PROGRAMMING_PORT = /dev/ttyUSB1 # programmer connected to serial device

# Set this such that the nano-RK directory is the base path
ROOT_DIR = ../../../../..
SAMPL_ROOT = $(ROOT_DIR)/src/net/sampl

# Set platform specific defines 
# The following will be defined based on the PLATFORM variable:
# PROG_TYPE  (e.g. avrdude, or uisp)
# MCU (e.g. atmega32, atmega128, atmega1281) 
# RADIO (e.g. cc2420)
include $(ROOT_DIR)/include/platform.mk


SRC = $(TARGET).c
SRC += $(SAMPL_ROOT)/include/pkt_packer.c
SRC += $(SAMPL_ROOT)/pkt-handlers/xmpp_pkt.c
SRC += $(SAMPL_ROOT)/pkt-handlers/ack_pkt.c
SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/ff_lcd.c
SRC += $(ROOT_DIR)/src/drivers/platform/$(PLATFORM_TYPE)/source/lcd_driver.c


# Add extra source files. 
# For example:
SRC += $(ROOT_DIR)/src/net/bmac/bmac.c


# Add extra includes files. 
# For example:
EXTRAINCDIRS =
EXTRAINCDIRS += $(ROOT_DIR)/src/net/bmac
EXTRAINCDIRS += $(SAMPL_ROOT)/include
EXTRAINCDIRS += $(SAMPL_ROOT)/pkt-handlers
EXTRAINCDIRS += $(SAMPL_ROOT)/sampl-core

#  This is where the final compile and download happens
include $(ROOT_DIR)/include/platform/$(PLATFORM)/common.mk
