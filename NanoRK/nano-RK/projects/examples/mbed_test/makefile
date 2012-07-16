# mbed offline compilation-specific parameters
PROJECT=LEDBlink
GCC4MBED_DIR=/Users/dbanks/mbed_offline
LIBS_PREFIX=
LIBS_SUFFIX=
GCC4MBED_DELAYED_STDIO_INIT=1

# Platform name  cc2420DK, firefly, micaZ, firefly2_1, firefly2_2, firefly2_3
PLATFORM = mbed

# Target file name (without extension).
TARGET = main

# Set the Port that you programmer is connected to 
PROGRAMMING_PORT = /Volumes/MBED

# Set this such that the nano-RK directory is the base path
NANORK_DIR = $(GCC4MBED_DIR)/NanoRK/nano-RK

# Set platform specific defines 
# The following will be defined based on the PLATFORM variable:
# PROG_TYPE  (e.g. avrdude, or uisp)
# MCU (e.g. atmega32, atmega128, atmega1281) 
# RADIO (e.g. cc2420)
include $(NANORK_DIR)/include/platform.mk


# Add extra source files. 
# For example:
# CSRCS += $(ROOT_DIR)/src/platform/$(PLATFORM_TYPE)/source/my_src1.c 
# CSRCS += $(ROOT_DIR)/src/platform/$(PLATFORM_TYParm-none-eabi-gccE)/source/my_src2.c 

# Add extra includes files. 
# For example:
# EXTRAINCDIRS += $(ROOT_DIR)/src/platform/include
EXTRAINCDIRS = 


#  This is where the final compile and download happens
include ~/mbed_offline/build/gcc4mbed.mk
