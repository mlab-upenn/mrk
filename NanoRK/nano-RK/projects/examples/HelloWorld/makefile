# Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT=HelloWorld
GCC4MBED_DIR=/Users/dbanks/mbed_offline
LIBS_PREFIX=
LIBS_SUFFIX=
GCC4MBED_DELAYED_STDIO_INIT=1

# Platform name  cc2420DK, firefly, micaZ, firefly2_1, firefly2_2, firefly2_3
PLATFORM = mbed

# main file
TARGET = main

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
include $(GCC4MBED_DIR)/build/gcc4mbed.mk
