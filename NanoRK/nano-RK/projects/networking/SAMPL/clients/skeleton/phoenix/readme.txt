***************************************************************************

                     PHOENIX WIRELESS UPDATE README

***************************************************************************

****************************************************************************
CLIENT
****************************************************************************

1) Changes to globals.h : 

added uint8_t ph_buf[256] buffer for pagewise flash write. 

2) Files: 

phoenix.c - Handles network functions (phoenix.h)
nanopatch.c - patch applier functions
bootloader.c - functions for writing/read from flash (bootloader.h)
flash.h - macros for flash read/write
./util/eepgen - program for generating .eep file

phoenix-client.mk - modified common.mk to create .bin and add bootloader

3) Compiling Client -

* make clean
* make
* ./util/eepgen <main.bin> <mac> <channel> // temporary (creates main.eep)
* make program

main.bin should be preserved for creating patch

****************************************************************************
PATCH
****************************************************************************

1) Creating patch:

* ./truncate main_old.bin gives trunc_main_old.bin (Truncate to remove bootloader)
* ./truncate main_new.bin gives trunc_main_new.bin
* ./nanodiff <trunc_main_old.bin> <trunc_main_new.bin> <patch.bin> (create patch for converting old to new)

Copy patch.bin to masters' folder.

****************************************************************************
MASTER
****************************************************************************

1) Changes to tree_route.h:

added #define UNKNOWN_PKT 0x0A for unknown pkt_type

2) Compiling Master:

* make clean
* ./util/binder <patch.bin> <1/2 Update Mode>
(To transfer patch to master using programmer)
(Update mode redundant, can be used for choosing fullimage/patch update)

* make
* make program

Master first sends the wireless update packet. Then switches to UNKNOWN_PKT

3) phoenix-master.mk - modified common.mk for serial patch transfer.











