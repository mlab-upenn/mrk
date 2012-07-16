PCF Example
-----------

Below is a description of each directory
ff-clients:	Firmware to run on each FireFly node client
pcf-host:	Firmware to run on FireFly gateway node MAC_ADR=0x00000000
slip-client:	Software running on PC that communicates over SLIP to the network
common:		Contains common packet file types.

Notes
-----
Gateway MAC must be 0x00000000.  Node slots is based off of MAC address.  By default, network support only 64 nodes, so don't exceed MAC address 0x00000040.
	
