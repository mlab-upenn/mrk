ROOT_DIR = ../../../../../..
SAMPL_ROOT = $(ROOT_DIR)/src/net/sampl


CC=gcc
#CC=/home/soxuser/trunk/staging_dir/toolchain-mipsel_gcc-4.1.2_uClibc-0.9.30.1/usr/bin/mipsel-openwrt-linux-gcc
#CC=/home/soxuser/trunk/staging_dir/toolchain-mipsel_gcc-3.4.6_uClibc-0.9.30.1/usr/bin/mipsel-openwrt-linux-gcc
INCLUDE=-I. 
INCLUDE+=-I$(SAMPL_ROOT)/include/ 
INCLUDE+=-I$(SAMPL_ROOT)/pkt-handlers/  
INCLUDE+=-I../../../transducer-pkts/ 
INCLUDE+=-I$(ROOT_DIR)/tools/SLIPstream/SLIPstream-client/ 
INCLUDE+=-I../../common/



CFLAGS=-Wall $(INCLUDE)
SOURCES=main.c 
SOURCES+=$(ROOT_DIR)/tools/SLIPstream/SLIPstream-client/slipstream.c 
SOURCES+=$(SAMPL_ROOT)/include/pkt_packer.c 
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/ack_pkt.c 
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/ping_pkt.c 
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/transducer_pkt.c
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/stats_pkt.c
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/neighbor_pkt.c

SOURCES+=../../../transducer-pkts/ff_basic_sensor_pkt.c 
SOURCES+=../../../transducer-pkts/ff_power.c 


OBJS=$(SOURCES:.c=.o)

all: ping 

default: ping 

ping: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf *~ $(OBJS) ping $(SAMPL_ROOT)/pkt-handlers/*.o ../../transducer-pkts/*.o 
