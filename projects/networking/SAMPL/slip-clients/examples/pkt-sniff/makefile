ROOT_DIR = ../../../../../..
SAMPL_ROOT = $(ROOT_DIR)/src/net/sampl


CC=gcc
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
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/transducer_pkt.c
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/stats_pkt.c
SOURCES+=$(SAMPL_ROOT)/pkt-handlers/neighbor_pkt.c

SOURCES+=../../../transducer-pkts/ff_basic_sensor_pkt.c 
SOURCES+=../../../transducer-pkts/ff_power.c 
SOURCES+=../../common/pkt_debug.c 


OBJS=$(SOURCES:.c=.o)

all: pkt-sniff

default: pkt-sniff 

pkt-sniff: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf *~ $(OBJS) pkt-sniff $(SAMPL_ROOT)/pkt-handlers/*.o ../../transducer-pkts/*.o 
