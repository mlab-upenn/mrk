CC=gcc
CFLAGS=-I. -I../../common/

%.o: %.c 
	$(CC) -c -o $@ $< $(CFLAGS)

all: main.o slipstream.o ../../common/pkt.o
	gcc -o orb-ctrl main.o slipstream.o ../../common/pkt.o -I. -I../../common/
clean: 
	rm -f ../../common/*.o *.o *~ core orb-ctrl 

