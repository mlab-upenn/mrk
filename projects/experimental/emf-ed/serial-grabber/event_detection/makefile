CC=gcc
CFLAGS=-I.

%.o: %.c 
	$(CC) -c -o $@ $< $(CFLAGS)

all: edgeDetect 
	gcc -o edgeDetect edgeDetect.c -I.
clean: 
	rm -f *.o edgeDetect 

