CC=gcc
CFLAGS=-c -Wall
LIBS=-lwiringPi -pthread

all: gateway

gateway: radio.o main.o network.o
	$(CC) radio.o main.o network.o $(LIBS) -o gateway

send: send.c radio.o
	$(CC) radio.o send.c $(LIBS) -o send

recv: recv.c lora.o
	$(CC) lora.o recv.c $(LIBS) -o recv

main.o: main.c
	$(CC) $(CFLAGS) main.c

radio.o: radio.c radio.h config.h sx1278.h
	$(CC) $(CFLAGS) radio.c

heap.o: heap.c heap.h
	$(CC) $(CFLAGS) heap.c

network.o: network.c network.h
	$(CC) $(CFLAGS) network.c

clean:
	rm -f *.o
	rm -f send
	rm -f recv
	rm -f gateway

lora: lora.c lora.h
	$(CC) $(CFLAGS) lora.c
