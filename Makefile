CC=g++
CFLAGS=-c -Wall
LIBS=-lwiringPi

all: gateway

gateway: radio.o mac.o net.o main.o aes.o
	$(CC) radio.o mac.o net.o main.o aes.o $(LIBS) -o gateway

main.o: main.c
	$(CC) $(CFLAGS) main.c

mac.o: mac.c
	$(CC) $(CFLAGS) mac.c

net.o: net.c
	$(CC) $(CFLAGS) net.c

radio.o: radio.c
	$(CC) $(CFLAGS) radio.c

aes.o: aes.c
	$(CC) $(CFLAGS) aes.c

clean:
	rm *.o
	rm gateway
