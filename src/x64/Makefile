CC      ?= cc
CFLAGS  ?= -std=gnu11 -O2 -Wall -Wextra
LDLIBS  = -lm

all: udpbbr_server udpbbr_client linkemu

udpbbr_server: server.c bbr3.c bbr3.h proto.h
	$(CC) $(CFLAGS) -o $@ server.c bbr3.c $(LDLIBS)

udpbbr_client: client.c proto.h
	$(CC) $(CFLAGS) -o $@ client.c

linkemu: linkemu.c proto.h
	$(CC) $(CFLAGS) -o $@ linkemu.c

clean:
	rm -f udpbbr_server udpbbr_client linkemu

.PHONY: all clean
