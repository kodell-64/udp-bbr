# Works with old GNU make (no ?=) and old compilers (no -std=).
# Override on the command line, e.g.  make CC=gcc CFLAGS="-O2 -g"
CC      = cc
CFLAGS  = -O2 -Wall
LDLIBS  = -lm

# Optional fully static clients built against musl libc (see README):
#   make udpbbr_client MUSL64=$HOME/musl64
#   make client32      MUSL32=$HOME/musl32
MUSL32  =
MUSL64  =

all: udpbbr_server udpbbr_client linkemu

client32: udpbbr_client32

udpbbr_server: server.c bbr3.c bbr3.h proto.h compat.h
	$(CC) $(CFLAGS) -o $@ server.c bbr3.c $(LDLIBS)

ifeq ($(MUSL64),)
udpbbr_client: client.c proto.h compat.h
	$(CC) $(CFLAGS) -o $@ client.c
else
udpbbr_client: client.c proto.h compat.h
	$(CC) $(CFLAGS) -static -no-pie -fno-pie -nostdlib -nostdinc \
	    -isystem $(MUSL64)/include -o $@ \
	    $(MUSL64)/lib/crt1.o $(MUSL64)/lib/crti.o client.c \
	    $(MUSL64)/lib/libc.a $$($(CC) -print-libgcc-file-name) $(MUSL64)/lib/crtn.o
endif

ifeq ($(MUSL32),)
udpbbr_client32: client.c proto.h compat.h
	$(CC) -m32 $(CFLAGS) -o $@ client.c
else
udpbbr_client32: client.c proto.h compat.h i386_divdi.c
	$(CC) -m32 -march=i486 $(CFLAGS) -static -no-pie -fno-pie -nostdlib -nostdinc \
	    -isystem $(MUSL32)/include -DPROVIDE_DIVDI -o $@ \
	    $(MUSL32)/lib/crt1.o $(MUSL32)/lib/crti.o client.c i386_divdi.c \
	    $(MUSL32)/lib/libc.a $(MUSL32)/lib/crtn.o
endif

linkemu: linkemu.c proto.h compat.h
	$(CC) $(CFLAGS) -o $@ linkemu.c

# For developers on a modern gcc: check the sources stay C89-clean.
c89check:
	for f in client.c server.c bbr3.c linkemu.c; do \
	    $(CC) -std=gnu89 -pedantic -Wall -Wextra -Wno-long-long \
	        -Werror=declaration-after-statement -fsyntax-only $$f || exit 1; \
	done

clean:
	rm -f udpbbr_server udpbbr_client udpbbr_client32 linkemu

.PHONY: all client32 c89check clean
