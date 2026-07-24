# Makefile minimal pour calypso-ipc-device.
# Build standalone : ne dépend ni d'autotools ni du tree osmo-trx.
# Vendoring complet dans ce répertoire (shm.{c,h}, ipc_{shm,chan,sock}.{c,h},
# debug.h, ipc-driver-test.h). Le seul lien externe = libosmocore.

CC      ?= gcc
CFLAGS  += -Wall -Wextra -O2 -g -I. -D_GNU_SOURCE
CFLAGS  += $(shell pkg-config --cflags libosmocore 2>/dev/null)
LDFLAGS +=
LDLIBS  += $(shell pkg-config --libs libosmocore 2>/dev/null || echo "-losmocore")
LDLIBS  += -L/usr/local/lib -losmocoding -losmogsm -losmoisdn -losmocodec -losmocore -ltalloc -lmnl
LDLIBS  += -lpthread -lrt -lm

SRCS = \
    calypso_ipc_device.c \
    qemu_wrap.c \
    shm.c \
    ipc_shm.c \
    ipc_chan.c \
    ipc_sock.c

OBJS = $(SRCS:.c=.o)
TARGET = calypso-ipc-device

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)
