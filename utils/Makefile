ifeq ($(owl),y)
	EXTRA_CFLAGS += -DOWL_DEV
endif

ifeq ($(debug),y)
	EXTRA_CFLAGS += -DDEBUG
endif

CFLAGS = -Wall -Wextra -pedantic -std=gnu99 #-std=c99 -D_POSIX_C_SOURCE
CFLAGS += $(EXTRA_CFLAGS)

CC = gcc

OBJ = main.o ngham.o crc.o

6loch_owl_con: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $(OBJ)

main.o: char6lo_owl_connector.c 6loch_conn.h config.h ngham.h
	$(CC) -c $(CFLAGS) char6lo_owl_connector.c -o $@

ngham.o: ngham.c ngham.h crc_ccitt.h
	$(CC) -c $(CFLAGS) ngham.c

crc.o: crc_ccitt.c crc_ccitt.h 
	$(CC) -c $(CFLAGS) crc_ccitt.c -o $@

clean:
	rm 6loch_owl_con main.o ngham.o crc.o


