
CC = gcc
CFLAGS = -I ../include -g -Wall -Wmissing-prototypes -Wstrict-prototypes -Werror

all : mkbootimage installboot

mkbootimage : mkbootimage.c
	$(CC) $(CFLAGS) -o mkbootimage mkbootimage.c

installboot : installboot.c
	$(CC) $(CFLAGS) -o installboot installboot.c

docprep : docprep.c
	$(CC) $(CFLAGS) -o docprep docprep.c
