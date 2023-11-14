.phony: all

CC=gcc
CFLAGS=-static
LDFLAGS=-lpthread -L. -lldlidar_driver 

all: executable

executable: main.o
	$(CC) $(CFLAGS) main.c thirdparty/log.c -Ithirdparty -Iinclude  -o robotproject

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS) $(LDFLAGS) -I./include -Ithirdparty


