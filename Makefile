.phony: all

CXX = arm-linux-gnueabihf-g++
CPPFLAGS = -Iinclude -Ithirdparty -std=c++17  #beaglebone doesn't have a very good compiler.
LDFLAGS = -lpthread -L. -lrobotcontrol

SOURCES := $(wildcard ./*.cpp ./*.c)
INCLUDES := $(wildcard ./include/*.h)

OBJECTS := $(patsubst %.cpp, %.o, $(SOURCES))

.PHONY: all clean remote

robot_main: $(OBJECTS) $(INCLUDES)
	$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS) -Ithirdparty -Iinclude -o robot_main -static

all: robot_main

remote:
	rsync -av --exclude="*.o" --exclude="robotproject" . debian@192.168.7.2:robotproject
	ssh debian@192.168.7.2 "cd robotproject;make"

clean: 
	rm -rf *.o
	rm -rf robot_main
	rm -rf robotproject

# %.o: %.c
# 	$(CC) -c $< -o $@ $(CFLAGS) $(LDFLAGS) -I./include -Ithirdparty


