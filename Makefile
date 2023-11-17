.phony: all

# CC = g++
CPPFLAGS = -Iinclude -Ithirdparty -static -std=c++11 #beaglebone doesn't have a very good compiler.
LDFLAGS = -lpthread -lrobotcontrol

SOURCES := $(wildcard ./*.cpp ./*.c)
OBJECTS := $(patsubst %.cpp, %.o, $(SOURCES))

$(info Sources: ${SOURCES})

$(info Objects: ${OBJECTS})

.PHONY: all clean

EXECUTABLE = 

robot_main: $(OBJECTS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS) -Ithirdparty -Iinclude -o robotproject

all: robot_main

clean: 
	rm -rf *.o
	rm -rf robot_main
	rm -rf robotproject

# %.o: %.c
# 	$(CC) -c $< -o $@ $(CFLAGS) $(LDFLAGS) -I./include -Ithirdparty


