.phony: all

# CC = g++
CPPFLAGS = -static -Iinclude -Ithirdparty
LDFLAGS = -lpthread -L. -lldlidar_driver -lrobotcontrol

SOURCES := $(wildcard ./*.cpp ./*.c)
OBJECTS := $(patsubst %.cpp, %.o, $(SOURCES))

$(info Sources: ${SOURCES})

$(info Objects: ${OBJECTS})

.PHONY: all clean

robot_main: $(OBJECTS)
	$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS) -Ithirdparty -Iinclude  -o robotproject

all: robot_main

clean: 
	rm -rf *.o
	rm -rf robot_main
	rm -rf robotproject

# %.o: %.c
# 	$(CC) -c $< -o $@ $(CFLAGS) $(LDFLAGS) -I./include -Ithirdparty


