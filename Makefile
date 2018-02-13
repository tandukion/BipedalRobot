TARGET := pendulum

OBJLIBS = libgpiommap libxcommunication libxstypes

# C_FILES := main.c
# OBJECTS := $(C_FILES:.c=.o)
CPP_FILES := main.cpp
OBJECTS := $(CPP_FILES:.cpp=.o)
# OBJECTS += src/gpio.o

HEADERS = $(wildcard *.h)
INCLUDE = -I. -Iinclude -Isrc
CFLAGS =$(INCLUDE) -w -include config.h
CXXFLAGS =-std=c++11 $(CFLAGS)
LFLAGS = -Llib -lgpiommap -lxdevice -lxcommunication -lxstypes -lpthread -lrt -ldl

RM = rm -rf

all: $(OBJLIBS) $(TARGET)

libgpiommap:
	$(MAKE) -C src $(MFLAGS) all

libxcommunication : libxstypes
	-$(MAKE) -C xcommunication $(MFLAGS)
	-cp xcommunication/libxcommunication.a lib/

libxstypes :
	-$(MAKE) -C xstypes $(MFLAGS) libxstypes.a
	-cp xstypes/libxstypes.a lib/


$(TARGET): $(OBJECTS)
#	$(CXX) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LFLAGS)
# working
	$(CXX) $(CFLAGS) $(INCLUDE) $^ src/gpio.c -o $@ $(LFLAGS)


clean :
	$(RM) $(OBJECTS) $(TARGET)

clean_all:
	-$(RM) $(OBJECTS) $(TARGET)
	-$(MAKE) -C src $(MFLAGS) fclean
	-$(MAKE) -C xcommunication $(MFLAGS) clean
	-$(MAKE) -C xstypes $(MFLAGS) clean
	- rm -rf lib/libxstypes.a lib/libxcommunication.a
