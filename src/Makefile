NAME = ../lib/libgpiommap.a
TARGET := ../lib/libxdevice.a

SRC = gpio.c
OBJ = $(SRC:.c=.o)

CPP_FILES := deviceclass.cpp
C_FILES := conio.c
OBJECTS := $(CPP_FILES:.cpp=.cpp.o) $(C_FILES:.c=.c.o)
CFLAGS = -g -I. -I../include
CXXFLAGS+= -std=c++11 $(CFLAGS)

# TMS = main.c
# TMS2 = main.cpp
# TMO = $(TMS:.c=.o)$(TMS2:.cpp=.o)
# TMN = gpiommap
RM = rm -f

all: $(NAME) $(TARGET)

$(NAME): $(OBJ)
	ar rcs $@ $^
#	ar -rc $(NAME) $(OBJ)
#	ranlib $(NAME)

$(TARGET): $(OBJECTS)
	ar rcs $@ $^

clean:
	-$(RM) $(OBJ)
	-$(RM) *~
	-$(RM) \#*
	-$(RM) *.core

fclean: clean
	-$(RM) $(NAME)
	-$(RM) $(TARGET)
	-$(RM) *.o *.d

re: fclean all

ftest: fclean test

install: all
	cp gpio.h /usr/include/
	cp am335x.h /usr/include/
	cp $(NAME) /usr/lib/

%.cpp.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.c.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<
