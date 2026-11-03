CC ?= gcc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Wconversion -Wsign-conversion \
	-Wshadow -Werror

TARGET := test_line_follower
SOURCES := test_line_follower.c \
	../../../app_program/subsystem3/line_follower.c

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $@ $(SOURCES)

test: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET) $(TARGET).exe
