# Host tests for the DAP frame layer. No ESP-IDF, no hardware.
#
#   make run        build and run
#   make clean

CC      ?= gcc
CFLAGS  ?= -std=gnu17 -O1 -g -Wall -Wextra -Werror -I../include
BIN     := test_dap_frame
SRCS    := test_dap_frame.c ../dap_frame.c

.PHONY: run clean

run: $(BIN)
	./$(BIN)

$(BIN): $(SRCS) ../include/dap_frame.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(BIN)
