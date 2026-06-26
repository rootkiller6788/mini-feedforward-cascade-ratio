# Makefile for mini-static-dynamic-feedforward
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g
INCLUDES = -I./include
LDFLAGS = -lm

SRCS = src/feedforward_static.c src/feedforward_dynamic.c src/feedforward_models.c \
       src/feedforward_combined.c src/feedforward_tuning.c src/feedforward_advanced.c \
       src/feedforward_applications.c

.PHONY: all clean test

all: build/test_all
	build/test_all

build/test_all: tests/test_all.c $(SRCS)
	@mkdir -p build
	$(CC) $(CFLAGS) $(INCLUDES) tests/test_all.c $(SRCS) -o build/test_all $(LDFLAGS)

test: build/test_all
	build/test_all

clean:
	rm -rf build

lines:
	@find include -name "*.h" -exec cat {} + | wc -l
	@find src -name "*.c" -exec cat {} + | wc -l
