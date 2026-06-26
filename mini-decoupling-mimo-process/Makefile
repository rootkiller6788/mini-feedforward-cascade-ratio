# Makefile for mini-decoupling-mimo-process
# MIMO Decoupling Control Engineering Module

CC       = gcc
CFLAGS   = -Wall -Wextra -std=c11 -O2 -Iinclude
LDFLAGS  = -lm
TARGET   = build/libmimo_decoupling.a
TEST_TARGET = build/test_runner

SRC_DIR  = src
TEST_DIR = tests
BUILD_DIR = build

SRCS     = $(wildcard $(SRC_DIR)/*.c)
OBJS     = $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o)
TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_OBJS = $(TEST_SRCS:$(TEST_DIR)/%.c=$(BUILD_DIR)/test_%.o)

.PHONY: all clean test lib examples

all: lib examples

EXAMPLES = build/example_distillation_column build/example_reactor_decoupling build/example_heat_exchanger

build/example_%: examples/example_%.c $(TARGET) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -L$(BUILD_DIR) -lmimo_decoupling $(LDFLAGS) -o $@

examples: $(EXAMPLES)

lib: $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	ar rcs $@ $^

$(BUILD_DIR)/test_%.o: $(TEST_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TEST_TARGET): $(TEST_OBJS) $(TARGET)
	$(CC) $(CFLAGS) -o $@ $(TEST_OBJS) -L$(BUILD_DIR) -lmimo_decoupling $(LDFLAGS)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -rf $(BUILD_DIR)

print:
	@echo "SRCS: $(SRCS)"
	@echo "OBJS: $(OBJS)"
	@echo "Total include+src lines:"
	@cat include/*.h src/*.c 2>/dev/null | wc -l

count:
	@echo "include/ lines:"
	@wc -l include/*.h
	@echo "src/ lines (.c):"
	@wc -l src/*.c
	@echo "src/ lines (.lean):"
	@wc -l src/*.lean 2>/dev/null || echo "0"

check:
	@echo "=== Safety Scan ==="
	@grep -rn "_fn[0-9]" --include="*.c" --include="*.h" . || true
	@grep -rn "_aux[0-9]" --include="*.c" --include="*.h" . || true
	@grep -rn "_ext[0-9][^a-z]" --include="*.c" --include="*.h" . || true
	@grep -rn "TODO\|FIXME\|placeholder\|stub" --include="*.c" --include="*.h" . || true
	@echo "Safety scan complete."
