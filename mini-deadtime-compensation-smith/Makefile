# Makefile for mini-deadtime-compensation-smith
# Smith Predictor Dead-Time Compensation (O.J.M. Smith, 1957)
# make test  -> build + run all tests

CC       := gcc
CFLAGS   := -std=c11 -Wall -Wextra -O2 -Wno-misleading-indentation -Wno-unused-variable -Wno-unused-but-set-variable
LDFLAGS  := -lm
INCLUDES := -Iinclude

BUILD_DIR := build

SRC_C := $(wildcard src/*.c)
SRC_O := $(SRC_C:src/%.c=$(BUILD_DIR)/%.o)

TEST_C := tests/test_smith.c
TEST_X := $(BUILD_DIR)/test_smith

.PHONY: all
all: test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(TEST_X): $(SRC_O) $(TEST_C) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC_O) $(TEST_C) -o $@ $(LDFLAGS)

.PHONY: build
build: $(TEST_X)

.PHONY: test
test: $(TEST_X)
	@echo "=== Running Smith Predictor Tests ==="
	./$(TEST_X)
	@echo "=== All Tests Done ==="

# Example executables
EXAMPLE_C := $(wildcard examples/*.c)
EXAMPLE_X := $(EXAMPLE_C:examples/%.c=$(BUILD_DIR)/%)

$(BUILD_DIR)/%: examples/%.c $(SRC_O) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC_O) $< -o $@ $(LDFLAGS)

.PHONY: examples
examples: $(EXAMPLE_X)
	@echo "=== Examples built ==="

# Benchmarks
BENCH_C := $(wildcard benches/*.c)
BENCH_X := $(BENCH_C:benches/%.c=$(BUILD_DIR)/%)

$(BUILD_DIR)/%: benches/%.c $(SRC_O) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $(SRC_O) $< -o $@ $(LDFLAGS)

.PHONY: bench
bench: $(BENCH_X)
	@for b in $(BENCH_X); do ./$$b; done

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: lines
lines:
	@echo "include/ + src/ lines:"
	@find include src -type f \( -name "*.h" -o -name "*.c" -o -name "*.lean" \) -exec cat {} + 2>/dev/null | wc -l

.PHONY: audit
audit:
	@echo "=== SKILL.md Safety Review ==="
	@echo "--- Filler detection ---"
	@grep -rn "_fn[0-9]" --include="*.c" --include="*.h" . || echo "  CLEAN"
	@grep -rn "_aux[0-9]" --include="*.c" --include="*.h" . || echo "  CLEAN"
	@grep -rn "_ext[0-9][^a-z]" --include="*.c" --include="*.h" . || echo "  CLEAN"
	@grep -rn "algorithm variant" --include="*.c" . || echo "  CLEAN"
	@grep -rn "extension point" --include="*.c" . || echo "  CLEAN"
	@grep -rn "Module extension" --include="*.c" . || echo "  CLEAN"
	@grep -rn "\bsorry\b" src/*.lean || echo "  CLEAN"
	@grep -rn "traceability_matrix.*:=" src/*.lean || echo "  CLEAN"
	@echo "--- Line count ---"
	@make -s lines
	@echo "--- TODO/FIXME/stub/placeholder ---"
	@grep -rni "TODO\|FIXME\|stub\|placeholder" src/ include/ || echo "  CLEAN"
	@echo "=== Audit complete ==="
