# Makefile for mini-ratio-control-gas-liquid
# Gas-Liquid Ratio Control — Complete Implementation

CC       := gcc
CFLAGS   := -Wall -Wextra -std=c11 -O2 -g
LDFLAGS  := -lm
INCLUDES := -Iinclude

BUILD_DIR := build

SRC_FILES := src/ratio_core.c src/ratio_controller.c src/gas_liquid_process.c \
             src/ratio_cross_limiting.c src/ratio_dynamic_comp.c src/ratio_adaptive.c

TEST_FILES := tests/test_ratio_core.c tests/test_ratio_controller.c tests/test_gas_liquid.c

EXAMPLE_FILES := examples/example_air_fuel_ratio.c examples/example_gas_liquid_reactor.c \
                 examples/example_blending_ratio.c

BENCH_FILES := benches/bench_ratio_perf.c

DEMO_FILES := demos/demo_ratio_visual.c

SRC_OBJS  := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SRC_FILES))
TEST_BINS := $(patsubst tests/%.c,$(BUILD_DIR)/%,$(TEST_FILES))
EXAMPLE_BINS := $(patsubst examples/%.c,$(BUILD_DIR)/%,$(EXAMPLE_FILES))
BENCH_BINS := $(patsubst benches/%.c,$(BUILD_DIR)/%,$(BENCH_FILES))
DEMO_BINS := $(patsubst demos/%.c,$(BUILD_DIR)/%,$(DEMO_FILES))

.PHONY: all
all: test examples benches demos

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/test_ratio_core: tests/test_ratio_core.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_ratio_controller: tests/test_ratio_controller.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/test_gas_liquid: tests/test_gas_liquid.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/example_air_fuel_ratio: examples/example_air_fuel_ratio.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/example_gas_liquid_reactor: examples/example_gas_liquid_reactor.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/example_blending_ratio: examples/example_blending_ratio.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/bench_ratio_perf: benches/bench_ratio_perf.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

$(BUILD_DIR)/demo_ratio_visual: demos/demo_ratio_visual.c $(SRC_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) $< $(SRC_OBJS) -o $@ $(LDFLAGS)

.PHONY: test
test: $(TEST_BINS)
	@echo "=== Running All Tests ==="
	-$(BUILD_DIR)/test_ratio_core
	-$(BUILD_DIR)/test_ratio_controller
	-$(BUILD_DIR)/test_gas_liquid
	@echo "=== All Tests Done ==="

.PHONY: examples
examples: $(EXAMPLE_BINS)
	@echo "=== All examples built ==="

.PHONY: benches
benches: $(BENCH_BINS)
	@echo "=== All benches built ==="

.PHONY: demos
demos: $(DEMO_BINS)
	@echo "=== All demos built ==="

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: lines
lines:
	@find include src -type f \( -name "*.h" -o -name "*.c" -o -name "*.lean" \) -exec cat {} + | wc -l
