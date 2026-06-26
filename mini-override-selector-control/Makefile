# ============================================================================
# Makefile for mini-override-selector-control
# ============================================================================

CC       := gcc
CFLAGS   := -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -O2 -std=c11
LDFLAGS  := -lm
INCLUDES := -I./include

SRC_DIR  := src
TST_DIR  := tests
EXM_DIR  := examples
BLD_DIR  := build

SRCS := $(SRC_DIR)/override_core.c $(SRC_DIR)/override_selector.c $(SRC_DIR)/override_pid.c $(SRC_DIR)/override_constraint.c $(SRC_DIR)/override_valve_pos.c $(SRC_DIR)/override_surge.c $(SRC_DIR)/override_auctioneering.c $(SRC_DIR)/override_diagnostics.c

OBJS := $(BLD_DIR)/override_core.o $(BLD_DIR)/override_selector.o $(BLD_DIR)/override_pid.o $(BLD_DIR)/override_constraint.o $(BLD_DIR)/override_valve_pos.o $(BLD_DIR)/override_surge.o $(BLD_DIR)/override_auctioneering.o $(BLD_DIR)/override_diagnostics.o

TESTS := $(TST_DIR)/test_override_core $(TST_DIR)/test_override_pid
EXAMPLES := $(EXM_DIR)/example_compressor_surge $(EXM_DIR)/example_reactor_override $(EXM_DIR)/example_boiler_override $(EXM_DIR)/example_furnace_constraint

.PHONY: all test examples count check clean demo bench

all: test examples

$(BLD_DIR):
	mkdir -p $(BLD_DIR)

$(BLD_DIR)/%.o: $(SRC_DIR)/%.c | $(BLD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(TST_DIR)/test_override_core: $(TST_DIR)/test_override_core.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(TST_DIR)/test_override_core.c $(OBJS) -o $@ $(LDFLAGS)

$(TST_DIR)/test_override_pid: $(TST_DIR)/test_override_pid.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(TST_DIR)/test_override_pid.c $(OBJS) -o $@ $(LDFLAGS)

test: $(TESTS)
	@echo "============================================"
	@echo "  Running Override Selector Test Suite"
	@echo "============================================"
	./$(TST_DIR)/test_override_core
	./$(TST_DIR)/test_override_pid
	@echo "All tests passed."

$(EXM_DIR)/example_compressor_surge: $(EXM_DIR)/example_compressor_surge.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(EXM_DIR)/example_compressor_surge.c $(OBJS) -o $@ $(LDFLAGS)

$(EXM_DIR)/example_reactor_override: $(EXM_DIR)/example_reactor_override.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(EXM_DIR)/example_reactor_override.c $(OBJS) -o $@ $(LDFLAGS)

$(EXM_DIR)/example_boiler_override: $(EXM_DIR)/example_boiler_override.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(EXM_DIR)/example_boiler_override.c $(OBJS) -o $@ $(LDFLAGS)

$(EXM_DIR)/example_furnace_constraint: $(EXM_DIR)/example_furnace_constraint.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $(EXM_DIR)/example_furnace_constraint.c $(OBJS) -o $@ $(LDFLAGS)

examples: $(EXAMPLES)

demo: demos/override_demo.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) demos/override_demo.c $(OBJS) -o demos/override_demo $(LDFLAGS)
	./demos/override_demo

bench: benches/bench_override.c $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) benches/bench_override.c $(OBJS) -o benches/bench_override $(LDFLAGS)
	./benches/bench_override

count:
	@echo "include/ lines:"
	@wc -l include/*.h
	@echo "src/ lines (.c):"
	@wc -l src/*.c
	@echo "src/ lines (.lean):"
	@wc -l src/*.lean

check:
	@echo "=== Safety Scan ==="
	@grep -rn "_fn[0-9]" --include="*.c" --include="*.h" . || true
	@grep -rn "_aux[0-9]" --include="*.c" --include="*.h" . || true
	@grep -rn "TODO\|FIXME\|placeholder\|stub" --include="*.c" --include="*.h" . || true
	@echo "Safety scan complete."

clean:
	rm -rf $(BLD_DIR)/*.o
	rm -f tests/test_override_core tests/test_override_pid
	rm -f examples/example_compressor_surge examples/example_reactor_override examples/example_boiler_override examples/example_furnace_constraint
	rm -f demos/override_demo benches/bench_override
	@echo "Cleaned."
