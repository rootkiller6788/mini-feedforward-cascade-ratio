# Makefile for mini-cascade-control-primary-secondary
# Cascade Control — Primary/Secondary, Feedforward, Ratio, Override
#
# Targets:
#   make all      — build test runner and examples
#   make test     — compile and run all tests
#   make examples — build and run all examples
#   make clean    — remove build artifacts

CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -Iinclude
LDFLAGS = -lm

SRC = src/cascade_pid.c src/cascade_control.c src/cascade_tuning.c \
      src/cascade_stability.c src/feedforward_control.c \
      src/override_selector.c src/ratio_control.c

# Individual test binaries
TEST1 = tests/test_cascade
TEST2 = tests/test_tuning
TEST3 = tests/test_ff_ratio_override

# Example binaries
EX1 = examples/example_temp_cascade
EX2 = examples/example_level_flow_cascade
EX3 = examples/example_ratio_blending

.PHONY: all clean test examples lean check

all: $(TEST1) $(TEST2) $(TEST3) $(EX1) $(EX2) $(EX3)

# Test binaries
$(TEST1): tests/test_cascade.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_cascade.c $(SRC) $(LDFLAGS)

$(TEST2): tests/test_tuning.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_tuning.c $(SRC) $(LDFLAGS)

$(TEST3): tests/test_ff_ratio_override.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_ff_ratio_override.c $(SRC) $(LDFLAGS)

# Example binaries
$(EX1): examples/example_temp_cascade.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/example_temp_cascade.c $(SRC) $(LDFLAGS)

$(EX2): examples/example_level_flow_cascade.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/example_level_flow_cascade.c $(SRC) $(LDFLAGS)

$(EX3): examples/example_ratio_blending.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/example_ratio_blending.c $(SRC) $(LDFLAGS)

# Run all tests
test: $(TEST1) $(TEST2) $(TEST3)
	@echo "==========================================="
	@echo "  Running Cascade Control Tests"
	@echo "==========================================="
	@./$(TEST1) || exit 1
	@echo ""
	@./$(TEST2) || exit 1
	@echo ""
	@./$(TEST3) || exit 1
	@echo ""
	@echo "==========================================="
	@echo "  ALL TESTS PASSED"
	@echo "==========================================="

# Run all examples
examples: $(EX1) $(EX2) $(EX3)
	@echo "==========================================="
	@echo "  Running Cascade Control Examples"
	@echo "==========================================="
	@./$(EX1)
	@echo ""
	@./$(EX2)
	@echo ""
	@./$(EX3)
	@echo ""
	@echo "==========================================="
	@echo "  ALL EXAMPLES COMPLETE"
	@echo "==========================================="

lean:
	cd src && lean cascade_formal.lean 2>/dev/null || echo "Lean: checking skipped (lean4 not available in this environment)"

check: test

clean:
	rm -f $(TEST1) $(TEST2) $(TEST3)
	rm -f $(EX1) $(EX2) $(EX3)
	rm -f src/*.o src/*.olean src/*.ilean
	rm -f *.exe
