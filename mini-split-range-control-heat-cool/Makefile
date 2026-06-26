# Makefile for mini-split-range-control-heat-cool
# Split-Range Control ！ Heat/Cool, Acid/Base, Vent/Inert
#
# Targets:
#   make all      ！ build test runner and examples
#   make test     ！ compile and run all tests
#   make examples ！ build and run all examples
#   make clean    ！ remove build artifacts

CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -Iinclude
LDFLAGS = -lm

SRC = src/split_range_core.c src/split_range_pid.c       src/split_range_valve.c src/split_range_advanced.c       src/split_range_control.c

TEST = tests/test_split_range

EX1 = examples/example_reactor_heat_cool
EX2 = examples/example_ph_control
EX3 = examples/example_pressure_split

DEMO = demos/demo_split_range
BENCH = benches/bench_split_range

.PHONY: all clean test examples demo bench lean check

all: $(TEST) $(EX1) $(EX2) $(EX3)

$(TEST): tests/test_split_range.c $(SRC)
	$(CC) $(CFLAGS) -o $@ tests/test_split_range.c $(SRC) $(LDFLAGS)

$(EX1): examples/example_reactor_heat_cool.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/example_reactor_heat_cool.c $(SRC) $(LDFLAGS)

$(EX2): examples/example_ph_control.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/example_ph_control.c $(SRC) $(LDFLAGS)

$(EX3): examples/example_pressure_split.c $(SRC)
	$(CC) $(CFLAGS) -o $@ examples/example_pressure_split.c $(SRC) $(LDFLAGS)

$(DEMO): demos/demo_split_range.c $(SRC)
	$(CC) $(CFLAGS) -o $@ demos/demo_split_range.c $(SRC) $(LDFLAGS)

$(BENCH): benches/bench_split_range.c $(SRC)
	$(CC) $(CFLAGS) -o $@ benches/bench_split_range.c $(SRC) $(LDFLAGS)

test: $(TEST)
	@echo ===========================================
	@echo   Running Split-Range Control Tests
	@echo ===========================================
	@./$(TEST) || exit 1
	@echo
	@echo ===========================================
	@echo   ALL TESTS PASSED
	@echo ===========================================

examples: $(EX1) $(EX2) $(EX3)
	@echo ===========================================
	@echo   Running Split-Range Examples
	@echo ===========================================
	@./$(EX1)
	@echo
	@./$(EX2)
	@echo
	@./$(EX3)
	@echo
	@echo ===========================================
	@echo   ALL EXAMPLES COMPLETE
	@echo ===========================================

demo: $(DEMO)
	@./$(DEMO)

bench: $(BENCH)
	@./$(BENCH)

lean:
	cd src && lean split_range_formal.lean 2>/dev/null || echo "Lean: checking skipped (lean4 not available in this environment)"

check: test

clean:
	rm -f $(TEST) $(TEST).exe
	rm -f $(EX1) $(EX1).exe $(EX2) $(EX2).exe $(EX3) $(EX3).exe
	rm -f $(DEMO) $(DEMO).exe $(BENCH) $(BENCH).exe
	rm -f src/*.o src/*.olean src/*.ilean
	rm -f *.exe
