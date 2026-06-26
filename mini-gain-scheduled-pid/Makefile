# Makefile for mini-gain-scheduled-pid
# Submodule of 3. mini-feedforward-cascade-ratio

CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -O2 -g -Iinclude
LDFLAGS = -lm

SRC = src/gain_schedule_core.c       src/gain_schedule_interp.c       src/gain_schedule_design.c       src/gain_schedule_pid.c       src/gain_schedule_stability.c       src/gain_schedule_adaptive.c

TEST_SRC = tests/test_gain_schedule.c
TEST_BIN = test_gain_schedule

EXAMPLE_BINS = example_temp_control example_servo_position example_ph_control

DEMO_BIN = demo_schedule_analysis

BENCH_BIN = bench_gain_schedule

.PHONY: all test examples clean run-examples check demo bench

all: $(TEST_BIN) examples demo bench

$(TEST_BIN): $(SRC) $(TEST_SRC) include/*.h
	$(CC) $(CFLAGS) -o $(TEST_BIN) $(SRC) $(TEST_SRC) $(LDFLAGS)

test: $(TEST_BIN)
	./$(TEST_BIN)

examples: $(EXAMPLE_BINS)

example_temp_control: examples/example_temp_control.c $(SRC) include/*.h
	$(CC) $(CFLAGS) -o $@ examples/example_temp_control.c $(SRC) $(LDFLAGS)

example_servo_position: examples/example_servo_position.c $(SRC) include/*.h
	$(CC) $(CFLAGS) -o $@ examples/example_servo_position.c $(SRC) $(LDFLAGS)

example_ph_control: examples/example_ph_control.c $(SRC) include/*.h
	$(CC) $(CFLAGS) -o $@ examples/example_ph_control.c $(SRC) $(LDFLAGS)

demo: $(DEMO_BIN)

$(DEMO_BIN): demos/demo_schedule_analysis.c $(SRC) include/*.h
	$(CC) $(CFLAGS) -o $@ demos/demo_schedule_analysis.c $(SRC) $(LDFLAGS)

bench: $(BENCH_BIN)

$(BENCH_BIN): benches/bench_gain_schedule.c $(SRC) include/*.h
	$(CC) $(CFLAGS) -o $@ benches/bench_gain_schedule.c $(SRC) $(LDFLAGS)

run-demo: $(DEMO_BIN)
	./$(DEMO_BIN)

run-bench: $(BENCH_BIN)
	./$(BENCH_BIN)

run-examples: examples
	./example_temp_control
	./example_servo_position
	./example_ph_control

check:
	@echo === Filler Pattern Scan ===
	@grep -rn "_fn[0-9]" --include="*.c" --include="*.h" . || true
	@grep -rn "_aux[0-9]" --include="*.c" --include="*.h" . || true
	@grep -rn "_ext[0-9]" --include="*.c" --include="*.h" . || true
	@grep -rn "algorithm variant" --include="*.c" . || true
	@grep -rn "supplemental assert" --include="*.c" . || true
	@grep -rn "by trivial" --include="*.lean" . || true
	@grep -rn "sorry" --include="*.lean" . || true
	@echo === TODO/FIXME/stub/placeholder Scan ===
	@grep -rn "TODO\|FIXME\|stub\|placeholder" --include="*.c" --include="*.h" --include="*.lean" . || true
	@echo === Line Count include and src ===
	@echo -n "Total lines: "
	@wc -l include/*.h src/*.c src/*.lean 2>/dev/null || true

clean:
	rm -f $(TEST_BIN) $(EXAMPLE_BINS) $(DEMO_BIN) $(BENCH_BIN)
	rm -f *.exe
