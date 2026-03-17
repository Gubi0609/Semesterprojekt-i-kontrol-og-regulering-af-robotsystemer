CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -I/usr/include/quanser
LDFLAGS  := -lm

# Quanser HIL library (only needed for hardware targets)
HIL_LIBS := -lhil -lquanser_common

.PHONY: all sim hardware balance clean

all: run_sim run_hardware run_balance read_fast

# ── Simulation (no Quanser SDK needed) ──────────────────────
run_sim: run_sim.cpp plant.h controllers.h qube_types.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

# ── Hardware targets (need Quanser SDK) ─────────────────────
run_hardware: run_hardware.cpp controllers.h qube_types.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS) $(HIL_LIBS)

run_balance: run_balance.cpp controllers.h qube_types.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS) $(HIL_LIBS)

read_fast: read_fast.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS) $(HIL_LIBS)

# ── Convenience targets ─────────────────────────────────────
sim: run_sim
	./run_sim

hardware: run_hardware
	./run_hardware

balance: run_balance
	./run_balance

clean:
	rm -f run_sim run_hardware run_balance read_fast *.csv
