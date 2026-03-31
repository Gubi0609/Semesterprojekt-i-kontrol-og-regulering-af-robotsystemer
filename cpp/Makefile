CXX      := g++
CXXFLAGS := -O2 -std=c++17 -Wall -Wextra -I/usr/include/quanser
LDFLAGS  := -lm

# Quanser HIL library (only needed for hardware targets)
HIL_LIBS := -lhil -lquanser_common

# ImGui / ImPlot / OpenGL (for GUI targets)
IMGUI_DIR  := vendor/imgui
IMPLOT_DIR := vendor/implot
GUI_SRCS   := $(IMGUI_DIR)/imgui.cpp \
              $(IMGUI_DIR)/imgui_draw.cpp \
              $(IMGUI_DIR)/imgui_tables.cpp \
              $(IMGUI_DIR)/imgui_widgets.cpp \
              $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp \
              $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp \
              $(IMPLOT_DIR)/implot.cpp \
              $(IMPLOT_DIR)/implot_items.cpp
GUI_INC    := -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends -I$(IMPLOT_DIR)
GUI_LIBS   := -lGL -lGLEW -lglfw -lpthread

.PHONY: all sim hardware balance gui tests clean

all: run_sim run_hardware run_balance read_fast run_gui run_tests analyze_csv

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

run_friction_test: run_friction_test.cpp qube_types.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS) $(HIL_LIBS)

# ── GUI: simulation mode (no Quanser SDK) ───────────────────
run_gui: run_gui.cpp controllers.h qube_types.h plant.h render3d.h $(GUI_SRCS)
	$(CXX) $(CXXFLAGS) $(GUI_INC) -o $@ run_gui.cpp $(GUI_SRCS) $(LDFLAGS) $(GUI_LIBS)

# ── GUI: hardware mode (needs Quanser SDK) ──────────────────
run_gui_hw: run_gui.cpp controllers.h qube_types.h plant.h render3d.h $(GUI_SRCS)
	$(CXX) $(CXXFLAGS) $(GUI_INC) -DUSE_HARDWARE -o $@ run_gui.cpp $(GUI_SRCS) $(LDFLAGS) $(GUI_LIBS) $(HIL_LIBS)

# ── Test harness (no Quanser SDK needed) ────────────────────
run_tests: run_tests.cpp plant.h controllers.h qube_types.h
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

# ── CSV analyzer (no Quanser SDK needed) ────────────────────
analyze_csv: analyze_csv.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDFLAGS)

# ── Convenience targets ─────────────────────────────────────
sim: run_sim
	./run_sim

hardware: run_hardware
	./run_hardware

balance: run_balance
	./run_balance

gui: run_gui
	./run_gui

tests: run_tests
	./run_tests

clean:
	rm -f run_sim run_hardware run_balance read_fast run_friction_test run_gui run_gui_hw run_tests analyze_csv *.csv
