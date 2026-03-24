# Qube-Servo 3 Inverted Pendulum Controller

Full state-feedback balance controller and energy-based swing-up for the Quanser Qube-Servo 3 with inverted pendulum module. Includes a real-time 3D GUI, offline simulation, automated testing, and hardware CSV analysis.

## Status

| Component | Status | Notes |
|-----------|--------|-------|
| **Balance controller** | ✅ Working | Full state feedback, confirmed on hardware |
| **Swing-up controller** | ✅ Working | Energy-proportional Åström method with auto-kick |
| **3D GUI** | ✅ Working | Dear ImGui + ImPlot + OpenGL, real-time 3D model + plots |
| **Simulation** | ✅ Working | Nonlinear plant with RK4, 10 substeps per control step |
| **Test suite** | ✅ Working | 27 automated tests: balance, swing-up, robustness, noise |
| **CSV analyzer** | ✅ Working | Post-hoc analysis of hardware logs |

## Quick Start

### GUI (simulation — no hardware needed)

```bash
cd cpp
make run_gui
./run_gui
# Click "Start" — 3D model + live plots + parameter sliders
```

Requires `libglfw3-dev` and `libglew-dev`:
```bash
sudo apt install libglfw3-dev libglew-dev
```

### GUI (with real hardware)

```bash
cd cpp
make run_gui_hw
./run_gui_hw --hardware
# Same GUI but reads real encoders and drives the motor
```

### Balance only (hold upright, let go)

```bash
cd cpp
make run_balance
./run_balance
# Hold pendulum upright, let go when it says "Waiting..."
# Ctrl+C to stop. Logs to balance_output.csv
```

### Full swing-up + balance

```bash
cd cpp
make run_hardware
./run_hardware --duration 30
# Pendulum starts hanging down, swings itself up and catches.
# Logs to hardware_output.csv
```

### Offline simulation

```bash
cd cpp
make run_sim
./run_sim                              # swing-up → balance (10s)
./run_sim --near-upright --duration 5  # balance only from 5.7°
```

### Automated tests

```bash
cd cpp
make run_tests
./run_tests                    # all 27 tests
./run_tests --suite balance    # 7 balance tests
./run_tests --suite swingup    # 6 swing-up tests
./run_tests --suite robust     # 8 robustness tests (parameter mismatch)
./run_tests --suite noise      # 6 noise tests
./run_tests --csv              # also write per-test CSV files
```

### Analyze hardware logs

```bash
cd cpp
make analyze_csv
./analyze_csv hardware_output.csv
./analyze_csv balance_output.csv
./analyze_csv run1.csv run2.csv   # side-by-side comparison table
```

## Build

```bash
cd cpp
make all          # builds everything
make run_sim      # simulation only (no SDK needed)
make run_gui      # GUI in simulation mode (needs GLFW + GLEW)
make run_gui_hw   # GUI with hardware support (needs SDK + GLFW + GLEW)
make run_tests    # test harness (no SDK needed)
make analyze_csv  # CSV analyzer (no SDK needed)
make run_balance run_hardware read_fast  # hardware-only targets (need SDK)
```

**Dependencies:**
- `g++` with C++17
- Quanser SDK (`libhil`, `libquanser_common`) — for hardware targets only
- `libglfw3-dev`, `libglew-dev` — for GUI targets only
- Dear ImGui and ImPlot — vendored in `cpp/vendor/` (no install needed)

## Architecture

```
cpp/
├── qube_types.h       # QubeState, QubeParams, constants, wrap_angle, clamp
├── plant.h            # Nonlinear Euler-Lagrange EOM + RK4 integrator (header-only)
├── controllers.h      # BalanceController + SwingUpController (header-only)
├── render3d.h         # 3D OpenGL visualization of the Qube (header-only)
├── run_gui.cpp        # Real-time GUI: 3D model + plots + sliders (sim or hardware)
├── run_sim.cpp        # Offline simulation (no hardware)
├── run_balance.cpp    # Balance-only mode (hardware)
├── run_hardware.cpp   # Full swing-up + balance (hardware)
├── run_tests.cpp      # Automated performance test suite (simulation)
├── analyze_csv.cpp    # Post-hoc analysis of hardware CSV logs
├── read_fast.cpp      # Encoder poll rate benchmark
├── vendor/
│   ├── imgui/         # Dear ImGui (vendored source)
│   └── implot/        # ImPlot (vendored source)
└── Makefile
```

All headers are header-only — no separate compilation units needed except the main `.cpp` files.

## GUI Features

The GUI (`run_gui` / `run_gui_hw`) provides:

- **3D visualization** — real-time model of the Qube body, motor arm, and pendulum. Click-drag to orbit, scroll to zoom.
- **4 live plots** — angles (°), velocities (rad/s), motor voltage (V), pendulum energy (J) with adjustable time window.
- **Mode indicator** — SWING-UP (orange) / BALANCING (green) with live state readout.
- **Parameter sliders** — tune all swing-up (mu, catch angle, deadzone, kick) and balance (5 gains + voltage limit) parameters in real time.
- **Start / Stop / Reset** buttons.

## Test Suite Results (Simulation)

| Suite | Pass | Total | Notes |
|-------|------|-------|-------|
| Balance | 7 | 7 | 5°–25° perturbations, arm offset, initial velocity |
| Swing-up | 5 | 6 | mu=3 too gentle (expected) |
| Robustness | 5 | 8 | Friction mismatch causes failures (known limitation) |
| Noise | 6 | 6 | Solid up to 0.01 rad + 1 rad/s noise |
| **Total** | **23** | **27** | |

## Hardware Notes

- **Encoder CPR:** 2048 counts/rev (512 lines × 4X quadrature)
- **Alpha convention:** 0 = upright, ±π = hanging. Raw encoder zero = hanging down. Applied in code: `wrap_angle(-(raw - π))`
- **Velocities:** Hardware tachometers (channels 14000/14001, 72 MHz 32-bit counter) — zero-lag, no filtering needed.
- **Amplifier deadband:** ~0.3V compensated in software on all nonzero commands.
- **Pendulum attachment:** Magnetic — detaches if mu > ~10. Keep mu ≤ 8 on hardware.
- **Arm endstop:** ~±130° mechanical. Software limit at ±60° with 30° soft braking zone.
- **Motor resistance:** Rm = 7.5Ω (Servo 3 manual Table 2.2).
- **Pendulum length:** Lp = 12.9cm (workbook value, datasheet says 9.5cm — TODO: measure).
- **Control rate:** 1 kHz default (configurable via `--rate`).

## Controller Design

### Balance (LQR-derived full state feedback)

```
u = -k_theta·θ - k_alpha·α - k_theta_dot·θ̇ - k_alpha_dot·α̇ - k_integral·∫α
```

Gains found via parameter sweep (~500 combinations, zero overshoot criterion):

| Gain | Value | Role |
|------|-------|------|
| k_alpha | 20.0 | Proportional: counteract pendulum tilt |
| k_alpha_dot | 2.0 | Derivative: damp pendulum oscillation |
| k_theta | -2.0 | Proportional: return arm to center |
| k_theta_dot | -1.0 | Derivative: damp arm motion |
| k_integral | 0.3 | Integral: correct steady-state offset |

Voltage clamped to ±6V to prevent amplifier saturation.

### Swing-up (energy-proportional Åström method)

Unlike bang-bang (±mu), voltage scales proportionally to the energy error:
- Far from upright → stronger push (up to mu volts)
- Near upright → gentle approach (minimum 20% of mu)
- Auto initial kick (0.8V for 0.5s) eliminates the need for manual nudge
- Arm protection: 3 zones (safe / soft brake / hard brake) prevent hitting endstops

Catches when |α| < 20° and hands off to the balance controller.

## Workflow: Evaluating on Hardware

```bash
# 1. Run experiment
./run_hardware --duration 30        # logs to hardware_output.csv
mv hardware_output.csv run_A.csv

# 2. Change parameters, run again
# (edit controllers.h or use GUI sliders)
./run_hardware --duration 30
mv hardware_output.csv run_B.csv

# 3. Compare
./analyze_csv run_A.csv run_B.csv
```

The analyzer reports: catch time, settling time, overshoot, RMS α, max voltage, balance duration, and an overall grade (★–★★★).
