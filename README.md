# Qube-Servo 3 Inverted Pendulum Balance Controller

PID/state-feedback balance controller for the Quanser Qube-Servo 3 with inverted pendulum module.

## Status

| Component | Status | Notes |
|-----------|--------|-------|
| **Balance controller** | ✅ Working | Hold pendulum upright manually, let go — it balances |
| **Swing-up controller** | ⚠️ WIP | Energy-based, gets ~34° from upright but can't close the gap yet |
| **Simulation** | ✅ Working | Offline sim with RK4 integration, no hardware needed |

## Quick Start

### Balance only (working)

```bash
cd cpp
make run_balance
./run_balance
# Hold pendulum upright, let go when it says "Waiting..."
# Ctrl+C to stop
```

### Simulation (no hardware needed)

```bash
cd cpp
make run_sim
./run_sim                              # swing-up → balance
./run_sim --near-upright --duration 5  # balance only
```

### Full swing-up + balance (WIP)

```bash
cd cpp
make run_hardware
./run_hardware --duration 30
# Needs a nudge to start. Pendulum may detach if too aggressive.
```

### Encoder poll rate test

```bash
cd cpp
make read_fast
./read_fast --duration 5
```

## Build

```bash
cd cpp
make all        # builds everything
make run_sim    # sim only (no Quanser SDK needed)
make run_balance run_hardware read_fast  # hardware targets (need SDK)
```

Requires: `g++` with C++17, Quanser SDK (`libhil`, `libquanser_common`) for hardware targets.

## Architecture

```
cpp/
├── qube_types.h       # QubeState, QubeParams, constants
├── plant.h            # Nonlinear EOM + RK4 integrator (header-only)
├── controllers.h      # BalanceController + SwingUpController (header-only)
├── run_sim.cpp        # Offline simulation
├── run_balance.cpp    # Balance-only mode (WORKING)
├── run_hardware.cpp   # Full swing-up + balance (WIP)
├── read_fast.cpp      # Encoder poll rate benchmark
└── Makefile
```

## Hardware Notes

- **Encoder CPR:** 2048 counts/rev (512 lines × 4X quadrature)
- **Encoder zero:** Hanging down = 0°, upright = 180°
- **Alpha convention:** 0 = upright, ±π = hanging down (offset + sign flip applied in code)
- **Motor voltage:** ±10V via analog output (amp internally limits to 15V)
- **Amplifier deadband:** ~0.3V (compensated in software)
- **Pendulum attachment:** Magnetic — detaches if swing-up is too aggressive (mu > ~10)
- **Arm endstop:** ~±130° from center
- **Achieved poll rate:** ~13 MHz (C HIL API)

## Controller Design

### Balance (LQR-derived state feedback)
```
u = -k_theta·θ - k_alpha·α - k_theta_dot·θ̇ - k_alpha_dot·α̇ - k_integral·∫α
```
Gains found via parameter sweep (no overshoot, 2 zero crossings):
- k_alpha = 20.0, k_alpha_dot = 2.0
- k_theta = -2.0, k_theta_dot = -1.0
- Voltage limit: 6V (prevents saturation → smooth response)

### Swing-up (Åström energy pumping, WIP)
- Energy-based with velocity deadzone (prevents startup chatter)
- Arm angle protection with soft braking zone
- Deadband compensation (+0.3V on all nonzero commands)
