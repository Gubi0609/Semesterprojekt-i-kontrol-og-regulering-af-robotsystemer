# Branch Overview — Qube-Servo 3 Inverted Pendulum Project

> Auto-generated heads-up document. Last updated: 2026-05-12

---

## Branch Summary at a Glance

| Branch | Purpose | Status | Key Deliverable |
|--------|---------|--------|-----------------|
| `main` | Stable C++ balance controller | ✅ Working (balance), ⚠️ WIP (swing-up) | `run_balance`, `run_sim` |
| `modelling` | MATLAB/Simulink modelling & classical control design | 🔧 Active development | `PendulumModel.mlx` (primary), Simulink, lead compensator |
| `SwingUp` | Full C++ controller suite + GUI + test framework | ✅ Most complete | GUI, test suite, swing-up + balance on hardware |
| `modern` | Pole placement / modern control notes | 📝 Notes only | `NOTES_modern_control.md` (torque→voltage conversion) |
| `cpp` | Identical to `main` | 🪞 Mirror | Same as `main` |
| `python` | Initial Python implementation | 🗄️ Abandoned | Single-commit scaffold |

---

## Detailed Branch Descriptions

### `modelling` — MATLAB Modelling & Classical Control

**This is where the analytical/modelling work lives.**

The **primary document** is `Matlab modelling/PendulumModel.mlx` — the Euler-Lagrange derivation of the system dynamics. Everything else in this branch is a spin-off or experiment.

#### Timeline of work (Mar 23 – Apr 29)
1. **Euler-Lagrange derivation** (Mar 23–28): DH parameters, Jacobians, inertia tensors, linearization, transfer function. Multiple iterations to get the math right.
2. **Real-world parameter insertion** (Apr 7): Physical values plugged in, transfer function simplified to 2nd-order.
3. **Root locus & lead compensator** (Apr 7–14): Zeta/sigma/omega calculations from system requirements (overshoot, settling time). Visual lead compensator added.
4. **Simulink PID** (Apr 17–21): `SimplePID.slx` — PID balance in Simulink. Also `ControlSystemDesignerSession1.mat`.
5. **Voltage-input formulation** (Apr 28–29): `LQRTestVolt.mlx`, `PoleTestVolt.mlx` — attempts at expressing torque as voltage for LQR/pole-placement.

#### Key files
| File | Description |
|------|-------------|
| `Matlab modelling/PendulumModel.mlx` | **★ Main modelling document.** Euler-Lagrange, DH, linearization, transfer function |
| `Matlab modelling/SimplePID.slx` | Simulink PID balance model |
| `Matlab modelling/ControlSystemDesignerSession1.mat` | Saved Control System Designer session |
| `LQRTestVolt.mlx` | LQR with voltage input (spin-off) |
| `PoleTestVolt.mlx` | Pole placement with voltage input (spin-off) |
| `PoleTest.mlx` | Earlier pole test (spin-off) |
| `Matlab modelling/quanser-cube-*.svg/png` | System illustrations with DH parameters |

#### Spin-off `.mlx` files (fruitless per team note)
- `LQRTestVolt.mlx`, `PoleTest.mlx`, `PoleTestVolt.mlx` — experimental scripts that didn't lead anywhere. Use `PendulumModel.mlx` as the canonical source.

#### Also on this branch (from SwingUp merge or shared history)
- Compiled C++ binaries, CSV logs, analysis plots — artifacts from simulation/hardware runs that got committed.

---

### `SwingUp` — Full Controller Suite (Most Complete Branch)

**The most feature-rich branch.** Has everything from `main` plus GUI, test suite, friction identification, swing-up that actually works, and the full MATLAB modelling folder.

#### What it adds over `main`
- **Energy-based swing-up controller** — Åström method with auto-kick, confirmed working
- **3D GUI** (`run_gui.cpp`) — Dear ImGui + ImPlot + OpenGL with real-time 3D model, live plots, parameter sliders
- **Hardware GUI** (`run_gui_hw`) — same GUI driving real hardware
- **Automated test suite** (`run_tests.cpp`) — 27 tests: balance (7), swing-up (6), robustness (8), noise (6)
- **CSV analyzer** (`analyze_csv.cpp`) — post-hoc analysis of hardware logs
- **Friction test** (`run_friction_test.cpp`) — hardware program to measure b₁ and b₂
- **3D renderer** (`render3d.h`) — OpenGL pendulum visualization
- **Plot script** (`plot_run.py`) — Python plotting for CSV data
- Vendor libraries: full `imgui` and `implot` source trees

#### Key MATLAB files (shared with modelling)
- `PendulumModel.mlx` — same Euler-Lagrange model
- `SimplePID.slx` — Simulink PID
- `SESSION_NOTES.md` — detailed session log from 2026-03-31 (friction tests, transfer function derivation, PID tuning notes, known issues)
- `friction_identification.md` — full procedure for measuring motor/pendulum friction coefficients

#### Key finding from friction tests
Both b₁ (motor) and b₂ (pendulum) ≈ 0 — negligible friction. Actual km ≈ 0.041 V/(rad/s) vs datasheet 0.0422.

---

### `modern` — Pole Placement Notes

**One commit ahead of `SwingUp`.** Contains everything from SwingUp plus a single additional file:

- **`cpp/NOTES_modern_control.md`** — detailed notes on:
  - Converting the torque-input state-space model to voltage-input (the 3-line motor dynamics conversion from the Quanser ROTPEN Workbook)
  - Why poles [-5,-6,-7,-8] are too slow (unstable open-loop pole is +8 to +20 rad/s)
  - Reference to Quanser's suggested poles: dominant at -2.8±j2.86, fast at -30 and -40
  - Step-by-step procedure for implementing `place()` in MATLAB then porting to C++

This branch is essentially a design notebook — no code changes beyond the notes file.

---

### `main` (and `cpp` — identical mirror)

**The stable baseline.** Balance controller works on hardware; swing-up is WIP.

#### Contents
- `cpp/controllers.h` — `BalanceController` (LQR-derived state feedback) + `SwingUpController` (energy pumping, incomplete)
- `cpp/plant.h` — nonlinear EOM + RK4 integrator (header-only)
- `cpp/qube_types.h` — `QubeState`, `QubeParams`, constants
- `cpp/run_balance.cpp` — balance-only mode (working)
- `cpp/run_hardware.cpp` — full swing-up + balance (WIP — gets ~34° from upright)
- `cpp/run_sim.cpp` — offline simulation
- `cpp/read_fast.cpp` — encoder poll rate benchmark (~13 MHz achieved)
- `calibration.json` — hardware calibration data

#### Balance gains
```
k_alpha = 20.0, k_alpha_dot = 2.0
k_theta = -2.0, k_theta_dot = -1.0
Voltage limit: 6V
```

---

### `python` — Initial Python Scaffold

**Single commit, abandoned.** Contains the initial Python implementation skeleton (sim, hardware interface, LQR balance + swing-up controllers). No further development. The project moved to C++ early on.

---

## Branch Relationships

```
                    Initial commit
                         │
                     ┌───┴───┐
                     │       │
                  python   cpp/main
                (abandoned)  │
                         ┌───┴──────────────────┐
                         │                      │
                      SwingUp              modelling
                   (full C++ suite +      (MATLAB focus,
                    MATLAB modelling)      classical control)
                         │
                      modern
                   (+pole placement notes)
```

## Recommendations

1. **`SwingUp` is the most complete branch** — if you need a single branch to build on, start there.
2. **`PendulumModel.mlx` on `modelling`** is the canonical modelling document — reference this for the report.
3. **`NOTES_modern_control.md` on `modern`** has the torque→voltage conversion that bridges MATLAB modelling to the C++ controller — useful for documenting the design process.
4. **`modelling` has accumulated build artifacts** (binaries, CSVs, plots) that should probably be cleaned up or `.gitignore`d.
5. **`python` and `cpp`** branches can be safely ignored — `python` was abandoned, `cpp` is identical to `main`.
