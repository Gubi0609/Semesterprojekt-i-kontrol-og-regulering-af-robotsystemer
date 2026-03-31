# Session Notes — 2026-03-31

## What we did
1. Reviewed the symbolic Euler-Lagrange derivation in `PendulumModel.mlx`
2. Identified missing friction coefficients (b1, b2)
3. Built `run_friction_test.cpp` to measure friction on hardware
4. Ran friction tests — both b1 and b2 are ≈ 0 (negligible friction)
5. Also discovered actual km ≈ 0.041 V/(rad/s) vs datasheet 0.0422
6. Built numeric transfer function scripts for Simulink
7. Got PID balance working in Simulink

## Key files
- `friction_identification.md` — full friction test procedure + results
- `PendulumTransferFunction.m` — first attempt at numeric TF (has inertia issues)
- `PendulumTransferFunction_v2.m` — TF from original DH model (different convention)
- `cpp/run_friction_test.cpp` — hardware friction test program

## Working Simulink setup
**Transfer function (reduced 2nd-order, from plant.h parameters):**
```
Numerator:    [120.2]
Denominator:  [1  0  -425.6]
```
This is: G(s) = 120.2 / (s² - 425.6)

Derived by eliminating theta (arm angle) from the coupled EOM,
using the exact parameters from plant.h / qube_types.h.

**PID gains (approximate, needs more tuning):**
- P ≈ 5-10
- I = 0
- D ≈ 0.5
- N = 20

Sum block: `+-` (negative feedback)
Step block: initial=0.1, final=0, step time=0.5

## Issues encountered
- PendulumTransferFunction.m (script 1) has wrong inertia formulation —
  M22 = Jp only, missing the mp*hLp² term. This makes the model predict
  stable oscillation instead of instability. Don't use it.
- The 4th-order transfer function (Vm → alpha) is NOT stabilisable with
  PID due to a zero at s=0 in the numerator. Must use the reduced
  2nd-order version.
- Control System Toolbox installation got stuck (two installers deadlocked).
  Used symbolic toolbox workaround instead.

## TODO next session
- [ ] Tune PID gains properly (reduce overshoot/oscillation)
- [ ] Try adding integral term for steady-state error
- [ ] Fix PendulumTransferFunction.m inertia terms to match plant.h
- [ ] Measure actual Lr (0.085 vs 0.0826 discrepancy)
- [ ] Measure actual Lp (datasheet 9.5cm vs workbook 12.9cm)
- [ ] Re-run 1V motor friction test (was overwritten)
- [ ] Get Control System Toolbox installed properly
