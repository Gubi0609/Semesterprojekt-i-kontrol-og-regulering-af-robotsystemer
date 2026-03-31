# Friction Coefficient Identification — Qube Servo 3

We need to find two friction coefficients: **b₁** (motor arm) and **b₂** (pendulum).

## Known Motor Parameters

| Symbol | Description                        | Value                  |
|--------|------------------------------------|------------------------|
| Vnom   | Nominal input voltage              | 24.0 V                |
| τnom   | Nominal torque                     | 20.4 mN·m             |
| ωnom   | Nominal speed                      | 5400 RPM               |
| Inom   | Nominal current                    | 0.5 A                  |
| Rm     | Terminal resistance                | 7.5 Ω                  |
| kt     | Torque constant                    | 0.0422 N·m/A           |
| km     | Motor back-EMF constant            | 0.0422 V/(rad/s)       |
| Jm     | Rotor inertia                      | 1.4e-6 kg·m²           |
| Lm     | Rotor inductance                   | 1.15 mH                |
| mh     | Module attachment hub mass         | 0.0106 kg              |
| rh     | Module attachment hub radius       | 0.0111 m               |
| Jh     | Module attachment moment of inertia| 0.6e-6 kg·m²           |

---

## Test 1: b₁ — Motor arm viscous friction (steady-state spin test)

### Setup
1. Remove the pendulum (or let it hang freely, spin slowly)
2. Apply a known constant voltage V to the motor
3. Wait for steady-state (constant angular velocity dθ₁)
4. Record V and dθ₁

### Math
At steady state, q̈ = 0, so the motor equation is just:

```
τ_m = b₁ · dθ₁
```

The motor torque from voltage is:

```
τ_m = kt · (V - km · dθ₁) / Rm
```

So:

```
b₁ = τ_m / dθ₁ = kt · (V - km · dθ₁) / (Rm · dθ₁)
```

### Steps
1. Apply e.g. V = 2V (start low!)
2. Read steady-state speed from encoder (rad/s)
3. Plug into formula above
4. Repeat at a few voltages (e.g. 1V, 2V, 3V) and average

### Example calculation
If V = 2V and you measure dθ₁ = 50 rad/s:

```
τ_m = 0.0422 * (2 - 0.0422 * 50) / 7.5
    = 0.0422 * (2 - 2.11) / 7.5
    = 0.0422 * (-0.11) / 7.5
    ≈ -0.000619 N·m   (negative means you need a higher voltage or the speed is wrong)
```

Pick voltages where (V - km·dθ₁) is clearly positive, i.e. the motor hasn't reached back-EMF equilibrium yet.

---

## Test 2: b₂ — Pendulum viscous friction (free decay test)

### Setup
1. Hold the arm fixed (don't power motor, or clamp it)
2. Lift pendulum to some angle (e.g. 30-45° from hanging)
3. Release and record θ₂ vs time
4. Measure peak amplitudes over several swings

### Math
The pendulum is an underdamped oscillator. Amplitude decays as e^(-ζωₙt).

Pick two peaks separated by n complete cycles:
- A₁ = amplitude of first peak
- A₂ = amplitude of peak n cycles later

```
δ = ln(A₁ / A₂) / n                     (logarithmic decrement per cycle)
ζ = δ / sqrt(4π² + δ²)                   (damping ratio)
ωₙ = 2π / T                              (natural frequency, T = period of one cycle)
b₂ = 2 · ζ · ωₙ · J₂                    (viscous friction coefficient)
```

Where J₂ is the pendulum moment of inertia about its pivot (= m₂·l₂²/3 for a uniform rod).

### Steps
1. Record angle data at a decent sample rate (100+ Hz)
2. Find at least 5-10 peaks
3. Compute δ between first and last peak for better accuracy
4. Plug into formulas above

---

---

## Results

### Test 2: b₂ — Pendulum decay

**First attempt**: arm was free to rotate. The arm-pendulum coupling drained energy
from the pendulum faster than friction alone, giving a falsely high b₂ ≈ 3.7e-4 and
a measured ω_n (15.1 rad/s) that didn't match theory (10.7 rad/s). Rejected.

**Second attempt**: arm held fixed. Much cleaner:
- Initial deflection: ~26° from hanging
- Decay over ~10s: 26.5° → 18.6° (very slow)
- Measured ω_n = 10.89 rad/s ≈ theoretical 10.68 rad/s ✅
- Pairwise log decrements very small and noisy (encoder quantization)
- **b₂ ≈ 7.9e-6 N·m·s/rad — essentially zero**

### Test 1: b₁ — Motor steady-state spin

Ran with inertia disk attached (no pendulum) at 2V and 3V.

| Voltage | Steady-state speed | back-EMF     | τ_motor        | b₁             |
|---------|--------------------|-------------- |----------------|----------------|
| 2.0 V   | 49.3 rad/s (471 RPM) | 2.08 V     | -0.000457 N·m  | ≈ 0 (negative) |
| 3.0 V   | 73.4 rad/s (701 RPM) | 3.10 V     | -0.000559 N·m  | ≈ 0 (negative) |

The back-EMF **exceeds** the applied voltage in both cases, meaning the motor reaches
(and slightly surpasses) its theoretical no-load speed. Friction is below the noise
floor of this measurement method. **b₁ ≈ 0.**

The slightly negative values indicate the datasheet km (0.0422) is a bit high.
Implied km from measured data:
- From 2V run: km ≈ 0.0406 V/(rad/s)
- From 3V run: km ≈ 0.0409 V/(rad/s)
- Datasheet:   km = 0.0422 V/(rad/s)

Actual km is closer to **0.041 V/(rad/s)**.

### Conclusion

Both friction coefficients are negligible:
- **b₁ (Dr) ≈ 0** — motor arm viscous friction
- **b₂ (Dp) ≈ 0** — pendulum viscous friction

Setting Dr = 0.0 and Dp = 0.0 in the model (as already done in qube_types.h) is correct.

If the transfer function math requires nonzero friction, use token values like 1e-5.

### Bonus: km correction
The actual motor back-EMF constant is ~0.041 V/(rad/s), slightly lower than the
datasheet value of 0.0422. Consider updating if simulation accuracy matters.

---

## Notes
- b₂ will likely be very small (order of 1e-4 to 1e-3 N·m·s/rad)
- b₁ should also be small but larger than b₂
- If results seem weird, double-check units (rad/s not RPM, V not mV, etc.)
- The encoder on the Qube gives 2048 counts/rev (512 lines × 4x quadrature)
