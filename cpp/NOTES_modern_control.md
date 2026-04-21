# Modern Control — Pole Placement Notes

## Status
- Branch: `modern`
- Euler-Lagrange model done in MATLAB (`PendulumModel.mlx`)
- Linearized A_ss, B_ss with torque input ✓
- Controllability check ✓ (det ≠ 0)
- Pole placement code written with `place()` ✓
- Observability issue with C=[0 1 0 0] — not a problem since we measure all 4 states on hardware

## Key Finding: Torque → Voltage Conversion
From the official Quanser ROTPEN Workbook (Instructor Version, 2011):

The A and B matrices are first derived with **torque** as input (τ_m).
Then the motor electrical dynamics are added to convert to **voltage** input:

```matlab
% For SRV02 (with gear train):
B = Kg * kt * B / Rm;
A(3,3) = A(3,3) - Kg^2*kt*km/Rm*B(3);
A(4,3) = A(4,3) - Kg^2*kt*km/Rm*B(4);
```

For the Qube-Servo 3 (direct drive, no gears: Kg=1, η_g=1, η_m=1):

```matlab
km = 0.042;  % V/(rad/s) — measured ~0.041, datasheet 0.0422
Rm = 7.5;    % Ohm

% Convert B from torque input to voltage input
B_volt = km * B_val / Rm;

% Add back-EMF damping to A
A_volt = A_val;
A_volt(3,3) = A_volt(3,3) - km^2/Rm * B_volt(3);
A_volt(4,3) = A_volt(4,3) - km^2/Rm * B_volt(4);

% Now design controller with voltage-based matrices
K = place(A_volt, B_volt, desired_poles);
% K gives: u_voltage = -K * x
```

## What the conversion does physically
- `B = km/Rm * B`: scales input from torque to voltage (τ = km*V/Rm when θ̇=0)
- `A(3,3) -= km²/Rm * B(3)`: back-EMF adds damping to θ̈ (motor resists spinning)
- `A(4,3) -= km²/Rm * B(4)`: back-EMF coupling to α̈ through the arm

## Your MATLAB pole placement issue
Your current poles [-5, -6, -7, -8] are too slow. The unstable open-loop pole
is around +8 to +20 rad/s depending on parameters. Closed-loop poles need to
be faster than that. Quanser's example uses dominant poles at -2.8±j2.86
with fast poles at -30 and -40.

## Next steps
1. Add the 3-line motor conversion after computing A_val, B_val
2. Pick better poles (e.g. dominant: ζ=0.7, ωn=4 → -2.8±j2.86, fast: -30, -40)
3. Run `K = place(A_volt, B_volt, poles)`
4. Implement `u = -K*x` in C++ (same structure as current LQR in controllers.h)
5. Test on hardware

## Reference
Quanser ROTPEN Workbook (Instructor Version), 2011
- Section 2.1.2: Nonlinear EOMs and torque equation (Eq 2.4)
- Section 2.3.1: ROTPEN_ABCD_eqns.m with motor dynamics conversion
- Section 3.2.4: Pole placement procedure
- Section 3.4.1: Control design with `acker`/`place`
