#!/usr/bin/env python3
"""
Cart-Pole Simplification of the Qube-Servo 3 Rotary Inverted Pendulum
======================================================================

Instead of modelling the full rotary system (two coupled rotations),
we approximate the arm tip as a cart moving in a straight line:

    x ≈ Lr · θ       (valid for small θ)

This converts the system into the classic cart-pole (inverted pendulum
on a cart) which has well-known, simple equations of motion.

Mapping from rotary to cart-pole
---------------------------------
    Cart position  x  = Lr · θ
    Cart mass      M  = Jr / Lr²     (arm-only effective mass, WITHOUT pendulum)
    Pendulum mass  m  = mp
    Pendulum CoM   l  = Lp / 2
    Pendulum inertia I = Jp          (about pivot)
    Motor force    F  = τ / Lr = km·(Vm − km·ẋ/Lr) / (Rm·Lr)

IMPORTANT: M = Jr/Lr², NOT Jt/Lr².  Jt = Jr + mp·Lr² already includes
the pendulum mass at the arm tip.  In the standard cart-pole EOM the
(M + m) term adds the pendulum mass separately, so using Jt/Lr² would
double-count mp.  With M = Jr/Lr² we get:
    M + m = Jr/Lr² + mp = Jt/Lr²    ← correct total translational inertia.

Cart-pole EOM (nonlinear, α = 0 is upright):
    (M + m)·ẍ − m·l·cos(α)·α̈ + m·l·sin(α)·α̇² = F
    −m·l·cos(α)·ẍ + (I + m·l²)·α̈ − m·g·l·sin(α) = 0

Linearised transfer function Vm → α (2nd order, eliminating x):
    G(s) = b / (s² − a)

This should match the rotary model's reduced TF almost exactly.
"""

import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# ═══════════════════════════════════════════════════════════════════
# System parameters — from qube_types.h (SwingUp branch)
# ═══════════════════════════════════════════════════════════════════
Rm  = 7.5        # Motor resistance [Ohm]
km  = 0.042      # Motor torque/back-EMF constant [N·m/A = V·s/rad]
Jr  = 4.0e-6     # Rotor inertia [kg·m²]
Lr  = 0.0826     # Arm length (shaft to pivot) [m]
Dr  = 0.0        # Arm viscous damping [N·m·s/rad]
mp  = 0.024      # Pendulum mass [kg]
Lp  = 0.129      # Pendulum total length [m]
Jp  = 3.33e-5    # Pendulum inertia about pivot [kg·m²]
Dp  = 0.0        # Pendulum damping [N·m·s/rad]
g   = 9.81       # Gravity [m/s²]

# ═══════════════════════════════════════════════════════════════════
# Derived cart-pole parameters
# ═══════════════════════════════════════════════════════════════════
l   = Lp / 2.0                     # Pendulum CoM distance [m]
Jt  = Jr + mp * Lr * Lr            # Total arm-side inertia [kg·m²]
M   = Jr / (Lr * Lr)               # Effective cart mass (arm ONLY) [kg]
I   = Jp                           # Pendulum inertia about pivot [kg·m²]

print("=" * 60)
print("CART-POLE PARAMETER MAPPING")
print("=" * 60)
print(f"  Rotor inertia Jr               = {Jr:.6e} kg·m²")
print(f"  Arm+pend inertia Jt = Jr+mp·Lr²= {Jt:.6e} kg·m²")
print(f"  Cart mass M  = Jr/Lr²          = {M:.6f} kg")
print(f"  Pendulum mass m                = {mp} kg")
print(f"  Sanity: M + m = {M+mp:.6f}  vs  Jt/Lr² = {Jt/Lr**2:.6f}")
print(f"  Pendulum CoM length l = Lp/2   = {l} m")
print(f"  Pendulum inertia I = Jp        = {I:.6e} kg·m²")
print()

# ═══════════════════════════════════════════════════════════════════
# Linearised state-space:  x = [x, alpha, x_dot, alpha_dot]
#                          u = Vm (motor voltage)
# ═══════════════════════════════════════════════════════════════════

# Denominator of the mass matrix inverse
Delta = (M + mp) * (I + mp * l**2) - (mp * l)**2

print(f"  Mass matrix determinant Δ = {Delta:.6e}")
print(f"  (Δ > 0 required: {'OK' if Delta > 0 else 'PROBLEM!'})")
print()

# Motor force coefficients (F = b_v · Vm − b_d · ẋ)
b_v = km / (Rm * Lr)               # Force per volt (at zero speed)
b_d = km**2 / (Rm * Lr**2)         # Back-EMF damping coefficient

print(f"  Motor force gain   b_v = km/(Rm·Lr)   = {b_v:.6f} N/V")
print(f"  Back-EMF damping   b_d = km²/(Rm·Lr²) = {b_d:.6f} N·s/m")
print()

# ── State-space A, B ───────────────────────────────────────────────
# Derived by solving the 2×2 linearised EOM for ẍ and α̈.
# Back-EMF enters through the force: F = b_v·Vm − b_d·ẋ
# The −b_d·ẋ term adds damping to both ẍ and α̈ equations.

A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, (mp * l)**2 * g / Delta,
       -(I + mp * l**2) * b_d / Delta, 0],
    [0, mp * g * l * (M + mp) / Delta,
       -mp * l * b_d / Delta, 0],
])

B = np.array([
    [0],
    [0],
    [(I + mp * l**2) * b_v / Delta],
    [mp * l * b_v / Delta],
])

C_full = np.eye(4)
D_full = np.zeros((4, 1))

print("=" * 60)
print("STATE-SPACE MATRICES (linearised about upright)")
print("  State: [x, α, ẋ, α̇],  Input: Vm")
print("=" * 60)
print(f"\nA =\n{np.array2string(A, precision=4, suppress_small=True)}")
print(f"\nB = {B.flatten()}")
print()

# ═══════════════════════════════════════════════════════════════════
# Open-loop eigenvalues
# ═══════════════════════════════════════════════════════════════════
eigenvalues = np.linalg.eigvals(A)
print("Open-loop eigenvalues:")
for ev in sorted(eigenvalues, key=lambda x: x.real):
    if abs(ev.imag) < 1e-10:
        print(f"  s = {ev.real:+.4f}")
    else:
        print(f"  s = {ev.real:+.4f} ± {abs(ev.imag):.4f}j")
print()

# ═══════════════════════════════════════════════════════════════════
# Reduced 2nd-order transfer function Vm → α
# ═══════════════════════════════════════════════════════════════════
a_tf = mp * g * l * (M + mp) / Delta
b_tf = mp * l * b_v / Delta

print("=" * 60)
print("TRANSFER FUNCTION  Vm → α  (2nd order, x eliminated)")
print("=" * 60)
print(f"  G(s) = {b_tf:.2f} / (s² − {a_tf:.2f})")
print(f"  Unstable pole at s = +{np.sqrt(a_tf):.2f} rad/s")
print(f"  Stable   pole at s = −{np.sqrt(a_tf):.2f} rad/s")
print()
print("Compare with full rotary model (SESSION_NOTES.md):")
print("  G_rot(s)  = 120.2 / (s² − 425.6)")
print(f"  G_cart(s) = {b_tf:.1f} / (s² − {a_tf:.1f})")
pct_a = abs(a_tf - 425.6) / 425.6 * 100
pct_b = abs(b_tf - 120.2) / 120.2 * 100
print(f"  Denominator error: {pct_a:.1f}%")
print(f"  Numerator error:   {pct_b:.1f}%")
print()

# ═══════════════════════════════════════════════════════════════════
# Controllability check
# ═══════════════════════════════════════════════════════════════════
n = A.shape[0]
ctrb = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
rank = np.linalg.matrix_rank(ctrb)
print(f"Controllability: rank = {rank}/{n}  {'✓' if rank == n else '✗'}")
print()

# ═══════════════════════════════════════════════════════════════════
# Nonlinear simulation models
# ═══════════════════════════════════════════════════════════════════

def cart_pole_dynamics(state, Vm):
    """Nonlinear cart-pole EOM. state = [x, alpha, x_dot, alpha_dot]"""
    x, alpha, x_dot, alpha_dot = state
    Vm = np.clip(Vm, -10, 10)

    sin_a = np.sin(alpha)
    cos_a = np.cos(alpha)

    # Motor force on cart (including back-EMF via theta_dot = x_dot/Lr)
    theta_dot = x_dot / Lr
    tau = km * (Vm - km * theta_dot) / Rm
    F = tau / Lr

    # Mass matrix (standard cart-pole, α=0 upright)
    #   [(M+m)       −m·l·cos(α)] [ẍ ]   [F + m·l·α̇²·sin(α)]
    #   [−m·l·cos(α)  I+m·l²    ] [α̈] = [m·g·l·sin(α)      ]
    M11 = M + mp
    M12 = -mp * l * cos_a
    M22 = I + mp * l**2

    rhs1 = F + mp * l * alpha_dot**2 * sin_a
    rhs2 = mp * g * l * sin_a

    det = M11 * M22 - M12**2
    x_dd     = (M22 * rhs1 - M12 * rhs2) / det
    alpha_dd = (-M12 * rhs1 + M11 * rhs2) / det

    return np.array([x_dot, alpha_dot, x_dd, alpha_dd])


def rotary_dynamics(state, Vm):
    """Full rotary EOM from plant.h. state = [theta, alpha, theta_dot, alpha_dot]"""
    theta, alpha, theta_dot, alpha_dot = state
    Vm = np.clip(Vm, -10, 10)

    sin_a = np.sin(alpha)
    cos_a = np.cos(alpha)
    hLp = Lp / 2.0

    tau = km * (Vm - km * theta_dot) / Rm

    M11 = Jt + Jp * sin_a**2
    M12 = -mp * hLp * Lr * cos_a
    M22_r = Jp + mp * hLp**2

    rhs1 = (tau - Dr * theta_dot
            - mp * hLp * Lr * alpha_dot**2 * sin_a
            - 2.0 * Jp * sin_a * cos_a * theta_dot * alpha_dot)
    rhs2 = (-Dp * alpha_dot
            + mp * hLp * g * sin_a
            + Jp * sin_a * cos_a * theta_dot**2)

    det = M11 * M22_r - M12**2
    theta_dd = (M22_r * rhs1 - M12 * rhs2) / det
    alpha_dd = (M11 * rhs2 - M12 * rhs1) / det

    return np.array([theta_dot, alpha_dot, theta_dd, alpha_dd])


def rk4_step(f, state, u, dt):
    k1 = f(state, u)
    k2 = f(state + 0.5 * dt * k1, u)
    k3 = f(state + 0.5 * dt * k2, u)
    k4 = f(state + dt * k3, u)
    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


# ═══════════════════════════════════════════════════════════════════
# PD balance controller (same gains for both models)
# ═══════════════════════════════════════════════════════════════════

def balance_voltage(alpha, alpha_dot, theta, theta_dot):
    """
    PD state-feedback controller.
    For cart-pole: convert x→θ before calling:  theta = x/Lr, theta_dot = x_dot/Lr
    """
    k_alpha     = 20.0
    k_alpha_dot = 2.0
    k_theta     = -2.0
    k_theta_dot = -1.0

    u = (-k_alpha * alpha - k_alpha_dot * alpha_dot
         - k_theta * theta - k_theta_dot * theta_dot)
    return np.clip(u, -6.0, 6.0)


# ═══════════════════════════════════════════════════════════════════
# Run comparative simulation
# ═══════════════════════════════════════════════════════════════════
dt = 0.001        # 1 kHz
T  = 3.0          # seconds
steps = int(T / dt)

alpha0 = 5.0 * np.pi / 180.0   # 5° initial tilt

# Rotary: [theta, alpha, theta_dot, alpha_dot]
state_rot = np.array([0.0, alpha0, 0.0, 0.0])
# Cart:   [x, alpha, x_dot, alpha_dot]
state_cart = np.array([0.0, alpha0, 0.0, 0.0])

t_log          = np.zeros(steps)
alpha_rot_log  = np.zeros(steps)
alpha_cart_log = np.zeros(steps)
theta_rot_log  = np.zeros(steps)
theta_cart_log = np.zeros(steps)  # x/Lr for comparison
Vm_rot_log     = np.zeros(steps)
Vm_cart_log    = np.zeros(steps)

for i in range(steps):
    t_log[i] = i * dt

    # Rotary controller
    Vm_rot = balance_voltage(state_rot[1], state_rot[3],
                             state_rot[0], state_rot[2])
    # Cart controller (convert x → equivalent θ for the same gains)
    Vm_cart = balance_voltage(state_cart[1], state_cart[3],
                              state_cart[0] / Lr, state_cart[2] / Lr)

    alpha_rot_log[i]  = state_rot[1]
    alpha_cart_log[i] = state_cart[1]
    theta_rot_log[i]  = state_rot[0]
    theta_cart_log[i] = state_cart[0] / Lr   # equivalent angle
    Vm_rot_log[i]     = Vm_rot
    Vm_cart_log[i]    = Vm_cart

    state_rot  = rk4_step(rotary_dynamics, state_rot, Vm_rot, dt)
    state_cart = rk4_step(cart_pole_dynamics, state_cart, Vm_cart, dt)

# ═══════════════════════════════════════════════════════════════════
# Plot comparison
# ═══════════════════════════════════════════════════════════════════
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axes[0].plot(t_log, np.degrees(alpha_rot_log), label='Rotary (full)', lw=1.5)
axes[0].plot(t_log, np.degrees(alpha_cart_log), '--', label='Cart-pole (simplified)', lw=1.5)
axes[0].set_ylabel('α [deg]')
axes[0].set_title('Pendulum angle — Rotary vs Cart-Pole (α₀ = 5°)')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].plot(t_log, np.degrees(theta_rot_log), label='θ rotary', lw=1.5)
axes[1].plot(t_log, np.degrees(theta_cart_log), '--', label='x/Lr cart-pole', lw=1.5)
axes[1].set_ylabel('θ [deg]')
axes[1].set_title('Arm angle (rotary θ vs equivalent cart x/Lr)')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

axes[2].plot(t_log, Vm_rot_log, label='Vm rotary', lw=1.5)
axes[2].plot(t_log, Vm_cart_log, '--', label='Vm cart-pole', lw=1.5)
axes[2].set_ylabel('Vm [V]')
axes[2].set_xlabel('Time [s]')
axes[2].set_title('Control voltage')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('cart_pole_comparison.png', dpi=150)
print("Saved: cart_pole_comparison.png")

# ═══════════════════════════════════════════════════════════════════
# Approximation error vs initial angle
# ═══════════════════════════════════════════════════════════════════
print("\n" + "=" * 60)
print("APPROXIMATION ERROR vs INITIAL ANGLE")
print("=" * 60)
header = f"{'α₀ [deg]':>10} {'max|Δα| [deg]':>16} {'max|Δθ| [deg]':>16} {'RMS Δα [deg]':>14} {'Note':>14}"
print(header)
print("-" * len(header))

for alpha0_deg in [1, 2, 5, 10, 15, 20, 30, 45]:
    alpha0 = alpha0_deg * np.pi / 180.0
    sr = np.array([0.0, alpha0, 0.0, 0.0])
    sc = np.array([0.0, alpha0, 0.0, 0.0])

    max_da = 0.0
    max_dt = 0.0
    sum_da2 = 0.0
    stable = True

    for i in range(steps):
        Vm_r = balance_voltage(sr[1], sr[3], sr[0], sr[2])
        Vm_c = balance_voltage(sc[1], sc[3], sc[0] / Lr, sc[2] / Lr)

        da = abs(sr[1] - sc[1])
        dt_eq = abs(sr[0] - sc[0] / Lr)
        max_da = max(max_da, da)
        max_dt = max(max_dt, dt_eq)
        sum_da2 += da**2

        sr = rk4_step(rotary_dynamics, sr, Vm_r, dt)
        sc = rk4_step(cart_pole_dynamics, sc, Vm_c, dt)

        if abs(sr[1]) > np.pi or abs(sc[1]) > np.pi:
            stable = False
            break

    rms_da = np.sqrt(sum_da2 / steps)

    if not stable:
        note = "UNSTABLE"
    elif np.degrees(max_da) < 0.1:
        note = "excellent"
    elif np.degrees(max_da) < 0.5:
        note = "good"
    elif np.degrees(max_da) < 2.0:
        note = "acceptable"
    else:
        note = "poor"

    print(f"{alpha0_deg:>10} {np.degrees(max_da):>16.4f} {np.degrees(max_dt):>16.4f} {np.degrees(rms_da):>14.4f} {note:>14}")

# ═══════════════════════════════════════════════════════════════════
# Multi-angle comparison plot
# ═══════════════════════════════════════════════════════════════════
fig2, axes2 = plt.subplots(2, 2, figsize=(12, 8))

for idx, alpha0_deg in enumerate([5, 15, 30, 45]):
    ax = axes2[idx // 2][idx % 2]
    alpha0 = alpha0_deg * np.pi / 180.0
    sr = np.array([0.0, alpha0, 0.0, 0.0])
    sc = np.array([0.0, alpha0, 0.0, 0.0])

    t_arr = np.zeros(steps)
    a_rot = np.zeros(steps)
    a_cart = np.zeros(steps)

    for i in range(steps):
        t_arr[i] = i * dt
        a_rot[i] = sr[1]
        a_cart[i] = sc[1]

        Vm_r = balance_voltage(sr[1], sr[3], sr[0], sr[2])
        Vm_c = balance_voltage(sc[1], sc[3], sc[0] / Lr, sc[2] / Lr)
        sr = rk4_step(rotary_dynamics, sr, Vm_r, dt)
        sc = rk4_step(cart_pole_dynamics, sc, Vm_c, dt)

    ax.plot(t_arr, np.degrees(a_rot), label='Rotary', lw=1.5)
    ax.plot(t_arr, np.degrees(a_cart), '--', label='Cart-pole', lw=1.5)
    ax.set_title(f'α₀ = {alpha0_deg}°')
    ax.set_ylabel('α [deg]')
    ax.set_xlabel('Time [s]')
    ax.legend()
    ax.grid(True, alpha=0.3)

fig2.suptitle('Cart-Pole vs Rotary: Balance from various initial angles', fontsize=14)
plt.tight_layout()
plt.savefig('cart_pole_multi_angle.png', dpi=150)
print("Saved: cart_pole_multi_angle.png")

plt.show()
