#pragma once
// ═══════════════════════════════════════════════════════════════════
// qube_types.h — Common types and constants for Qube-Servo 3 control
// ═══════════════════════════════════════════════════════════════════

#include <cmath>

// ─── Encoder conversion ────────────────────────────────────────────
// The Qube-Servo 3 has 512-line optical encoders decoded in 4X
// quadrature mode, giving 2048 discrete positions per revolution.
// COUNTS_TO_RAD converts a raw encoder count to radians.
constexpr int    COUNTS_PER_REV  = 2048;
constexpr double COUNTS_TO_RAD   = (2.0 * M_PI) / COUNTS_PER_REV;

// ─── Voltage limits ────────────────────────────────────────────────
// The analog output range is ±10V. The onboard amplifier can go up
// to ±24V in PWM mode but the firmware limits it to 15V. Since we
// use analog output, ±10V is the hard ceiling.
constexpr double V_MAX = 10.0;

// ─── Arm safety limits ────────────────────────────────────────────
// The motor arm has a mechanical endstop at roughly ±130° from center.
// ARM_LIMIT_RAD is the software hard limit — the swing-up controller
// overrides with full braking beyond this angle.
// ARM_SOFT_ZONE_RAD defines how far before the limit the controller
// starts tapering its output and blending in a restoring force.
//
// Current settings: soft zone starts at 30°, hard limit at 60°.
//   |--- full authority ---|-- taper + brake --|X hard brake
//   0°                    30°                60°
constexpr double ARM_LIMIT_RAD     = 60.0 * M_PI / 180.0;
constexpr double ARM_SOFT_ZONE_RAD = 30.0 * M_PI / 180.0;

// ─── State vector ──────────────────────────────────────────────────
// The complete observable state of the system at one instant.
// Both controllers take this as input.
struct QubeState {
    double theta;      // motor/arm angle [rad] (0 = wherever it was at startup)
    double alpha;      // pendulum angle from upright [rad] (0 = balanced, ±π = down)
    double theta_dot;  // motor angular velocity [rad/s]
    double alpha_dot;  // pendulum angular velocity [rad/s]
};

// ─── Physical parameters ───────────────────────────────────────────
// Constants describing the Qube-Servo 3 hardware. Used by the
// simulation (plant.h) and the swing-up energy calculation.
// Values are from the Quanser Qube-Servo 3 datasheet.
struct QubeParams {
    double Rm   = 8.4;       // Motor terminal resistance [Ohm]
    double km   = 0.042;     // Motor torque constant = back-EMF constant [N·m/A = V·s/rad]
    double Jr   = 4.0e-6;    // Rotor (motor hub) moment of inertia [kg·m²]
    double Lr   = 0.0826;    // Arm length from motor shaft to pendulum pivot [m]
    double Dr   = 0.0;       // Arm viscous damping coefficient [N·m·s/rad]
    double mp   = 0.024;     // Pendulum mass [kg]
    double Lp   = 0.129;     // Pendulum total length (pivot to tip) [m]
    double Jp   = 3.33e-5;   // Pendulum moment of inertia about pivot [kg·m²]
    double Dp   = 0.0;       // Pendulum viscous damping coefficient [N·m·s/rad]
    double g    = 9.81;      // Gravitational acceleration [m/s²]

    // Convenience: half pendulum length (distance to center of mass)
    double half_Lp() const { return Lp / 2.0; }
    // Convenience: total effective inertia of arm + pendulum mass at arm tip
    double Jt()      const { return Jr + mp * Lr * Lr; }
};

// ─── Utility functions ─────────────────────────────────────────────

// Clamp a value to [lo, hi].
inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Wrap an angle to the range [-π, +π].
// This prevents encoder readings from accumulating past ±180°
// (e.g. if the pendulum detaches and reattaches while spinning).
inline double wrap_angle(double a) {
    a = fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}
