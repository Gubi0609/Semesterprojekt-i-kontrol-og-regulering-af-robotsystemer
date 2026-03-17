#pragma once
// Common types and constants for Qube-Servo 3 pendulum control.

#include <cmath>

// Qube-Servo 3 encoder: 512 lines × 4X quadrature = 2048 counts/rev
constexpr int    COUNTS_PER_REV  = 2048;
constexpr double COUNTS_TO_RAD   = (2.0 * M_PI) / COUNTS_PER_REV;

// Motor voltage limits
// Qube-Servo 3 amp internally clips at 15V. Analog output range is ±10V.
// PWM mode maps ±1 to ±24V, but firmware limits to 15V.
// Using analog output: safe to command up to ±10V (hardware clamps the rest).
constexpr double V_MAX = 10.0;

// Arm angle safety limit (radians) — prevent hitting endstop
constexpr double ARM_LIMIT_RAD     = 60.0 * M_PI / 180.0;
constexpr double ARM_SOFT_ZONE_RAD = 30.0 * M_PI / 180.0;

struct QubeState {
    double theta;      // motor/arm angle [rad]
    double alpha;      // pendulum angle from upright [rad] (0 = balanced)
    double theta_dot;  // motor angular velocity [rad/s]
    double alpha_dot;  // pendulum angular velocity [rad/s]
};

// Physical parameters (Qube-Servo 3 + pendulum module)
struct QubeParams {
    double Rm   = 8.4;       // Motor resistance [Ohm]
    double km   = 0.042;     // Motor torque/back-EMF constant [N·m/A]
    double Jr   = 4.0e-6;    // Rotor inertia [kg·m²]
    double Lr   = 0.0826;    // Arm length [m]
    double Dr   = 0.0;       // Arm viscous damping [N·m·s/rad]
    double mp   = 0.024;     // Pendulum mass [kg]
    double Lp   = 0.129;     // Pendulum length [m]
    double Jp   = 3.33e-5;   // Pendulum inertia [kg·m²]
    double Dp   = 0.0;       // Pendulum damping [N·m·s/rad]
    double g    = 9.81;      // Gravity [m/s²]

    double half_Lp() const { return Lp / 2.0; }
    double Jt()      const { return Jr + mp * Lr * Lr; }
};

inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

inline double wrap_angle(double a) {
    a = fmod(a + M_PI, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a - M_PI;
}
