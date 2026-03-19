#pragma once
// ═══════════════════════════════════════════════════════════════════
// controllers.h — Balance and swing-up controllers
// ═══════════════════════════════════════════════════════════════════

#include "qube_types.h"
#include <cmath>

// ─── Deadband compensation ─────────────────────────────────────────
// The Qube-Servo 3 amplifier has a ~0.3V deadband near zero voltage.
// Any command below this threshold produces zero torque at the motor.
// To compensate, we add 0.3V to any nonzero command so the motor
// receives the intended voltage. Near-zero commands (|u| < 0.01V)
// are treated as exactly zero to avoid jitter.
inline double compensate_deadband(double u, double deadband = 0.3) {
    if (u > 0.01)       return u + deadband;
    else if (u < -0.01) return u - deadband;
    return 0.0;
}

// ═══════════════════════════════════════════════════════════════════
// Balance Controller — keeps the pendulum upright
// ═══════════════════════════════════════════════════════════════════
//
// Uses full state feedback (derived from LQR optimal control):
//
//   u = -k_theta·θ - k_alpha·α - k_theta_dot·θ̇ - k_alpha_dot·α̇ - k_integral·∫α
//
// Each gain has a physical role:
//   k_alpha     (20.0) — proportional: pushes arm to counteract pendulum tilt
//   k_alpha_dot  (2.0) — derivative: damps pendulum oscillation, prevents overshoot
//   k_theta     (-2.0) — proportional: pulls arm back toward center (θ=0)
//   k_theta_dot (-1.0) — derivative: damps arm motion
//   k_integral   (0.3) — integral: slowly corrects steady-state offset from friction
//
// Gains were found by sweeping ~500 combinations in simulation and
// selecting the set with: zero overshoot, ≤2 zero crossings, and
// fastest settling time.
//
// Output is clamped to ±6V to prevent amplifier saturation, which
// would cause aggressive overshoot on real hardware.

struct BalanceController {
    // State feedback gains
    double k_theta     = -2.0;   // arm position → voltage [V/rad]
    double k_alpha     = 20.0;   // pendulum position → voltage [V/rad]
    double k_theta_dot = -1.0;   // arm velocity → voltage [V/(rad/s)]
    double k_alpha_dot = 2.0;    // pendulum velocity → voltage [V/(rad/s)]
    double k_integral  = 0.3;    // pendulum integral → voltage [V/(rad·s)]

    double voltage_limit  = 6.0; // max output voltage [V]
    double integral_limit = 0.2; // anti-windup: clamp integral state [rad·s]

    double alpha_integral = 0.0; // accumulated ∫α·dt
    double dt;                   // control period [s], needed for integration

    // Constructor — dt must match the actual loop period
    explicit BalanceController(double dt_) : dt(dt_) {}

    // Compute motor voltage from current state.
    // Returns voltage in [-voltage_limit, +voltage_limit].
    double compute(const QubeState& s) {
        // Accumulate integral of alpha (with anti-windup clamp)
        alpha_integral += s.alpha * dt;
        alpha_integral = clamp(alpha_integral, -integral_limit, integral_limit);

        // Full state feedback: u = -K·x - ki·∫α
        double u = -k_theta     * s.theta
                   - k_alpha     * s.alpha
                   - k_theta_dot * s.theta_dot
                   - k_alpha_dot * s.alpha_dot
                   - k_integral  * alpha_integral;

        // Add deadband offset so the motor actually moves
        u = compensate_deadband(u);

        // Clamp to prevent saturation
        return clamp(u, -voltage_limit, voltage_limit);
    }

    // Reset integral state (called when switching from swing-up to balance)
    void reset() { alpha_integral = 0.0; }
};

// ═══════════════════════════════════════════════════════════════════
// Swing-Up Controller — gets the pendulum from hanging to upright
// ═══════════════════════════════════════════════════════════════════
//
// Uses Åström's energy-based method:
//
// 1. Compute the pendulum's total mechanical energy relative to the
//    upright equilibrium:
//      E = ½·Jp·α̇² - mp·g·(Lp/2)·(cos(α) - 1)
//    At upright with zero velocity, E = 0.
//    Hanging down at rest, E < 0 (we need to add energy).
//
// 2. Apply motor voltage in the direction that transfers energy
//    into the pendulum. The optimal direction is:
//      sign(E · α̇ · cos(α))
//    This naturally adds energy when E < 0 and removes it when E > 0,
//    converging the pendulum toward the upright energy level.
//
// 3. Once |α| < catch_angle (20°), hand off to the balance controller.
//
// Safety features:
//   - Velocity deadzone: no output when pendulum is barely moving
//     (prevents violent chattering from sign flips near α̇ ≈ 0)
//   - Arm angle protection: three zones prevent hitting the endstop
//     (see compute() for details)

struct SwingUpController {
    QubeParams p;             // physical parameters (for energy calculation)
    double mu             = 8.0;    // voltage amplitude [V] (clips to ±V_MAX anyway)
    double catch_angle    = 20.0 * M_PI / 180.0;  // hand off to balance when |α| < this [rad]
    double arm_limit      = ARM_LIMIT_RAD;         // hard arm limit [rad] (from qube_types.h)
    double arm_soft_zone  = ARM_SOFT_ZONE_RAD;     // braking starts this far before limit [rad]
    double arm_brake_gain = 10.0;   // proportional gain for arm braking [V/rad]
    double vel_deadzone   = 0.5;    // ignore α̇ below this [rad/s]

    explicit SwingUpController(const QubeParams& params) : p(params) {}

    // Compute motor voltage from current state.
    // Arm protection takes priority over swing-up.
    double compute(const QubeState& s) const {
        double hLp = p.half_Lp();  // half pendulum length (center of mass)

        double abs_theta = fabs(s.theta);

        // ── ZONE 1: Hard limit ─────────────────────────────────────
        // Arm is past the limit — override everything with strong
        // restoring force + velocity damping to push it back.
        if (abs_theta > arm_limit) {
            return clamp(-arm_brake_gain * s.theta - 4.0 * s.theta_dot,
                         -V_MAX, V_MAX);
        }

        // ── ZONE 2: Soft braking zone ──────────────────────────────
        // Arm is approaching the limit. Taper the swing-up voltage
        // quadratically (scale²) and blend in a restoring force.
        // If swing-up would push arm further out, suppress it to 10%.
        if (abs_theta > (arm_limit - arm_soft_zone)) {
            // scale = 1.0 at the inner edge, 0.0 at the hard limit
            double scale = (arm_limit - abs_theta) / arm_soft_zone;
            scale = clamp(scale, 0.0, 1.0);
            scale = scale * scale;  // quadratic: brakes harder near limit

            // Compute what swing-up wants to do
            double u_swing = energy_pump(s, hLp);

            // If swing-up would push arm further toward the endstop,
            // reduce it to 10% of the already-tapered value
            bool pushing_out = (u_swing > 0) == (s.theta > 0);
            if (pushing_out) scale *= 0.1;

            // Restoring force: spring + damper pulling arm back to center
            double u_restore = -arm_brake_gain * s.theta - 2.0 * s.theta_dot;

            // Blend: mostly restoring near limit, mostly swing-up near center
            return clamp(scale * u_swing + (1.0 - scale) * u_restore,
                         -V_MAX, V_MAX);
        }

        // ── ZONE 3: Safe zone ──────────────────────────────────────
        // Arm is well within limits — full swing-up authority.
        return clamp(energy_pump(s, hLp), -V_MAX, V_MAX);
    }

    // Check if pendulum is close enough to upright for the balance
    // controller to take over.
    bool should_catch(double alpha) const {
        return fabs(alpha) < catch_angle;
    }

private:
    // Core energy pumping calculation.
    // Returns a signed voltage that either adds or removes energy
    // from the pendulum to drive it toward the upright equilibrium.
    double energy_pump(const QubeState& s, double hLp) const {
        // Don't command anything if pendulum is barely moving.
        // Near α̇=0, the sign of (E·α̇·cos α) flips rapidly due to
        // noise, causing violent motor chattering. The deadzone
        // waits until there's a real swing before acting.
        if (fabs(s.alpha_dot) < vel_deadzone) return 0.0;

        // Pendulum energy relative to upright equilibrium:
        //   kinetic:   ½·Jp·α̇²
        //   potential:  -mp·g·(Lp/2)·(cos α - 1)
        // E = 0 at upright rest, E < 0 when hanging below, E > 0 if overshooting
        double E = 0.5 * p.Jp * s.alpha_dot * s.alpha_dot
                 - p.mp * p.g * hLp * (cos(s.alpha) - 1.0);

        // The control law: u = mu · sign(E · α̇ · cos α)
        //   - When E < 0 (need energy): this pushes the arm in the
        //     direction that accelerates the pendulum during its swing
        //   - When E > 0 (too much energy): this brakes the pendulum
        //   - cos(α) ensures the direction is correct for the current
        //     pendulum position (flips at ±90°)
        double u = mu * copysign(1.0, E * s.alpha_dot * cos(s.alpha));

        // Add deadband compensation so the motor actually responds
        return compensate_deadband(u);
    }
};
