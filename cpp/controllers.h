#pragma once
// Balance controller (LQR state feedback) and swing-up controller.

#include "qube_types.h"
#include <cmath>

// ─── Deadband compensation ─────────────────────────────────────────
inline double compensate_deadband(double u, double deadband = 0.3) {
    if (u > 0.01)       return u + deadband;
    else if (u < -0.01) return u - deadband;
    return 0.0;
}

// ─── Balance Controller ────────────────────────────────────────────
struct BalanceController {
    // Gains from sweep: no overshoot, 2 zero crossings, fast settling
    double k_theta     = -2.0;
    double k_alpha     = 20.0;
    double k_theta_dot = -1.0;
    double k_alpha_dot = 2.0;
    double k_integral  = 0.3;

    double voltage_limit  = 6.0;
    double integral_limit = 0.2;

    double alpha_integral = 0.0;
    double dt;

    explicit BalanceController(double dt_) : dt(dt_) {}

    double compute(const QubeState& s) {
        alpha_integral += s.alpha * dt;
        alpha_integral = clamp(alpha_integral, -integral_limit, integral_limit);

        double u = -k_theta     * s.theta
                   - k_alpha     * s.alpha
                   - k_theta_dot * s.theta_dot
                   - k_alpha_dot * s.alpha_dot
                   - k_integral  * alpha_integral;

        u = compensate_deadband(u);
        return clamp(u, -voltage_limit, voltage_limit);
    }

    void reset() { alpha_integral = 0.0; }
};

// ─── Swing-Up Controller ───────────────────────────────────────────
// Energy-based (Åström) — the version that got to 34° from upright.
// With velocity deadzone (no startup chatter) and arm protection.

struct SwingUpController {
    QubeParams p;
    double mu             = 8.0;    // mu=8 got to 34° from upright without detaching
    double catch_angle    = 20.0 * M_PI / 180.0;
    double arm_limit      = ARM_LIMIT_RAD;
    double arm_soft_zone  = ARM_SOFT_ZONE_RAD;
    double arm_brake_gain = 10.0;
    double vel_deadzone   = 0.5;

    explicit SwingUpController(const QubeParams& params) : p(params) {}

    double compute(const QubeState& s) const {
        double hLp = p.half_Lp();

        // Arm protection first
        double abs_theta = fabs(s.theta);

        if (abs_theta > arm_limit) {
            return clamp(-arm_brake_gain * s.theta - 4.0 * s.theta_dot, -V_MAX, V_MAX);
        }

        if (abs_theta > (arm_limit - arm_soft_zone)) {
            double scale = (arm_limit - abs_theta) / arm_soft_zone;
            scale = clamp(scale, 0.0, 1.0);
            scale = scale * scale;

            double u_swing = energy_pump(s, hLp);

            bool pushing_out = (u_swing > 0) == (s.theta > 0);
            if (pushing_out) scale *= 0.1;

            double u_restore = -arm_brake_gain * s.theta - 2.0 * s.theta_dot;
            return clamp(scale * u_swing + (1.0 - scale) * u_restore, -V_MAX, V_MAX);
        }

        return clamp(energy_pump(s, hLp), -V_MAX, V_MAX);
    }

    bool should_catch(double alpha) const {
        return fabs(alpha) < catch_angle;
    }

private:
    double energy_pump(const QubeState& s, double hLp) const {
        if (fabs(s.alpha_dot) < vel_deadzone) return 0.0;

        double E = 0.5 * p.Jp * s.alpha_dot * s.alpha_dot
                 - p.mp * p.g * hLp * (cos(s.alpha) - 1.0);

        double u = mu * copysign(1.0, E * s.alpha_dot * cos(s.alpha));
        return compensate_deadband(u);
    }
};
