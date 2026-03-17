#pragma once
// Balance controller (LQR state feedback) and swing-up controller.

#include "qube_types.h"
#include <cmath>

// ─── Balance Controller ────────────────────────────────────────────
// Full state feedback: u = -K·x + integral term
// Gains from LQR with Q=diag(10,100,1,1), R=1

struct BalanceController {
    double k_theta     = -3.16;
    double k_alpha     = 49.0;
    double k_theta_dot = -1.83;
    double k_alpha_dot = 4.32;
    double k_integral  = 2.0;

    double voltage_limit  = V_MAX;
    double integral_limit = 0.5;

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

        return clamp(u, -voltage_limit, voltage_limit);
    }

    void reset() { alpha_integral = 0.0; }
};

// ─── Swing-Up Controller ───────────────────────────────────────────
// Energy-based (Åström) with arm angle protection.

struct SwingUpController {
    QubeParams p;
    double mu             = 2.5;
    double catch_angle    = 20.0 * M_PI / 180.0;
    double arm_limit      = ARM_LIMIT_RAD;
    double arm_soft_zone  = ARM_SOFT_ZONE_RAD;
    double arm_brake_gain = 8.0;

    explicit SwingUpController(const QubeParams& params) : p(params) {}

    double compute(const QubeState& s) const {
        double hLp = p.half_Lp();

        // Pendulum energy relative to upright equilibrium
        double E = 0.5 * p.Jp * s.alpha_dot * s.alpha_dot
                 - p.mp * p.g * hLp * (cos(s.alpha) - 1.0);

        double u_swing = mu * copysign(1.0, E * s.alpha_dot * cos(s.alpha));

        // Arm protection
        double abs_theta = fabs(s.theta);

        if (abs_theta > arm_limit) {
            // Hard limit: restoring + damping
            return clamp(-arm_brake_gain * s.theta - 2.0 * s.theta_dot, -V_MAX, V_MAX);
        }

        if (abs_theta > (arm_limit - arm_soft_zone)) {
            double scale = (arm_limit - abs_theta) / arm_soft_zone;
            scale = clamp(scale, 0.0, 1.0);

            // Suppress outward pushes
            bool pushing_out = (u_swing > 0) == (s.theta > 0);
            if (pushing_out) scale *= 0.3;

            double u_restore = -1.0 * s.theta;
            return clamp(scale * u_swing + (1.0 - scale) * u_restore, -V_MAX, V_MAX);
        }

        return clamp(u_swing, -V_MAX, V_MAX);
    }

    bool should_catch(double alpha) const {
        return fabs(alpha) < catch_angle;
    }
};
