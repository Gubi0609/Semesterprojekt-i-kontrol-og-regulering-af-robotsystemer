#pragma once
// ═══════════════════════════════════════════════════════════════════
// controllers.h — Parallel PID balance + energy-based swing-up
// ═══════════════════════════════════════════════════════════════════
//
// The balance controller uses two parallel PID loops whose outputs
// are summed into a single motor voltage:
//
//   u = PID_alpha(0 - α) + PID_theta(0 - θ)
//
//   Alpha PID: aggressive — keeps the pendulum upright (primary)
//   Theta PID: gentle — nudges the arm back toward center (secondary)
//
// This is equivalent to PD control on each DOF with an integral on
// alpha for steady-state correction. It maps directly onto classical
// PID theory — each loop can be tuned and analysed independently
// using root locus, Bode plots, etc.

#include "qube_types.h"
#include <cmath>

// ─── Deadband compensation ─────────────────────────────────────────
// The motor amplifier has a ~0.3V deadband. Any command below that
// produces zero torque. We add the offset only when the command is
// large enough to be intentional (threshold), so encoder noise
// near zero doesn't get amplified into ±0.3V jitter.
inline double compensate_deadband(double u, double deadband = 0.3,
                                  double threshold = 0.15) {
    if (u > threshold)        return u + deadband;
    else if (u < -threshold)  return u - deadband;
    return 0.0;
}

// ─── Single PID block ──────────────────────────────────────────────
// Reusable PID with anti-windup clamping and filtered derivative.
// The derivative uses a first-order low-pass filter to suppress
// high-frequency noise:  D_filt = α·D_raw + (1-α)·D_prev
// where α = dt / (Tf + dt).  Tf = derivative filter time constant.
struct PID {
    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;

    double integral       = 0.0;
    double prev_error     = 0.0;
    double prev_deriv     = 0.0;
    double integral_limit = 1e6;  // anti-windup clamp
    double output_limit   = 1e6;  // clamp final output
    double Tf             = 0.01; // derivative filter time constant [s]
    bool   first_call     = true;

    void reset() {
        integral   = 0.0;
        prev_error = 0.0;
        prev_deriv = 0.0;
        first_call = true;
    }

    double compute(double error, double dt) {
        // Proportional
        double P = Kp * error;

        // Integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integral_limit, integral_limit);
        double I = Ki * integral;

        // Filtered derivative
        double D = 0.0;
        if (!first_call) {
            double raw_deriv = (error - prev_error) / dt;
            double alpha = dt / (Tf + dt);
            double filt_deriv = alpha * raw_deriv + (1.0 - alpha) * prev_deriv;
            prev_deriv = filt_deriv;
            D = Kd * filt_deriv;
        }
        prev_error = error;
        first_call = false;

        double out = P + I + D;
        return clamp(out, -output_limit, output_limit);
    }
};

// ═══════════════════════════════════════════════════════════════════
// Balance Controller — Parallel PID
// ═══════════════════════════════════════════════════════════════════
//
//   u = PID_alpha(0 - α) + PID_theta(0 - θ)
//
// Alpha PID (pendulum stabilisation):
//   Kp — counteracts pendulum tilt (main balancing force)
//   Ki — eliminates steady-state offset from friction/model error
//   Kd — damps pendulum oscillation, prevents overshoot
//
// Theta PID (arm centering):
//   Kp — pulls arm back toward center
//   Kd — damps arm motion to avoid overshoot
//   Ki — typically zero (arm centering is not critical)
//
// Starting gains are based on the original LQR state-feedback gains
// which are known to work on this hardware:
//   k_alpha=20, k_alpha_dot=2, k_theta=-2, k_theta_dot=-1, ki=0.3
//
// The PID Kp/Kd map directly to these:
//   alpha_pid.Kp ↔ k_alpha,  alpha_pid.Kd ↔ k_alpha_dot
//   theta_pid.Kp ↔ k_theta,  theta_pid.Kd ↔ k_theta_dot

struct BalanceController {
    // ── Alpha PI: pendulum angle → voltage (aggressive) ────────
    // Only P+I here — the D term uses measured velocity directly.
    PID alpha_pid;

    // ── Theta PI: arm angle → voltage (gentle) ────────────────
    // NOTE: theta_pid.Kp is stored as a positive number but its
    // contribution is ADDED (not subtracted) — see compute().
    // When the arm drifts right (θ>0), we push voltage positive
    // to move the base under the pendulum, not pull it back.
    PID theta_pid;

    // ── D gains applied to measured velocities ─────────────────
    // Using tachometer readings (hardware) or simulation velocities
    // instead of differentiating noisy encoder positions.
    double alpha_Kd = 2.0;   // [V·s / rad] — damp pendulum oscillation
    double theta_Kd = 1.0;   // [V·s / rad] — damp arm motion

    double voltage_limit = 6.0;
    double dt;

    explicit BalanceController(double dt_) : dt(dt_) {
        // ── Alpha PI (pendulum — primary) ──────────────────────
        // Positive alpha (tilt right) → negative voltage to push
        // arm left and counteract tilt. PID error = (0 - α) < 0
        // so output is negative. Correct.
        alpha_pid.Kp = 20.0;   // [V / rad]
        alpha_pid.Ki = 0.3;    // [V / (rad·s)] — steady-state correction
        alpha_pid.Kd = 0.0;    // D handled separately via measured velocity
        alpha_pid.integral_limit = 0.2;  // anti-windup [rad·s]
        alpha_pid.output_limit   = voltage_limit;

        // ── Theta PI (arm — secondary) ─────────────────────────
        // This is NOT a centering controller. It moves the base
        // under the pendulum. The sign is handled in compute():
        // we SUBTRACT theta_pid output instead of adding it.
        theta_pid.Kp = 2.0;    // [V / rad]
        theta_pid.Ki = 0.0;    // no integral needed for arm
        theta_pid.Kd = 0.0;    // D handled separately via measured velocity
        theta_pid.integral_limit = 0.5;
        theta_pid.output_limit   = voltage_limit;
    }

    double compute(const QubeState& s) {
        // ── P + I from alpha (pendulum) ────────────────────────
        // error = (0 - alpha): positive alpha → negative error → negative output
        // This pushes the arm to counteract the tilt. ✓
        double alpha_error = 0.0 - s.alpha;
        double u_alpha = alpha_pid.compute(alpha_error, dt);

        // ── P + I from theta (arm) ────────────────────────────
        // The original LQR had: u += -k_theta * theta, with k_theta = -2.0
        // i.e. u += +2.0 * theta.  When theta > 0, voltage increases.
        //
        // PID computes: Kp * (0 - theta) = -2.0 * theta (wrong sign!)
        // Fix: subtract instead of add, so we get -(-2.0 * theta) = +2.0 * theta. ✓
        double theta_error = 0.0 - s.theta;
        double u_theta = theta_pid.compute(theta_error, dt);

        // ── D from measured velocities (no differentiation) ───
        // Alpha: negative feedback. Positive alpha_dot → oppose it.
        //   Original: u += -k_alpha_dot * alpha_dot = -2.0 * alpha_dot  ✓
        double u_alpha_d = -alpha_Kd * s.alpha_dot;

        // Theta: positive feedback on velocity (moves base under pendulum).
        //   Original: u += -k_theta_dot * theta_dot, with k_theta_dot = -1.0
        //   i.e. u += +1.0 * theta_dot.  ✓
        double u_theta_d = +theta_Kd * s.theta_dot;

        // Sum: alpha terms are normal PID, theta terms have inverted sign
        double u = u_alpha - u_theta + u_alpha_d + u_theta_d;

        // Deadband compensation
        u = compensate_deadband(u);

        // Clamp to prevent saturation
        return clamp(u, -voltage_limit, voltage_limit);
    }

    void reset() {
        alpha_pid.reset();
        theta_pid.reset();
    }
};

// ═══════════════════════════════════════════════════════════════════
// Swing-Up Controller — energy-based bang-bang (Åström)
// ═══════════════════════════════════════════════════════════════════
//
// How it works:
//
//   1. Compute the pendulum's energy relative to upright:
//        E = ½·Jp·α̇² - mp·g·(Lp/2)·(cos(α) - 1)
//      E = 0 at upright rest, E < 0 when hanging.
//
//   2. Pick motor voltage direction using Åström's law:
//        direction = sign(E · α̇ · cos(α))
//      This adds energy when E < 0 and removes when E > 0.
//
//   3. Apply constant magnitude ±mu (bang-bang).
//
//   4. When |α| < catch_angle, hand off to balance controller.
//
// Arm safety: hard brake if arm exceeds limit.
//
struct SwingUpController {
    QubeParams p;

    double mu             = 1.5;    // bang-bang voltage magnitude [V]
    double catch_angle    = 20.0 * M_PI / 180.0;  // hand off threshold [rad]
    double arm_limit      = ARM_LIMIT_RAD;         // hard arm limit [rad]
    double arm_soft_start = 70.0 * M_PI / 180.0;   // start tapering here [rad]
    double arm_brake_gain = 10.0;   // braking P gain [V/rad]
    double vel_deadzone   = 0.1;    // ignore α̇ below this [rad/s]
    double initial_kick   = 0.8;    // voltage to get pendulum moving [V]
    double kick_duration  = 0.5;    // how long to kick [s]
    double kick_timer     = 0.0;

    double dt;

    SwingUpController(const QubeParams& params, double dt_) : p(params), dt(dt_) {}

    double compute(const QubeState& s) {
        double abs_theta = fabs(s.theta);

        // ── Arm safety: hard brake if past limit ───────────────
        if (abs_theta > arm_limit) {
            return clamp(-arm_brake_gain * s.theta - 4.0 * s.theta_dot,
                         -V_MAX, V_MAX);
        }

        // ── Initial kick: get pendulum moving from rest ────────
        if (fabs(s.alpha_dot) < vel_deadzone) {
            if (kick_timer < kick_duration) {
                kick_timer += dt;
                return compensate_deadband(initial_kick);
            }
            return 0.0;
        }
        kick_timer = kick_duration;

        // ── Energy calculation ─────────────────────────────────
        double hLp = p.half_Lp();
        double E = 0.5 * p.Jp * s.alpha_dot * s.alpha_dot
                 - p.mp * p.g * hLp * (cos(s.alpha) - 1.0);

        // ── Bang-bang: constant voltage, Åström direction ──────
        double direction = copysign(1.0, E * s.alpha_dot * cos(s.alpha));
        double u = mu * direction;

        // ── Soft braking near arm limit ────────────────────────
        // Linearly taper swing-up and blend in restoring force
        if (abs_theta > arm_soft_start) {
            double scale = (arm_limit - abs_theta) / (arm_limit - arm_soft_start);
            scale = clamp(scale, 0.0, 1.0);
            // If pushing arm further out, suppress harder
            if ((u > 0) == (s.theta > 0)) scale *= 0.1;
            double u_restore = -arm_brake_gain * s.theta - 2.0 * s.theta_dot;
            u = scale * u + (1.0 - scale) * u_restore;
        }

        return clamp(compensate_deadband(u), -V_MAX, V_MAX);
    }

    bool should_catch(double alpha) const {
        return fabs(alpha) < catch_angle;
    }
};
