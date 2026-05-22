#pragma once
// Nonlinear dynamics + RK4 integrator for the rotary inverted pendulum.

#include "qube_types.h"
#include <array>
#include <cmath>

using State = std::array<double, 4>; // [theta, alpha, theta_dot, alpha_dot]

// dx/dt = f(x, Vm)
inline State dynamics(const State& x, double Vm, const QubeParams& p) {
    Vm = clamp(Vm, -V_MAX, V_MAX);

    // double theta  = x[0]; // not used directly in EOM
    double alpha     = x[1];
    double theta_dot = x[2];
    double alpha_dot = x[3];

    double sin_a = sin(alpha);
    double cos_a = cos(alpha);
    double hLp   = p.half_Lp();

    // Motor torque (with back-EMF)
    double tau = p.km * (Vm - p.km * theta_dot) / p.Rm;

    // Mass matrix
    double M11 = p.Jt() + p.Jp * sin_a * sin_a;
    double M12 = -p.mp * hLp * p.Lr * cos_a;
    double M22 = p.Jp + p.mp * hLp * hLp;

    // Right-hand side
    double rhs1 = tau
                - p.Dr * theta_dot
                - p.mp * hLp * p.Lr * alpha_dot * alpha_dot * sin_a
                - 2.0 * p.Jp * sin_a * cos_a * theta_dot * alpha_dot;

    double rhs2 = -p.Dp * alpha_dot
                + p.mp * hLp * p.g * sin_a
                + p.Jp * sin_a * cos_a * theta_dot * theta_dot;

    // Solve 2x2
    double det = M11 * M22 - M12 * M12;
    double theta_ddot = (M22 * rhs1 - M12 * rhs2) / det;
    double alpha_ddot = (M11 * rhs2 - M12 * rhs1) / det;

    return {theta_dot, alpha_dot, theta_ddot, alpha_ddot};
}

// Single RK4 step
inline State rk4_step(const State& x, double Vm, double dt, const QubeParams& p) {
    auto add = [](const State& a, const State& b, double s) -> State {
        return {a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2], a[3] + s * b[3]};
    };

    State k1 = dynamics(x, Vm, p);
    State k2 = dynamics(add(x, k1, 0.5 * dt), Vm, p);
    State k3 = dynamics(add(x, k2, 0.5 * dt), Vm, p);
    State k4 = dynamics(add(x, k3, dt), Vm, p);

    return {
        x[0] + (dt / 6.0) * (k1[0] + 2*k2[0] + 2*k3[0] + k4[0]),
        x[1] + (dt / 6.0) * (k1[1] + 2*k2[1] + 2*k3[1] + k4[1]),
        x[2] + (dt / 6.0) * (k1[2] + 2*k2[2] + 2*k3[2] + k4[2]),
        x[3] + (dt / 6.0) * (k1[3] + 2*k2[3] + 2*k3[3] + k4[3]),
    };
}
