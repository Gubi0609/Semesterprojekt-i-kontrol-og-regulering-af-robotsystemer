// Offline simulation of the Qube-Servo 3 inverted pendulum.
// No hardware needed — pure physics sim with RK4 integration.
//
// Build:  g++ -O2 -std=c++17 -o run_sim run_sim.cpp -lm
// Run:    ./run_sim
//         ./run_sim --near-upright --duration 5

#include "qube_types.h"
#include "plant.h"
#include "controllers.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>
#include <string>

int main(int argc, char** argv) {
    double duration    = 10.0;
    double dt          = 0.001;
    bool   near_upright = false;
    int    substeps    = 10;

    // Minimal arg parsing
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt") && i + 1 < argc) dt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--near-upright")) near_upright = true;
    }

    QubeParams params;
    double initial_alpha = near_upright ? 0.1 : M_PI;

    // State: [theta, alpha, theta_dot, alpha_dot]
    State x = {0.0, initial_alpha, 0.0, 0.0};

    BalanceController balance(dt);
    SwingUpController swing_up(params, dt);

    enum Mode { SWING_UP, BALANCE };
    Mode mode = near_upright ? BALANCE : SWING_UP;

    int n_steps = static_cast<int>(duration / dt);

    // CSV log
    std::ofstream csv("sim_output.csv");
    csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,mode\n";

    printf("Simulation: %.1fs, dt=%.4fs, mode=%s\n",
           duration, dt, mode == SWING_UP ? "swing_up" : "balance");
    printf("  Parallel PID — Alpha: Kp=%.1f Ki=%.1f Kd=%.1f | Theta: Kp=%.1f Ki=%.1f Kd=%.1f\n",
           balance.alpha_pid.Kp, balance.alpha_pid.Ki, balance.alpha_Kd,
           balance.theta_pid.Kp, balance.theta_pid.Ki, balance.theta_Kd);

    for (int i = 0; i < n_steps; i++) {
        double t = i * dt;

        QubeState s = {x[0], x[1], x[2], x[3]};
        double voltage = 0.0;

        if (mode == SWING_UP) {
            voltage = swing_up.compute(s);
            if (swing_up.should_catch(s.alpha)) {
                mode = BALANCE;
                balance.reset();
                printf("  [%.2fs] Caught! alpha=%.1f deg\n", t, s.alpha * 180.0 / M_PI);
            }
        } else {
            voltage = balance.compute(s);
            if (fabs(s.alpha) > 30.0 * M_PI / 180.0) {
                mode = SWING_UP;
                balance.reset();
                printf("  [%.2fs] Lost balance! alpha=%.1f deg\n", t, s.alpha * 180.0 / M_PI);
            }
        }

        // RK4 sub-stepping
        double sub_dt = dt / substeps;
        for (int j = 0; j < substeps; j++) {
            x = rk4_step(x, voltage, sub_dt, params);
        }

        // Wrap alpha (pendulum) to [-π, π] but let theta (arm) accumulate.
        // Wrapping theta causes discontinuities that destabilize the
        // inner PID loop (sudden ±360° jumps in error).
        x[1] = wrap_angle(x[1]);

        // Log every sample
        char line[256];
        snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                 t, x[0], x[1], x[2], x[3], voltage, (int)mode);
        csv << line;
    }

    csv.close();
    printf("Logged %d samples to sim_output.csv\n", n_steps);

    // Summary
    printf("Final: theta=%.4f deg, alpha=%.4f deg\n",
           x[0] * 180.0 / M_PI, x[1] * 180.0 / M_PI);

    return 0;
}
