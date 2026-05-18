// Offline simulation of the Qube-Servo 3 inverted pendulum.
// No hardware needed — pure physics sim with RK4 integration.
//
// Build:  g++ -O2 -std=c++17 -o run_sim run_sim.cpp -lm
// Run:    ./run_sim              (both controllers, side-by-side)
//         ./run_sim --pid        (PID only)
//         ./run_sim --place      (pole placement only)
//         ./run_sim --alpha0 15  (initial deflection in degrees)
//         ./run_sim --duration 5

#include "qube_types.h"
#include "plant.h"
#include "controllers.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    double duration    = 5.0;
    double dt          = 0.001;
    double alpha0_deg  = 10.0;
    int    substeps    = 10;
    bool   run_pid     = false;
    bool   run_place   = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt") && i + 1 < argc) dt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--alpha0") && i + 1 < argc) alpha0_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pid"))   run_pid = true;
        else if (!strcmp(argv[i], "--place")) run_place = true;
        else {
            printf("Usage: %s [--pid] [--place] [--alpha0 DEG] [--duration S] [--dt S]\n", argv[0]);
            return 1;
        }
    }

    // Default: run both
    if (!run_pid && !run_place) {
        run_pid   = true;
        run_place = true;
    }

    QubeParams params;
    double initial_alpha = alpha0_deg * M_PI / 180.0;
    int n_steps = static_cast<int>(duration / dt);

    struct RunResult {
        std::string name;
        std::vector<double> t, theta, alpha, theta_dot, alpha_dot, voltage;
    };

    // mode: 0 = PID, 1 = pole placement
    auto simulate = [&](const std::string& name, int mode) -> RunResult {
        RunResult r;
        r.name = name;
        r.t.resize(n_steps);
        r.theta.resize(n_steps);
        r.alpha.resize(n_steps);
        r.theta_dot.resize(n_steps);
        r.alpha_dot.resize(n_steps);
        r.voltage.resize(n_steps);

        State x = {0.0, initial_alpha, 0.0, 0.0};
        BalanceController ctrl(dt);

        for (int i = 0; i < n_steps; i++) {
            r.t[i] = i * dt;
            r.theta[i]     = x[0];
            r.alpha[i]     = x[1];
            r.theta_dot[i] = x[2];
            r.alpha_dot[i] = x[3];

            QubeState s = {x[0], x[1], x[2], x[3]};
            double v = (mode == 1) ? ctrl.compute_from_gains(s) : ctrl.compute(s);
            r.voltage[i] = v;

            double sub_dt = dt / substeps;
            for (int j = 0; j < substeps; j++)
                x = rk4_step(x, v, sub_dt, params);
            x[1] = wrap_angle(x[1]);
        }
        return r;
    };

    printf("Balance simulation: %.1fs, dt=%.4fs, alpha0=%.1f deg\n", duration, dt, alpha0_deg);
    printf("Plant: Jr=%.2e, Jp=%.2e (pivot-frame), Lr=%.4f, Lp=%.3f\n\n",
           params.Jr, params.Jp, params.Lr, params.Lp);

    if (run_place) {
        printf("Pole placement gains: K = [%.4f, %.4f, %.4f, %.4f]\n",
               BalanceController::K_theta, BalanceController::K_alpha,
               BalanceController::K_theta_dot, BalanceController::K_alpha_dot);
    }
    if (run_pid) {
        printf("PID gains: alpha Kp=%.1f Ki=%.1f Kd=%.1f | theta Kp=%.1f Kd=%.1f\n",
               20.0, 0.3, 2.0, 2.0, 1.0);
    }
    printf("\n");

    auto print_metrics = [&](const RunResult& r) {
        double max_alpha = 0, max_vm = 0, settle_t = 0;
        double band = 0.02 * initial_alpha;
        for (int i = 0; i < n_steps; i++) {
            if (fabs(r.alpha[i]) > fabs(max_alpha)) max_alpha = r.alpha[i];
            if (fabs(r.voltage[i]) > max_vm) max_vm = fabs(r.voltage[i]);
            if (fabs(r.alpha[i]) > band) settle_t = r.t[i];
        }
        printf("  %-35s\n", r.name.c_str());
        printf("    Settling time (2%%): %.3f s\n", settle_t);
        printf("    Peak alpha:         %.2f deg\n", max_alpha * 180.0 / M_PI);
        printf("    Max |Vm|:           %.2f V\n", max_vm);
        printf("    Final alpha:        %.4f deg\n", r.alpha.back() * 180.0 / M_PI);
        printf("    Final theta:        %.4f deg\n\n", r.theta.back() * 180.0 / M_PI);
    };

    auto write_csv = [&](const RunResult& r, const std::string& filename) {
        std::ofstream csv(filename);
        csv << "t,theta,alpha,theta_dot,alpha_dot,voltage\n";
        char line[256];
        for (int i = 0; i < n_steps; i++) {
            snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                     r.t[i], r.theta[i], r.alpha[i],
                     r.theta_dot[i], r.alpha_dot[i], r.voltage[i]);
            csv << line;
        }
        csv.close();
        printf("Wrote %s\n", filename.c_str());
    };

    if (run_pid) {
        RunResult pid_run = simulate("PID", 0);
        print_metrics(pid_run);
        write_csv(pid_run, "sim_pid.csv");
    }

    if (run_place) {
        RunResult place_run = simulate("Pole placement", 1);
        print_metrics(place_run);
        write_csv(place_run, "sim_place.csv");
    }

    return 0;
}
