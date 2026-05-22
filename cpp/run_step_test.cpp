// ═══════════════════════════════════════════════════════════════════
// run_step_test.cpp — Step-response evaluation for balance controller
// ═══════════════════════════════════════════════════════════════════
//
// Simulation test procedure:
//   The pendulum starts at a specified offset (default 20°) with
//   zero angular velocity — a clean initial condition that mimics
//   the hardware test where the pendulum coasts through the trigger
//   zone with near-zero α̇.
//
//   The balance controller engages at t=0 and runs for --duration
//   seconds. Metrics are computed: rise time, overshoot, settling
//   time (within a ±2° band).
//
// CSV filename encodes the controller gains and a timestamp so
// multiple runs never overwrite each other:
//   step_aKp20.0_aKi0.3_aKd2.0_tKp2.0_tKd1.0_20260522_143021.csv
//
// Build:  make run_step_test
// Run:    ./run_step_test                       # defaults (20° offset)
//         ./run_step_test --alpha0 15           # 15° initial offset
//         ./run_step_test --aKp 25 --aKd 3     # override gains
//         ./run_step_test --duration 8          # run for 8s
//         ./run_step_test --outdir results      # write CSV to results/
// ═══════════════════════════════════════════════════════════════════

#include "qube_types.h"
#include "plant.h"
#include "controllers.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <sys/stat.h>

// ─── Step-response metrics ─────────────────────────────────────────
struct StepMetrics {
    double initial_alpha_deg;  // α at t=0 [deg]
    double initial_alpha_dot;  // α̇ at t=0 [rad/s]
    double rise_time;          // time to first enter ±band [s]
    double overshoot_deg;      // peak |alpha| after first zero crossing [deg]
    double overshoot_pct;      // overshoot as % of initial offset
    double settling_time;      // time to permanently stay within ±band [s]
    double steady_state_err;   // mean |alpha| over last 1s [deg]
    double max_voltage;        // peak |voltage| during run
    bool   settled;            // did it actually settle?
};

// ─── Compute metrics from recorded data ────────────────────────────
StepMetrics compute_metrics(const std::vector<double>& t_vec,
                            const std::vector<double>& alpha_vec,
                            const std::vector<double>& voltage_vec,
                            double alpha_band_deg = 2.0) {
    StepMetrics m = {};
    int n = (int)t_vec.size();
    if (n < 2) return m;

    double band_rad = alpha_band_deg * M_PI / 180.0;
    m.initial_alpha_deg = fabs(alpha_vec[0]) * 180.0 / M_PI;

    // Rise time: first time |alpha| enters ±band
    m.rise_time = -1;
    for (int i = 0; i < n; i++) {
        if (fabs(alpha_vec[i]) <= band_rad) {
            m.rise_time = t_vec[i];
            break;
        }
    }

    // Overshoot: peak |alpha| after first zero crossing
    bool crossed_zero = false;
    double peak_after_cross = 0;
    for (int i = 1; i < n; i++) {
        if (!crossed_zero) {
            if (alpha_vec[i - 1] * alpha_vec[i] < 0 &&
                fabs(alpha_vec[i - 1]) > 0.001) {
                crossed_zero = true;
            }
        }
        if (crossed_zero) {
            double a = fabs(alpha_vec[i]);
            if (a > peak_after_cross) peak_after_cross = a;
        }
    }
    m.overshoot_deg = peak_after_cross * 180.0 / M_PI;
    if (m.initial_alpha_deg > 0.01)
        m.overshoot_pct = (m.overshoot_deg / m.initial_alpha_deg) * 100.0;

    // Settling time: last time |alpha| exceeded the band
    double last_outside = -1;
    for (int i = 0; i < n; i++) {
        if (fabs(alpha_vec[i]) > band_rad)
            last_outside = t_vec[i];
    }
    if (last_outside < 0) {
        m.settling_time = 0;
        m.settled = true;
    } else if (last_outside < t_vec.back() - 0.5) {
        m.settling_time = last_outside;
        m.settled = true;
    } else {
        m.settling_time = -1;
        m.settled = false;
    }

    // Steady-state error: mean |alpha| over last 1s
    double ss_window = 1.0;
    double sum_abs = 0;
    int count = 0;
    for (int i = 0; i < n; i++) {
        if (t_vec[i] > t_vec.back() - ss_window) {
            sum_abs += fabs(alpha_vec[i]) * 180.0 / M_PI;
            count++;
        }
    }
    if (count > 0) m.steady_state_err = sum_abs / count;

    // Max voltage
    m.max_voltage = 0;
    for (int i = 0; i < n; i++) {
        double v = fabs(voltage_vec[i]);
        if (v > m.max_voltage) m.max_voltage = v;
    }

    return m;
}

// ─── Generate timestamped filename with gains ──────────────────────
std::string make_filename(const BalanceController& bal, const std::string& outdir) {
    time_t now = time(nullptr);
    struct tm* lt = localtime(&now);

    char ts[64];
    snprintf(ts, sizeof(ts), "%04d%02d%02d_%02d%02d%02d",
             lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
             lt->tm_hour, lt->tm_min, lt->tm_sec);

    char buf[512];
    snprintf(buf, sizeof(buf),
             "step_aKp%.1f_aKi%.1f_aKd%.1f_tKp%.1f_tKd%.1f_%s.csv",
             bal.alpha_pid.Kp, bal.alpha_pid.Ki, bal.alpha_Kd,
             bal.theta_pid.Kp, bal.theta_Kd, ts);

    std::string fname = buf;
    if (!outdir.empty()) {
        mkdir(outdir.c_str(), 0755);
        fname = outdir + "/" + fname;
    }
    return fname;
}

// ─── Print metrics report ──────────────────────────────────────────
void print_report(const StepMetrics& m, const BalanceController& bal,
                  const std::string& csv_path) {
    printf("\n");
    printf("══════════════════════════════════════════════════════════════\n");
    printf("  Step-Response Test Results (Simulation)\n");
    printf("══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("  Controller gains:\n");
    printf("    Alpha PID:  Kp=%.2f  Ki=%.2f  Kd=%.2f\n",
           bal.alpha_pid.Kp, bal.alpha_pid.Ki, bal.alpha_Kd);
    printf("    Theta PID:  Kp=%.2f  Ki=%.2f  Kd=%.2f\n",
           bal.theta_pid.Kp, bal.theta_pid.Ki, bal.theta_Kd);
    printf("    Voltage limit: %.1f V\n", bal.voltage_limit);
    printf("\n");
    printf("  Initial offset:    %.2f°  (α̇=%.2f rad/s)\n",
           m.initial_alpha_deg, m.initial_alpha_dot);
    printf("\n");
    printf("  ── Time-Domain Metrics (±2° band) ───────────────────────\n");

    if (m.rise_time >= 0)
        printf("  Rise time:         %.4f s\n", m.rise_time);
    else
        printf("  Rise time:         NEVER (did not reach ±2°)\n");

    printf("  Overshoot:         %.2f° (%.1f%%)\n",
           m.overshoot_deg, m.overshoot_pct);

    if (m.settled)
        printf("  Settling time:     %.4f s\n", m.settling_time);
    else
        printf("  Settling time:     DID NOT SETTLE\n");

    printf("  Steady-state err:  %.3f° (mean |α| last 1s)\n", m.steady_state_err);
    printf("  Max |voltage|:     %.2f V\n", m.max_voltage);
    printf("\n");

    const char* grade;
    if (m.settled && m.settling_time < 0.5 && m.overshoot_pct < 20)
        grade = "\033[32m★★★ EXCELLENT\033[0m";
    else if (m.settled && m.settling_time < 1.5 && m.overshoot_pct < 50)
        grade = "\033[32m★★  GOOD\033[0m";
    else if (m.settled)
        grade = "\033[33m★   SETTLED (slow or high overshoot)\033[0m";
    else
        grade = "\033[31m    FAILED (did not settle)\033[0m";

    printf("  Grade: %s\n", grade);
    printf("\n");
    printf("  CSV: %s\n", csv_path.c_str());
    printf("══════════════════════════════════════════════════════════════\n\n");
}

// ═══════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    // ── Defaults ───────────────────────────────────────────────
    double alpha0_deg   = 20.0;   // initial pendulum offset [deg]
    double theta0_deg   = 0.0;    // initial arm angle [deg]
    double duration     = 5.0;    // simulation duration [s]
    double dt           = 0.001;
    int    substeps     = 10;
    double alpha_band   = 2.0;    // settling band [deg]
    std::string outdir  = "";

    // Controller gain overrides
    double aKp = 20.0, aKi = 0.3, aKd = 2.0;
    double tKp = 2.0,  tKi = 0.0, tKd = 1.0;
    double vlim = 6.0;

    // ── Parse arguments ────────────────────────────────────────
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--alpha0")   && i+1 < argc) alpha0_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--theta0")   && i+1 < argc) theta0_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--duration") && i+1 < argc) duration   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt")       && i+1 < argc) dt         = atof(argv[++i]);
        else if (!strcmp(argv[i], "--band")     && i+1 < argc) alpha_band = atof(argv[++i]);
        else if (!strcmp(argv[i], "--outdir")   && i+1 < argc) outdir     = argv[++i];
        else if (!strcmp(argv[i], "--aKp")      && i+1 < argc) aKp  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--aKi")      && i+1 < argc) aKi  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--aKd")      && i+1 < argc) aKd  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tKp")      && i+1 < argc) tKp  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tKi")      && i+1 < argc) tKi  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tKd")      && i+1 < argc) tKd  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--vlim")     && i+1 < argc) vlim = atof(argv[++i]);
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --alpha0 DEG    Initial pendulum offset (default: 20)\n");
            printf("  --theta0 DEG    Initial arm angle (default: 0)\n");
            printf("  --duration SEC  Simulation duration (default: 5)\n");
            printf("  --band DEG      Settling band (default: 2)\n");
            printf("  --outdir DIR    Output directory for CSV\n");
            printf("  --aKp VAL       Alpha Kp (default: 20)\n");
            printf("  --aKi VAL       Alpha Ki (default: 0.3)\n");
            printf("  --aKd VAL       Alpha Kd (default: 2)\n");
            printf("  --tKp VAL       Theta Kp (default: 2)\n");
            printf("  --tKi VAL       Theta Ki (default: 0)\n");
            printf("  --tKd VAL       Theta Kd (default: 1)\n");
            printf("  --vlim VAL      Voltage limit (default: 6)\n");
            return 0;
        }
    }

    // ── Set up controller ──────────────────────────────────────
    BalanceController balance(dt);
    balance.alpha_pid.Kp = aKp;
    balance.alpha_pid.Ki = aKi;
    balance.alpha_Kd     = aKd;
    balance.theta_pid.Kp = tKp;
    balance.theta_pid.Ki = tKi;
    balance.theta_Kd     = tKd;
    balance.voltage_limit = vlim;

    // ── Set up plant ───────────────────────────────────────────
    QubeParams params;
    double alpha0_rad = alpha0_deg * M_PI / 180.0;
    double theta0_rad = theta0_deg * M_PI / 180.0;

    // Start at offset with zero velocity — mimics the hardware
    // test where the pendulum coasts through the trigger zone
    // with near-zero angular momentum.
    State x = {theta0_rad, alpha0_rad, 0.0, 0.0};

    int n_steps = (int)(duration / dt);

    // ── Storage for metrics computation ────────────────────────
    std::vector<double> t_log, alpha_log, voltage_log;
    t_log.reserve(n_steps);
    alpha_log.reserve(n_steps);
    voltage_log.reserve(n_steps);

    // ── Generate CSV filename ──────────────────────────────────
    std::string csv_path = make_filename(balance, outdir);
    std::ofstream csv(csv_path);
    csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,mode\n";

    // ── Print header ───────────────────────────────────────────
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║     Qube-Servo 3 — Step-Response Test (Simulation)         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("  Initial offset: α=%.1f°  θ=%.1f°  (α̇=0)\n", alpha0_deg, theta0_deg);
    printf("  Duration: %.1fs   dt=%.4fs   settling band: ±%.1f°\n",
           duration, dt, alpha_band);
    printf("  Gains — Alpha: Kp=%.2f Ki=%.2f Kd=%.2f\n", aKp, aKi, aKd);
    printf("          Theta: Kp=%.2f Ki=%.2f Kd=%.2f  Vlim=%.1fV\n",
           tKp, tKi, tKd, vlim);
    printf("  Simulating...\n");

    // ── Simulation loop ────────────────────────────────────────
    balance.reset();

    for (int i = 0; i < n_steps; i++) {
        double t = i * dt;

        QubeState s = {x[0], x[1], x[2], x[3]};
        double voltage = balance.compute(s);

        // Store for metrics
        t_log.push_back(t);
        alpha_log.push_back(x[1]);
        voltage_log.push_back(voltage);

        // CSV log
        char line[256];
        snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                 t, x[0], x[1], x[2], x[3], voltage, 1);
        csv << line;

        // RK4 integration
        double sub_dt = dt / substeps;
        for (int j = 0; j < substeps; j++)
            x = rk4_step(x, voltage, sub_dt, params);

        x[1] = wrap_angle(x[1]);
    }

    csv.close();

    // ── Compute and print metrics ──────────────────────────────
    StepMetrics metrics = compute_metrics(t_log, alpha_log, voltage_log, alpha_band);
    metrics.initial_alpha_dot = 0.0;
    print_report(metrics, balance, csv_path);

    return 0;
}
