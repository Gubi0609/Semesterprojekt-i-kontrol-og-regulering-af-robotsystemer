// ═══════════════════════════════════════════════════════════════════
// run_tests.cpp — Automated performance evaluation for controllers
// ═══════════════════════════════════════════════════════════════════
//
// Runs batches of simulations to evaluate balance and swing-up
// controllers under various conditions. Produces a summary report
// and per-test CSV logs.
//
// Tests:
//   1. Balance — settling time, overshoot, disturbance rejection
//   2. Swing-up — catch time, success rate, arm travel
//   3. Robustness — parameter mismatch (wrong Lp, mp, friction)
//   4. Noise — encoder quantization, velocity noise
//
// Build:  make run_tests
// Run:    ./run_tests                    # all tests
//         ./run_tests --suite balance    # only balance tests
//         ./run_tests --suite swingup    # only swing-up tests
//         ./run_tests --suite robust     # only robustness tests
//         ./run_tests --suite noise      # only noise tests
//         ./run_tests --csv              # write per-test CSV files
// ═══════════════════════════════════════════════════════════════════

#include "qube_types.h"
#include "plant.h"
#include "controllers.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <functional>
#include <fstream>
#include <random>
#include <algorithm>
#include <numeric>

// ─── Test result for a single run ──────────────────────────────────
struct TestResult {
    std::string name;

    // Balance metrics
    double settling_time  = -1;  // time to stay within ±2° permanently [s]
    double overshoot_deg  = 0;   // max |alpha| after first zero crossing [deg]
    int    zero_crossings = 0;   // number of times alpha crosses zero
    double max_voltage    = 0;   // peak |voltage| during run
    double rms_alpha_deg  = 0;   // RMS alpha over last 2s [deg]

    // Swing-up metrics
    double catch_time     = -1;  // time until first successful catch [s]
    int    catch_attempts = 0;   // number of times balance was entered
    double max_arm_deg    = 0;   // peak |theta| during swing-up [deg]
    bool   balanced_5s    = false; // did it stay balanced for ≥5s after catch?

    // General
    double final_alpha_deg = 0;
    double final_theta_deg = 0;
    bool   success = false;
};

// ─── Simulation runner ─────────────────────────────────────────────
// Runs a full simulation and collects metrics.
// noise_alpha/noise_theta: std dev of additive Gaussian noise on angle readings.
// The plant_params can differ from controller_params to test robustness.

struct SimConfig {
    std::string name;
    double duration     = 10.0;
    double dt           = 0.001;
    int    substeps     = 10;
    double initial_alpha = M_PI;   // start hanging down
    double initial_theta = 0.0;
    double initial_alpha_dot = 0.0;
    double initial_theta_dot = 0.0;
    bool   start_balancing = false; // true = skip swing-up, start in balance mode
    double noise_alpha  = 0.0;     // angle noise std dev [rad]
    double noise_theta  = 0.0;
    double noise_vel    = 0.0;     // velocity noise std dev [rad/s]

    // Plant (reality)
    QubeParams plant_params;
    // Controller (what the controller thinks the plant is)
    QubeParams ctrl_params;

    // Controller gain overrides (use defaults if NaN)
    float mu = 5.0f;
    float k_alpha = 20.0f, k_alpha_dot = 2.0f;
    float k_theta = -2.0f, k_theta_dot = -1.0f;
    float k_integral = 0.3f;
    float voltage_limit = 6.0f;

    bool write_csv = false;
};

TestResult run_sim_test(const SimConfig& cfg) {
    TestResult result;
    result.name = cfg.name;

    State x = {cfg.initial_theta, cfg.initial_alpha,
               cfg.initial_theta_dot, cfg.initial_alpha_dot};

    BalanceController balance(cfg.dt);
    balance.k_alpha     = cfg.k_alpha;
    balance.k_alpha_dot = cfg.k_alpha_dot;
    balance.k_theta     = cfg.k_theta;
    balance.k_theta_dot = cfg.k_theta_dot;
    balance.k_integral  = cfg.k_integral;
    balance.voltage_limit = cfg.voltage_limit;

    SwingUpController swing_up(cfg.ctrl_params, cfg.dt);
    swing_up.mu = cfg.mu;

    enum Mode { SWING_UP, BALANCE } mode;
    mode = cfg.start_balancing ? BALANCE : SWING_UP;
    if (cfg.start_balancing) balance.reset();

    int n_steps = (int)(cfg.duration / cfg.dt);

    // Noise RNG
    std::mt19937 rng(42);
    std::normal_distribution<double> noise_a(0, cfg.noise_alpha);
    std::normal_distribution<double> noise_t(0, cfg.noise_theta);
    std::normal_distribution<double> noise_v(0, cfg.noise_vel);

    // Tracking
    double first_catch_time = -1;
    double last_balance_start = -1;
    double longest_balance = 0;
    bool   had_first_zero_cross = false;
    double prev_alpha = x[1];
    int    zero_cross_count = 0;
    double max_overshoot = 0;
    double max_voltage = 0;
    double max_arm = 0;
    int    catch_count = 0;

    // For settling time: track when alpha last left ±2°
    double settle_threshold = 2.0 * M_PI / 180.0;
    double last_outside_settle = 0;

    // For RMS over last 2s
    std::vector<double> recent_alpha;
    double rms_window = 2.0;

    // CSV
    std::ofstream csv;
    if (cfg.write_csv) {
        std::string fname = "test_" + cfg.name + ".csv";
        // Replace spaces with underscores
        for (auto& c : fname) if (c == ' ') c = '_';
        csv.open(fname);
        csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,mode\n";
    }

    for (int i = 0; i < n_steps; i++) {
        double t = i * cfg.dt;

        // Add measurement noise
        double meas_theta = x[0] + (cfg.noise_theta > 0 ? noise_t(rng) : 0);
        double meas_alpha = x[1] + (cfg.noise_alpha > 0 ? noise_a(rng) : 0);
        double meas_theta_dot = x[2] + (cfg.noise_vel > 0 ? noise_v(rng) : 0);
        double meas_alpha_dot = x[3] + (cfg.noise_vel > 0 ? noise_v(rng) : 0);

        // Quantize angles to encoder resolution if noise is being applied
        if (cfg.noise_alpha > 0) {
            meas_theta = round(meas_theta / COUNTS_TO_RAD) * COUNTS_TO_RAD;
            meas_alpha = round(meas_alpha / COUNTS_TO_RAD) * COUNTS_TO_RAD;
        }

        QubeState s = {meas_theta, meas_alpha, meas_theta_dot, meas_alpha_dot};
        double voltage = 0.0;

        if (mode == SWING_UP) {
            voltage = swing_up.compute(s);
            if (swing_up.should_catch(s.alpha)) {
                mode = BALANCE;
                balance.reset();
                catch_count++;
                if (first_catch_time < 0) first_catch_time = t;
                last_balance_start = t;
            }
        } else {
            voltage = balance.compute(s);
            if (fabs(s.alpha) > 30.0 * M_PI / 180.0) {
                // Track how long we balanced
                if (last_balance_start >= 0) {
                    double dur = t - last_balance_start;
                    if (dur > longest_balance) longest_balance = dur;
                }
                mode = SWING_UP;
                balance.reset();
            }
        }

        // Track metrics
        if (fabs(voltage) > max_voltage) max_voltage = fabs(voltage);
        if (fabs(x[0]) > max_arm) max_arm = fabs(x[0]);

        // Zero crossings and overshoot (only in balance mode)
        if (mode == BALANCE) {
            if (prev_alpha * x[1] < 0 && fabs(prev_alpha) > 0.001) {
                zero_cross_count++;
                had_first_zero_cross = true;
            }
            if (had_first_zero_cross) {
                double abs_alpha_deg = fabs(x[1]) * 180.0 / M_PI;
                if (abs_alpha_deg > max_overshoot) max_overshoot = abs_alpha_deg;
            }

            // Settling time
            if (fabs(x[1]) > settle_threshold) last_outside_settle = t;

            // RMS window
            if (t > cfg.duration - rms_window) {
                recent_alpha.push_back(x[1] * 180.0 / M_PI);
            }
        }

        prev_alpha = x[1];

        // Log
        if (cfg.write_csv) {
            char line[256];
            snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                     t, x[0], x[1], x[2], x[3], voltage, (int)mode);
            csv << line;
        }

        // Integrate with the REAL plant params
        double sub_dt = cfg.dt / cfg.substeps;
        for (int j = 0; j < cfg.substeps; j++)
            x = rk4_step(x, voltage, sub_dt, cfg.plant_params);
        x[0] = wrap_angle(x[0]);
        x[1] = wrap_angle(x[1]);
    }

    // Final balance duration
    if (mode == BALANCE && last_balance_start >= 0) {
        double dur = cfg.duration - last_balance_start;
        if (dur > longest_balance) longest_balance = dur;
    }

    // Compute results
    result.catch_time     = first_catch_time;
    result.catch_attempts = catch_count;
    result.max_arm_deg    = max_arm * 180.0 / M_PI;
    result.zero_crossings = zero_cross_count;
    result.overshoot_deg  = max_overshoot;
    result.max_voltage    = max_voltage;
    result.final_alpha_deg = x[1] * 180.0 / M_PI;
    result.final_theta_deg = x[0] * 180.0 / M_PI;
    result.balanced_5s    = longest_balance >= 5.0;

    if (mode == BALANCE && last_outside_settle > 0)
        result.settling_time = last_outside_settle + cfg.dt;  // when it last left threshold
    else if (mode == BALANCE)
        result.settling_time = 0;  // never exceeded threshold

    if (!recent_alpha.empty()) {
        double sum_sq = 0;
        for (double a : recent_alpha) sum_sq += a * a;
        result.rms_alpha_deg = sqrt(sum_sq / recent_alpha.size());
    }

    result.success = (mode == BALANCE) && (fabs(x[1]) < 10.0 * M_PI / 180.0);

    if (cfg.write_csv) csv.close();

    return result;
}

// ─── Print utilities ───────────────────────────────────────────────

void print_header(const char* title) {
    printf("\n");
    printf("══════════════════════════════════════════════════════════════\n");
    printf("  %s\n", title);
    printf("══════════════════════════════════════════════════════════════\n");
}

void print_result(const TestResult& r) {
    const char* status = r.success ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m";
    printf("  %-40s [%s]\n", r.name.c_str(), status);

    if (r.catch_time >= 0)
        printf("    catch: %.2fs (%d attempts)  ", r.catch_time, r.catch_attempts);
    if (r.settling_time >= 0)
        printf("    settle: %.3fs  ", r.settling_time);
    if (r.overshoot_deg > 0.1)
        printf("    overshoot: %.1f°  ", r.overshoot_deg);
    if (r.zero_crossings > 0)
        printf("    zero-cross: %d  ", r.zero_crossings);
    printf("\n");

    printf("    max|V|: %.1fV  max|θ|: %.1f°  RMS(α): %.2f°  final: α=%.1f° θ=%.1f°\n",
           r.max_voltage, r.max_arm_deg, r.rms_alpha_deg,
           r.final_alpha_deg, r.final_theta_deg);
    if (r.balanced_5s)
        printf("    ✓ Sustained balance ≥5s\n");
}

void print_summary(const std::vector<TestResult>& results) {
    int pass = 0, fail = 0;
    for (auto& r : results) {
        if (r.success) pass++; else fail++;
    }
    printf("\n──────────────────────────────────────────────────────────────\n");
    printf("  Summary: %d/%d passed", pass, (int)results.size());
    if (fail > 0) printf("  (\033[31m%d failed\033[0m)", fail);
    printf("\n──────────────────────────────────────────────────────────────\n");
}

// ═══════════════════════════════════════════════════════════════════
//                         TEST SUITES
// ═══════════════════════════════════════════════════════════════════

std::vector<TestResult> run_balance_tests(bool write_csv) {
    print_header("BALANCE CONTROLLER TESTS");
    std::vector<TestResult> results;

    auto base = [&](const char* name, double alpha0) {
        SimConfig c;
        c.name = name;
        c.initial_alpha = alpha0;
        c.start_balancing = true;
        c.duration = 10.0;
        c.write_csv = write_csv;
        return c;
    };

    // B1: Small perturbation (5°)
    {
        auto c = base("balance: 5° initial", 5.0 * M_PI / 180);
        results.push_back(run_sim_test(c));
    }
    // B2: Medium perturbation (10°)
    {
        auto c = base("balance: 10° initial", 10.0 * M_PI / 180);
        results.push_back(run_sim_test(c));
    }
    // B3: Large perturbation (20° — at catch zone edge)
    {
        auto c = base("balance: 20° initial", 20.0 * M_PI / 180);
        results.push_back(run_sim_test(c));
    }
    // B4: Negative perturbation
    {
        auto c = base("balance: -15° initial", -15.0 * M_PI / 180);
        results.push_back(run_sim_test(c));
    }
    // B5: With initial arm offset
    {
        auto c = base("balance: α=10° θ=20°", 10.0 * M_PI / 180);
        c.initial_theta = 20.0 * M_PI / 180;
        results.push_back(run_sim_test(c));
    }
    // B6: With initial velocity (pendulum falling)
    {
        auto c = base("balance: α=5° α̇=2 rad/s", 5.0 * M_PI / 180);
        c.initial_alpha_dot = 2.0;
        results.push_back(run_sim_test(c));
    }
    // B7: Extreme (25° — beyond catch zone)
    {
        auto c = base("balance: 25° initial (hard)", 25.0 * M_PI / 180);
        results.push_back(run_sim_test(c));
    }

    for (auto& r : results) print_result(r);
    return results;
}

std::vector<TestResult> run_swingup_tests(bool write_csv) {
    print_header("SWING-UP CONTROLLER TESTS");
    std::vector<TestResult> results;

    // S1: Default swing-up from hanging
    {
        SimConfig c;
        c.name = "swingup: default (mu=5)";
        c.duration = 20.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // S2: Higher mu
    {
        SimConfig c;
        c.name = "swingup: mu=8";
        c.duration = 20.0;
        c.mu = 8.0f;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // S3: Lower mu (gentle)
    {
        SimConfig c;
        c.name = "swingup: mu=3 (gentle)";
        c.duration = 30.0;
        c.mu = 3.0f;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // S4: Starting at 45° (partial swing)
    {
        SimConfig c;
        c.name = "swingup: start at 135° from up";
        c.initial_alpha = 135.0 * M_PI / 180;
        c.duration = 15.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // S5: Starting nearly upright (should catch immediately)
    {
        SimConfig c;
        c.name = "swingup: start at 15° (easy catch)";
        c.initial_alpha = 15.0 * M_PI / 180;
        c.duration = 10.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // S6: With initial arm offset
    {
        SimConfig c;
        c.name = "swingup: θ₀=30°";
        c.initial_theta = 30.0 * M_PI / 180;
        c.duration = 20.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }

    for (auto& r : results) print_result(r);
    return results;
}

std::vector<TestResult> run_robustness_tests(bool write_csv) {
    print_header("ROBUSTNESS TESTS (parameter mismatch)");
    std::vector<TestResult> results;

    auto mismatch = [&](const char* name, std::function<void(QubeParams&)> modify) {
        SimConfig c;
        c.name = name;
        c.duration = 15.0;
        c.write_csv = write_csv;
        // Controller uses default params
        // Plant uses modified params
        modify(c.plant_params);
        return c;
    };

    // R1: Pendulum 20% longer than controller thinks
    {
        auto c = mismatch("robust: Lp +20%", [](QubeParams& p) { p.Lp *= 1.2; });
        results.push_back(run_sim_test(c));
    }
    // R2: Pendulum 20% shorter
    {
        auto c = mismatch("robust: Lp -20%", [](QubeParams& p) { p.Lp *= 0.8; });
        results.push_back(run_sim_test(c));
    }
    // R3: Pendulum 30% heavier
    {
        auto c = mismatch("robust: mp +30%", [](QubeParams& p) { p.mp *= 1.3; });
        results.push_back(run_sim_test(c));
    }
    // R4: Significant arm friction
    {
        auto c = mismatch("robust: Dr=0.005", [](QubeParams& p) { p.Dr = 0.005; });
        results.push_back(run_sim_test(c));
    }
    // R5: Significant pendulum friction
    {
        auto c = mismatch("robust: Dp=0.001", [](QubeParams& p) { p.Dp = 0.001; });
        results.push_back(run_sim_test(c));
    }
    // R6: Wrong motor resistance (+20%)
    {
        auto c = mismatch("robust: Rm +20%", [](QubeParams& p) { p.Rm *= 1.2; });
        results.push_back(run_sim_test(c));
    }
    // R7: Lp = 9.5cm (datasheet vs workbook 12.9cm)
    {
        auto c = mismatch("robust: Lp=9.5cm (datasheet)", [](QubeParams& p) { p.Lp = 0.095; });
        results.push_back(run_sim_test(c));
    }
    // R8: Combined: Lp wrong + friction + heavier
    {
        auto c = mismatch("robust: Lp+10% mp+15% Dr=0.002", [](QubeParams& p) {
            p.Lp *= 1.1; p.mp *= 1.15; p.Dr = 0.002;
        });
        results.push_back(run_sim_test(c));
    }

    for (auto& r : results) print_result(r);
    return results;
}

std::vector<TestResult> run_noise_tests(bool write_csv) {
    print_header("NOISE ROBUSTNESS TESTS");
    std::vector<TestResult> results;

    // N1: Encoder quantization only (balance from 10°)
    {
        SimConfig c;
        c.name = "noise: encoder quant only";
        c.initial_alpha = 10.0 * M_PI / 180;
        c.start_balancing = true;
        c.noise_alpha = COUNTS_TO_RAD;  // 1 count std dev
        c.duration = 10.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // N2: Moderate noise (balance)
    {
        SimConfig c;
        c.name = "noise: moderate (0.003 rad)";
        c.initial_alpha = 10.0 * M_PI / 180;
        c.start_balancing = true;
        c.noise_alpha = 0.003;
        c.noise_theta = 0.003;
        c.duration = 10.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // N3: Velocity noise (balance)
    {
        SimConfig c;
        c.name = "noise: velocity noise 0.5 rad/s";
        c.initial_alpha = 10.0 * M_PI / 180;
        c.start_balancing = true;
        c.noise_vel = 0.5;
        c.duration = 10.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // N4: Heavy noise (balance)
    {
        SimConfig c;
        c.name = "noise: heavy (0.01 rad + 1 rad/s)";
        c.initial_alpha = 10.0 * M_PI / 180;
        c.start_balancing = true;
        c.noise_alpha = 0.01;
        c.noise_theta = 0.01;
        c.noise_vel = 1.0;
        c.duration = 10.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // N5: Swing-up with noise
    {
        SimConfig c;
        c.name = "noise: swing-up + moderate noise";
        c.noise_alpha = 0.003;
        c.noise_theta = 0.003;
        c.noise_vel = 0.3;
        c.duration = 20.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }
    // N6: Swing-up with heavy noise
    {
        SimConfig c;
        c.name = "noise: swing-up + heavy noise";
        c.noise_alpha = 0.01;
        c.noise_theta = 0.01;
        c.noise_vel = 1.0;
        c.duration = 25.0;
        c.write_csv = write_csv;
        results.push_back(run_sim_test(c));
    }

    for (auto& r : results) print_result(r);
    return results;
}

// ═══════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    std::string suite = "all";
    bool write_csv = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--suite") && i+1 < argc) suite = argv[++i];
        else if (!strcmp(argv[i], "--csv")) write_csv = true;
    }

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║     Qube-Servo 3 — Controller Performance Evaluation       ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("  Suite: %s   CSV output: %s\n", suite.c_str(), write_csv ? "on" : "off");

    std::vector<TestResult> all_results;

    if (suite == "all" || suite == "balance") {
        auto r = run_balance_tests(write_csv);
        all_results.insert(all_results.end(), r.begin(), r.end());
    }
    if (suite == "all" || suite == "swingup") {
        auto r = run_swingup_tests(write_csv);
        all_results.insert(all_results.end(), r.begin(), r.end());
    }
    if (suite == "all" || suite == "robust") {
        auto r = run_robustness_tests(write_csv);
        all_results.insert(all_results.end(), r.begin(), r.end());
    }
    if (suite == "all" || suite == "noise") {
        auto r = run_noise_tests(write_csv);
        all_results.insert(all_results.end(), r.begin(), r.end());
    }

    print_summary(all_results);

    return 0;
}
