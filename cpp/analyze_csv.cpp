// ═══════════════════════════════════════════════════════════════════
// analyze_csv.cpp — Analyze hardware CSV logs with test metrics
// ═══════════════════════════════════════════════════════════════════
//
// Reads CSV output from run_hardware, run_balance, or run_gui_hw
// and computes the same performance metrics as run_tests.
//
// Usage:
//   ./analyze_csv hardware_output.csv
//   ./analyze_csv balance_output.csv
//   ./analyze_csv *.csv
//
// Expected CSV columns:
//   t, theta, alpha, theta_dot, alpha_dot, voltage, mode (or active)
//
// Build: make analyze_csv
// ═══════════════════════════════════════════════════════════════════

#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

struct Sample {
    double t, theta, alpha, theta_dot, alpha_dot, voltage;
    int mode;  // 0 = swing_up, 1 = balance
};

struct Metrics {
    std::string filename;

    // Timing
    double duration        = 0;
    double first_catch_t   = -1;
    int    catch_count     = 0;
    int    fall_count      = 0;
    double longest_balance = 0;
    bool   balanced_5s     = false;

    // Balance quality
    double settling_time   = -1;
    double overshoot_deg   = 0;
    int    zero_crossings  = 0;
    double rms_alpha_deg   = 0;
    double rms_alpha_last2s = 0;
    double max_alpha_deg   = 0;

    // Effort
    double max_voltage     = 0;
    double rms_voltage     = 0;
    double max_arm_deg     = 0;

    // Final state
    double final_alpha_deg = 0;
    double final_theta_deg = 0;
    int    final_mode      = 0;
};

std::vector<Sample> load_csv(const char* path) {
    std::vector<Sample> samples;
    std::ifstream f(path);
    if (!f.is_open()) {
        fprintf(stderr, "Cannot open: %s\n", path);
        return samples;
    }

    std::string line;
    std::getline(f, line);  // skip header

    while (std::getline(f, line)) {
        Sample s;
        // Replace commas with spaces for sscanf
        for (auto& c : line) if (c == ',') c = ' ';
        if (sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %d",
                   &s.t, &s.theta, &s.alpha, &s.theta_dot, &s.alpha_dot,
                   &s.voltage, &s.mode) >= 7) {
            samples.push_back(s);
        }
    }
    return samples;
}

Metrics analyze(const char* path) {
    Metrics m;
    m.filename = path;

    auto samples = load_csv(path);
    if (samples.empty()) {
        fprintf(stderr, "  No data in %s\n", path);
        return m;
    }

    int n = (int)samples.size();
    m.duration = samples.back().t;

    // ── Track mode transitions ─────────────────────────────────
    int prev_mode = samples[0].mode;
    double balance_start = -1;
    if (prev_mode == 1) balance_start = samples[0].t;

    for (int i = 1; i < n; i++) {
        int cur_mode = samples[i].mode;

        // Entered balance
        if (cur_mode == 1 && prev_mode == 0) {
            m.catch_count++;
            if (m.first_catch_t < 0) m.first_catch_t = samples[i].t;
            balance_start = samples[i].t;
        }

        // Left balance (fell)
        if (cur_mode == 0 && prev_mode == 1) {
            m.fall_count++;
            if (balance_start >= 0) {
                double dur = samples[i].t - balance_start;
                if (dur > m.longest_balance) m.longest_balance = dur;
            }
            balance_start = -1;
        }

        prev_mode = cur_mode;
    }

    // Final balance segment
    if (samples.back().mode == 1 && balance_start >= 0) {
        double dur = samples.back().t - balance_start;
        if (dur > m.longest_balance) m.longest_balance = dur;
    }
    m.balanced_5s = m.longest_balance >= 5.0;

    // ── Balance quality metrics (only during balance mode) ─────
    double settle_threshold = 2.0 * M_PI / 180.0;
    double last_outside_settle = -1;
    bool had_first_zero_cross = false;
    double prev_alpha = 0;
    bool in_balance = false;

    double sum_alpha_sq = 0;
    int    balance_count = 0;
    double sum_alpha_last2s = 0;
    int    count_last2s = 0;
    double sum_volt_sq = 0;

    for (int i = 0; i < n; i++) {
        auto& s = samples[i];

        double abs_alpha_deg = fabs(s.alpha) * 180.0 / M_PI;
        double abs_theta_deg = fabs(s.theta) * 180.0 / M_PI;

        if (fabs(s.voltage) > m.max_voltage) m.max_voltage = fabs(s.voltage);
        if (abs_theta_deg > m.max_arm_deg)   m.max_arm_deg = abs_theta_deg;
        sum_volt_sq += s.voltage * s.voltage;

        if (s.mode == 1) {
            // Settling
            if (fabs(s.alpha) > settle_threshold)
                last_outside_settle = s.t;

            // Zero crossings
            if (i > 0 && in_balance) {
                if (prev_alpha * s.alpha < 0 && fabs(prev_alpha) > 0.001) {
                    m.zero_crossings++;
                    had_first_zero_cross = true;
                }
            }

            // Overshoot (after first zero crossing)
            if (had_first_zero_cross && abs_alpha_deg > m.overshoot_deg)
                m.overshoot_deg = abs_alpha_deg;

            // Max alpha in balance
            if (abs_alpha_deg > m.max_alpha_deg)
                m.max_alpha_deg = abs_alpha_deg;

            // RMS
            sum_alpha_sq += (s.alpha * 180.0 / M_PI) * (s.alpha * 180.0 / M_PI);
            balance_count++;

            // Last 2s RMS
            if (s.t > m.duration - 2.0) {
                double ad = s.alpha * 180.0 / M_PI;
                sum_alpha_last2s += ad * ad;
                count_last2s++;
            }

            in_balance = true;
        } else {
            in_balance = false;
            had_first_zero_cross = false;
        }

        prev_alpha = s.alpha;
    }

    if (last_outside_settle >= 0 && samples.back().mode == 1)
        m.settling_time = last_outside_settle - m.first_catch_t;
    else if (samples.back().mode == 1 && m.first_catch_t >= 0)
        m.settling_time = 0;

    if (balance_count > 0)
        m.rms_alpha_deg = sqrt(sum_alpha_sq / balance_count);
    if (count_last2s > 0)
        m.rms_alpha_last2s = sqrt(sum_alpha_last2s / count_last2s);
    if (n > 0)
        m.rms_voltage = sqrt(sum_volt_sq / n);

    m.final_alpha_deg = samples.back().alpha * 180.0 / M_PI;
    m.final_theta_deg = samples.back().theta * 180.0 / M_PI;
    m.final_mode = samples.back().mode;

    return m;
}

void print_metrics(const Metrics& m) {
    printf("\n");
    printf("══════════════════════════════════════════════════════════════\n");
    printf("  %s\n", m.filename.c_str());
    printf("══════════════════════════════════════════════════════════════\n");

    printf("  Duration: %.2f s   Samples: (dt estimated from data)\n", m.duration);
    printf("\n");

    // Swing-up
    printf("  ── Swing-Up ──────────────────────────────────────────────\n");
    if (m.first_catch_t >= 0)
        printf("  First catch:      %.3f s\n", m.first_catch_t);
    else
        printf("  First catch:      NEVER\n");
    printf("  Catch attempts:   %d\n", m.catch_count);
    printf("  Falls:            %d\n", m.fall_count);
    printf("  Longest balance:  %.2f s %s\n",
           m.longest_balance, m.balanced_5s ? "(≥5s ✓)" : "");
    printf("  Max arm travel:   %.1f°\n", m.max_arm_deg);
    printf("\n");

    // Balance
    printf("  ── Balance Quality ───────────────────────────────────────\n");
    if (m.settling_time >= 0)
        printf("  Settling time:    %.3f s (±2° threshold)\n", m.settling_time);
    else
        printf("  Settling time:    N/A (not balanced at end)\n");
    printf("  Overshoot:        %.1f°\n", m.overshoot_deg);
    printf("  Zero crossings:   %d\n", m.zero_crossings);
    printf("  Max |α| (bal):    %.1f°\n", m.max_alpha_deg);
    printf("  RMS α (balance):  %.2f°\n", m.rms_alpha_deg);
    printf("  RMS α (last 2s):  %.2f°\n", m.rms_alpha_last2s);
    printf("\n");

    // Effort
    printf("  ── Control Effort ────────────────────────────────────────\n");
    printf("  Max |voltage|:    %.2f V\n", m.max_voltage);
    printf("  RMS voltage:      %.2f V\n", m.rms_voltage);
    printf("\n");

    // Final
    printf("  ── Final State ───────────────────────────────────────────\n");
    printf("  Mode:             %s\n", m.final_mode == 1 ? "BALANCING" : "SWING-UP");
    printf("  α:                %.2f°\n", m.final_alpha_deg);
    printf("  θ:                %.2f°\n", m.final_theta_deg);

    // Grade
    printf("\n");
    if (m.balanced_5s && m.rms_alpha_last2s < 2.0)
        printf("  Grade: \033[32m★★★ EXCELLENT\033[0m\n");
    else if (m.balanced_5s)
        printf("  Grade: \033[32m★★  GOOD\033[0m\n");
    else if (m.first_catch_t >= 0)
        printf("  Grade: \033[33m★   CAUGHT but unstable\033[0m\n");
    else
        printf("  Grade: \033[31m    FAILED (never caught)\033[0m\n");

    printf("\n");
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <csv_file> [csv_file2 ...]\n", argv[0]);
        printf("\nAnalyzes CSV logs from run_hardware, run_balance, or run_gui_hw.\n");
        printf("Computes balance quality, swing-up performance, and control effort metrics.\n");
        printf("\nExamples:\n");
        printf("  %s hardware_output.csv\n", argv[0]);
        printf("  %s balance_output.csv\n", argv[0]);
        printf("  %s test_*.csv\n", argv[0]);
        return 1;
    }

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║     Qube-Servo 3 — Hardware Log Analysis                   ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");

    for (int i = 1; i < argc; i++) {
        Metrics m = analyze(argv[i]);
        print_metrics(m);
    }

    // Comparison table if multiple files
    if (argc > 2) {
        printf("══════════════════════════════════════════════════════════════\n");
        printf("  COMPARISON\n");
        printf("══════════════════════════════════════════════════════════════\n");
        printf("  %-30s %7s %7s %7s %7s %7s\n",
               "File", "Catch", "Settle", "RMS(α)", "Max|V|", "Bal(s)");
        printf("  %-30s %7s %7s %7s %7s %7s\n",
               "----", "-----", "------", "------", "------", "------");
        for (int i = 1; i < argc; i++) {
            Metrics m = analyze(argv[i]);
            // Shorten filename
            std::string fname = argv[i];
            if (fname.size() > 30) fname = "..." + fname.substr(fname.size() - 27);

            printf("  %-30s", fname.c_str());
            if (m.first_catch_t >= 0) printf(" %6.2fs", m.first_catch_t);
            else printf("    N/A");
            if (m.settling_time >= 0) printf(" %6.3fs", m.settling_time);
            else printf("    N/A");
            printf("  %5.2f°  %5.1fV  %5.1fs\n",
                   m.rms_alpha_last2s, m.max_voltage, m.longest_balance);
        }
        printf("\n");
    }

    return 0;
}
