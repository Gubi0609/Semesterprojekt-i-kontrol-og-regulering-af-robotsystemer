// ═══════════════════════════════════════════════════════════════════
// run_step_test_hw.cpp — Hardware step-response test for Qube-Servo 3
// ═══════════════════════════════════════════════════════════════════
//
// Automated test procedure (real hardware):
//   Phase 1 — SWING-UP: Energy-based controller pumps the pendulum
//             up from hanging.
//   Phase 2 — COAST:    Once |α| < trigger angle, motor goes to 0V.
//             The pendulum coasts freely. We wait for it to re-enter
//             the trigger zone with |α̇| below a velocity threshold,
//             ensuring near-zero angular momentum at engage.
//   Phase 3 — BALANCE:  Controller engages. Data is logged and
//             metrics are computed (rise time, overshoot, settling).
//
// This gives a repeatable initial condition without manual
// intervention — just run the program and it does everything.
//
// CSV filename encodes gains + timestamp:
//   step_hw_aKp20.0_aKi0.3_aKd2.0_tKp2.0_tKd1.0_20260522_143021.csv
//
// Build:  make run_step_test_hw
// Run:    ./run_step_test_hw                    # defaults
//         ./run_step_test_hw --trigger 15       # engage at 15°
//         ./run_step_test_hw --vel 0.5          # tighter velocity gate
//         ./run_step_test_hw --aKp 25 --aKd 3  # override gains
//         ./run_step_test_hw --outdir results
// ═══════════════════════════════════════════════════════════════════

#include "qube_types.h"
#include "controllers.h"

#include <hil.h>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <string>
#include <vector>
#include <fstream>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

// ─── Encoder calibration (same as run_hardware.cpp) ────────────────
static constexpr double PEND_OFFSET_RAD = M_PI;
static constexpr t_uint32 TACHO_CHANNELS[] = {14000, 14001};

// ─── Timing helpers ────────────────────────────────────────────────
static void sleep_until(struct timespec* next) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, nullptr);
}

static void timespec_add_ns(struct timespec* ts, long ns) {
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

// ─── Ctrl+C handler ───────────────────────────────────────────────
static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

// ─── Safe shutdown helper ──────────────────────────────────────────
static void safe_shutdown(t_card card) {
    const t_uint32 aout_ch[] = {0};
    const t_uint32 dout_ch[] = {0};
    t_double zero_v[] = {0.0};
    t_boolean disable[] = {0};
    hil_write_analog(card, aout_ch, 1, zero_v);
    hil_write_digital(card, dout_ch, 1, disable);
    hil_close(card);
}

// ─── Step-response metrics ─────────────────────────────────────────
struct StepMetrics {
    double initial_alpha_deg;
    double initial_alpha_dot;
    double rise_time;
    double overshoot_deg;
    double overshoot_pct;
    double settling_time;
    double steady_state_err;
    double max_voltage;
    bool   settled;
};

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
            m.rise_time = t_vec[i] - t_vec[0];
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
    double t0 = t_vec[0];
    if (last_outside < 0) {
        m.settling_time = 0;
        m.settled = true;
    } else if (last_outside < t_vec.back() - 0.5) {
        m.settling_time = last_outside - t0;
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

// ─── Generate timestamped filename ─────────────────────────────────
std::string make_filename(const BalanceController& bal, const std::string& outdir) {
    time_t now = time(nullptr);
    struct tm* lt = localtime(&now);

    char ts[64];
    snprintf(ts, sizeof(ts), "%04d%02d%02d_%02d%02d%02d",
             lt->tm_year + 1900, lt->tm_mon + 1, lt->tm_mday,
             lt->tm_hour, lt->tm_min, lt->tm_sec);

    char buf[512];
    snprintf(buf, sizeof(buf),
             "step_hw_aKp%.1f_aKi%.1f_aKd%.1f_tKp%.1f_tKd%.1f_%s.csv",
             bal.alpha_pid.Kp, bal.alpha_pid.Ki, bal.alpha_Kd,
             bal.theta_pid.Kp, bal.theta_Kd, ts);

    std::string fname = buf;
    if (!outdir.empty()) {
        mkdir(outdir.c_str(), 0755);
        fname = outdir + "/" + fname;
    }
    return fname;
}

// ─── Print report ──────────────────────────────────────────────────
void print_report(const StepMetrics& m, const BalanceController& bal,
                  const std::string& csv_path) {
    printf("\n");
    printf("══════════════════════════════════════════════════════════════\n");
    printf("  Hardware Step-Response Results\n");
    printf("══════════════════════════════════════════════════════════════\n");
    printf("\n");
    printf("  Controller gains:\n");
    printf("    Alpha PID:  Kp=%.2f  Ki=%.2f  Kd=%.2f\n",
           bal.alpha_pid.Kp, bal.alpha_pid.Ki, bal.alpha_Kd);
    printf("    Theta PID:  Kp=%.2f  Ki=%.2f  Kd=%.2f\n",
           bal.theta_pid.Kp, bal.theta_pid.Ki, bal.theta_Kd);
    printf("    Voltage limit: %.1f V\n", bal.voltage_limit);
    printf("\n");
    printf("  At engage:  α=%.2f°  α̇=%.2f rad/s\n",
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
    double trigger_deg  = 20.0;   // engage angle threshold [deg]
    double vel_thresh   = 1.0;    // max |α̇| at engage [rad/s]
    double duration     = 10.0;   // balance phase duration [s]
    double dt           = 0.001;  // 1 kHz
    double alpha_band   = 5.0;    // settling band [deg]
    double theta_max_deg = 30.0;  // max |theta| at engage [deg]
    double theta_dot_max = 2.0;   // max |theta_dot| at engage [rad/s]
    std::string outdir  = "";

    double aKp = 20.0, aKi = 0.3, aKd = 2.0;
    double tKp = 2.0,  tKi = 0.0, tKd = 1.0;
    double vlim = 6.0;

    // ── Parse arguments ────────────────────────────────────────
    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--trigger")  && i+1 < argc) trigger_deg  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--vel")      && i+1 < argc) vel_thresh   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--duration") && i+1 < argc) duration     = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt")       && i+1 < argc) dt           = atof(argv[++i]);
        else if (!strcmp(argv[i], "--band")     && i+1 < argc) alpha_band   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--thmax")   && i+1 < argc) theta_max_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--thdmax")  && i+1 < argc) theta_dot_max = atof(argv[++i]);
        else if (!strcmp(argv[i], "--outdir")   && i+1 < argc) outdir       = argv[++i];
        else if (!strcmp(argv[i], "--aKp")      && i+1 < argc) aKp  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--aKi")      && i+1 < argc) aKi  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--aKd")      && i+1 < argc) aKd  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tKp")      && i+1 < argc) tKp  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tKi")      && i+1 < argc) tKi  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tKd")      && i+1 < argc) tKd  = atof(argv[++i]);
        else if (!strcmp(argv[i], "--vlim")     && i+1 < argc) vlim = atof(argv[++i]);
        else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("  --trigger DEG   Engage angle threshold (default: 20)\n");
            printf("  --vel RAD/S     Max |α̇| at engage (default: 1.0)\n");
            printf("  --duration SEC  Balance phase duration (default: 10)\n");
            printf("  --band DEG      Settling band (default: 2)\n");
            printf("  --thmax DEG     Max |theta| at engage (default: 30)\n");
            printf("  --thdmax RAD/S  Max |theta_dot| at engage (default: 2.0)\n");
            printf("  --outdir DIR    Output directory for CSV\n");
            printf("  --aKp/aKi/aKd   Alpha PID gains\n");
            printf("  --tKp/tKi/tKd   Theta PID gains\n");
            printf("  --vlim VAL      Voltage limit (default: 6)\n");
            return 0;
        }
    }

    signal(SIGINT, sigint_handler);

    // ── Set up controllers ─────────────────────────────────────
    QubeParams params;

    BalanceController balance(dt);
    balance.alpha_pid.Kp = aKp;
    balance.alpha_pid.Ki = aKi;
    balance.alpha_Kd     = aKd;
    balance.theta_pid.Kp = tKp;
    balance.theta_pid.Ki = tKi;
    balance.theta_Kd     = tKd;
    balance.voltage_limit = vlim;

    double trigger_rad = trigger_deg * M_PI / 180.0;

    // ── Open hardware ──────────────────────────────────────────
    t_card card;
    t_error result = hil_open("qube_servo3_usb", "0", &card);
    if (result < 0) {
        fprintf(stderr, "Failed to open Qube-Servo 3 (error %d)\n", result);
        return 1;
    }

    const t_uint32 enc_ch[]  = {0, 1};
    const t_uint32 aout_ch[] = {0};
    const t_uint32 dout_ch[] = {0};

    // Enable amplifier
    t_boolean enable[] = {1};
    hil_write_digital(card, dout_ch, 1, enable);

    // Zero encoders
    t_int32 zero_counts[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero_counts);

    // I/O buffers
    t_int32  enc_buf[2];
    t_double tacho_buf[2];
    t_double aout_buf[1] = {0.0};

    long dt_ns = (long)(dt * 1e9);

    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║     Qube-Servo 3 — Hardware Step-Response Test             ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n");
    printf("  Trigger: |α| < %.1f° AND |α̇| < %.1f rad/s AND |θ| < %.1f° AND |θ̇| < %.1f rad/s\n",
           trigger_deg, vel_thresh, theta_max_deg, theta_dot_max);
    printf("  Balance duration: %.1fs   Settling band: ±%.1f°\n", duration, alpha_band);
    printf("  Gains — Alpha: Kp=%.2f Ki=%.2f Kd=%.2f\n", aKp, aKi, aKd);
    printf("          Theta: Kp=%.2f Ki=%.2f Kd=%.2f  Vlim=%.1fV\n",
           tKp, tKi, tKd, vlim);

    // ════════════════════════════════════════════════════════════
    //  PHASE 1: Wait for manual positioning — Enter to engage
    // ════════════════════════════════════════════════════════════
    printf("\n  Position the pendulum near upright.\n");
    printf("  Display turns \033[32mGREEN\033[0m when ready. Press ENTER to engage. Ctrl+C to abort.\n\n");

    // Put terminal in non-blocking raw mode so we can poll for Enter
    struct termios orig_term, raw_term;
    tcgetattr(STDIN_FILENO, &orig_term);
    raw_term = orig_term;
    raw_term.c_lflag &= ~(ICANON | ECHO);  // no line buffering, no echo
    raw_term.c_cc[VMIN]  = 0;               // non-blocking
    raw_term.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw_term);

    struct timespec t_next;
    clock_gettime(CLOCK_MONOTONIC, &t_next);

    bool ready = false;
    double engage_alpha = 0, engage_alpha_dot = 0;
    double theta_max_rad = theta_max_deg * M_PI / 180.0;
    int print_count = 0;

    while (g_running) {
        hil_read_encoder(card, enc_ch, 2, enc_buf);
        hil_read_other(card, TACHO_CHANNELS, 2, tacho_buf);

        double theta     = enc_buf[0] * COUNTS_TO_RAD;
        double alpha     = wrap_angle(-(enc_buf[1] * COUNTS_TO_RAD - PEND_OFFSET_RAD));
        double theta_dot = tacho_buf[0] * COUNTS_TO_RAD;
        double alpha_dot = -tacho_buf[1] * COUNTS_TO_RAD;

        bool conditions_met = fabs(alpha) < trigger_rad &&
                              fabs(alpha_dot) < vel_thresh &&
                              fabs(theta) < theta_max_rad &&
                              fabs(theta_dot) < theta_dot_max;

        // Check for Enter keypress
        char ch;
        if (read(STDIN_FILENO, &ch, 1) == 1 && (ch == '\n' || ch == '\r')) {
            if (conditions_met) {
                engage_alpha = alpha;
                engage_alpha_dot = alpha_dot;
                printf("\r\033[0m  ENGAGED: α=%+.2f° α̇=%+.2f  θ=%+.1f° θ̇=%+.1f                    \n",
                       alpha * 180.0 / M_PI, alpha_dot,
                       theta * 180.0 / M_PI, theta_dot);
                ready = true;
                break;
            }
            // Enter pressed but conditions not met — flash red
            printf("\r\033[31m  NOT READY — conditions not met!                                    \033[0m");
            fflush(stdout);
        }

        // Live display ~20 Hz
        if (++print_count % 50 == 0) {
            const char* color = conditions_met ? "\033[32m" : "\033[0m";
            const char* tag   = conditions_met ? " [READY] " : "         ";
            printf("\r%s  α=%+7.2f°  α̇=%+6.2f    θ=%+7.2f°  θ̇=%+6.2f  %s\033[0m",
                   color,
                   alpha * 180.0 / M_PI, alpha_dot,
                   theta * 180.0 / M_PI, theta_dot,
                   tag);
            fflush(stdout);

        }

        timespec_add_ns(&t_next, dt_ns);
        sleep_until(&t_next);
    }

    // Restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_term);

    if (!ready || !g_running) {
        printf("\n  Aborted.\n");
        safe_shutdown(card);
        return 1;
    }

    // ════════════════════════════════════════════════════════════
    //  PHASE 3: Balance — controller engaged, logging data
    // ════════════════════════════════════════════════════════════
    printf("  Phase 3: Balance controller engaged. Recording %.1fs...\n", duration);

    balance.reset();

    int n_steps = (int)(duration / dt);
    std::vector<double> t_log, alpha_log, voltage_log;
    t_log.reserve(n_steps);
    alpha_log.reserve(n_steps);
    voltage_log.reserve(n_steps);

    std::string csv_path = make_filename(balance, outdir);
    std::ofstream csv(csv_path);
    csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,mode\n";

    // Re-sync timer
    clock_gettime(CLOCK_MONOTONIC, &t_next);

    bool fell = false;

    for (int i = 0; i < n_steps && g_running; i++) {
        double t = i * dt;

        hil_read_encoder(card, enc_ch, 2, enc_buf);
        hil_read_other(card, TACHO_CHANNELS, 2, tacho_buf);

        double theta     = enc_buf[0] * COUNTS_TO_RAD;
        double alpha     = wrap_angle(-(enc_buf[1] * COUNTS_TO_RAD - PEND_OFFSET_RAD));
        double theta_dot = tacho_buf[0] * COUNTS_TO_RAD;
        double alpha_dot = -tacho_buf[1] * COUNTS_TO_RAD;

        QubeState s = {theta, alpha, theta_dot, alpha_dot};

        double voltage = 0.0;
        int mode = 1;

        if (!fell) {
            voltage = balance.compute(s);
            if (fabs(alpha) > 30.0 * M_PI / 180.0) {
                fell = true;
                voltage = 0.0;
                mode = 0;
                printf("  [%.2fs] Pendulum fell (α=%.1f°) — motor off.\n",
                       t, alpha * 180.0 / M_PI);
            }
        } else {
            mode = 0;
        }

        aout_buf[0] = voltage;
        hil_write_analog(card, aout_ch, 1, aout_buf);

        t_log.push_back(t);
        alpha_log.push_back(alpha);
        voltage_log.push_back(voltage);

        char line[256];
        snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                 t, theta, alpha, theta_dot, alpha_dot, voltage, mode);
        csv << line;

        timespec_add_ns(&t_next, dt_ns);
        sleep_until(&t_next);
    }

    // ════════════════════════════════════════════════════════════
    //  SAFE SHUTDOWN + METRICS
    // ════════════════════════════════════════════════════════════
    safe_shutdown(card);
    csv.close();

    StepMetrics metrics = compute_metrics(t_log, alpha_log, voltage_log, alpha_band);
    metrics.initial_alpha_deg = fabs(engage_alpha) * 180.0 / M_PI;
    metrics.initial_alpha_dot = engage_alpha_dot;
    print_report(metrics, balance, csv_path);

    return 0;
}
