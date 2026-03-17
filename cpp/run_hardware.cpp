// Real-time balance controller for the Qube-Servo 3.
// Requires the Quanser C HIL API (libhil).
//
// Build:  g++ -O2 -std=c++17 -o run_hardware run_hardware.cpp -lhil -lquanser_common -lm
// Run:    ./run_hardware
//         ./run_hardware --duration 30

#include "qube_types.h"
#include "controllers.h"

#include <hil.h>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>

// ─── Calibration ───────────────────────────────────────────────────
// Encoder zero = hanging down (rest). 180 deg (pi rad) = upright.
// Controller convention: alpha=0 is upright, alpha=±pi is down.
// Transform: alpha_controller = encoder_rad - pi
//   At rest (enc=0):      alpha = -pi  (down) ✓
//   At upright (enc=pi):  alpha = 0           ✓
static constexpr double PEND_OFFSET_RAD = M_PI;

// ─── Filtered differentiator for velocity estimation ───────────────
struct VelocityFilter {
    double cutoff_hz;
    double dt;
    double alpha;  // IIR coefficient
    double prev_x;
    double filtered;
    bool   initialized;

    VelocityFilter(double cutoff, double dt_)
        : cutoff_hz(cutoff), dt(dt_), prev_x(0), filtered(0), initialized(false)
    {
        double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
        alpha = rc / (rc + dt);
    }

    double update(double x) {
        if (!initialized) {
            prev_x = x;
            initialized = true;
            return 0.0;
        }
        double raw = (x - prev_x) / dt;
        prev_x = x;
        filtered = alpha * filtered + (1.0 - alpha) * raw;
        return filtered;
    }

    void reset() {
        prev_x = 0;
        filtered = 0;
        initialized = false;
    }
};

// ─── Precise sleep ─────────────────────────────────────────────────
static void sleep_until(struct timespec* next) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, nullptr);
}

static void timespec_add_ns(struct timespec* ts, long ns) {
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

// ─── Signal handler for clean shutdown ─────────────────────────────
static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

// ───────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    double duration = 30.0;
    double dt       = 0.002;  // 500 Hz

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt") && i + 1 < argc)  dt = atof(argv[++i]);
    }

    signal(SIGINT, sigint_handler);

    // ── Open board ──
    t_card card;
    t_error result = hil_open("qube_servo3_usb", "0", &card);
    if (result < 0) {
        fprintf(stderr, "Failed to open Qube-Servo 3 (error %d)\n", result);
        return 1;
    }
    printf("Connected to Qube-Servo 3.\n");

    // ── Channel setup ──
    const t_uint32 enc_ch[]  = {0, 1};
    const t_uint32 aout_ch[] = {0};
    const t_uint32 dout_ch[] = {0};

    // Enable motor amplifier
    t_boolean enable[] = {1};
    hil_write_digital(card, dout_ch, 1, enable);

    // Zero motor encoder (don't zero pendulum — calibration offset handles it)
    t_int32 zero_counts[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero_counts);

    // ── Controllers ──
    QubeParams params;
    BalanceController balance(dt);
    SwingUpController swing_up(params);

    VelocityFilter theta_filt(50.0, dt);
    VelocityFilter alpha_filt(50.0, dt);

    enum Mode { SWING_UP, BALANCE } mode = SWING_UP;

    // ── CSV log ──
    std::ofstream csv("hardware_output.csv");
    csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,mode\n";

    int n_steps = static_cast<int>(duration / dt);
    long dt_ns  = static_cast<long>(dt * 1e9);

    printf("Running: %.1fs at %.0f Hz. Ctrl+C to stop.\n", duration, 1.0 / dt);

    // ── Control loop ──
    struct timespec t_next;
    clock_gettime(CLOCK_MONOTONIC, &t_next);

    t_int32  enc_buf[2];
    t_double aout_buf[1];

    for (int i = 0; i < n_steps && g_running; i++) {
        double t = i * dt;

        // Read encoders
        hil_read_encoder(card, enc_ch, 2, enc_buf);

        double theta = enc_buf[0] * COUNTS_TO_RAD;
        double alpha = wrap_angle(-(enc_buf[1] * COUNTS_TO_RAD - PEND_OFFSET_RAD));

        double theta_dot = theta_filt.update(theta);
        double alpha_dot = alpha_filt.update(alpha);

        QubeState s = {theta, alpha, theta_dot, alpha_dot};

        // Compute voltage
        double voltage = 0.0;

        if (mode == SWING_UP) {
            voltage = swing_up.compute(s);
            if (swing_up.should_catch(s.alpha)) {
                mode = BALANCE;
                balance.reset();
                printf("  [%.2fs] Caught! alpha=%.1f deg\n", t, alpha * 180.0 / M_PI);
            }
        } else {
            voltage = balance.compute(s);
            if (fabs(s.alpha) > 30.0 * M_PI / 180.0) {
                mode = SWING_UP;
                balance.reset();
                printf("  [%.2fs] Lost balance! alpha=%.1f deg\n", t, alpha * 180.0 / M_PI);
            }
        }

        // Write voltage
        aout_buf[0] = voltage;
        hil_write_analog(card, aout_ch, 1, aout_buf);

        // Log
        char line[256];
        snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                 t, theta, alpha, theta_dot, alpha_dot, voltage, (int)mode);
        csv << line;

        // Sleep until next period
        timespec_add_ns(&t_next, dt_ns);
        sleep_until(&t_next);
    }

    // ── Safe shutdown ──
    printf("\nShutting down...\n");
    aout_buf[0] = 0.0;
    hil_write_analog(card, aout_ch, 1, aout_buf);

    t_boolean disable[] = {0};
    hil_write_digital(card, dout_ch, 1, disable);

    hil_close(card);
    csv.close();

    printf("Done. Logged to hardware_output.csv\n");
    return 0;
}
