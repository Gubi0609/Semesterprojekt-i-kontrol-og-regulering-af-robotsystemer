// Balance-only mode for the Qube-Servo 3.
// No swing-up — manually hold the pendulum upright, then let go.
// The controller engages when alpha is within the catch zone.
//
// Build:  g++ -O2 -std=c++17 -I/usr/include/quanser -o run_balance run_balance.cpp -lhil -lquanser_common -lm
// Run:    ./run_balance
//         ./run_balance --duration 30 --catch 30

#include "qube_types.h"
#include "controllers.h"

#include <hil.h>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>

static constexpr double PEND_OFFSET_RAD = M_PI;

struct VelocityFilter {
    double rc_alpha, prev_x, filtered;
    double dt;
    bool init;

    VelocityFilter(double cutoff_hz, double dt_) : dt(dt_), init(false), prev_x(0), filtered(0) {
        double rc = 1.0 / (2.0 * M_PI * cutoff_hz);
        rc_alpha = rc / (rc + dt);
    }

    double update(double x) {
        if (!init) { prev_x = x; init = true; return 0.0; }
        double raw = (x - prev_x) / dt;
        prev_x = x;
        filtered = rc_alpha * filtered + (1.0 - rc_alpha) * raw;
        return filtered;
    }

    void reset() { prev_x = 0; filtered = 0; init = false; }
};

static void sleep_until(struct timespec* next) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, nullptr);
}
static void timespec_add_ns(struct timespec* ts, long ns) {
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) { ts->tv_sec++; ts->tv_nsec -= 1000000000L; }
}

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

int main(int argc, char** argv) {
    double duration   = 60.0;
    double dt         = 0.002;
    double catch_deg  = 30.0;   // engage when within this many degrees of upright
    double bail_deg   = 45.0;   // disengage if it falls past this

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--duration") && i+1 < argc) duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt") && i+1 < argc)  dt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--catch") && i+1 < argc) catch_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--bail") && i+1 < argc) bail_deg = atof(argv[++i]);
    }

    double catch_rad = catch_deg * M_PI / 180.0;
    double bail_rad  = bail_deg * M_PI / 180.0;

    signal(SIGINT, sigint_handler);

    t_card card;
    t_error result = hil_open("qube_servo3_usb", "0", &card);
    if (result < 0) { fprintf(stderr, "Failed to open (error %d)\n", result); return 1; }

    const t_uint32 enc_ch[]  = {0, 1};
    const t_uint32 aout_ch[] = {0};
    const t_uint32 dout_ch[] = {0};

    t_boolean enable[] = {1};
    hil_write_digital(card, dout_ch, 1, enable);

    t_int32 zero[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero);

    BalanceController balance(dt);
    VelocityFilter theta_filt(50.0, dt);
    VelocityFilter alpha_filt(50.0, dt);

    std::ofstream csv("balance_output.csv");
    csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,active\n";

    int n_steps = static_cast<int>(duration / dt);
    long dt_ns  = static_cast<long>(dt * 1e9);

    printf("=== Balance-Only Mode ===\n");
    printf("Hold pendulum upright and let go.\n");
    printf("Controller engages when |alpha| < %.0f deg.\n", catch_deg);
    printf("Ctrl+C to stop. Duration: %.0fs\n\n", duration);

    struct timespec t_next;
    clock_gettime(CLOCK_MONOTONIC, &t_next);

    t_int32  enc_buf[2];
    t_double aout_buf[1] = {0.0};
    bool active = false;
    bool printed_waiting = false;

    for (int i = 0; i < n_steps && g_running; i++) {
        double t = i * dt;

        hil_read_encoder(card, enc_ch, 2, enc_buf);

        double theta     = enc_buf[0] * COUNTS_TO_RAD;
        double alpha     = wrap_angle(-(enc_buf[1] * COUNTS_TO_RAD - PEND_OFFSET_RAD));
        double theta_dot = theta_filt.update(theta);
        double alpha_dot = alpha_filt.update(alpha);

        QubeState s = {theta, alpha, theta_dot, alpha_dot};
        double voltage = 0.0;

        if (!active) {
            // Waiting for manual placement
            if (fabs(alpha) < catch_rad) {
                active = true;
                balance.reset();
                theta_filt.reset();
                alpha_filt.reset();
                // Re-read to get clean derivatives
                theta_dot = 0.0;
                alpha_dot = 0.0;
                printf("  [%.2fs] ACTIVE! alpha=%.1f deg — balancing\n",
                       t, alpha * 180.0 / M_PI);
            } else {
                // Print waiting message once per second
                if (!printed_waiting || (i % 500 == 0)) {
                    printf("  [%.1fs] Waiting... alpha=%.1f deg (need < %.0f deg)\r",
                           t, alpha * 180.0 / M_PI, catch_deg);
                    fflush(stdout);
                    printed_waiting = true;
                }
            }
        } else {
            // Balancing
            voltage = balance.compute(s);

            if (fabs(alpha) > bail_rad) {
                active = false;
                voltage = 0.0;
                balance.reset();
                printf("\n  [%.2fs] LOST at alpha=%.1f deg — waiting for retry\n",
                       t, alpha * 180.0 / M_PI);
                printed_waiting = false;
            }
        }

        aout_buf[0] = voltage;
        hil_write_analog(card, aout_ch, 1, aout_buf);

        char line[256];
        snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                 t, theta, alpha, theta_dot, alpha_dot, voltage, active ? 1 : 0);
        csv << line;

        timespec_add_ns(&t_next, dt_ns);
        sleep_until(&t_next);
    }

    // Safe shutdown
    printf("\nShutting down...\n");
    aout_buf[0] = 0.0;
    hil_write_analog(card, aout_ch, 1, aout_buf);
    t_boolean disable[] = {0};
    hil_write_digital(card, dout_ch, 1, disable);
    hil_close(card);
    csv.close();
    printf("Done. Logged to balance_output.csv\n");
    return 0;
}
