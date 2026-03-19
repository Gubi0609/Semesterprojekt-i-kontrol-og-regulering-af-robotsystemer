// Full swing-up + balance for the Qube-Servo 3.
// Uses hardware tachometers for velocity feedback.
//
// Build:  g++ -O2 -std=c++17 -I/usr/include/quanser -o run_hardware run_hardware.cpp -lhil -lquanser_common -lm
// Run:    ./run_hardware
//         ./run_hardware --duration 30 --rate 1000

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
static constexpr t_uint32 TACHO_CHANNELS[] = {14000, 14001};

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
    double duration = 30.0;
    double dt       = 0.001;  // 1 kHz

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--duration") && i+1 < argc) duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt") && i+1 < argc)  dt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--rate") && i+1 < argc) dt = 1.0 / atof(argv[++i]);
    }

    signal(SIGINT, sigint_handler);

    t_card card;
    t_error result = hil_open("qube_servo3_usb", "0", &card);
    if (result < 0) {
        fprintf(stderr, "Failed to open Qube-Servo 3 (error %d)\n", result);
        return 1;
    }
    printf("Connected to Qube-Servo 3.\n");

    const t_uint32 enc_ch[]  = {0, 1};
    const t_uint32 aout_ch[] = {0};
    const t_uint32 dout_ch[] = {0};

    t_boolean enable[] = {1};
    hil_write_digital(card, dout_ch, 1, enable);

    t_int32 zero_counts[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero_counts);

    QubeParams params;
    BalanceController balance(dt);
    SwingUpController swing_up(params);

    enum Mode { SWING_UP, BALANCE } mode = SWING_UP;

    std::ofstream csv("hardware_output.csv");
    csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,mode\n";

    int n_steps = static_cast<int>(duration / dt);
    long dt_ns  = static_cast<long>(dt * 1e9);

    printf("Running: %.1fs at %.0f Hz. Ctrl+C to stop.\n", duration, 1.0 / dt);

    struct timespec t_next;
    clock_gettime(CLOCK_MONOTONIC, &t_next);

    t_int32  enc_buf[2];
    t_double tacho_buf[2];
    t_double aout_buf[1];

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

        aout_buf[0] = voltage;
        hil_write_analog(card, aout_ch, 1, aout_buf);

        char line[256];
        snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                 t, theta, alpha, theta_dot, alpha_dot, voltage, (int)mode);
        csv << line;

        timespec_add_ns(&t_next, dt_ns);
        sleep_until(&t_next);
    }

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
