// ═══════════════════════════════════════════════════════════════════
// run_friction_test.cpp — Identify friction coefficients b1 and b2
// ═══════════════════════════════════════════════════════════════════
//
// Two modes:
//
//   Mode 1: --pendulum  (default)
//     Motor is OFF. You manually deflect the pendulum and let it
//     swing freely. Logs alpha + alpha_dot vs time for decay analysis.
//     → gives b2 (pendulum viscous friction)
//
//   Mode 2: --motor V
//     Applies a constant voltage V to the motor (no pendulum needed).
//     Logs theta_dot vs time until steady-state.
//     → gives b1 (motor arm viscous friction)
//     Run at several voltages (e.g. 1, 2, 3 V) for better accuracy.
//
// Usage:
//   ./run_friction_test --pendulum --duration 15
//   ./run_friction_test --motor 2.0 --duration 10
//   ./run_friction_test --motor 3.0 --duration 10
//
// Build:
//   make run_friction_test
// ═══════════════════════════════════════════════════════════════════

#include "qube_types.h"
#include <hil.h>

#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>

// ─── Pendulum encoder offset ──────────────────────────────────────
// Encoder reads 0 when hanging down, ~π when upright.
// We subtract π so that 0 = upright, ±π = hanging.
static constexpr double PEND_OFFSET_RAD = M_PI;

// ─── Hardware tachometer channels ─────────────────────────────────
static constexpr t_uint32 TACHO_CHANNELS[] = {14000, 14001};

// ─── Timing helpers (same as other programs) ──────────────────────
static void sleep_until(struct timespec* next) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, nullptr);
}
static void timespec_add_ns(struct timespec* ts, long ns) {
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) { ts->tv_sec++; ts->tv_nsec -= 1000000000L; }
}

static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

// ═══════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {

    // ── Defaults ───────────────────────────────────────────────────
    enum Mode { PENDULUM, MOTOR } mode = PENDULUM;
    double motor_voltage = 0.0;
    double duration      = 15.0;
    double dt            = 0.001;  // 1 kHz

    // ── Parse args ─────────────────────────────────────────────────
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--pendulum"))
            mode = PENDULUM;
        else if (!strcmp(argv[i], "--motor") && i+1 < argc) {
            mode = MOTOR;
            motor_voltage = atof(argv[++i]);
        }
        else if (!strcmp(argv[i], "--duration") && i+1 < argc)
            duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--rate") && i+1 < argc)
            dt = 1.0 / atof(argv[++i]);
    }

    signal(SIGINT, sigint_handler);

    // ── Open hardware ──────────────────────────────────────────────
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

    // Enable amplifier
    t_boolean enable[] = {1};
    hil_write_digital(card, dout_ch, 1, enable);

    // Zero encoders
    t_int32 zero_counts[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero_counts);

    // ── Set up CSV logging ─────────────────────────────────────────
    char filename[128];
    if (mode == PENDULUM) {
        snprintf(filename, sizeof(filename), "friction_pendulum.csv");
    } else {
        snprintf(filename, sizeof(filename), "friction_motor_%.1fV.csv", motor_voltage);
    }
    std::ofstream csv(filename);

    if (mode == PENDULUM) {
        csv << "t,alpha,alpha_dot\n";
        printf("\n");
        printf("=== PENDULUM FRICTION TEST ===\n");
        printf("Motor is OFF. Deflect the pendulum and let it swing.\n");
        printf("Recording for %.0fs. Ctrl+C to stop early.\n\n", duration);
    } else {
        csv << "t,theta,theta_dot,voltage\n";
        printf("\n");
        printf("=== MOTOR FRICTION TEST ===\n");
        printf("Applying %.2f V to motor.\n", motor_voltage);
        printf("Recording for %.0fs. Ctrl+C to stop early.\n\n", duration);

        // Safety check
        if (fabs(motor_voltage) > 5.0) {
            printf("WARNING: voltage > 5V. Are you sure? (y/n) ");
            char c = getchar();
            if (c != 'y' && c != 'Y') {
                printf("Aborted.\n");
                t_double zero_v[] = {0.0};
                hil_write_analog(card, aout_ch, 1, zero_v);
                t_boolean disable[] = {0};
                hil_write_digital(card, dout_ch, 1, disable);
                hil_close(card);
                return 0;
            }
        }
    }

    // ── Main loop ──────────────────────────────────────────────────
    int  n_steps = static_cast<int>(duration / dt);
    long dt_ns   = static_cast<long>(dt * 1e9);

    struct timespec t_next;
    clock_gettime(CLOCK_MONOTONIC, &t_next);

    t_int32  enc_buf[2];
    t_double tacho_buf[2];
    t_double aout_buf[1];

    for (int i = 0; i < n_steps && g_running; i++) {
        double t = i * dt;

        // Read sensors
        hil_read_encoder(card, enc_ch, 2, enc_buf);
        hil_read_other(card, TACHO_CHANNELS, 2, tacho_buf);

        // Convert
        double theta     = enc_buf[0] * COUNTS_TO_RAD;
        double alpha     = wrap_angle(-(enc_buf[1] * COUNTS_TO_RAD - PEND_OFFSET_RAD));
        double theta_dot = tacho_buf[0] * COUNTS_TO_RAD;
        double alpha_dot = -tacho_buf[1] * COUNTS_TO_RAD;

        // Write voltage
        if (mode == MOTOR) {
            aout_buf[0] = clamp(motor_voltage, -V_MAX, V_MAX);
        } else {
            aout_buf[0] = 0.0;  // motor off for pendulum test
        }
        hil_write_analog(card, aout_ch, 1, aout_buf);

        // Log to CSV
        if (mode == PENDULUM) {
            csv << t << "," << alpha << "," << alpha_dot << "\n";
        } else {
            csv << t << "," << theta << "," << theta_dot << "," << motor_voltage << "\n";
        }

        // Print progress every second
        if (i % 1000 == 0) {
            if (mode == PENDULUM)
                printf("  t=%.1fs  alpha=%.1f deg  alpha_dot=%.1f rad/s\n",
                       t, alpha * 180.0 / M_PI, alpha_dot);
            else
                printf("  t=%.1fs  theta_dot=%.1f rad/s (%.0f RPM)\n",
                       t, theta_dot, theta_dot * 60.0 / (2.0 * M_PI));
        }

        // Wait for next tick
        timespec_add_ns(&t_next, dt_ns);
        sleep_until(&t_next);
    }

    // ── Shutdown: zero voltage, disable amp ────────────────────────
    printf("\nStopping motor...\n");
    aout_buf[0] = 0.0;
    hil_write_analog(card, aout_ch, 1, aout_buf);

    t_boolean disable[] = {0};
    hil_write_digital(card, dout_ch, 1, disable);

    hil_close(card);

    printf("Done. Data saved to %s\n", filename);
    printf("\nNext step: analyze with\n");
    if (mode == PENDULUM)
        printf("  python3 analyze_friction.py --pendulum friction_pendulum.csv\n");
    else
        printf("  python3 analyze_friction.py --motor friction_motor.csv\n");

    return 0;
}
