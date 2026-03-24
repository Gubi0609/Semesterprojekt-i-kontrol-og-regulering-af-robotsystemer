// ═══════════════════════════════════════════════════════════════════
// run_hardware.cpp — Full swing-up + balance for the Qube-Servo 3
// ═══════════════════════════════════════════════════════════════════
//
// This program connects to the Qube-Servo 3 over USB, reads the
// two encoders (arm + pendulum position) and two hardware tachometers
// (arm + pendulum velocity), computes a control voltage using either
// the swing-up or balance controller, and writes it to the motor.
//
// The loop runs at 1 kHz by default (configurable via --rate).
//
// Usage:
//   ./run_hardware                    # 30s at 1 kHz
//   ./run_hardware --duration 60      # 60s
//   ./run_hardware --rate 2000        # 2 kHz loop
//
// Build:
//   g++ -O2 -std=c++17 -I/usr/include/quanser -o run_hardware \
//       run_hardware.cpp -lhil -lquanser_common -lm
// ═══════════════════════════════════════════════════════════════════

#include "qube_types.h"    // QubeState, QubeParams, COUNTS_TO_RAD, wrap_angle, clamp
#include "controllers.h"   // BalanceController, SwingUpController, compensate_deadband

#include <hil.h>           // Quanser HIL C API: hil_open, hil_read_encoder, etc.

#include <cstdio>
#include <cstring>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>

// ─── Encoder calibration ───────────────────────────────────────────
// The pendulum encoder reads 0 when hanging straight down, and
// ~π rad (~180°) when upright. Our controller convention is:
//   alpha = 0   → upright (balanced)
//   alpha = ±π  → hanging down
// So we subtract π from the raw reading and negate it (the encoder
// counts positive in the opposite direction from our convention).
static constexpr double PEND_OFFSET_RAD = M_PI;

// ─── Hardware tachometer channels ──────────────────────────────────
// The Qube-Servo 3 has two 32-bit digital tachometers based on a
// 72 MHz counter. They report angular velocity in counts/second.
// Channel 14000 = motor shaft, channel 14001 = pendulum.
// We multiply by COUNTS_TO_RAD to convert counts/s → rad/s.
static constexpr t_uint32 TACHO_CHANNELS[] = {14000, 14001};

// ─── Timing helpers ────────────────────────────────────────────────
// sleep_until: blocks the thread until the absolute time in *next.
// Uses CLOCK_MONOTONIC so the loop doesn't drift with wall-clock
// adjustments (NTP, etc). TIMER_ABSTIME means we specify the exact
// wake-up time rather than a relative delay — this prevents jitter
// from accumulating across iterations.
static void sleep_until(struct timespec* next) {
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, nullptr);
}

// Add nanoseconds to a timespec, handling the seconds rollover.
static void timespec_add_ns(struct timespec* ts, long ns) {
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

// ─── Safe shutdown on Ctrl+C ───────────────────────────────────────
// When the user presses Ctrl+C, the signal handler sets this flag.
// The main loop checks it every iteration and exits cleanly,
// ensuring the motor voltage is zeroed and the amplifier is disabled.
static volatile bool g_running = true;
static void sigint_handler(int) { g_running = false; }

// ═══════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {

    // ── Default parameters ──
    double duration = 30.0;   // total run time [seconds]
    double dt       = 0.001;  // control period [seconds] → 1 kHz

    // ── Parse command-line arguments ──
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--duration") && i+1 < argc)
            duration = atof(argv[++i]);
        else if (!strcmp(argv[i], "--dt") && i+1 < argc)
            dt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--rate") && i+1 < argc)
            dt = 1.0 / atof(argv[++i]);  // e.g. --rate 2000 → dt = 0.0005
    }

    // Register Ctrl+C handler before touching any hardware
    signal(SIGINT, sigint_handler);

    // ── Open the Qube-Servo 3 ──────────────────────────────────────
    // hil_open talks to the USB device via the Quanser driver.
    // "qube_servo3_usb" = board type, "0" = first unit found.
    // Returns a t_card handle used for all subsequent I/O calls.
    t_card card;
    t_error result = hil_open("qube_servo3_usb", "0", &card);
    if (result < 0) {
        fprintf(stderr, "Failed to open Qube-Servo 3 (error %d)\n", result);
        return 1;
    }
    printf("Connected to Qube-Servo 3.\n");

    // ── Define I/O channel arrays ──────────────────────────────────
    // The HIL API takes arrays of channel numbers + a count.
    const t_uint32 enc_ch[]  = {0, 1};  // encoder 0 = motor, 1 = pendulum
    const t_uint32 aout_ch[] = {0};     // analog output 0 = motor voltage
    const t_uint32 dout_ch[] = {0};     // digital output 0 = amplifier enable

    // ── Enable the motor amplifier ─────────────────────────────────
    // Digital output 0 acts as an enable for the motor drive circuitry.
    // High = amplifier on, Low = amplifier off (motor coasts).
    t_boolean enable[] = {1};
    hil_write_digital(card, dout_ch, 1, enable);

    // ── Zero both encoder counters ─────────────────────────────────
    // Incremental encoders count relative to wherever they were at
    // power-on. We zero them here so θ starts at 0.
    // (The pendulum offset is handled in software via PEND_OFFSET_RAD.)
    t_int32 zero_counts[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero_counts);

    // ── Create controllers ─────────────────────────────────────────
    QubeParams params;
    BalanceController balance(dt);       // needs dt for integral term
    SwingUpController swing_up(params, dt);  // needs physical params + dt for energy calc

    // Start in swing-up mode — the pendulum begins hanging down.
    enum Mode { SWING_UP, BALANCE } mode = SWING_UP;

    // ── Open CSV log file ──────────────────────────────────────────
    std::ofstream csv("hardware_output.csv");
    csv << "t,theta,alpha,theta_dot,alpha_dot,voltage,mode\n";

    // Pre-compute loop constants
    int  n_steps = static_cast<int>(duration / dt);  // total iterations
    long dt_ns   = static_cast<long>(dt * 1e9);      // period in nanoseconds

    printf("Running: %.1fs at %.0f Hz. Ctrl+C to stop.\n", duration, 1.0 / dt);

    // ── Initialize the loop timer ──────────────────────────────────
    // We record the current time and schedule each future iteration
    // as an absolute offset from this start time. This means:
    //   iteration 0 runs at t_start
    //   iteration 1 runs at t_start + dt
    //   iteration 2 runs at t_start + 2*dt
    //   ...
    // Even if one iteration takes longer than dt (jitter), the next
    // one still targets the correct absolute time — no drift.
    struct timespec t_next;
    clock_gettime(CLOCK_MONOTONIC, &t_next);

    // ── I/O buffers ────────────────────────────────────────────────
    t_int32  enc_buf[2];      // raw encoder counts [motor, pendulum]
    t_double tacho_buf[2];    // tachometer readings [motor, pendulum] in counts/s
    t_double aout_buf[1];     // voltage to write to motor

    // ════════════════════════════════════════════════════════════════
    //                      MAIN CONTROL LOOP
    // ════════════════════════════════════════════════════════════════
    for (int i = 0; i < n_steps && g_running; i++) {
        double t = i * dt;  // current time [seconds]

        // ── 1. READ SENSORS ────────────────────────────────────────
        // Read both encoder positions (24-bit counters, 4X quadrature)
        hil_read_encoder(card, enc_ch, 2, enc_buf);
        // Read both hardware tachometers (72 MHz counter, counts/s)
        hil_read_other(card, TACHO_CHANNELS, 2, tacho_buf);

        // ── 2. CONVERT TO PHYSICAL UNITS ───────────────────────────
        // Motor arm angle: raw counts → radians (no offset needed)
        double theta = enc_buf[0] * COUNTS_TO_RAD;

        // Pendulum angle: raw counts → radians, then:
        //   - subtract π so that upright (encoder=π) maps to α=0
        //   - negate because encoder counts positive the wrong way
        //   - wrap to [-π, +π] so it can't accumulate past ±180°
        //     (this happens if the pendulum detaches and reattaches)
        double alpha = wrap_angle(-(enc_buf[1] * COUNTS_TO_RAD - PEND_OFFSET_RAD));

        // Velocities: tachometers give counts/s, multiply by COUNTS_TO_RAD
        // for rad/s. Negate alpha_dot to match the alpha sign convention.
        double theta_dot = tacho_buf[0] * COUNTS_TO_RAD;
        double alpha_dot = -tacho_buf[1] * COUNTS_TO_RAD;

        // Pack into a single struct for the controllers
        QubeState s = {theta, alpha, theta_dot, alpha_dot};

        // ── 3. COMPUTE CONTROL VOLTAGE ─────────────────────────────
        double voltage = 0.0;

        if (mode == SWING_UP) {
            // Energy-based swing-up: pump energy into pendulum
            voltage = swing_up.compute(s);

            // Check if pendulum is close enough to upright to catch
            if (swing_up.should_catch(s.alpha)) {
                mode = BALANCE;
                balance.reset();  // zero the integral term
                printf("  [%.2fs] Caught! alpha=%.1f deg\n",
                       t, alpha * 180.0 / M_PI);
            }
        } else {
            // Full state feedback: keep pendulum balanced at α=0
            voltage = balance.compute(s);

            // If pendulum falls too far, give up and go back to swing-up
            if (fabs(s.alpha) > 30.0 * M_PI / 180.0) {
                mode = SWING_UP;
                balance.reset();
                printf("  [%.2fs] Lost balance! alpha=%.1f deg\n",
                       t, alpha * 180.0 / M_PI);
            }
        }

        // ── 4. WRITE VOLTAGE TO MOTOR ──────────────────────────────
        aout_buf[0] = voltage;
        hil_write_analog(card, aout_ch, 1, aout_buf);

        // ── 5. LOG DATA ────────────────────────────────────────────
        char line[256];
        snprintf(line, sizeof(line), "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n",
                 t, theta, alpha, theta_dot, alpha_dot, voltage, (int)mode);
        csv << line;

        // ── 6. SLEEP UNTIL NEXT TICK ───────────────────────────────
        // Advance the target wake-up time by one period and sleep.
        timespec_add_ns(&t_next, dt_ns);
        sleep_until(&t_next);
    }

    // ════════════════════════════════════════════════════════════════
    //                      SAFE SHUTDOWN
    // ════════════════════════════════════════════════════════════════
    // Always reached — whether the loop ended normally, the user
    // pressed Ctrl+C, or an error occurred. The motor must be stopped.
    printf("\nShutting down...\n");

    // Zero the motor voltage
    aout_buf[0] = 0.0;
    hil_write_analog(card, aout_ch, 1, aout_buf);

    // Disable the amplifier (motor will coast to a stop)
    t_boolean disable[] = {0};
    hil_write_digital(card, dout_ch, 1, disable);

    // Release the USB connection
    hil_close(card);
    csv.close();

    printf("Done. Logged to hardware_output.csv\n");
    return 0;
}
