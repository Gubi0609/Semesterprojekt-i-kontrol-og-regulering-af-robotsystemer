// Fast encoder read test — measures achievable poll rate.
// Build:  g++ -O2 -std=c++17 -o read_fast read_fast.cpp -lhil -lquanser_common -lm
// Run:    ./read_fast

#include <hil.h>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <ctime>

static constexpr int    COUNTS_PER_REV = 2048;
static constexpr double COUNTS_TO_DEG  = 360.0 / COUNTS_PER_REV;
// No count offset. Encoder zero = hanging down, 180° = upright.
// "pend(cal)" shows angle from upright: 0° = upright, ±180° = down.
static constexpr double PEND_OFFSET_DEG = 180.0;

static double now_sec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char** argv) {
    double duration = 5.0;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--duration") && i + 1 < argc) duration = atof(argv[++i]);

    t_card card;
    t_error result = hil_open("qube_servo3_usb", "0", &card);
    if (result < 0) {
        fprintf(stderr, "Failed to open (error %d)\n", result);
        return 1;
    }

    const t_uint32 enc_ch[] = {0, 1};
    t_int32 buf[2];
    t_int32 zero[] = {0, 0};
    hil_set_encoder_counts(card, enc_ch, 2, zero);

    double t_start  = now_sec();
    double t_print  = t_start;
    long   samples  = 0;

    printf("Reading for %.0fs as fast as possible...\n", duration);
    printf("%8s  %8s  %9s  %9s  %9s\n", "elapsed", "rate", "motor", "pendulum", "pend(cal)");
    printf("------------------------------------------------------\n");

    while (true) {
        hil_read_encoder(card, enc_ch, 2, buf);
        samples++;

        double now = now_sec();
        double elapsed = now - t_start;

        if (elapsed >= duration) break;

        if (now - t_print >= 0.5) {
            double rate      = samples / elapsed;
            double motor_deg = buf[0] * COUNTS_TO_DEG;
            double pend_deg  = buf[1] * COUNTS_TO_DEG;
            double pend_cal  = buf[1] * COUNTS_TO_DEG - PEND_OFFSET_DEG;
            printf("%7.2fs  %7.0fHz  %8.2f°  %8.2f°  %8.2f°\n",
                   elapsed, rate, motor_deg, pend_deg, pend_cal);
            t_print = now;
        }
    }

    double elapsed = now_sec() - t_start;
    double rate = samples / elapsed;
    hil_close(card);

    printf("------------------------------------------------------\n");
    printf("Total samples: %ld\n", samples);
    printf("Elapsed:       %.3fs\n", elapsed);
    printf("Average rate:  %.0f Hz\n", rate);
    printf("Period:        %.1f us/sample\n", 1e6 / rate);

    return 0;
}
