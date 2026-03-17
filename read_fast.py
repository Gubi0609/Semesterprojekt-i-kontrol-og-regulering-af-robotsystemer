#!/usr/bin/env python3
"""Fast encoder read test — measures actual achievable poll rate.

Reads as fast as possible for a few seconds, prints a summary
line every 500ms, and reports the achieved sample rate.

Usage:
    python3 read_fast.py
    python3 read_fast.py --duration 10
"""

import argparse
import time
import numpy as np
from quanser.hardware import HIL

CPR = 2048
COUNTS_TO_DEG = 360.0 / CPR
COUNTS_TO_RAD = 2.0 * np.pi / CPR

PEND_OFFSET = -1026  # from calibration (upright)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    card = HIL("qube_servo3_usb", "0")
    enc_ch = np.array([0, 1], dtype=np.uint32)
    buf = np.zeros(2, dtype=np.int32)

    card.set_encoder_counts(enc_ch, 2, np.array([0, 0], dtype=np.int32))

    total_samples = 0
    t_start = time.perf_counter()
    last_print = t_start
    print_interval = 0.5  # print every 500ms

    print(f"Reading for {args.duration}s as fast as possible...")
    print(f"{'elapsed':>8s}  {'rate':>8s}  {'motor':>9s}  {'pendulum':>9s}  {'pend(cal)':>9s}")
    print("-" * 55)

    try:
        while True:
            card.read_encoder(enc_ch, 2, buf)
            total_samples += 1

            now = time.perf_counter()
            elapsed = now - t_start

            if elapsed >= args.duration:
                break

            if now - last_print >= print_interval:
                rate = total_samples / elapsed
                motor_deg = buf[0] * COUNTS_TO_DEG
                pend_raw_deg = buf[1] * COUNTS_TO_DEG
                pend_cal_deg = (buf[1] - PEND_OFFSET) * COUNTS_TO_DEG
                print(f"{elapsed:7.2f}s  {rate:7.0f}Hz  {motor_deg:8.2f}°  {pend_raw_deg:8.2f}°  {pend_cal_deg:8.2f}°")
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.perf_counter() - t_start
        rate = total_samples / elapsed
        card.close()

        print("-" * 55)
        print(f"Total samples: {total_samples}")
        print(f"Elapsed:       {elapsed:.3f}s")
        print(f"Average rate:  {rate:.0f} Hz")
        print(f"Period:        {1e6/rate:.1f} µs/sample")


if __name__ == "__main__":
    main()
