#!/usr/bin/env python3
"""Encoder calibration for the Qube-Servo 3 pendulum.

Run this ONCE when you have hardware access. It determines:
  1. Actual encoder counts per revolution (expected: 2048 in 4X quadrature)
  2. Sign convention (which direction is positive alpha)
  3. Offset from encoder zero to true upright

No motor commands are sent — completely safe.

IMPORTANT: The Qube-Servo 3 pendulum encoder is 512 lines × 4X quadrature
= 2048 counts/revolution. The pendulum attaches magnetically and can
rotate freely through 360°.

Usage:
    python3 calibrate.py
"""

from quanser.hardware import HIL
import numpy as np
import time
import json
from pathlib import Path


# Qube-Servo 3 spec: 512 lines × 4X quadrature = 2048 counts/rev
COUNTS_PER_REV = 2048


def read_encoders(card, enc_channels):
    buf = np.zeros(2, dtype=np.int32)
    card.read_encoder(enc_channels, 2, buf)
    return buf[0], buf[1]


def main():
    card = HIL("qube_servo3_usb", "0")
    enc_channels = np.array([0, 1], dtype=np.uint32)

    print("=" * 60)
    print("  Qube-Servo 3 Encoder Calibration")
    print("  No motor commands will be sent.")
    print("=" * 60)

    # --- Step 0: Live stream to verify encoders are working ---
    print("\n[Step 0] Verifying encoders are alive.")
    print("  Gently wiggle the pendulum and arm...")
    print("  (Watching for 5 seconds)\n")

    card.set_encoder_counts(enc_channels, 2, np.array([0, 0], dtype=np.int32))
    time.sleep(0.2)

    enc0_moved = False
    enc1_moved = False
    for i in range(50):
        m, p = read_encoders(card, enc_channels)
        if abs(m) > 5:
            enc0_moved = True
        if abs(p) > 5:
            enc1_moved = True
        if i % 10 == 0:
            print(f"    enc0(motor)={m:6d}  enc1(pendulum)={p:6d}")
        time.sleep(0.1)

    print(f"\n  enc0 moved: {enc0_moved}")
    print(f"  enc1 moved: {enc1_moved}")

    if not enc0_moved and not enc1_moved:
        print("  ⚠ Neither encoder moved! Make sure the Qube is powered ON")
        print("    and you're physically moving the arm/pendulum.")

    # --- Step 1: Determine CPR by rotating pendulum one full revolution ---
    print("\n" + "=" * 60)
    print("[Step 1] Measuring counts per revolution.")
    print("  We need to find the EXACT encoder count for 360°.\n")
    print("  Instructions:")
    print("  1. Hold the arm still")
    print("  2. Note the pendulum's current position (mark it if possible)")
    print("  3. Slowly rotate the pendulum EXACTLY ONE FULL TURN (360°)")
    print("     in one direction, back to the same starting position")
    input("\n  Press Enter when pendulum is at the starting mark...")

    card.set_encoder_counts(enc_channels, 2, np.array([0, 0], dtype=np.int32))
    time.sleep(0.2)
    print("  Zeroed. Now rotate pendulum exactly 360° and return to start.")
    input("  Press Enter when back at the starting position...")

    _, p_full = read_encoders(card, enc_channels)
    print(f"\n  Counts for one full revolution: {p_full}")
    print(f"  Expected: ±2048 (512 lines × 4X quadrature)")

    if abs(p_full) < 100:
        # Returned very close to zero — good, consistent with 360°
        print(f"  ✓ Close to zero — you completed a full revolution.")
        measured_cpr = COUNTS_PER_REV  # trust the spec
    elif abs(abs(p_full) - 2048) < 100:
        measured_cpr = abs(p_full)
        print(f"  ✓ Consistent with {measured_cpr} CPR")
    else:
        print(f"  ⚠ Unexpected value. Using spec value of {COUNTS_PER_REV}.")
        measured_cpr = COUNTS_PER_REV

    # --- Step 2: Zero with pendulum hanging down ---
    print("\n" + "=" * 60)
    print("[Step 2] Setting zero reference.")
    print("  Let the pendulum hang straight DOWN, completely still.")
    input("  Press Enter when hanging down and stationary...")

    card.set_encoder_counts(enc_channels, 2, np.array([0, 0], dtype=np.int32))
    time.sleep(0.3)
    _, p_down = read_encoders(card, enc_channels)
    print(f"  Zeroed at hanging-down position. Reading: {p_down}")

    # --- Step 3: Measure upright ---
    print("\n" + "=" * 60)
    print("[Step 3] Measuring the upright position.")
    print("  Slowly lift the pendulum ALL THE WAY to upright (180° from down).")
    print("  The pendulum should be pointing STRAIGHT UP.")
    print("  Hold it steady — don't let it go past vertical.")
    input("  Press Enter when holding steady at upright...")

    time.sleep(0.3)
    readings = []
    for _ in range(10):
        _, p = read_encoders(card, enc_channels)
        readings.append(p)
        time.sleep(0.05)

    p_up = int(np.median(readings))
    p_up_deg = p_up / measured_cpr * 360.0
    print(f"  Upright reading: {p_up} counts ({p_up_deg:.1f}°)")
    print(f"  Expected: ~±{measured_cpr // 2} counts (±180°)")
    print(f"  Jitter: ±{max(readings) - min(readings)} counts")

    expected_half = measured_cpr / 2
    error_pct = abs(abs(p_up) - expected_half) / expected_half * 100
    if error_pct > 20:
        print(f"\n  ⚠ WARNING: Expected ~±{expected_half:.0f} counts but got {p_up}.")
        print(f"    Error: {error_pct:.0f}%")
        print(f"    Possible causes:")
        print(f"    - Pendulum wasn't fully upright (180° from hanging)")
        print(f"    - Pendulum hit a mechanical stop before reaching upright")
        print(f"    - Wrong CPR assumption")
        print(f"\n    Using measured value anyway.")

    # --- Step 4: Verify return to down ---
    print("\n" + "=" * 60)
    print("[Step 4] Verification — let the pendulum hang down again.")
    input("  Let go and press Enter once settled...")

    time.sleep(0.5)
    _, p_back = read_encoders(card, enc_channels)
    p_back_deg = p_back / measured_cpr * 360.0
    print(f"  Back to down: {p_back} counts ({p_back_deg:.1f}°)")
    if abs(p_back) < 20:
        print(f"  ✓ Consistent — returned near zero.")
    else:
        print(f"  ⚠ Drift of {p_back} counts from zero. May indicate")
        print(f"    the pendulum didn't return to the same position.")

    # --- Step 5: Determine sign convention ---
    print("\n" + "=" * 60)
    print("  RESULTS")
    print("=" * 60)

    direction = "positive" if p_up > 0 else "negative"
    # We want: alpha = 0 at upright, alpha = +pi at hanging down
    # Currently: 0 = hanging down, p_up = upright
    # Transform: alpha_controller = -(encoder - p_up) * sign
    # At upright: alpha = -(p_up - p_up) * sign = 0 ✓
    # At down:    alpha = -(0 - p_up) * sign = p_up * sign
    #   We want this to be +pi, so sign = +1 if p_up > 0, else -1
    # Actually simpler: just offset so upright=0

    counts_to_rad = 2.0 * np.pi / measured_cpr

    # Sign flip: we want positive alpha = away from upright toward hanging
    if p_up > 0:
        sign_flip = -1  # encoder increases toward upright, we want upright=0 and down=+pi
    else:
        sign_flip = 1

    print(f"\n  Measured CPR: {measured_cpr} counts/rev")
    print(f"  Down→Up: {p_up} counts ({p_up_deg:.1f}°)")
    print(f"  Lifting direction: {direction}")
    print(f"  Sign flip: {sign_flip}")
    print(f"  Offset (upright): {p_up} counts")
    print(f"\n  Conversion formula:")
    print(f"    alpha_rad = (enc_counts - {p_up}) × {sign_flip} × {counts_to_rad:.6f}")
    print(f"    alpha = 0 → upright")
    print(f"    alpha = ±π → hanging down")

    # Save calibration
    cal = {
        "counts_per_rev": measured_cpr,
        "counts_to_rad": float(counts_to_rad),
        "pendulum_offset_counts": int(p_up),
        "pendulum_offset_deg": float(p_up_deg),
        "pendulum_sign_flip": sign_flip,
        "notes": "alpha=0 is upright, alpha=+pi is hanging down. "
                 "Formula: alpha = (enc - offset) * sign_flip * counts_to_rad",
    }

    cal_path = Path("calibration.json")
    with open(cal_path, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"\n  Saved to {cal_path}")

    card.close()
    print("  Done! Connection closed safely.")


if __name__ == "__main__":
    main()
