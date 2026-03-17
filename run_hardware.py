#!/usr/bin/env python3
"""Run the pendulum balance controller on the real Qube-Servo 3.

Requires the Quanser SDK to be installed.

Usage:
    python3 run_hardware.py
    python3 run_hardware.py --duration 30
"""

import argparse
import time
import numpy as np

from hardware.qube_hw import HardwareQube
from control.balance import BalanceController
from control.swing_up import SwingUpController
from sim.plant import QubeParams
from utils.logger import DataLogger


def main():
    parser = argparse.ArgumentParser(description="Run pendulum balance on real hardware")
    parser.add_argument("--duration", type=float, default=30.0, help="Run duration [s]")
    parser.add_argument("--dt", type=float, default=0.002, help="Control period [s]")
    args = parser.parse_args()

    params = QubeParams()

    qube = HardwareQube(dt=args.dt)

    balance = BalanceController(
        k_theta=-3.16, k_alpha=49.0,
        k_theta_dot=-1.83, k_alpha_dot=4.32,
        k_integral=2.0, dt=args.dt,
    )
    swing_up = SwingUpController(
        params=params,
        mu=2.5,
        catch_angle=np.radians(20),
        arm_limit=np.radians(75),
        arm_soft_zone=np.radians(30),
        arm_brake_gain=8.0,
    )

    logger = DataLogger("hardware")

    state = qube.reset()
    mode = "swing_up"
    n_steps = int(args.duration / args.dt)

    print(f"Running on hardware: {args.duration}s, dt={args.dt}s")
    print("  Press Ctrl+C to stop safely.\n")

    try:
        for i in range(n_steps):
            t_start = time.perf_counter()
            t = i * args.dt

            if mode == "swing_up":
                voltage = swing_up.compute(
                    state.theta, state.alpha, state.theta_dot, state.alpha_dot
                )
                if swing_up.should_catch(state.alpha):
                    mode = "balance"
                    balance.reset()
                    print(f"  [{t:.2f}s] Caught! Balancing...")
            else:
                voltage = balance.compute(state)

                if abs(state.alpha) > np.radians(30):
                    mode = "swing_up"
                    balance.reset()
                    print(f"  [{t:.2f}s] Lost balance, swing-up...")

            state = qube.step(voltage)
            logger.log(t, state, voltage)

            # Maintain consistent loop timing
            elapsed = time.perf_counter() - t_start
            sleep_time = args.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        qube.close()
        logger.close()
        print("Hardware safely shut down.")


if __name__ == "__main__":
    main()
