#!/usr/bin/env python3
"""Run the pendulum balance controller in simulation.

Usage:
    python3 run_sim.py                      # default: swing-up → balance
    python3 run_sim.py --near-upright       # start near upright (skip swing-up)
    python3 run_sim.py --duration 15        # run for 15 seconds
    python3 run_sim.py --no-plot            # headless (log only)
"""

import argparse
import numpy as np

from sim.simulated_qube import SimulatedQube
from sim.plant import QubeParams
from control.balance import BalanceController
from control.swing_up import SwingUpController
from sim.visualizer import PendulumVisualizer
from utils.logger import DataLogger


def main():
    parser = argparse.ArgumentParser(description="Simulate Qube-Servo 3 pendulum balance")
    parser.add_argument("--duration", type=float, default=10.0, help="Sim duration [s]")
    parser.add_argument("--dt", type=float, default=0.002, help="Timestep [s]")
    parser.add_argument("--near-upright", action="store_true",
                        help="Start pendulum near upright (skip swing-up)")
    parser.add_argument("--no-plot", action="store_true", help="Disable plotting")
    parser.add_argument("--noise", type=float, default=0.0,
                        help="Encoder noise stddev [rad]")
    args = parser.parse_args()

    # --- Setup ---
    params = QubeParams()
    initial_alpha = 0.1 if args.near_upright else np.pi

    qube = SimulatedQube(
        dt=args.dt,
        params=params,
        initial_alpha=initial_alpha,
        substeps=10,
        sensor_noise=args.noise,
    )

    # Full-state-feedback balance controller (gains from LQR).
    # u = -K @ [theta, alpha, theta_dot, alpha_dot] + integral term
    balance = BalanceController(
        k_theta=-3.16,
        k_alpha=49.0,
        k_theta_dot=-1.83,
        k_alpha_dot=4.32,
        k_integral=2.0,
        dt=args.dt,
    )

    swing_up = SwingUpController(
        params=params,
        mu=2.5,                          # gentler than default
        catch_angle=np.radians(20),
        arm_limit=np.radians(75),        # conservative: real endstop ~120-150°
        arm_soft_zone=np.radians(30),
        arm_brake_gain=8.0,              # strong braking near limit
    )

    logger = DataLogger("sim")
    viz = None if args.no_plot else PendulumVisualizer()

    # --- Run ---
    state = qube.reset()
    n_steps = int(args.duration / args.dt)
    mode = "swing_up" if not args.near_upright else "balance"

    print(f"Running simulation: {args.duration}s, dt={args.dt}s, mode={mode}")
    print(f"  Balance gains: k_theta={balance.k_theta}, k_alpha={balance.k_alpha}, "
          f"k_theta_dot={balance.k_theta_dot}, k_alpha_dot={balance.k_alpha_dot}")

    for i in range(n_steps):
        t = i * args.dt

        if mode == "swing_up":
            voltage = swing_up.compute(
                state.theta, state.alpha, state.theta_dot, state.alpha_dot
            )
            if swing_up.should_catch(state.alpha):
                mode = "balance"
                balance.reset()
                print(f"  [{t:.2f}s] Caught! Switching to balance mode "
                      f"(alpha={np.degrees(state.alpha):.1f}°)")
        else:
            voltage = balance.compute(state)

            # Safety: if pendulum falls too far, switch back to swing-up
            if abs(state.alpha) > np.radians(30):
                mode = "swing_up"
                balance.reset()
                print(f"  [{t:.2f}s] Lost balance! Back to swing-up "
                      f"(alpha={np.degrees(state.alpha):.1f}°)")

        state = qube.step(voltage)
        logger.log(t, state, voltage)
        if viz:
            viz.update(state, voltage, t)

    # --- Cleanup ---
    qube.close()
    logger.close()

    if viz:
        viz.show()


if __name__ == "__main__":
    main()
