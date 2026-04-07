#!/usr/bin/env python3
"""
Plot hardware/balance CSV logs from the Qube-Servo 3.

Usage:
    python3 plot_run.py hardware_output.csv
    python3 plot_run.py balance_output.csv
    python3 plot_run.py hardware_output.csv --save   # saves PNG instead of showing
"""

import sys
import csv
import math
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

def load_csv(filename):
    with open(filename) as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    data = {}
    for key in rows[0].keys():
        try:
            data[key] = np.array([float(r[key]) for r in rows])
        except ValueError:
            data[key] = np.array([r[key] for r in rows])
    return data

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_run.py <csv_file> [--save]")
        sys.exit(1)

    filename = sys.argv[1]
    save = '--save' in sys.argv

    data = load_csv(filename)
    t = data['t']
    theta = data['theta'] * 180 / math.pi       # rad -> deg
    alpha = data['alpha'] * 180 / math.pi        # rad -> deg
    theta_dot = data['theta_dot']
    alpha_dot = data['alpha_dot']
    voltage = data['voltage']

    # Mode column: 'mode' in hardware, 'active' in balance
    if 'mode' in data:
        mode = data['mode'].astype(int)
        mode_label = 'mode'  # 0=swing-up, 1=balance
    elif 'active' in data:
        mode = data['active'].astype(int)
        mode_label = 'active'  # 0=waiting, 1=balancing
    else:
        mode = np.ones_like(t)
        mode_label = None

    # --- Create figure ---
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    fig.suptitle(filename, fontsize=13, fontweight='bold')

    # Shade background by mode
    def shade_modes(ax):
        if mode_label is None:
            return
        # Find transitions
        changes = np.where(np.diff(mode))[0]
        boundaries = np.concatenate([[0], changes + 1, [len(t)]])
        for i in range(len(boundaries) - 1):
            start = boundaries[i]
            end = boundaries[i + 1] - 1
            m = mode[start]
            if mode_label == 'mode':
                color = '#FFE0B2' if m == 0 else '#C8E6C9'  # orange=swingup, green=balance
            else:
                color = '#E0E0E0' if m == 0 else '#C8E6C9'  # grey=waiting, green=active
            ax.axvspan(t[start], t[end], alpha=0.3, color=color, linewidth=0)

    # Plot 1: Pendulum angle
    shade_modes(axes[0])
    axes[0].plot(t, alpha, linewidth=0.5, color='#1565C0')
    axes[0].set_ylabel('α (deg)')
    axes[0].set_title('Pendulum angle (0° = upright)')
    axes[0].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[0].grid(True, alpha=0.3)

    # Plot 2: Arm angle
    shade_modes(axes[1])
    axes[1].plot(t, theta, linewidth=0.5, color='#E65100')
    axes[1].set_ylabel('θ (deg)')
    axes[1].set_title('Arm angle')
    axes[1].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[1].grid(True, alpha=0.3)

    # Plot 3: Motor voltage
    shade_modes(axes[2])
    axes[2].plot(t, voltage, linewidth=0.5, color='#2E7D32')
    axes[2].set_ylabel('Voltage (V)')
    axes[2].set_title('Motor voltage')
    axes[2].axhline(y=0, color='gray', linestyle='--', linewidth=0.5)
    axes[2].grid(True, alpha=0.3)

    # Plot 4: Angular velocities
    shade_modes(axes[3])
    axes[3].plot(t, theta_dot, linewidth=0.5, color='#E65100', label='θ̇', alpha=0.7)
    axes[3].plot(t, alpha_dot, linewidth=0.5, color='#1565C0', label='α̇', alpha=0.7)
    axes[3].set_ylabel('Velocity (rad/s)')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('Angular velocities')
    axes[3].legend(loc='upper right')
    axes[3].grid(True, alpha=0.3)

    # Add legend for mode shading
    if mode_label == 'mode':
        patches = [
            mpatches.Patch(color='#FFE0B2', alpha=0.5, label='Swing-up'),
            mpatches.Patch(color='#C8E6C9', alpha=0.5, label='Balance'),
        ]
        axes[0].legend(handles=patches, loc='upper right')
    elif mode_label == 'active':
        patches = [
            mpatches.Patch(color='#E0E0E0', alpha=0.5, label='Waiting'),
            mpatches.Patch(color='#C8E6C9', alpha=0.5, label='Active'),
        ]
        axes[0].legend(handles=patches, loc='upper right')

    plt.tight_layout()

    if save:
        outfile = filename.rsplit('.', 1)[0] + '.png'
        plt.savefig(outfile, dpi=150)
        print(f"Saved to {outfile}")
    else:
        plt.show()

if __name__ == '__main__':
    main()
