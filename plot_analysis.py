#!/usr/bin/env python3
"""Generate an annotated analysis plot from the most recent sim log."""

import csv
import numpy as np
from pathlib import Path
import glob


def load_latest_log():
    logs = sorted(glob.glob("logs/sim_*.csv"))
    if not logs:
        raise FileNotFoundError("No sim logs found. Run `python3 run_sim.py` first.")
    path = logs[-1]
    print(f"Loading: {path}")
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {
        "t": np.array([float(r["t"]) for r in rows]),
        "theta": np.array([float(r["theta"]) for r in rows]),
        "alpha": np.array([float(r["alpha"]) for r in rows]),
        "theta_dot": np.array([float(r["theta_dot"]) for r in rows]),
        "alpha_dot": np.array([float(r["alpha_dot"]) for r in rows]),
        "voltage": np.array([float(r["voltage"]) for r in rows]),
    }


def main():
    from plotly.subplots import make_subplots
    import plotly.graph_objects as go

    d = load_latest_log()
    t = d["t"]
    alpha_deg = np.degrees(d["alpha"])
    theta_deg = np.degrees(d["theta"])
    volts = d["voltage"]

    # Detect catch time: first moment |alpha| < 20 deg after being > 90 deg
    catch_idx = None
    was_large = False
    for i, a in enumerate(alpha_deg):
        if abs(a) > 90:
            was_large = True
        if was_large and abs(a) < 20:
            catch_idx = i
            break
    catch_t = t[catch_idx] if catch_idx else None

    fig = make_subplots(
        rows=3, cols=2,
        shared_xaxes="columns",
        column_widths=[0.5, 0.5],
        vertical_spacing=0.08,
        horizontal_spacing=0.08,
        subplot_titles=(
            "Full Run — α (pendulum)", "Balance Phase — α (pendulum)",
            "Full Run — θ (arm)", "Balance Phase — θ (arm)",
            "Full Run — Voltage", "Balance Phase — Voltage",
        ),
    )

    # Left column: full run
    fig.add_trace(go.Scattergl(x=t, y=alpha_deg, mode="lines",
                               line=dict(color="red", width=1), showlegend=False),
                  row=1, col=1)
    fig.add_trace(go.Scattergl(x=t, y=theta_deg, mode="lines",
                               line=dict(color="blue", width=1), showlegend=False),
                  row=2, col=1)
    fig.add_trace(go.Scattergl(x=t, y=volts, mode="lines",
                               line=dict(color="black", width=1), showlegend=False),
                  row=3, col=1)

    # Right column: balance phase only (after catch)
    if catch_idx is not None:
        bal_t = t[catch_idx:]
        bal_alpha = alpha_deg[catch_idx:]
        bal_theta = theta_deg[catch_idx:]
        bal_volts = volts[catch_idx:]

        fig.add_trace(go.Scattergl(x=bal_t, y=bal_alpha, mode="lines",
                                   line=dict(color="red", width=1), showlegend=False),
                      row=1, col=2)
        fig.add_trace(go.Scattergl(x=bal_t, y=bal_theta, mode="lines",
                                   line=dict(color="blue", width=1), showlegend=False),
                      row=2, col=2)
        fig.add_trace(go.Scattergl(x=bal_t, y=bal_volts, mode="lines",
                                   line=dict(color="black", width=1), showlegend=False),
                      row=3, col=2)

    # Add catch time annotation on left column
    if catch_t is not None:
        for row in range(1, 4):
            fig.add_vline(x=catch_t, line_dash="dash", line_color="green",
                          opacity=0.7, row=row, col=1,
                          annotation_text="catch" if row == 1 else None)

    # Zero reference lines
    for col in (1, 2):
        fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=1, col=col)
        fig.add_hline(y=0, line_dash="dot", line_color="gray", opacity=0.4, row=2, col=col)

    # Voltage limits
    for col in (1, 2):
        fig.add_hline(y=10, line_dash="dot", line_color="red", opacity=0.2, row=3, col=col)
        fig.add_hline(y=-10, line_dash="dot", line_color="red", opacity=0.2, row=3, col=col)

    fig.update_yaxes(title_text="α [deg]", row=1, col=1)
    fig.update_yaxes(title_text="θ [deg]", row=2, col=1)
    fig.update_yaxes(title_text="V [V]", row=3, col=1)
    fig.update_xaxes(title_text="Time [s]", row=3, col=1)
    fig.update_xaxes(title_text="Time [s]", row=3, col=2)

    fig.update_layout(
        title=(
            "Qube-Servo 3 Simulation Analysis<br>"
            "<sup>Left: full run (swing-up → balance) | "
            "Right: balance phase only (zoomed)</sup>"
        ),
        height=900,
        template="plotly_white",
    )

    out = Path("plots")
    out.mkdir(exist_ok=True)
    path = out / "analysis.html"
    fig.write_html(str(path), auto_open=True)
    print(f"Analysis plot: {path}")


if __name__ == "__main__":
    main()
