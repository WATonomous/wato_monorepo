#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Stage 2: Visualize extracted bag data for feedforward curve fitting.

Generates diagnostic plots from CSV data produced by extract_bag_data.py.
Review these plots to choose the appropriate feedforward model structure.

Usage:
    python plot_data.py

Dependencies:
    pip install matplotlib pandas numpy plotly
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HAS_PLOTLY = False
try:
    import plotly.graph_objects as go

    HAS_PLOTLY = True
except ImportError:
    pass

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"
PLOT_DIR = OUTPUT_DIR / "plots"


def load_data():
    """Load raw and steady-state CSVs."""
    df_torque = pd.read_csv(OUTPUT_DIR / "steering_torque_raw.csv")
    df_angle = pd.read_csv(OUTPUT_DIR / "steering_angle_raw.csv")
    df_speed = pd.read_csv(OUTPUT_DIR / "gps_imu_velocity_raw.csv")
    df_synced = pd.read_csv(OUTPUT_DIR / "feedforward_data.csv")
    df_steady = pd.read_csv(OUTPUT_DIR / "feedforward_data_steady.csv")

    print(f"Raw: torque={len(df_torque)}, angle={len(df_angle)}, speed={len(df_speed)}")
    print(f"Synchronized: {len(df_synced)} rows")
    print(f"Steady-state: {len(df_steady)} rows")

    return df_torque, df_angle, df_speed, df_synced, df_steady


def plot_time_series(df_torque, df_angle, df_speed, df_steady=None):
    """Plot 1: Time series overview — torque, angle, speed vs time.

    If df_steady is provided, highlights the steady-state spans with
    yellow background shading on all three subplots.
    """
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)

    t0 = min(
        df_torque["timestamp_ns"].iloc[0],
        df_angle["timestamp_ns"].iloc[0],
        df_speed["timestamp_ns"].iloc[0],
    )

    ax = axes[0]
    t = (df_torque["timestamp_ns"] - t0) / 1e9
    ax.plot(t, df_torque["torque"], linewidth=0.3, color="tab:red")
    ax.set_ylabel("Torque (normalized)")
    ax.set_title("Steering Torque (raw, from OSCC report)")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    t = (df_angle["timestamp_ns"] - t0) / 1e9
    ax.plot(t, df_angle["angle_rad"], linewidth=0.3, color="tab:blue")
    ax.set_ylabel("Angle (rad)")
    ax.set_title("Steering Angle")
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    t = (df_speed["timestamp_ns"] - t0) / 1e9
    ax.plot(t, df_speed["speed_mps"], linewidth=0.3, color="tab:green")
    ax.set_ylabel("Speed (m/s)")
    ax.set_xlabel("Time (s)")
    ax.set_title("GPS/IMU Velocity")
    ax.grid(True, alpha=0.3)

    # Highlight steady-state spans
    if df_steady is not None and len(df_steady) > 0:
        steady_t = (df_steady["timestamp_ns"].values - t0) / 1e9
        # Find contiguous segments (gap > 0.1s = new segment)
        dt = np.diff(steady_t)
        breaks = np.where(dt > 0.1)[0]
        seg_starts = np.concatenate([[0], breaks + 1])
        seg_ends = np.concatenate([breaks, [len(steady_t) - 1]])

        for ax in axes:
            for s, e in zip(seg_starts, seg_ends):
                ax.axvspan(steady_t[s], steady_t[e], alpha=0.15, color="gold")

        # Add legend entry on first axis
        from matplotlib.patches import Patch

        axes[0].legend(
            handles=[Patch(facecolor="gold", alpha=0.3, label="Steady-state data")],
            loc="upper right",
            fontsize=9,
        )

    fig.suptitle("Time Series Overview", fontsize=14)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "01_time_series_overview.png", dpi=150)
    plt.close(fig)
    print("  Saved 01_time_series_overview.png")


def plot_torque_vs_angle_all(df_synced):
    """Plot 2: Torque vs steering angle (all data), colored by speed."""
    fig, ax = plt.subplots(figsize=(10, 8))

    sc = ax.scatter(
        df_synced["angle_rad"],
        df_synced["torque"],
        c=df_synced["speed_mps"],
        cmap="viridis",
        s=1,
        alpha=0.3,
        rasterized=True,
    )
    fig.colorbar(sc, ax=ax, label="Speed (m/s)")

    ax.set_xlabel("Steering Angle (rad)")
    ax.set_ylabel("Torque (normalized, deadzone removed)")
    ax.set_title("Torque vs Steering Angle \u2014 All Synchronized Data")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(PLOT_DIR / "02_torque_vs_angle_all.png", dpi=150)
    plt.close(fig)
    print("  Saved 02_torque_vs_angle_all.png")


def plot_torque_vs_angle_steady(df_steady):
    """Plot 3: Torque vs steering angle (steady-state only), colored by speed."""
    fig, ax = plt.subplots(figsize=(10, 8))

    sc = ax.scatter(
        df_steady["angle_rad"],
        df_steady["torque"],
        c=df_steady["speed_mps"],
        cmap="viridis",
        s=2,
        alpha=0.5,
        rasterized=True,
    )
    fig.colorbar(sc, ax=ax, label="Speed (m/s)")

    ax.set_xlabel("Steering Angle (rad)")
    ax.set_ylabel("Torque (normalized, deadzone removed)")
    ax.set_title("Torque vs Steering Angle \u2014 Steady-State Only")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(PLOT_DIR / "03_torque_vs_angle_steady.png", dpi=150)
    plt.close(fig)
    print("  Saved 03_torque_vs_angle_steady.png")


def plot_torque_vs_angle_speed_bins(df_steady):
    """Plot 4: Torque vs angle at different speed bins."""
    speed_bins = [(0, 3), (3, 6), (6, 9), (9, 12), (12, np.inf)]
    n_bins = len(speed_bins)

    fig, axes = plt.subplots(1, n_bins, figsize=(4 * n_bins, 5), sharey=True)

    for ax, (lo, hi) in zip(axes, speed_bins):
        mask = (df_steady["speed_mps"] >= lo) & (df_steady["speed_mps"] < hi)
        subset = df_steady[mask]

        hi_label = f"{hi:.0f}" if np.isfinite(hi) else "+"
        ax.scatter(
            subset["angle_rad"], subset["torque"], s=2, alpha=0.4, rasterized=True
        )
        ax.set_xlabel("Angle (rad)")
        ax.set_title(f"{lo:.0f}-{hi_label} m/s (n={len(subset)})")
        ax.grid(True, alpha=0.3)

    axes[0].set_ylabel("Torque (normalized)")
    fig.suptitle("Torque vs Angle \u2014 Binned by Speed", fontsize=14)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "04_torque_vs_angle_speed_bins.png", dpi=150)
    plt.close(fig)
    print("  Saved 04_torque_vs_angle_speed_bins.png")


def plot_torque_vs_speed_angle_bins(df_steady):
    """Plot 5: Torque vs speed at different angle bins."""
    angle_bins = [
        (-0.15, -0.05),
        (-0.05, -0.02),
        (-0.02, 0.02),
        (0.02, 0.05),
        (0.05, 0.15),
    ]
    n_bins = len(angle_bins)

    fig, axes = plt.subplots(1, n_bins, figsize=(4 * n_bins, 5), sharey=True)

    for ax, (lo, hi) in zip(axes, angle_bins):
        mask = (df_steady["angle_rad"] >= lo) & (df_steady["angle_rad"] < hi)
        subset = df_steady[mask]

        ax.scatter(
            subset["speed_mps"], subset["torque"], s=2, alpha=0.4, rasterized=True
        )
        ax.set_xlabel("Speed (m/s)")
        ax.set_title(f"[{lo:.2f}, {hi:.2f}) rad\n(n={len(subset)})")
        ax.grid(True, alpha=0.3)

    axes[0].set_ylabel("Torque (normalized)")
    fig.suptitle("Torque vs Speed \u2014 Binned by Steering Angle", fontsize=14)
    fig.tight_layout()
    fig.savefig(PLOT_DIR / "05_torque_vs_speed_angle_bins.png", dpi=150)
    plt.close(fig)
    print("  Saved 05_torque_vs_speed_angle_bins.png")


def plot_speed_histogram(df_steady):
    """Plot 6: Speed distribution in steady-state data."""
    fig, ax = plt.subplots(figsize=(8, 5))

    ax.hist(df_steady["speed_mps"], bins=50, edgecolor="black", alpha=0.7)
    ax.set_xlabel("Speed (m/s)")
    ax.set_ylabel("Count")
    ax.set_title("Speed Distribution \u2014 Steady-State Data")
    ax.grid(True, alpha=0.3)

    spd = df_steady["speed_mps"]
    stats_text = f"Mean: {spd.mean():.1f} m/s\nMedian: {spd.median():.1f} m/s\nStd: {spd.std():.1f} m/s"
    ax.text(
        0.95,
        0.95,
        stats_text,
        transform=ax.transAxes,
        va="top",
        ha="right",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    fig.tight_layout()
    fig.savefig(PLOT_DIR / "06_speed_histogram.png", dpi=150)
    plt.close(fig)
    print("  Saved 06_speed_histogram.png")


def plot_angle_histogram(df_steady):
    """Plot 7: Steering angle distribution in steady-state data."""
    fig, ax = plt.subplots(figsize=(8, 5))

    ax.hist(df_steady["angle_rad"], bins=50, edgecolor="black", alpha=0.7)
    ax.set_xlabel("Steering Angle (rad)")
    ax.set_ylabel("Count")
    ax.set_title("Steering Angle Distribution \u2014 Steady-State Data")
    ax.grid(True, alpha=0.3)

    ang = df_steady["angle_rad"]
    stats_text = (
        f"Mean: {ang.mean():.4f} rad\nMedian: {ang.median():.4f} rad\n"
        f"Std: {ang.std():.4f} rad\nRange: [{ang.min():.3f}, {ang.max():.3f}]"
    )
    ax.text(
        0.95,
        0.95,
        stats_text,
        transform=ax.transAxes,
        va="top",
        ha="right",
        fontsize=10,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    fig.tight_layout()
    fig.savefig(PLOT_DIR / "07_angle_histogram.png", dpi=150)
    plt.close(fig)
    print("  Saved 07_angle_histogram.png")


def plot_3d_scatter(df_steady):
    """Plot 8: 3D scatter — torque vs (speed, angle). Interactive HTML + static PNG."""
    df = df_steady
    if len(df) > 20000:
        df = df.sample(n=20000, random_state=42)

    # Interactive HTML via plotly
    if HAS_PLOTLY:
        fig = go.Figure(
            data=[
                go.Scatter3d(
                    x=df["speed_mps"],
                    y=df["angle_rad"],
                    z=df["torque"],
                    mode="markers",
                    marker=dict(
                        size=2,
                        color=df["torque"],
                        colorscale="RdBu",
                        colorbar=dict(title="Torque"),
                        opacity=0.6,
                    ),
                )
            ]
        )
        fig.update_layout(
            title="Feedforward Torque Surface — Steady-State Data",
            scene=dict(
                xaxis_title="Speed (m/s)",
                yaxis_title="Steering Angle (rad)",
                zaxis_title="Torque (normalized)",
            ),
            width=1000,
            height=800,
        )
        fig.write_html(str(PLOT_DIR / "08_3d_scatter.html"))
        print("  Saved 08_3d_scatter.html (interactive)")

    # Static PNG fallback via matplotlib
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    mfig = plt.figure(figsize=(12, 9))
    ax = mfig.add_subplot(111, projection="3d")
    sc = ax.scatter(
        df["speed_mps"],
        df["angle_rad"],
        df["torque"],
        c=np.abs(df["torque"]),
        cmap="coolwarm",
        s=2,
        alpha=0.5,
        rasterized=True,
    )
    ax.set_xlabel("Speed (m/s)")
    ax.set_ylabel("Steering Angle (rad)")
    ax.set_zlabel("Torque (normalized)")
    ax.set_title("Feedforward Torque Surface \u2014 Steady-State Data")
    mfig.colorbar(sc, ax=ax, shrink=0.6, label="|Torque|")
    mfig.tight_layout()
    mfig.savefig(PLOT_DIR / "08_3d_scatter.png", dpi=150)
    plt.close(mfig)
    print("  Saved 08_3d_scatter.png")


def main():
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    print("Loading data...")
    df_torque, df_angle, df_speed, df_synced, df_steady = load_data()

    print("\nGenerating plots...")
    plot_time_series(df_torque, df_angle, df_speed, df_steady)
    plot_torque_vs_angle_all(df_synced)
    plot_torque_vs_angle_steady(df_steady)
    plot_torque_vs_angle_speed_bins(df_steady)
    plot_torque_vs_speed_angle_bins(df_steady)
    plot_speed_histogram(df_steady)
    plot_angle_histogram(df_steady)
    plot_3d_scatter(df_steady)

    print(f"\nDone! Plots saved to: {PLOT_DIR}")
    print("\nReview the plots and decide:")
    print("  - Is torque-angle approximately linear in delta? -> Model A sufficient")
    print("  - Is there visible nonlinearity in delta?        -> Need Model C")
    print("  - Is there asymmetry / friction offset?          -> Need Model B")


if __name__ == "__main__":
    main()
