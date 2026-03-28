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
"""Generate 30 feedforward curve variants (±2% steps) as interactive 3D HTML plots.

Based on symmetric Model B fit:
  T_ff = (c0 + c1*v + c2*v^2) * delta + friction_offset * sign(delta)

Each variant uniformly scales coefficients and friction by a percentage.

Usage:
    python compare_ff_curves.py
"""

import numpy as np
import plotly.graph_objects as go
from pathlib import Path

PLOT_DIR = Path(__file__).resolve().parent / "output" / "plots" / "compare_ff_curves"
PLOT_DIR.mkdir(parents=True, exist_ok=True)

# ── Baseline: symmetric Model B from fresh extraction (R²=0.845) ────────────
# T_ff = (c0 + c1*v + c2*v^2) * delta + friction_offset * sign(delta)
BASELINE_COEFFS = [0.137961, -0.097721, 0.002266]
BASELINE_FRICTION = -0.049855

# ── Generate 90 scale factors: 45 below, 45 above, each ±2% ─────────────────
N_EACH_SIDE = 45
STEP = 0.02
scales = np.concatenate(
    [
        np.linspace(1.0 - N_EACH_SIDE * STEP, 1.0 - STEP, N_EACH_SIDE),
        np.linspace(1.0 + STEP, 1.0 + N_EACH_SIDE * STEP, N_EACH_SIDE),
    ]
)


def compute_ff(velocity, delta, scale=1.0):
    c = [ci * scale for ci in BASELINE_COEFFS]
    fric = BASELINE_FRICTION * scale
    gain = c[0] + c[1] * velocity + c[2] * velocity**2
    result = gain * delta
    if fric != 0.0:
        result = result + fric * np.sign(delta)
    return result


def scale_to_color(s):
    t = (s - 0.10) / (1.90 - 0.10)
    r = int(60 + 195 * t)
    b = int(255 - 195 * t)
    g = int(60 + 100 * (1 - abs(2 * t - 1)))
    return f"rgb({r},{g},{b})"


# ── 3D grids ─────────────────────────────────────────────────────────────────
v_lin = np.linspace(0, 15, 60)
d_lin = np.linspace(-0.5, 0.5, 60)
v_grid, d_grid = np.meshgrid(v_lin, d_lin)
v_flat, d_flat = v_grid.ravel(), d_grid.ravel()

T_base = compute_ff(v_flat, d_flat, 1.0).reshape(v_grid.shape)

# --- Plot 1: All 30 surfaces overlaid ---
fig1 = go.Figure()

for i, s in enumerate(scales):
    idx = i + 1
    T_grid = compute_ff(v_flat, d_flat, s).reshape(v_grid.shape)
    c = [ci * s for ci in BASELINE_COEFFS]
    f = BASELINE_FRICTION * s
    fig1.add_trace(
        go.Surface(
            x=v_grid,
            y=d_grid,
            z=T_grid,
            colorscale=[[0, scale_to_color(s)], [1, scale_to_color(s)]],
            opacity=0.25,
            showscale=False,
            name=f"Curve {idx:02d} ({s * 100:.0f}%)",
            hovertemplate=(
                f"Curve {idx:02d} ({s * 100:.0f}%)<br>"
                f"[{c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f}] fric={f:.4f}<br>"
                "v=%{x:.1f} m/s<br>δ=%{y:.3f} rad<br>T=%{z:.4f}<extra></extra>"
            ),
        )
    )

fig1.add_trace(
    go.Surface(
        x=v_grid,
        y=d_grid,
        z=T_base,
        colorscale=[[0, "rgb(40,40,40)"], [1, "rgb(40,40,40)"]],
        opacity=0.8,
        showscale=False,
        name="Baseline (100%)",
        hovertemplate="Baseline (100%)<br>v=%{x:.1f} m/s<br>δ=%{y:.3f} rad<br>T=%{z:.4f}<extra></extra>",
    )
)

fig1.update_layout(
    title="Model B Feedforward — 90 Variants (±2% steps, 10%-190%)",
    scene=dict(
        xaxis_title="Velocity (m/s)",
        yaxis_title="Steering Angle δ (rad)",
        zaxis_title="Feedforward Torque",
    ),
    width=1200,
    height=900,
)
fig1.write_html(str(PLOT_DIR / "ff_90_surfaces_all.html"))
print("Saved ff_90_surfaces_all.html")

# --- Plot 2: Individual HTML per curve ---
for i, s in enumerate(scales):
    idx = i + 1
    T_grid = compute_ff(v_flat, d_flat, s).reshape(v_grid.shape)
    fig = go.Figure()

    fig.add_trace(
        go.Surface(
            x=v_grid,
            y=d_grid,
            z=T_base,
            colorscale=[[0, "rgb(180,180,180)"], [1, "rgb(180,180,180)"]],
            opacity=0.3,
            showscale=False,
            name="Baseline (100%)",
        )
    )

    fig.add_trace(
        go.Surface(
            x=v_grid,
            y=d_grid,
            z=T_grid,
            colorscale="RdBu",
            opacity=0.85,
            colorbar=dict(title="Torque"),
            name=f"Curve {idx:02d} ({s * 100:.0f}%)",
        )
    )

    fig.add_trace(
        go.Surface(
            x=v_grid,
            y=d_grid,
            z=T_grid - T_base,
            colorscale="RdBu",
            opacity=0.5,
            showscale=False,
            name="Difference from baseline",
            visible="legendonly",
        )
    )

    c = [ci * s for ci in BASELINE_COEFFS]
    f = BASELINE_FRICTION * s
    fig.update_layout(
        title=(
            f"Curve {idx:02d} ({s * 100:.0f}%) — "
            f"coefficients: [{c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f}], "
            f"friction_offset: {f:.6f}"
        ),
        scene=dict(
            xaxis_title="Velocity (m/s)",
            yaxis_title="Steering Angle δ (rad)",
            zaxis_title="Torque",
        ),
        width=1100,
        height=800,
    )
    fig.write_html(str(PLOT_DIR / f"ff_curve_{idx:02d}_{s * 100:.0f}pct.html"))

print("Saved 90 individual curve HTMLs")

# --- Plot 3: Less torque group ---
fig3 = go.Figure()
fig3.add_trace(
    go.Surface(
        x=v_grid,
        y=d_grid,
        z=T_base,
        colorscale=[[0, "rgb(40,40,40)"], [1, "rgb(40,40,40)"]],
        opacity=0.7,
        showscale=False,
        name="Baseline (100%)",
    )
)
for i, s in enumerate(scales[:15]):
    T_grid = compute_ff(v_flat, d_flat, s).reshape(v_grid.shape)
    fig3.add_trace(
        go.Surface(
            x=v_grid,
            y=d_grid,
            z=T_grid,
            colorscale=[[0, scale_to_color(s)], [1, scale_to_color(s)]],
            opacity=0.3,
            showscale=False,
            name=f"Curve {i + 1:02d} ({s * 100:.0f}%)",
        )
    )
fig3.update_layout(
    title="Less Torque Variants (10%-98%)",
    scene=dict(
        xaxis_title="Velocity (m/s)",
        yaxis_title="Steering Angle δ (rad)",
        zaxis_title="Torque",
    ),
    width=1200,
    height=900,
)
fig3.write_html(str(PLOT_DIR / "ff_90_surfaces_less_torque.html"))
print("Saved ff_90_surfaces_less_torque.html")

# --- Plot 4: More torque group ---
fig4 = go.Figure()
fig4.add_trace(
    go.Surface(
        x=v_grid,
        y=d_grid,
        z=T_base,
        colorscale=[[0, "rgb(40,40,40)"], [1, "rgb(40,40,40)"]],
        opacity=0.7,
        showscale=False,
        name="Baseline (100%)",
    )
)
for i, s in enumerate(scales[15:]):
    idx = i + 16
    T_grid = compute_ff(v_flat, d_flat, s).reshape(v_grid.shape)
    fig4.add_trace(
        go.Surface(
            x=v_grid,
            y=d_grid,
            z=T_grid,
            colorscale=[[0, scale_to_color(s)], [1, scale_to_color(s)]],
            opacity=0.3,
            showscale=False,
            name=f"Curve {idx:02d} ({s * 100:.0f}%)",
        )
    )
fig4.update_layout(
    title="More Torque Variants (102%-190%)",
    scene=dict(
        xaxis_title="Velocity (m/s)",
        yaxis_title="Steering Angle δ (rad)",
        zaxis_title="Torque",
    ),
    width=1200,
    height=900,
)
fig4.write_html(str(PLOT_DIR / "ff_90_surfaces_more_torque.html"))
print("Saved ff_90_surfaces_more_torque.html")

# --- Print YAML table ---
print(f"\n{'Curve':<12} {'Scale':<8} {'c0':>10} {'c1':>12} {'c2':>12} {'friction':>12}")
print("=" * 70)
for i, s in enumerate(scales):
    c = [ci * s for ci in BASELINE_COEFFS]
    f = BASELINE_FRICTION * s
    print(
        f"Curve {i + 1:02d}     {s * 100:5.0f}%  {c[0]:>10.6f} {c[1]:>12.6f} {c[2]:>12.6f} {f:>12.6f}"
    )
print("-" * 70)
print(
    f"Baseline      100%  {BASELINE_COEFFS[0]:>10.6f} {BASELINE_COEFFS[1]:>12.6f} {BASELINE_COEFFS[2]:>12.6f} {BASELINE_FRICTION:>12.6f}"
)

print(f"\nAll plots in: {PLOT_DIR}")
