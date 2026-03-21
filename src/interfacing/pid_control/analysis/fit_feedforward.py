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

"""Stage 3: Fit feedforward steering torque models to steady-state data.

Reads feedforward_data_steady.csv and fits multiple candidate models.
Produces comparison metrics, overlay plots, residual analysis, and
YAML config snippets ready to paste into vel_driven_feedforward_pid.yaml.

Models:
    A: T = (c0 + c1*v + c2*v^2) * delta          — current C++ structure, no code changes
    B: T = (c0 + c1*v + c2*v^2) * delta + c3*sign(delta)  — adds friction, needs C++ change
    C: T = sum(c_ij * v^i * delta^j)              — 2D polynomial, needs C++ change

Usage:
    python fit_feedforward.py [--model A|B|C|all] [--vel-degree 2] [--angle-degree 3]

Dependencies:
    pip install numpy pandas matplotlib scipy scikit-learn plotly
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"
PLOT_DIR = OUTPUT_DIR / "plots"


def calc_rmse(y_true, y_pred):
    return np.sqrt(np.mean((y_true - y_pred) ** 2))


def calc_r2(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    return 1.0 - ss_res / ss_tot


# ---------------------------------------------------------------------------
# Model A: T = (c0 + c1*v + c2*v^2 + ...) * delta
#   Matches current C++ compute_feedforward() — no code changes needed.
#   Coefficients map directly to YAML feedforward.coefficients.
# ---------------------------------------------------------------------------


def fit_model_a(v, delta, T, vel_degree=2):
    """Fit Model A via linear least squares.

    Features: [delta, v*delta, v^2*delta, ...]
    """
    X = np.column_stack([(v**i) * delta for i in range(vel_degree + 1)])
    coeffs, _, _, _ = np.linalg.lstsq(X, T, rcond=None)
    return coeffs, X @ coeffs


def predict_model_a(v, delta, coeffs):
    ff = sum(c * v**i for i, c in enumerate(coeffs))
    return ff * delta


# ---------------------------------------------------------------------------
# Model B: T = (c0 + c1*v + c2*v^2) * delta + c3*sign(delta)
#   Adds static friction offset. Last coefficient is the friction term.
#   NOTE: Requires C++ changes to add friction_offset parameter.
# ---------------------------------------------------------------------------


def fit_model_b(v, delta, T, vel_degree=2):
    """Fit Model B via linear least squares.

    Features: [delta, v*delta, v^2*delta, ..., sign(delta)]
    """
    features = [(v**i) * delta for i in range(vel_degree + 1)]
    features.append(np.sign(delta))
    X = np.column_stack(features)
    coeffs, _, _, _ = np.linalg.lstsq(X, T, rcond=None)
    return coeffs, X @ coeffs


def predict_model_b(v, delta, coeffs):
    n_vel = len(coeffs) - 1
    ff = sum(coeffs[i] * v**i for i in range(n_vel))
    return ff * delta + coeffs[-1] * np.sign(delta)


# ---------------------------------------------------------------------------
# Model C: T = sum(c_ij * v^i * delta^j)  for j in [1, angle_degree]
#   General 2D polynomial. j starts at 1 so T(delta=0) = 0.
#   NOTE: Requires C++ changes for 2D polynomial evaluation.
# ---------------------------------------------------------------------------


def fit_model_c(v, delta, T, vel_degree=2, angle_degree=3):
    """Fit Model C via linear least squares."""
    features = []
    labels = []
    for i in range(vel_degree + 1):
        for j in range(1, angle_degree + 1):
            features.append((v**i) * (delta**j))
            labels.append(f"v^{i} * d^{j}")
    X = np.column_stack(features)
    coeffs, _, _, _ = np.linalg.lstsq(X, T, rcond=None)
    return coeffs, X @ coeffs, labels


def predict_model_c(v, delta, coeffs, vel_degree, angle_degree):
    T = np.zeros_like(v, dtype=np.float64)
    idx = 0
    for i in range(vel_degree + 1):
        for j in range(1, angle_degree + 1):
            T += coeffs[idx] * (v**i) * (delta**j)
            idx += 1
    return T


# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------


def plot_overlay_speed_bins(v, delta, T, T_pred, model_name):
    """Data vs fitted model overlay in speed bins."""
    speed_bins = [(0, 3), (3, 6), (6, 9), (9, 12), (12, np.inf)]
    n = len(speed_bins)
    fig, axes = plt.subplots(1, n, figsize=(4 * n, 5), sharey=True)

    for ax, (lo, hi) in zip(axes, speed_bins):
        mask = (v >= lo) & (v < hi)
        hi_label = f"{hi:.0f}" if np.isfinite(hi) else "+"

        ax.scatter(delta[mask], T[mask], s=1, alpha=0.3, label="Data", rasterized=True)

        order = np.argsort(delta[mask])
        ax.plot(
            delta[mask][order],
            T_pred[mask][order],
            "r-",
            linewidth=1.5,
            alpha=0.7,
            label="Fit",
        )

        ax.set_xlabel("Angle (rad)")
        ax.set_title(f"{lo:.0f}-{hi_label} m/s (n={np.sum(mask)})")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    axes[0].set_ylabel("Torque")
    fig.suptitle(f"{model_name} \u2014 Data vs Fit by Speed Bin", fontsize=14)
    fig.tight_layout()
    return fig


def plot_residuals(T, T_pred, v, delta, model_name):
    """Residual analysis: vs predicted, vs speed, vs angle, histogram."""
    residuals = T - T_pred

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    ax = axes[0, 0]
    ax.scatter(T_pred, residuals, s=1, alpha=0.3, rasterized=True)
    ax.axhline(0, color="red", linewidth=0.5)
    ax.set_xlabel("Predicted Torque")
    ax.set_ylabel("Residual")
    ax.set_title("Residuals vs Predicted")
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.scatter(v, residuals, s=1, alpha=0.3, rasterized=True)
    ax.axhline(0, color="red", linewidth=0.5)
    ax.set_xlabel("Speed (m/s)")
    ax.set_ylabel("Residual")
    ax.set_title("Residuals vs Speed")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.scatter(delta, residuals, s=1, alpha=0.3, rasterized=True)
    ax.axhline(0, color="red", linewidth=0.5)
    ax.set_xlabel("Steering Angle (rad)")
    ax.set_ylabel("Residual")
    ax.set_title("Residuals vs Angle")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.hist(residuals, bins=100, edgecolor="black", alpha=0.7)
    ax.set_xlabel("Residual")
    ax.set_ylabel("Count")
    ax.set_title(f"Residual Distribution (std={np.std(residuals):.4f})")
    ax.grid(True, alpha=0.3)

    fig.suptitle(f"{model_name} \u2014 Residual Analysis", fontsize=14)
    fig.tight_layout()
    return fig


def plot_3d_surface(
    v_range,
    delta_range,
    predict_fn,
    model_name,
    tag=None,
    scatter_v=None,
    scatter_d=None,
    scatter_T=None,
):
    """3D surface + scatter overlay. Interactive HTML + static PNG."""
    v_grid, d_grid = np.meshgrid(
        np.linspace(v_range[0], v_range[1], 50),
        np.linspace(delta_range[0], delta_range[1], 50),
    )
    T_grid = predict_fn(v_grid.ravel(), d_grid.ravel()).reshape(v_grid.shape)

    # Subsample scatter for rendering performance
    has_scatter = scatter_v is not None and len(scatter_v) > 0
    if has_scatter and len(scatter_v) > 15000:
        rng = np.random.RandomState(42)
        idx = rng.choice(len(scatter_v), 15000, replace=False)
        sv, sd, sT = scatter_v[idx], scatter_d[idx], scatter_T[idx]
    elif has_scatter:
        sv, sd, sT = scatter_v, scatter_d, scatter_T
    else:
        sv = sd = sT = None

    # Interactive HTML via plotly
    try:
        import plotly.graph_objects as go

        traces = [
            go.Surface(
                x=v_grid,
                y=d_grid,
                z=T_grid,
                colorscale="RdBu",
                opacity=0.7,
                colorbar=dict(title="Fit"),
                name="Fitted surface",
            )
        ]
        if sv is not None:
            traces.append(
                go.Scatter3d(
                    x=sv,
                    y=sd,
                    z=sT,
                    mode="markers",
                    marker=dict(size=1.5, color=sT, colorscale="RdBu", opacity=0.4),
                    name="Data",
                )
            )
        pfig = go.Figure(data=traces)
        pfig.update_layout(
            title=f"{model_name} \u2014 Surface + Data",
            scene=dict(
                xaxis_title="Speed (m/s)",
                yaxis_title="Steering Angle (rad)",
                zaxis_title="Torque (normalized)",
            ),
            width=1000,
            height=800,
        )
        if tag:
            pfig.write_html(str(PLOT_DIR / f"fit_{tag}_surface.html"))
            print(f"  Saved fit_{tag}_surface.html (interactive)")
    except ImportError:
        pass

    # Static PNG via matplotlib
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    if sv is not None:
        ax.scatter(
            sv,
            sd,
            sT,
            c=sT,
            cmap="coolwarm",
            s=1,
            alpha=0.15,
            rasterized=True,
            label="Data",
        )
    ax.plot_surface(v_grid, d_grid, T_grid, cmap="coolwarm", alpha=0.6)
    ax.set_xlabel("Speed (m/s)")
    ax.set_ylabel("Steering Angle (rad)")
    ax.set_zlabel("Torque (normalized)")
    ax.set_title(f"{model_name} \u2014 Fitted Surface + Data")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------
# YAML output helpers
# ---------------------------------------------------------------------------


def format_yaml_model_a(coeffs):
    coeff_str = "[" + ", ".join(f"{c:.6f}" for c in coeffs) + "]"
    return (
        "# Model A: T = (c0 + c1*v + c2*v^2 + ...) * delta\n"
        "# Paste into vel_driven_feedforward_pid.yaml:\n"
        "feedforward:\n"
        f"  coefficients: {coeff_str}\n"
    )


def format_yaml_model_b(coeffs):
    vel_coeffs = coeffs[:-1]
    friction = coeffs[-1]
    coeff_str = "[" + ", ".join(f"{c:.6f}" for c in vel_coeffs) + "]"
    return (
        "# Model B: T = (c0 + c1*v + c2*v^2) * delta + friction * sign(delta)\n"
        "# NOTE: Requires C++ changes to support friction_offset parameter\n"
        "feedforward:\n"
        f"  coefficients: {coeff_str}\n"
        f"  friction_offset: {friction:.6f}\n"
    )


def format_yaml_model_c(coeffs, vel_degree, angle_degree):
    coeff_str = "[" + ", ".join(f"{c:.6f}" for c in coeffs) + "]"
    return (
        f"# Model C: 2D polynomial, vel_degree={vel_degree}, angle_degree={angle_degree}\n"
        "# T = sum(c_ij * v^i * delta^j) for i in [0,vel_degree], j in [1,angle_degree]\n"
        "# NOTE: Requires C++ changes for 2D polynomial evaluation\n"
        "feedforward:\n"
        f"  coefficients: {coeff_str}\n"
        f"  angle_degree: {angle_degree}\n"
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Fit feedforward steering torque models"
    )
    parser.add_argument(
        "--model",
        type=str,
        default="all",
        choices=["A", "B", "C", "all"],
        help="Which model(s) to fit (default: all)",
    )
    parser.add_argument(
        "--vel-degree",
        type=int,
        default=2,
        help="Velocity polynomial degree (default: 2)",
    )
    parser.add_argument(
        "--angle-degree",
        type=int,
        default=3,
        help="Angle polynomial degree for Model C (default: 3)",
    )
    parser.add_argument(
        "--test-size",
        type=float,
        default=0.2,
        help="Test set fraction (default: 0.2)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for train/test split (default: 42)",
    )
    args = parser.parse_args()

    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    print("Loading steady-state data...")
    df = pd.read_csv(OUTPUT_DIR / "feedforward_data_steady.csv")
    print(f"  {len(df)} samples")

    v = df["speed_mps"].values
    delta = df["angle_rad"].values
    T = df["torque"].values

    idx_train, idx_test = train_test_split(
        np.arange(len(df)), test_size=args.test_size, random_state=args.seed
    )
    v_tr, v_te = v[idx_train], v[idx_test]
    d_tr, d_te = delta[idx_train], delta[idx_test]
    T_tr, T_te = T[idx_train], T[idx_test]
    print(f"  Train: {len(idx_train)}, Test: {len(idx_test)}")

    models_to_fit = ["A", "B", "C"] if args.model == "all" else [args.model]
    results = {}

    for model_id in models_to_fit:
        print(f"\n{'=' * 60}")
        print(f"Model {model_id}")
        print(f"{'=' * 60}")

        if model_id == "A":
            coeffs, T_tr_pred = fit_model_a(
                v_tr, d_tr, T_tr, vel_degree=args.vel_degree
            )
            T_te_pred = predict_model_a(v_te, d_te, coeffs)
            T_all_pred = predict_model_a(v, delta, coeffs)

            poly_str = " + ".join(f"{c:.6f}*v^{i}" for i, c in enumerate(coeffs))
            print(f"\n  T = ({poly_str}) * delta")
            print(f"\n  Coefficients (YAML order): {coeffs.tolist()}")
            yaml_str = format_yaml_model_a(coeffs)

        elif model_id == "B":
            coeffs, T_tr_pred = fit_model_b(
                v_tr, d_tr, T_tr, vel_degree=args.vel_degree
            )
            T_te_pred = predict_model_b(v_te, d_te, coeffs)
            T_all_pred = predict_model_b(v, delta, coeffs)

            vel_c = coeffs[:-1]
            fric = coeffs[-1]
            poly_str = " + ".join(f"{c:.6f}*v^{i}" for i, c in enumerate(vel_c))
            print(f"\n  T = ({poly_str}) * delta + {fric:.6f}*sign(delta)")
            print(f"\n  Velocity coefficients: {vel_c.tolist()}")
            print(f"  Friction offset: {fric:.6f}")
            yaml_str = format_yaml_model_b(coeffs)

        elif model_id == "C":
            coeffs, T_tr_pred, labels = fit_model_c(
                v_tr,
                d_tr,
                T_tr,
                vel_degree=args.vel_degree,
                angle_degree=args.angle_degree,
            )
            T_te_pred = predict_model_c(
                v_te, d_te, coeffs, args.vel_degree, args.angle_degree
            )
            T_all_pred = predict_model_c(
                v, delta, coeffs, args.vel_degree, args.angle_degree
            )

            print(
                f"\n  Terms (vel_degree={args.vel_degree}, angle_degree={args.angle_degree}):"
            )
            for lbl, c in zip(labels, coeffs):
                print(f"    {lbl}: {c:.6f}")
            yaml_str = format_yaml_model_c(coeffs, args.vel_degree, args.angle_degree)

        # Metrics
        r2_tr = calc_r2(T_tr, T_tr_pred)
        r2_te = calc_r2(T_te, T_te_pred)
        rmse_tr = calc_rmse(T_tr, T_tr_pred)
        rmse_te = calc_rmse(T_te, T_te_pred)

        print(f"\n  Train -- R2: {r2_tr:.4f}, RMSE: {rmse_tr:.6f}")
        print(f"  Test  -- R2: {r2_te:.4f}, RMSE: {rmse_te:.6f}")

        results[model_id] = {
            "r2_train": r2_tr,
            "r2_test": r2_te,
            "rmse_train": rmse_tr,
            "rmse_test": rmse_te,
        }

        print("\n  YAML config:\n")
        for line in yaml_str.splitlines():
            print(f"    {line}")

        # --- Plots ---
        tag = model_id.lower()

        fig = plot_overlay_speed_bins(v, delta, T, T_all_pred, f"Model {model_id}")
        fig.savefig(PLOT_DIR / f"fit_{tag}_overlay.png", dpi=150)
        plt.close(fig)
        print(f"\n  Saved fit_{tag}_overlay.png")

        fig = plot_residuals(T, T_all_pred, v, delta, f"Model {model_id}")
        fig.savefig(PLOT_DIR / f"fit_{tag}_residuals.png", dpi=150)
        plt.close(fig)
        print(f"  Saved fit_{tag}_residuals.png")

        v_range = (v.min(), v.max())
        d_range = (delta.min(), delta.max())

        if model_id == "A":
            _c = coeffs

            def pred_fn(vv, dd, _c=_c):
                return predict_model_a(vv, dd, _c)

        elif model_id == "B":
            _c = coeffs

            def pred_fn(vv, dd, _c=_c):
                return predict_model_b(vv, dd, _c)

        elif model_id == "C":
            _c, _vd, _ad = coeffs, args.vel_degree, args.angle_degree

            def pred_fn(vv, dd, _c=_c, _vd=_vd, _ad=_ad):
                return predict_model_c(vv, dd, _c, _vd, _ad)

        fig = plot_3d_surface(
            v_range,
            d_range,
            pred_fn,
            f"Model {model_id}",
            tag=tag,
            scatter_v=v,
            scatter_d=delta,
            scatter_T=T,
        )
        fig.savefig(PLOT_DIR / f"fit_{tag}_surface.png", dpi=150)
        plt.close(fig)
        print(f"  Saved fit_{tag}_surface.png")

    # --- Summary comparison ---
    if len(results) > 1:
        print(f"\n{'=' * 60}")
        print("Model Comparison")
        print(f"{'=' * 60}")
        header = f"{'Model':<8} {'R2 (train)':<12} {'R2 (test)':<12} {'RMSE (train)':<14} {'RMSE (test)':<14}"
        print(header)
        print("-" * len(header))
        for name, r in results.items():
            print(
                f"{name:<8} {r['r2_train']:<12.4f} {r['r2_test']:<12.4f} "
                f"{r['rmse_train']:<14.6f} {r['rmse_test']:<14.6f}"
            )

    print(f"\nPlots saved to: {PLOT_DIR}")


if __name__ == "__main__":
    main()
