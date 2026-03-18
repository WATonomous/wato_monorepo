# Feedforward Steering Torque Analysis

Tools for curve-fitting `torque = f(speed, angle)` from bag data to populate the feedforward term in `vel_driven_feedforward_pid.yaml`.

## Pipeline

Run inside the interfacing dev container (`watod -t interfacing_bringup_dev`) where bags are mounted at `/ws/bags`:

### Prerequisites

```bash
pip install mcap mcap-ros2-support numpy pandas scipy matplotlib scikit-learn plotly
```

### Stage 1: Extract bag data

```bash
python3 extract_bag_data.py --bag-dir /ws/bags/recording_20260301_223023
```

Reads MCAP files and extracts:
- Steering torque from `/interfacing/oscc_interfacing/steering_torque`
- Steering angle from `/interfacing/can_state_estimator/steering_angle`
- Vehicle speed from `/novatel/oem7/inspva` (INS-fused GPS+IMU velocity)

Then synchronizes the signals, removes the OSCC deadzone offset, and filters for quasi-steady-state conditions (low-pass smoothed derivative thresholds, minimum segment duration, outlier rejection).

Outputs CSVs to `output/`.

### Stage 2: Plot data

```bash
python3 plot_data.py
```

Generates 8 diagnostic plots (PNGs + interactive HTML) to `output/plots/`:
- Time series with steady-state spans highlighted
- Torque vs angle scatter (all data and steady-state)
- Speed-binned and angle-binned views
- Speed/angle histograms
- Interactive 3D scatter

### Stage 3: Fit models

```bash
python3 fit_feedforward.py --model all
```

Fits three candidate models to steady-state data with 80/20 train/test split:

| Model | Formula | C++ support |
|-------|---------|-------------|
| A | `T = (c0 + c1*v + c2*v²) * delta` | `friction_offset: 0.0` |
| B | `A + friction * sign(delta)` | Set `friction_offset` in YAML |
| C | `sum(c_ij * v^i * delta^j)` | Requires C++ changes |

Outputs overlay plots, residual analysis, interactive 3D surfaces with data scatter, and YAML snippets.

Options:
- `--model A|B|C|all` — which model(s) to fit
- `--vel-degree N` — velocity polynomial degree (default: 2)
- `--angle-degree N` — angle polynomial degree for Model C (default: 3)

## Applying results

Paste the coefficients into `config/vel_driven_feedforward_pid.yaml`:

```yaml
feedforward:
  coefficients: [c0, c1, c2]   # from Model A or B
  friction_offset: 0.0          # 0.0 for Model A, nonzero for Model B
```

Parameters are hot-reloadable at runtime — no rebuild needed.

## Using a different bag

```bash
python3 extract_bag_data.py --bag-dir /ws/bags/your_recording
python3 plot_data.py
python3 fit_feedforward.py --model all
```

The bag must contain the three topics listed above. The INSPVA topic uses raw `iter_messages` + manual decode as a workaround for an mcap topic-filter indexing bug.
