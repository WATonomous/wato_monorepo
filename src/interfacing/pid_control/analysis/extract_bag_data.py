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

"""Stage 1: Extract and synchronize bag data for feedforward steering torque curve fitting.

Reads MCAP bag files, extracts steering torque, steering angle, INSPVA
velocity + solution status, and is_armed flag. Synchronizes, removes OSCC
deadzone, and applies aggressive steady-state filtering:

  1. Low-pass (Butterworth) smoothing before differentiation
  2. Derivative thresholds on angle, speed, AND torque
  3. is_armed == True (vehicle under autonomous control)
  4. INSPVA solution quality >= 3 (INS_SOLUTION_GOOD)
  5. Minimum contiguous segment duration (0.5 s)
  6. Per-bin outlier rejection (2.5 sigma)

Usage:
    python extract_bag_data.py [--bag-dir PATH] [--max-gap-ms 10.0]

Dependencies:
    pip install mcap mcap-ros2-support numpy pandas scipy
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from scipy.signal import butter, filtfilt

# Topics to extract
TOPIC_TORQUE = "/interfacing/oscc_interfacing/steering_torque"
TOPIC_ANGLE = "/interfacing/can_state_estimator/steering_angle"
TOPIC_INSPVA = "/novatel/oem7/inspva"
TOPIC_IS_ARMED = "/interfacing/oscc_interfacing/is_armed"

# OSCC deadzone values (from oscc_interfacing_config.yaml)
DEADZONE_POS = 0.09
DEADZONE_NEG = 0.13

SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"
BAG_DIR_DEFAULT = SCRIPT_DIR.parents[3] / "bags" / "recording_20260301_223023"


def get_header_timestamp_ns(ros_msg):
    """Extract nanosecond timestamp from ROS2 message header."""
    stamp = ros_msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def extract_raw_data(bag_dir):
    """Extract raw per-topic data from all MCAP files."""
    mcap_files = sorted(Path(bag_dir).glob("*.mcap"))
    if not mcap_files:
        print(f"ERROR: No MCAP files found in {bag_dir}")
        sys.exit(1)

    print(f"Found {len(mcap_files)} MCAP files in {bag_dir}")

    topics_to_read = [TOPIC_TORQUE, TOPIC_ANGLE, TOPIC_INSPVA, TOPIC_IS_ARMED]
    torque_data = []
    angle_data = []
    speed_data = []  # (ts_ns, speed_mps, inspva_status)
    armed_data = []  # (ts_ns, is_armed)

    factory = DecoderFactory()

    for i, mcap_file in enumerate(mcap_files):
        print(f"\n  [{i + 1}/{len(mcap_files)}] {mcap_file.name}")
        msg_count = 0

        with open(mcap_file, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()

            decoders = {}
            for ch_id, ch in summary.channels.items():
                if ch.topic in topics_to_read:
                    schema = summary.schemas[ch.schema_id]
                    decoders[ch_id] = (
                        ch.topic,
                        factory.decoder_for(ch.message_encoding, schema),
                    )

            for schema, channel, message in reader.iter_messages(
                topics=topics_to_read,
            ):
                if channel.id not in decoders:
                    continue
                topic, decode = decoders[channel.id]
                try:
                    ros_msg = decode(message.data)
                except Exception:
                    continue

                if topic == TOPIC_IS_ARMED:
                    ts_ns = message.log_time
                    armed_data.append((ts_ns, bool(ros_msg.data)))
                else:
                    ts_ns = get_header_timestamp_ns(ros_msg)
                    if topic == TOPIC_TORQUE:
                        torque_data.append((ts_ns, float(ros_msg.torque)))
                    elif topic == TOPIC_ANGLE:
                        angle_data.append((ts_ns, float(ros_msg.angle)))
                    elif topic == TOPIC_INSPVA:
                        ground_speed = math.sqrt(
                            ros_msg.north_velocity**2 + ros_msg.east_velocity**2
                        )
                        status = int(ros_msg.status.status)
                        speed_data.append((ts_ns, ground_speed, status))

                msg_count += 1
                if msg_count % 100000 == 0:
                    print(f"    ... {msg_count} messages processed")

        print(f"    {msg_count} messages extracted")

    print(
        f"\nExtracted totals: torque={len(torque_data)}, "
        f"angle={len(angle_data)}, speed={len(speed_data)}, armed={len(armed_data)}"
    )

    return torque_data, angle_data, speed_data, armed_data


def save_raw_csvs(torque_data, angle_data, speed_data, armed_data):
    """Save raw per-topic CSVs and return sorted DataFrames."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    df_torque = pd.DataFrame(torque_data, columns=["timestamp_ns", "torque"])
    df_torque.sort_values("timestamp_ns", inplace=True)
    df_torque.reset_index(drop=True, inplace=True)
    df_torque.to_csv(OUTPUT_DIR / "steering_torque_raw.csv", index=False)

    df_angle = pd.DataFrame(angle_data, columns=["timestamp_ns", "angle_rad"])
    df_angle.sort_values("timestamp_ns", inplace=True)
    df_angle.reset_index(drop=True, inplace=True)
    df_angle.to_csv(OUTPUT_DIR / "steering_angle_raw.csv", index=False)

    df_speed = pd.DataFrame(
        speed_data, columns=["timestamp_ns", "speed_mps", "inspva_status"]
    )
    df_speed.sort_values("timestamp_ns", inplace=True)
    df_speed.reset_index(drop=True, inplace=True)
    df_speed.to_csv(OUTPUT_DIR / "gps_imu_velocity_raw.csv", index=False)

    df_armed = pd.DataFrame(armed_data, columns=["timestamp_ns", "is_armed"])
    df_armed.sort_values("timestamp_ns", inplace=True)
    df_armed.reset_index(drop=True, inplace=True)
    df_armed.to_csv(OUTPUT_DIR / "is_armed_raw.csv", index=False)

    print("Saved raw CSVs:")
    print(f"  steering_torque_raw.csv  ({len(df_torque)} rows)")
    print(f"  steering_angle_raw.csv   ({len(df_angle)} rows)")
    print(f"  gps_imu_velocity_raw.csv ({len(df_speed)} rows)")
    print(f"  is_armed_raw.csv         ({len(df_armed)} rows)")

    return df_torque, df_angle, df_speed, df_armed


def interpolate_to_master(master_ts, slave_ts, slave_values, max_gap_ns):
    """Interpolate slave signal onto master timestamps via linear interpolation."""
    idx = np.searchsorted(slave_ts, master_ts, side="left")
    idx = np.clip(idx, 1, len(slave_ts) - 1)

    left_idx = idx - 1
    right_idx = idx

    left_gap = np.abs(master_ts.astype(np.int64) - slave_ts[left_idx].astype(np.int64))
    right_gap = np.abs(
        slave_ts[right_idx].astype(np.int64) - master_ts.astype(np.int64)
    )
    min_gap = np.minimum(left_gap, right_gap)

    span = (slave_ts[right_idx] - slave_ts[left_idx]).astype(np.float64)
    span = np.where(span == 0, 1.0, span)
    alpha = (
        master_ts.astype(np.float64) - slave_ts[left_idx].astype(np.float64)
    ) / span
    alpha = np.clip(alpha, 0.0, 1.0)
    interp_values = (
        slave_values[left_idx] * (1.0 - alpha) + slave_values[right_idx] * alpha
    )

    valid = min_gap <= max_gap_ns
    return interp_values, valid, min_gap


def nearest_to_master(master_ts, slave_ts, slave_values, max_gap_ns):
    """Map slave signal onto master timestamps via nearest-neighbor (no interpolation).

    Used for discrete signals like is_armed (bool) and inspva_status (int).
    """
    idx = np.searchsorted(slave_ts, master_ts, side="left")
    idx = np.clip(idx, 1, len(slave_ts) - 1)

    left_idx = idx - 1
    right_idx = idx

    left_gap = np.abs(master_ts.astype(np.int64) - slave_ts[left_idx].astype(np.int64))
    right_gap = np.abs(
        slave_ts[right_idx].astype(np.int64) - master_ts.astype(np.int64)
    )

    use_right = right_gap < left_gap
    nearest_idx = np.where(use_right, right_idx, left_idx)
    min_gap = np.minimum(left_gap, right_gap)

    values = slave_values[nearest_idx]
    valid = min_gap <= max_gap_ns
    return values, valid, min_gap


def synchronize_data(df_torque, df_angle, df_speed, df_armed, max_gap_ns):
    """Synchronize all signals onto the highest-rate timeline."""
    rates = {
        "torque": len(df_torque),
        "angle": len(df_angle),
        "speed": len(df_speed),
    }
    master_name = max(rates, key=rates.get)
    print(f"Master timeline: {master_name} ({rates[master_name]} samples)")
    print(f"Max gap tolerance: {max_gap_ns / 1e6:.1f} ms")

    sources = {
        "torque": (df_torque, "timestamp_ns", "torque"),
        "angle": (df_angle, "timestamp_ns", "angle_rad"),
        "speed": (df_speed, "timestamp_ns", "speed_mps"),
    }

    master_df, master_ts_col, master_val_col = sources[master_name]
    master_ts = master_df[master_ts_col].values

    output_col_map = {"torque": "torque", "angle": "angle_rad", "speed": "speed_mps"}
    synced = {"timestamp_ns": master_ts}
    synced[output_col_map[master_name]] = master_df[master_val_col].values

    valid_mask = np.ones(len(master_ts), dtype=bool)

    # Interpolate continuous signals
    for name, (df, ts_col, val_col) in sources.items():
        if name == master_name:
            continue
        interp_values, valid, gaps = interpolate_to_master(
            master_ts, df[ts_col].values, df[val_col].values, max_gap_ns
        )
        synced[output_col_map[name]] = interp_values
        valid_mask &= valid
        print(
            f"\n  {name}: mean gap={np.mean(gaps) / 1e6:.3f}ms, "
            f"max={np.max(gaps) / 1e6:.3f}ms, "
            f"within tol={100 * np.sum(valid) / len(valid):.1f}%"
        )

    # Nearest-neighbor for inspva_status (discrete)
    inspva_status, valid_s, gaps_s = nearest_to_master(
        master_ts,
        df_speed["timestamp_ns"].values,
        df_speed["inspva_status"].values,
        max_gap_ns,
    )
    synced["inspva_status"] = inspva_status
    valid_mask &= valid_s

    # Nearest-neighbor for is_armed (boolean)
    armed_ts = df_armed["timestamp_ns"].values
    armed_vals = df_armed["is_armed"].values.astype(np.int8)
    # is_armed publishes at high rate, use generous gap (500ms)
    armed_nn, valid_a, gaps_a = nearest_to_master(
        master_ts, armed_ts, armed_vals, 500_000_000
    )
    synced["is_armed"] = armed_nn.astype(bool)
    print(
        f"\n  is_armed: mean gap={np.mean(gaps_a) / 1e6:.3f}ms, "
        f"max={np.max(gaps_a) / 1e6:.3f}ms, "
        f"within tol={100 * np.sum(valid_a) / len(valid_a):.1f}%"
    )

    df_synced = pd.DataFrame(synced)
    df_synced = df_synced[valid_mask].reset_index(drop=True)

    pct = 100 * len(df_synced) / len(master_ts)
    print(
        f"\nSynchronized: {len(df_synced)}/{len(master_ts)} "
        f"samples retained ({pct:.1f}%)"
    )

    return df_synced


def remove_deadzone(torque_values):
    """Invert OSCC deadzone to recover pre-deadzone (feedforward) torque."""
    result = np.zeros_like(torque_values)
    pos_mask = torque_values >= DEADZONE_POS
    neg_mask = torque_values <= -DEADZONE_NEG
    result[pos_mask] = torque_values[pos_mask] - DEADZONE_POS
    result[neg_mask] = torque_values[neg_mask] + DEADZONE_NEG

    total = len(torque_values)
    n_pos = np.sum(pos_mask)
    n_neg = np.sum(neg_mask)
    n_dead = total - n_pos - n_neg
    print(f"  Positive (>= +{DEADZONE_POS}): {n_pos} ({100 * n_pos / total:.1f}%)")
    print(f"  Negative (<= -{DEADZONE_NEG}): {n_neg} ({100 * n_neg / total:.1f}%)")
    print(f"  Deadzone gap: {n_dead} ({100 * n_dead / total:.1f}%)")

    return result


def lowpass_filter(signal, fs, cutoff=2.0, order=4):
    """Zero-phase Butterworth low-pass filter."""
    nyq = fs / 2.0
    if cutoff >= nyq:
        return signal.copy()
    b, a = butter(order, cutoff / nyq, btype="low")
    # Need enough samples for filtfilt (3 * max(len(a), len(b)))
    min_len = 3 * max(len(a), len(b))
    if len(signal) < min_len:
        return signal.copy()
    return filtfilt(b, a, signal)


def label_contiguous_segments(mask):
    """Label contiguous True regions in a boolean mask. Returns (labels, n_segments).

    Each contiguous True block gets a unique integer label (1, 2, ...).
    False regions get label 0.
    """
    diff = np.diff(mask.astype(np.int8))
    starts = np.where(diff == 1)[0] + 1
    ends = np.where(diff == -1)[0] + 1

    if mask[0]:
        starts = np.concatenate([[0], starts])
    if mask[-1]:
        ends = np.concatenate([ends, [len(mask)]])

    labels = np.zeros(len(mask), dtype=np.int32)
    for seg_id, (s, e) in enumerate(zip(starts, ends), 1):
        labels[s:e] = seg_id

    return labels, len(starts)


def filter_steady_state(
    df,
    min_segment_s=0.5,
    outlier_sigma=3.0,
    d_angle_thresh=0.05,
    d_speed_thresh=0.5,
    d_torque_thresh=0.3,
    min_speed=0.5,
    min_torque=0.005,
):
    """Enhanced steady-state filtering pipeline."""
    df = df.copy()
    total = len(df)
    time_s = df["timestamp_ns"].values / 1e9

    # Estimate sample rate
    dt_median = np.median(np.diff(time_s))
    fs = 1.0 / dt_median
    print(f"  Sample rate: {fs:.1f} Hz")

    # --- 1. Low-pass smooth before differentiation ---
    print("\n  [1] Low-pass smoothing (Butterworth, 2 Hz cutoff)...")
    angle_smooth = lowpass_filter(df["angle_rad"].values, fs, cutoff=2.0)
    speed_smooth = lowpass_filter(df["speed_mps"].values, fs, cutoff=2.0)
    torque_smooth = lowpass_filter(df["torque"].values, fs, cutoff=2.0)

    d_angle_dt = np.gradient(angle_smooth, time_s)
    d_speed_dt = np.gradient(speed_smooth, time_s)
    d_torque_dt = np.gradient(torque_smooth, time_s)

    df["d_angle_dt"] = d_angle_dt
    df["d_speed_dt"] = d_speed_dt
    df["d_torque_dt"] = d_torque_dt

    # --- 2. Derivative thresholds (on smoothed signals) ---
    print("  [2] Derivative thresholds...")
    mask = np.ones(total, dtype=bool)

    m = np.abs(d_angle_dt) < d_angle_thresh
    mask &= m
    print(f"      |d(angle)/dt| < {d_angle_thresh} rad/s:  {np.sum(mask):>8} / {total}")

    m = np.abs(d_speed_dt) < d_speed_thresh
    mask &= m
    print(
        f"      + |d(speed)/dt| < {d_speed_thresh} m/s²:  {np.sum(mask):>8} / {total}"
    )

    m = np.abs(d_torque_dt) < d_torque_thresh
    mask &= m
    print(
        f"      + |d(torque)/dt| < {d_torque_thresh} /s:   {np.sum(mask):>8} / {total}"
    )

    # --- 3. is_armed filter (skip if data too sparse) ---
    print("  [3] is_armed filter...")
    armed_col = df["is_armed"].values.astype(bool)
    armed_true_pct = 100 * np.sum(armed_col) / total
    if armed_true_pct > 5:
        mask &= armed_col
        print(f"      + is_armed == True:           {np.sum(mask):>8} / {total}")
    else:
        print(
            f"      SKIPPED — only {armed_true_pct:.1f}% armed "
            f"(is_armed decode likely failed; using torque presence as proxy)"
        )

    # --- 4. INSPVA solution quality ---
    print("  [4] INSPVA solution quality...")
    m = df["inspva_status"].values >= 3
    mask &= m
    print(f"      + inspva_status >= 3:         {np.sum(mask):>8} / {total}")

    # --- 5. Basic thresholds ---
    print("  [5] Basic thresholds...")
    m = df["speed_mps"].values > min_speed
    mask &= m
    print(f"      + speed > {min_speed} m/s:           {np.sum(mask):>8} / {total}")

    m = np.abs(df["torque"].values) > min_torque
    mask &= m
    print(f"      + |torque| > {min_torque}:           {np.sum(mask):>8} / {total}")

    # --- 6. Minimum segment duration ---
    print(f"  [6] Minimum segment duration ({min_segment_s:.1f}s)...")
    labels, n_segs = label_contiguous_segments(mask)
    for seg_id in range(1, n_segs + 1):
        seg_mask = labels == seg_id
        seg_indices = np.where(seg_mask)[0]
        if len(seg_indices) < 2:
            mask[seg_mask] = False
            continue
        duration = time_s[seg_indices[-1]] - time_s[seg_indices[0]]
        if duration < min_segment_s:
            mask[seg_mask] = False
    print(
        f"      + segments >= {min_segment_s:.1f}s:          {np.sum(mask):>8} / {total}"
    )

    # --- 7. Per-bin outlier rejection ---
    print(f"  [7] Outlier rejection ({outlier_sigma} sigma per bin)...")
    speed_vals = df["speed_mps"].values
    angle_vals = df["angle_rad"].values
    torque_vals = df["torque"].values

    speed_edges = np.arange(0, speed_vals.max() + 3, 3)
    angle_edges = np.linspace(angle_vals.min() - 0.001, angle_vals.max() + 0.001, 11)

    s_bins = np.digitize(speed_vals, speed_edges)
    a_bins = np.digitize(angle_vals, angle_edges)

    for si in range(1, len(speed_edges)):
        for ai in range(1, len(angle_edges)):
            bin_mask = mask & (s_bins == si) & (a_bins == ai)
            idx = np.where(bin_mask)[0]
            if len(idx) < 10:
                continue
            t_bin = torque_vals[idx]
            mu = np.mean(t_bin)
            sigma = np.std(t_bin)
            if sigma < 1e-6:
                continue
            outliers = np.abs(t_bin - mu) > outlier_sigma * sigma
            mask[idx[outliers]] = False

    print(f"      + outlier removal:            {np.sum(mask):>8} / {total}")

    # --- Summary ---
    df_steady = df[mask].reset_index(drop=True)
    print(
        f"\n  Final: {len(df_steady)} / {total} ({100 * len(df_steady) / total:.1f}%)"
    )

    return df_steady


def main():
    parser = argparse.ArgumentParser(
        description="Extract bag data for feedforward steering torque curve fitting"
    )
    parser.add_argument(
        "--bag-dir",
        type=str,
        default=str(BAG_DIR_DEFAULT),
        help="Path to bag directory containing MCAP files",
    )
    parser.add_argument(
        "--max-gap-ms",
        type=float,
        default=10.0,
        help="Maximum sync gap tolerance in milliseconds (default: 10.0)",
    )
    parser.add_argument(
        "--d-angle-thresh",
        type=float,
        default=0.05,
        help="Max |d(angle)/dt| for steady-state (default: 0.05 rad/s)",
    )
    parser.add_argument(
        "--d-speed-thresh",
        type=float,
        default=0.5,
        help="Max |d(speed)/dt| for steady-state (default: 0.5 m/s²)",
    )
    parser.add_argument(
        "--d-torque-thresh",
        type=float,
        default=0.3,
        help="Max |d(torque)/dt| for steady-state (default: 0.3 /s)",
    )
    parser.add_argument(
        "--min-speed",
        type=float,
        default=0.5,
        help="Minimum speed threshold (default: 0.5 m/s)",
    )
    parser.add_argument(
        "--min-torque",
        type=float,
        default=0.005,
        help="Minimum |torque| threshold (default: 0.005)",
    )
    parser.add_argument(
        "--min-segment-s",
        type=float,
        default=0.5,
        help="Minimum steady-state segment duration (default: 0.5 s)",
    )
    parser.add_argument(
        "--outlier-sigma",
        type=float,
        default=3.0,
        help="Outlier rejection sigma (default: 3.0)",
    )
    args = parser.parse_args()

    bag_dir = Path(args.bag_dir).resolve()
    max_gap_ns = int(args.max_gap_ms * 1e6)

    print("=" * 60)
    print("STAGE 1a: Extracting raw data from MCAP files")
    print("=" * 60)
    torque_data, angle_data, speed_data, armed_data = extract_raw_data(bag_dir)

    if not torque_data or not angle_data or not speed_data:
        print("ERROR: One or more topics had no data. Check topic names.")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("STAGE 1b: Saving raw per-topic CSVs")
    print("=" * 60)
    df_torque, df_angle, df_speed, df_armed = save_raw_csvs(
        torque_data, angle_data, speed_data, armed_data
    )

    print("\n" + "=" * 60)
    print("STAGE 1c: Synchronizing data")
    print("=" * 60)
    df_synced = synchronize_data(df_torque, df_angle, df_speed, df_armed, max_gap_ns)

    print("\n" + "=" * 60)
    print("STAGE 1d: Removing OSCC deadzone")
    print("=" * 60)
    print(f"  Deadzone: +{DEADZONE_POS} (positive), -{DEADZONE_NEG} (negative)")
    df_synced["torque"] = remove_deadzone(df_synced["torque"].values)

    df_synced[["timestamp_ns", "torque", "angle_rad", "speed_mps"]].to_csv(
        OUTPUT_DIR / "feedforward_data.csv", index=False
    )
    print(f"\n  Saved feedforward_data.csv ({len(df_synced)} rows)")

    print("\n" + "=" * 60)
    print("STAGE 1e: Filtering for quasi-steady-state")
    print("=" * 60)
    df_steady = filter_steady_state(
        df_synced,
        min_segment_s=args.min_segment_s,
        outlier_sigma=args.outlier_sigma,
        d_angle_thresh=args.d_angle_thresh,
        d_speed_thresh=args.d_speed_thresh,
        d_torque_thresh=args.d_torque_thresh,
        min_speed=args.min_speed,
        min_torque=args.min_torque,
    )

    df_steady.to_csv(OUTPUT_DIR / "feedforward_data_steady.csv", index=False)
    print(f"\n  Saved feedforward_data_steady.csv ({len(df_steady)} rows)")

    print("\n" + "=" * 60)
    print(f"Done! Output directory: {OUTPUT_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
