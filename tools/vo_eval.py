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
"""Evaluate visual odometry (visual_odometry offline_vo output) on the host with `rosbags`.

Reports, as far as the given bags allow:
  * tracking: uptime, gaps (a gap = tracking loss / reset), processing time and camera sync offsets
    from the VO status messages;
  * drift: net motion of each tracking session (meaningful on parked data);
  * time offset: cross-correlation of VO yaw rate with an IMU's gyro z (needs turning). Add the
    result to eidos' visual_odometry_factor.time_offset;
  * accuracy vs a reference odometry (default: eidos liso/odometry): relative pose error and VO
    scale over path segments (needs driving). Scale != 1 points at stereo depth bias, i.e. the pair
    rotation along the epipolar lines that calibrate_pairs cannot observe while parked;
  * eidos visual_odometry_factor residuals, if the reference bag has them.

    tools/vo_eval.py --vo <offline_vo out> [--ref <eidos bag> ] [--imu <sensor bag>]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def quat_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def pose_matrix(pose):
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    )
    t[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return t


def angle_deg(r):
    return float(np.degrees(np.arccos(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))))


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read(bag, topics):
    """{topic: [(stamp_s or log_s, msg)]} for the topics present in the bag."""
    out = {t: [] for t in topics}
    with AnyReader([Path(bag)], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic in out]
        for conn, log_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = stamp_s(msg.header) if hasattr(msg, "header") else log_ns * 1e-9
            out[conn.topic].append((t, msg))
    return out


def sessions(stamps, max_gap):
    """Split sample indices into sessions at gaps > max_gap."""
    breaks = np.flatnonzero(np.diff(stamps) > max_gap) + 1
    return np.split(np.arange(len(stamps)), breaks)


def interpolate_poses(t_src, poses, t_query):
    """Linear translation + nearest rotation (fine at 20 Hz) of 4x4 poses at t_query."""
    idx = np.clip(np.searchsorted(t_src, t_query), 1, len(t_src) - 1)
    out = []
    for i, t in zip(idx, t_query):
        a, b = poses[i - 1], poses[i]
        alpha = (t - t_src[i - 1]) / (t_src[i] - t_src[i - 1])
        p = (a if alpha < 0.5 else b).copy()
        p[:3, 3] = (1 - alpha) * a[:3, 3] + alpha * b[:3, 3]
        out.append(p)
    return out


def yaw_rate(stamps, poses):
    """Body yaw rate (rad/s) from consecutive poses, at the midpoints."""
    t_mid, rates = [], []
    for i in range(1, len(stamps)):
        dt = stamps[i] - stamps[i - 1]
        if dt <= 0:
            continue
        r = poses[i - 1][:3, :3].T @ poses[i][:3, :3]
        rates.append(np.arctan2(r[1, 0], r[0, 0]) / dt)
        t_mid.append(0.5 * (stamps[i] + stamps[i - 1]))
    return np.array(t_mid), np.array(rates)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--vo",
        required=True,
        help="offline_vo output bag (or any bag with the VO topic)",
    )
    parser.add_argument("--vo-topic", default="/perception/visual_odometry/odometry")
    parser.add_argument("--status-topic", default="/perception/visual_odometry/status")
    parser.add_argument(
        "--ref", help="bag with the reference odometry (e.g. an eidos offline run)"
    )
    parser.add_argument("--ref-topic", default="/world_modeling/liso/odometry")
    parser.add_argument(
        "--residual-topic", default="/world_modeling/visual_odometry_factor/residual"
    )
    parser.add_argument(
        "--imu", help="bag with an IMU for the time-offset estimate (slow on big bags)"
    )
    parser.add_argument(
        "--imu-topic",
        default="/novatel/oem7/imu/data_raw",
        help="IMU with real gyro data (imu/data has zero angular velocity in the Sept 2026 bags)",
    )
    parser.add_argument(
        "--rig-height",
        type=float,
        default=1.76,
        help="VO rig (base_link) above the reference frame, m",
    )
    parser.add_argument(
        "--period", type=float, default=0.05, help="nominal VO period, s"
    )
    parser.add_argument(
        "--segments",
        type=float,
        nargs="+",
        default=[10.0, 100.0],
        help="RPE segment lengths, m",
    )
    args = parser.parse_args(argv)

    vo = read(args.vo, [args.vo_topic, args.status_topic])
    samples = sorted(vo[args.vo_topic], key=lambda s: s[0])
    if len(samples) < 2:
        print(f"no {args.vo_topic} messages in {args.vo}", file=sys.stderr)
        return 1
    t_vo = np.array([s[0] for s in samples])
    p_vo = [pose_matrix(s[1].pose.pose) for s in samples]
    # Express VO (rig = base_link) motion in the reference body frame (base_footprint): pure z offset.
    t_ref_rig = np.eye(4)
    t_ref_rig[2, 3] = args.rig_height
    p_vo = [t_ref_rig @ p @ np.linalg.inv(t_ref_rig) for p in p_vo]

    # ---- Tracking ----
    span = t_vo[-1] - t_vo[0]
    expected = int(round(span / args.period)) + 1
    gaps = np.diff(t_vo)
    sess = sessions(t_vo, 1.5 * args.period)
    print(f"== Tracking ({args.vo})")
    print(
        f"  {len(t_vo)} poses over {span:.1f} s: {100.0 * len(t_vo) / expected:.1f}% of {expected} expected"
    )
    print(
        f"  gaps > 1.5 periods: {int((gaps > 1.5 * args.period).sum())} (max {gaps.max() * 1e3:.0f} ms); sessions: {len(sess)}"
    )
    if vo[args.status_topic]:
        last = vo[args.status_topic][-1][1].status[0]
        values = {kv.key: kv.value for kv in last.values}
        print(
            f"  last status: {last.message}; track_ms {values.get('track_ms')}; landmarks {values.get('landmarks')}"
        )
        for key in sorted(k for k in values if k.startswith("offset_ms/")):
            print(f"  camera offset {key.split('/', 1)[1]}: {values[key]} ms")

    # ---- Drift per session ----
    print("== Net motion per tracking session (drift, if parked)")
    for s in sess:
        if len(s) < 2:
            continue
        d = np.linalg.inv(p_vo[s[0]]) @ p_vo[s[-1]]
        dt = t_vo[s[-1]] - t_vo[s[0]]
        path = sum(np.linalg.norm(p_vo[i][:3, 3] - p_vo[i - 1][:3, 3]) for i in s[1:])
        print(
            f"  {dt:7.1f} s: net {np.linalg.norm(d[:3, 3]):.3f} m / {angle_deg(d[:3, :3]):.3f} deg, "
            f"path {path:.2f} m ({np.linalg.norm(d[:3, 3]) / max(dt, 1e-9) * 60:.3f} m/min)"
        )

    # ---- Time offset vs IMU ----
    if args.imu:
        imu = read(args.imu, [args.imu_topic])[args.imu_topic]
        t_imu = np.array([s[0] for s in imu])
        gz = np.array([s[1].angular_velocity.z for s in imu])
        t_mid, w_vo = yaw_rate(t_vo, p_vo)
        print("== Time offset (VO yaw rate vs IMU gyro z)")
        if np.std(gz) < 0.02:
            print(
                f"  IMU yaw rate std {np.std(gz):.4f} rad/s: not enough turning to estimate an offset"
            )
        else:
            grid = np.arange(
                max(t_mid[0], t_imu[0]) + 1.0, min(t_mid[-1], t_imu[-1]) - 1.0, 0.005
            )
            imu_i = np.interp(grid, t_imu, gz)
            lags = np.arange(-0.3, 0.3001, 0.005)
            corr = [
                np.corrcoef(imu_i, np.interp(grid - lag, t_mid, w_vo))[0, 1]
                for lag in lags
            ]
            best = lags[int(np.argmax(corr))]
            print(
                f"  best lag {best * 1e3:+.0f} ms (corr {max(corr):.3f}): set visual_odometry_factor.time_offset: {best:.3f}"
            )

    # ---- Accuracy vs reference ----
    if args.ref:
        ref = read(args.ref, [args.ref_topic, args.residual_topic])
        rs = sorted(ref[args.ref_topic], key=lambda s: s[0])
        if len(rs) > 1:
            t_ref = np.array([s[0] for s in rs])
            p_ref = [pose_matrix(s[1].pose.pose) for s in rs]
            dist = np.concatenate(
                [
                    [0.0],
                    np.cumsum(
                        [
                            np.linalg.norm(p_ref[i][:3, 3] - p_ref[i - 1][:3, 3])
                            for i in range(1, len(p_ref))
                        ]
                    ),
                ]
            )
            print(
                f"== Accuracy vs {args.ref_topic} ({dist[-1]:.1f} m of reference path)"
            )
            for s in sess:
                inside = (t_ref >= t_vo[s[0]]) & (t_ref <= t_vo[s[-1]])
                idx = np.flatnonzero(inside)
                if len(idx) < 2:
                    continue
                vo_at = interpolate_poses(t_vo[s], [p_vo[i] for i in s], t_ref[idx])
                for seg in args.segments:
                    errs, rots, scales = [], [], []
                    j = 0
                    for i in range(len(idx)):
                        while j < len(idx) and dist[idx[j]] - dist[idx[i]] < seg:
                            j += 1
                        if j >= len(idx):
                            break
                        rel_ref = np.linalg.inv(p_ref[idx[i]]) @ p_ref[idx[j]]
                        rel_vo = np.linalg.inv(vo_at[i]) @ vo_at[j]
                        e = np.linalg.inv(rel_ref) @ rel_vo
                        errs.append(np.linalg.norm(e[:3, 3]) / seg * 100.0)
                        rots.append(angle_deg(e[:3, :3]) / seg * 100.0)
                        scales.append(
                            np.linalg.norm(rel_vo[:3, 3])
                            / max(np.linalg.norm(rel_ref[:3, 3]), 1e-9)
                        )
                    if errs:
                        print(
                            f"  {seg:5.0f} m segments (n={len(errs)}): translation error {np.median(errs):.2f}% "
                            f"(p90 {np.percentile(errs, 90):.2f}%), rotation {np.median(rots):.3f} deg/100 m, "
                            f"scale {np.median(scales):.3f} (p10-p90 {np.percentile(scales, 10):.3f}-{np.percentile(scales, 90):.3f})"
                        )
                    else:
                        print(f"  {seg:5.0f} m segments: reference path too short")
        if ref[args.residual_topic]:
            res = np.array(
                [
                    (
                        np.linalg.norm(
                            [m.pose.position.x, m.pose.position.y, m.pose.position.z]
                        ),
                        2
                        * np.degrees(
                            np.arctan2(
                                np.linalg.norm(
                                    [
                                        m.pose.orientation.x,
                                        m.pose.orientation.y,
                                        m.pose.orientation.z,
                                    ]
                                ),
                                abs(m.pose.orientation.w),
                            )
                        ),
                    )
                    for _, m in ref[args.residual_topic]
                ]
            )
            print(
                f"== eidos factor residuals (VO vs graph, per keyframe pair), n={len(res)}"
            )
            print(
                f"  translation m p50/p95/max {np.median(res[:, 0]):.4f}/{np.percentile(res[:, 0], 95):.4f}/{res[:, 0].max():.4f}; "
                f"rotation deg p50/p95/max {np.median(res[:, 1]):.3f}/{np.percentile(res[:, 1], 95):.3f}/{res[:, 1].max():.3f}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
