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
"""Score the eidos visual plugins against the lidar ones, from runs of tools/eidos_visual_eval.sh.

    tools/eidos_visual_eval.py slam <run dir> [<run dir> ...]
    tools/eidos_visual_eval.py relocalization --map <map run dir> <run dir> [<run dir> ...]
    tools/eidos_visual_eval.py ablation [--outage 130:200] [--json out.json] <run dir> [...]

The reference is the vehicle's INS (/novatel/oem7/odom, recorded with every run). Without RTK it is
good to about a metre absolutely and much better over seconds, which is enough to tell apart the
errors measured here. Two systematic differences between the INS and eidos' body frame are removed
before comparing, and both are reported:

  * a constant time offset (lidar stamps vs INS stamps), fitted on the graph's keyframe steps;
  * a constant rotation between the INS's base_link and eidos' base_footprint (mount and extrinsic
    errors of a degree or two), fitted per source on the directions of its steps.

``slam`` reports, per run:
  * odometry: MapAnything on consecutive keyframes (shadow_odometry) next to the graph's own steps;
  * loop closures: per plugin, what it measured and accepted, how far its measurements are from the
    INS, and how much drift the graph had when they were made;
  * place recognition: how many candidates are real revisits;
  * the final map: revisit consistency (relative pose between keyframes of different passes) and
    absolute trajectory error after a rigid alignment to the INS. These two decide whether a
    loop closure plugin made the map better;
  * the graph as a pose source: absolute error of the final keyframes and of the live pose (the
    pose eidos published when each keyframe was first optimized, what a consumer sees at the time),
    and the drift of the final keyframes over 100 m and 200 m.

``relocalization`` reports, per run, the pose tracking started from against the INS, mapped into
the prior map's frame through the map's own keyframes near that place.

Runs on the host with rosbags, numpy and scipy.
"""

import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation, Slerp

VISUAL = "/world_modeling/visual_loop_closure_factor/measurement"
# Consecutive-keyframe measurements of the lidar-free keyframe plugin; scored like VISUAL's
MULTIVIEW = "/world_modeling/multiview_odometry_factor/measurement"
SLAM_ODOM = "/world_modeling/slam/odometry"
GICP = "/world_modeling/euclidean_distance_loop_closure_factor/measurement"
INS = "/novatel/oem7/odom"
LISO = "/world_modeling/liso/odometry"
INITIAL = "/world_modeling/slam/initial_pose"
RELOC_ESTIMATE = "/world_modeling/visual_relocalization/estimate"
INDEX_MASK = (1 << 56) - 1  # gtsam::Symbol: character in the top byte, index below

REVISIT_GAP_S = 60.0  # keyframes this far apart in time are different passes
REVISIT_DIST_M = 10.0  # ... and a revisit if the INS puts them this close


# ---- geometry ---------------------------------------------------------------------------------------


def pose_matrix(pose) -> np.ndarray:
    q, p = pose.orientation, pose.position
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def invert(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


def angle_deg(r: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1))))


def error(estimate: np.ndarray, reference: np.ndarray):
    """(translation error m, rotation error deg) of a relative pose against a reference."""
    diff = invert(reference) @ estimate
    return float(np.linalg.norm(diff[:3, 3])), angle_deg(diff[:3, :3])


def conjugate(offset: np.ndarray, rel: np.ndarray) -> np.ndarray:
    """A relative motion of frame A expressed for frame B, with offset = T_A_B (rotation only here)."""
    return invert(offset) @ rel @ offset


def kabsch(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rotation R minimising sum |R a_i - b_i|^2."""
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    return vt.T @ np.diag([1.0, 1.0, d]) @ u.T


def umeyama(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Rigid 4x4 T minimising sum |T src_i - dst_i|^2."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    r = kabsch(src - cs, dst - cd)
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = cd - r @ cs
    return out


def stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Trajectory:
    """Timed poses with interpolation; ``shift`` (s) is added to every query time."""

    def __init__(self, stamps, poses):
        order = np.argsort(stamps)
        self.t = np.asarray(stamps, dtype=float)[order]
        self.poses = np.asarray(poses)[order]
        keep = np.concatenate([[True], np.diff(self.t) > 0])
        self.t, self.poses = self.t[keep], self.poses[keep]
        self._slerp = Slerp(self.t, Rotation.from_matrix(self.poses[:, :3, :3]))
        self.shift = 0.0

    def covers(self, t: float) -> bool:
        return self.t[0] <= t + self.shift <= self.t[-1]

    def at(self, t: float) -> np.ndarray:
        t = t + self.shift
        out = np.eye(4)
        out[:3, :3] = self._slerp(t).as_matrix()
        out[:3, 3] = [np.interp(t, self.t, self.poses[:, k, 3]) for k in range(3)]
        return out

    def relative(self, ta: float, tb: float) -> np.ndarray:
        return invert(self.at(ta)) @ self.at(tb)


# ---- one run ----------------------------------------------------------------------------------------


class Run:
    """Recorded topics and final keyframes of one eidos_visual_eval.sh run."""

    def __init__(self, path: Path):
        self.path = path
        self.name = path.name
        self.measurements = {VISUAL: [], GICP: []}
        self.initial, self.estimates = [], []
        self.live = {}  # keyframe stamp -> pose eidos first published for it
        tracks = {INS: ([], []), LISO: ([], [])}
        with AnyReader([path / "out"]) as reader:
            wanted = [
                c
                for c in reader.connections
                if c.topic
                in (VISUAL, MULTIVIEW, GICP, INS, LISO, INITIAL, RELOC_ESTIMATE, SLAM_ODOM)
            ]
            for conn, _, raw in reader.messages(wanted):
                msg = reader.deserialize(raw, conn.msgtype)
                if conn.topic in tracks:
                    tracks[conn.topic][0].append(stamp(msg.header))
                    tracks[conn.topic][1].append(pose_matrix(msg.pose.pose))
                elif conn.topic in self.measurements:
                    self.measurements[conn.topic].append(msg)
                elif conn.topic == MULTIVIEW:
                    self.measurements[VISUAL].append(msg)
                elif conn.topic == SLAM_ODOM:
                    self.live.setdefault(stamp(msg.header), pose_matrix(msg.pose.pose))
                elif conn.topic == INITIAL:
                    self.initial.append((stamp(msg.header), pose_matrix(msg.pose)))
                elif conn.topic == RELOC_ESTIMATE:
                    self.estimates.append((stamp(msg.header), pose_matrix(msg.pose)))
        self.ins = Trajectory(*tracks[INS]) if len(tracks[INS][0]) > 1 else None
        self.liso = Trajectory(*tracks[LISO]) if len(tracks[LISO][0]) > 1 else None
        self.keyframes = self._keyframes(path / "map.map")
        self.offset = {}  # source -> 4x4 rotation T_ins_body fitted for it

    @staticmethod
    def _keyframes(map_file: Path):
        """{state index: (stamp, 4x4 pose in the map)} of the run's own keyframes, from its saved map."""
        if not map_file.exists():
            return {}
        db = sqlite3.connect(f"file:{map_file}?mode=ro&immutable=1", uri=True)
        out = {}
        for key, x, y, z, roll, pitch, yaw, t in db.execute(
            "SELECT gtsam_key, x, y, z, roll, pitch, yaw, time FROM keyframes"
        ):
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
            pose[:3, 3] = [x, y, z]
            out[int(key) & INDEX_MASK] = (t, pose)
        db.close()
        return dict(sorted(out.items()))

    def map_relative(self, a: int, b: int):
        if a not in self.keyframes or b not in self.keyframes:
            return None
        return invert(self.keyframes[a][1]) @ self.keyframes[b][1]

    def ins_relative(self, ta: float, tb: float, source: str = "graph"):
        """INS motion between two times, expressed for eidos' body frame as fitted for source."""
        rel = self.ins.relative(ta, tb)
        offset = self.offset.get(source)
        return rel if offset is None else conjugate(offset, rel)

    def calibrate(self):
        """Fit the INS time offset and the per-source rotation offsets from the keyframe steps."""
        steps = [
            m for m in self.measurements[VISUAL] if m.candidate_source == "consecutive"
        ]
        graph = [(m.from_stamp, m.to_stamp, pose_matrix(m.graph)) for m in steps]
        if not graph and len(self.keyframes) > 2:
            kf = list(self.keyframes.values())
            graph = [(a[0], b[0], invert(a[1]) @ b[1]) for a, b in zip(kf, kf[1:])]
        graph = [(a, b, rel) for a, b, rel in graph if np.linalg.norm(rel[:3, 3]) > 2.0]
        if self.ins is None or not graph:
            return

        def step_error(shift):
            self.ins.shift = shift
            usable = [
                (a, b, rel)
                for a, b, rel in graph
                if self.ins.covers(a) and self.ins.covers(b)
            ]
            return np.median(
                [
                    abs(
                        np.linalg.norm(rel[:3, 3])
                        - np.linalg.norm(self.ins.relative(a, b)[:3, 3])
                    )
                    for a, b, rel in usable
                ]
            )

        shifts = np.arange(-0.15, 0.1501, 0.005)
        self.ins.shift = float(shifts[int(np.argmin([step_error(s) for s in shifts]))])

        def fit(pairs):
            # T_ins_body: the smallest rotation taking the mean body step direction onto the mean INS
            # one. Steps are nearly all straight ahead, so a full 3D fit (Kabsch) leaves the roll
            # about that direction unconstrained; a mount offset shows in yaw and pitch only.
            unit = lambda v: v / np.linalg.norm(v)
            body = unit(np.mean([unit(rel[:3, 3]) for _, _, rel in pairs], axis=0))
            ins = unit(
                np.mean(
                    [unit(self.ins.relative(a, b)[:3, 3]) for a, b, _ in pairs], axis=0
                )
            )
            axis = np.cross(body, ins)
            angle = np.arctan2(np.linalg.norm(axis), body @ ins)
            offset = np.eye(4)
            if np.linalg.norm(axis) > 1e-12:
                offset[:3, :3] = Rotation.from_rotvec(
                    unit(axis) * angle
                ).as_matrix()
            return offset

        usable = [
            (a, b, rel)
            for a, b, rel in graph
            if self.ins.covers(a) and self.ins.covers(b)
        ]
        self.offset["graph"] = fit(usable)
        visual = [(m.from_stamp, m.to_stamp, pose_matrix(m.measured)) for m in steps]
        visual = [
            (a, b, rel)
            for a, b, rel in visual
            if np.linalg.norm(rel[:3, 3]) > 2.0
            and self.ins.covers(a)
            and self.ins.covers(b)
        ]
        self.offset["visual"] = (
            fit(visual) if len(visual) > 10 else self.offset["graph"]
        )


# ---- reporting helpers ------------------------------------------------------------------------------


def stats(values, fmt="%.2f"):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return "-"
    return (fmt + " / " + fmt + " / " + fmt) % (
        np.median(values),
        np.percentile(values, 90),
        values.max(),
    )


def table(title, header, rows):
    print(f"\n{title}")
    if not rows:
        print("  (nothing)")
        return
    widths = [max(len(str(r[i])) for r in [header, *rows]) for i in range(len(header))]
    for row in [header, *rows]:
        print("  " + "  ".join(str(c).ljust(w) for c, w in zip(row, widths)))


def pair(m):
    return (min(m.from_index, m.to_index), max(m.from_index, m.to_index))


# ---- odometry ---------------------------------------------------------------------------------------


def odometry_rows(run: Run, spans=(100.0, 200.0)):
    steps = sorted(
        (m for m in run.measurements[VISUAL] if m.candidate_source == "consecutive"),
        key=lambda m: m.to_stamp,
    )
    steps = [
        m for m in steps if run.ins.covers(m.from_stamp) and run.ins.covers(m.to_stamp)
    ]
    if not steps:
        return [], []
    sources = (
        ("visual", "MapAnything", lambda m: pose_matrix(m.measured)),
        ("graph", "eidos graph", lambda m: pose_matrix(m.graph)),
    )
    step_rows = []
    for key, label, get in sources:
        v = []
        for m in steps:
            ref = run.ins_relative(m.from_stamp, m.to_stamp, key)
            dist = np.linalg.norm(ref[:3, 3])
            if dist < 2.0:
                continue
            est = get(m)
            trans, rot = error(est, ref)
            v.append((trans, rot, np.linalg.norm(est[:3, 3]) / dist, dist))
        v = np.array(v)
        step_rows.append(
            [
                run.name,
                label,
                len(v),
                "%.1f m" % np.median(v[:, 3]),
                stats(v[:, 0]),
                "%.1f" % (100 * np.median(v[:, 0] / v[:, 3])),
                stats(v[:, 1]),
                "%.3f [%.3f, %.3f]"
                % (np.median(v[:, 2]), *np.percentile(v[:, 2], [25, 75])),
            ]
        )

    chains = []
    for m in steps:
        if chains and chains[-1][-1].to_index == m.from_index:
            chains[-1].append(m)
        else:
            chains.append([m])
    chain_rows = []
    for span in spans:
        for key, label, get in sources:
            errs = []
            for chain in chains:
                start = 0
                while start < len(chain):
                    pose, end, travelled = np.eye(4), start, 0.0
                    while end < len(chain) and travelled < span:
                        pose = pose @ get(chain[end])
                        travelled += np.linalg.norm(
                            pose_matrix(chain[end].graph)[:3, 3]
                        )
                        end += 1
                    if travelled >= span:
                        ref = run.ins_relative(
                            chain[start].from_stamp, chain[end - 1].to_stamp, key
                        )
                        trans, rot = error(pose, ref)
                        errs.append((100 * trans / travelled, rot))
                    start = end
            errs = np.array(errs)
            if len(errs):
                chain_rows.append(
                    [
                        run.name,
                        f"{span:.0f} m",
                        label,
                        len(errs),
                        stats(errs[:, 0]),
                        stats(errs[:, 1]),
                    ]
                )
    return step_rows, chain_rows


# ---- loop closures ----------------------------------------------------------------------------------


def loop_rows(run: Run):
    visual = [
        m for m in run.measurements[VISUAL] if m.candidate_source != "consecutive"
    ]
    gicp = list(run.measurements[GICP])
    rows = []
    for label, group, key in (
        (
            "MapAnything, by distance",
            [m for m in visual if m.candidate_source == "distance"],
            "visual",
        ),
        (
            "MapAnything, by place recognition",
            [m for m in visual if m.candidate_source == "place_recognition"],
            "visual",
        ),
        ("GICP", gicp, "graph"),
    ):
        if not group:
            continue
        meas, drift, revisit = [], [], []
        for m in group:
            if not (run.ins.covers(m.from_stamp) and run.ins.covers(m.to_stamp)):
                continue
            ref = run.ins_relative(m.from_stamp, m.to_stamp, key)
            meas.append((*error(pose_matrix(m.measured), ref), m.accepted))
            drift.append(
                error(
                    pose_matrix(m.graph),
                    run.ins_relative(m.from_stamp, m.to_stamp, "graph"),
                )[0]
            )
            revisit.append(np.linalg.norm(ref[:3, 3]) < REVISIT_DIST_M + 15.0)
        meas, revisit = np.array(meas), np.array(revisit)
        acc = meas[:, 2] > 0
        reasons = {}
        for m in group:
            if not m.accepted:
                reasons[m.reject_reason] = reasons.get(m.reject_reason, 0) + 1
        rows.append(
            [
                run.name,
                label,
                len(group),
                int(acc.sum()),
                sum(m.added for m in group),
                stats(meas[acc, 0]) if acc.any() else "-",
                stats(meas[acc, 1]) if acc.any() else "-",
                stats(meas[:, 0]),
                "%.0f%%" % (100 * revisit.mean()),
                "%.1f" % np.median(drift),
                "%.2f" % np.median([m.duration for m in group]),
                ", ".join(f"{k}: {v}" for k, v in sorted(reasons.items())) or "-",
            ]
        )
    return rows


def versus_gicp_rows(run: Run, max_gicp_err: float = 2.0):
    """MapAnything against GICP on the pairs both measured, where GICP is within max_gicp_err of the INS."""
    gicp = {}
    for m in run.measurements[GICP]:
        if m.accepted and run.ins.covers(m.from_stamp) and run.ins.covers(m.to_stamp):
            if (
                error(
                    pose_matrix(m.measured),
                    run.ins_relative(m.from_stamp, m.to_stamp, "graph"),
                )[0]
                < max_gicp_err
            ):
                gicp[pair(m)] = m
    rows = []
    for label, source in (
        ("by distance", "distance"),
        ("by place recognition", "place_recognition"),
    ):
        diffs = []
        for m in run.measurements[VISUAL]:
            g = gicp.get(pair(m))
            if m.candidate_source != source or g is None:
                continue
            a, b = pair(m)
            v = (
                pose_matrix(m.measured)
                if (m.from_index, m.to_index) == (a, b)
                else invert(pose_matrix(m.measured))
            )
            w = (
                pose_matrix(g.measured)
                if (g.from_index, g.to_index) == (a, b)
                else invert(pose_matrix(g.measured))
            )
            diffs.append((*error(v, w), np.linalg.norm(w[:3, 3])))
        if diffs:
            d = np.array(diffs)
            rows.append(
                [
                    run.name,
                    label,
                    len(d),
                    "%.1f m" % np.median(d[:, 2]),
                    stats(d[:, 0]),
                    stats(d[:, 1]),
                    "%.1f" % (100 * np.median(d[:, 0] / np.maximum(d[:, 2], 1.0))),
                ]
            )
    return rows


# ---- final map ----------------------------------------------------------------------------------------


def revisit_pairs(run: Run):
    """For each keyframe, the nearest keyframe of an earlier pass (by the INS), if close enough."""
    keys = [k for k, (t, _) in run.keyframes.items() if run.ins.covers(t)]
    times = np.array([run.keyframes[k][0] for k in keys])
    ins_pos = np.array([run.ins.at(t)[:3, 3] for t in times])
    ins_yaw = np.array([np.arctan2(*run.ins.at(t)[[1, 0], 0]) for t in times])
    out = []
    for j in range(len(keys)):
        earlier = np.where(times < times[j] - REVISIT_GAP_S)[0]
        if not len(earlier):
            continue
        d = np.linalg.norm(ins_pos[earlier] - ins_pos[j], axis=1)
        i = earlier[int(np.argmin(d))]
        heading = abs((ins_yaw[i] - ins_yaw[j] + np.pi) % (2 * np.pi) - np.pi)
        if d.min() < REVISIT_DIST_M and heading < np.radians(60):
            out.append((keys[i], keys[j]))
    return out


def map_rows(run: Run):
    if not run.keyframes or run.ins is None:
        return None
    pairs = revisit_pairs(run)
    errs = []
    for a, b in pairs:
        ta, tb = run.keyframes[a][0], run.keyframes[b][0]
        errs.append(error(run.map_relative(a, b), run.ins_relative(ta, tb, "graph")))
    errs = np.array(errs) if errs else np.zeros((0, 2))
    kf = [(t, pose) for t, pose in run.keyframes.values() if run.ins.covers(t)]
    est = np.array([pose[:3, 3] for _, pose in kf])
    ref = np.array([run.ins.at(t)[:3, 3] for t, _ in kf])
    align = umeyama(est, ref)
    ate = np.linalg.norm(est @ align[:3, :3].T + align[:3, 3] - ref, axis=1)
    return [
        run.name,
        len(run.keyframes),
        len(pairs),
        stats(errs[:, 0]) if len(errs) else "-",
        stats(errs[:, 1]) if len(errs) else "-",
        "%.2f" % np.sqrt(np.mean(ate**2)),
        "%.2f" % ate.max(),
    ]


def trajectory_rows(run: Run, spans=(100.0, 200.0)):
    """The graph as a pose source: absolute error (final and live) and drift of the final keyframes."""
    if not run.keyframes or run.ins is None:
        return None
    kf = [(t, pose) for t, pose in run.keyframes.values() if run.ins.covers(t)]
    if len(kf) < 3:
        return None
    est = np.array([pose[:3, 3] for _, pose in kf])
    ref = np.array([run.ins.at(t)[:3, 3] for t, _ in kf])
    align = umeyama(est, ref)
    ate = np.linalg.norm(est @ align[:3, :3].T + align[:3, 3] - ref, axis=1)
    live = [(t, pose) for t, pose in sorted(run.live.items()) if run.ins.covers(t)]
    if live:
        est_live = np.array([pose[:3, 3] for _, pose in live])
        ref_live = np.array([run.ins.at(t)[:3, 3] for t, _ in live])
        ate_live = np.linalg.norm(
            est_live @ align[:3, :3].T + align[:3, 3] - ref_live, axis=1
        )
    travelled = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))]
    )
    drift = []
    for span in spans:
        errs = []
        for i in range(len(kf)):
            j = int(np.searchsorted(travelled, travelled[i] + span))
            if j >= len(kf):
                break
            rel = invert(kf[i][1]) @ kf[j][1]
            trans, rot = error(rel, run.ins_relative(kf[i][0], kf[j][0], "graph"))
            errs.append((100 * trans / (travelled[j] - travelled[i]), rot))
        errs = np.array(errs)
        drift += (
            [stats(errs[:, 0]), stats(errs[:, 1])] if len(errs) else ["-", "-"]
        )
    return [
        run.name,
        len(kf),
        "%.0f m" % travelled[-1],
        "%.2f / %.2f" % (np.sqrt(np.mean(ate**2)), ate.max()),
        "%.2f / %.2f" % (np.sqrt(np.mean(ate_live**2)), ate_live.max())
        if live
        else "-",
        *drift,
    ]


def describe_runs(runs):
    for run in runs:
        off = run.offset.get("graph")
        voff = run.offset.get("visual")
        print(
            f"  {run.name}: {len(run.keyframes)} keyframes; INS time offset {1e3 * run.ins.shift:+.0f} ms; "
            f"INS-to-body rotation {angle_deg(off[:3, :3]) if off is not None else float('nan'):.2f} deg (graph steps)"
            + (
                f", {angle_deg(voff[:3, :3]):.2f} deg (MapAnything steps)"
                if voff is not None
                else ""
            )
        )


def slam_report(runs):
    for run in runs:
        run.calibrate()
    print("Reference: INS (/novatel/oem7/odom), corrected per run for")
    describe_runs(runs)

    step_rows, chain_rows = [], []
    for run in runs:
        s, c = odometry_rows(run)
        step_rows += s
        chain_rows += c
    table(
        "1. Odometry, one keyframe step, against the INS",
        [
            "run",
            "source",
            "steps",
            "median step",
            "trans err m (med / p90 / max)",
            "% of step",
            "rot err deg (med / p90 / max)",
            "scale (med [IQR])",
        ],
        step_rows,
    )
    table(
        "2. Odometry chained over longer stretches (drift), against the INS",
        [
            "run",
            "stretch",
            "source",
            "stretches",
            "trans err % of distance (med / p90 / max)",
            "rot err deg (med / p90 / max)",
        ],
        chain_rows,
    )
    rows = []
    for run in runs:
        rows += loop_rows(run)
    table(
        "3. Loop closure measurements, against the INS",
        [
            "run",
            "source",
            "measured",
            "accepted",
            "added",
            "accepted: trans err m (med / p90 / max)",
            "accepted: rot err deg",
            "all: trans err m",
            "real revisits",
            "graph drift when measured (med m)",
            "s each",
            "rejected because",
        ],
        rows,
    )
    rows = []
    for run in runs:
        rows += versus_gicp_rows(run)
    table(
        "3b. MapAnything against GICP on the keyframe pairs both measured (GICP accepted and within 2 m of the INS)",
        [
            "run",
            "MapAnything candidates",
            "pairs",
            "median distance apart",
            "difference m (med / p90 / max)",
            "difference deg (med / p90 / max)",
            "% of distance",
        ],
        rows,
    )
    rows = [r for r in (map_rows(run) for run in runs) if r]
    table(
        f"4. Final map against the INS: revisit consistency (keyframes of different passes within {REVISIT_DIST_M:.0f} m) and absolute error after a rigid alignment",
        [
            "run",
            "keyframes",
            "revisit pairs",
            "revisit trans err m (med / p90 / max)",
            "revisit rot err deg (med / p90 / max)",
            "ATE RMSE m",
            "ATE max m",
        ],
        rows,
    )
    rows = [r for r in (trajectory_rows(run) for run in runs) if r]
    table(
        "5. The graph as a pose source, against the INS (absolute errors after one rigid alignment fitted on the final keyframes)",
        [
            "run",
            "keyframes",
            "driven",
            "final ATE m (RMSE / max)",
            "live ATE m (RMSE / max)",
            "100 m drift % (med / p90 / max)",
            "100 m rot deg",
            "200 m drift %",
            "200 m rot deg",
        ],
        rows,
    )


# ---- factor ablation --------------------------------------------------------------------------------


def describe_graph(run: Run) -> str:
    """Short label of a run's factors from its params.yaml (no file: the stack's default config)."""
    import yaml

    path = run.path / "params.yaml"
    if not path.exists():
        return "default (liso+gps/150m+gicp)"
    with open(path) as f:
        params = next(iter(yaml.safe_load(f).values()))["ros__parameters"]
    parts = []
    for plugin in params.get("factor_plugins", []):
        cfg = params.get(plugin, {})
        if plugin == "multiview_odometry_factor":
            if not cfg.get("add_factors", True):
                continue  # shadow: keyframes only
            opts = [
                o
                for o, on in (
                    ("depth", cfg.get("depth", {}).get("enabled", False)),
                    ("bias", cfg.get("bias", {}).get("enabled", False)),
                )
                if on
            ]
            parts.append("cam" + (f"({','.join(opts)})" if opts else ""))
        elif plugin == "liso_factor":
            parts.append("liso")
        elif plugin == "imu_factor":
            parts.append("imu")
        elif plugin == "gps_factor":
            parts.append("gps" + ("(outage)" if cfg.get("gps_topic") == "/eval/gps_fix" else ""))
        else:
            parts.append(plugin.replace("_factor", ""))
    return "+".join(parts)


def ablation_metrics(run: Run, windows=(), spans=(100.0, 1000.0)):
    """Pose-source metrics of one run against the INS; None without a saved map."""
    if not run.keyframes or run.ins is None:
        return None
    kf = [(t, pose) for t, pose in run.keyframes.values() if run.ins.covers(t)]
    if len(kf) < 3:
        return None
    out = {"run": run.name, "graph": describe_graph(run), "keyframes": len(kf)}
    est = np.array([pose[:3, 3] for _, pose in kf])
    ref = np.array([run.ins.at(t)[:3, 3] for t, _ in kf])
    travelled = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))])
    out["driven_m"] = float(travelled[-1])

    steps = []
    for (ta, a), (tb, b) in zip(kf, kf[1:]):
        ins = run.ins_relative(ta, tb, "graph")
        dist = np.linalg.norm(ins[:3, 3])
        if dist > 2.0:
            trans, rot = error(invert(a) @ b, ins)
            steps.append((100 * trans / dist, rot))
    steps = np.array(steps)
    out["step_pct"] = float(np.median(steps[:, 0]))
    out["step_rot_deg"] = float(np.median(steps[:, 1]))

    for span in spans:
        errs = []
        for i in range(len(kf)):
            j = int(np.searchsorted(travelled, travelled[i] + span))
            if j >= len(kf):
                break
            trans, rot = error(invert(kf[i][1]) @ kf[j][1], run.ins_relative(kf[i][0], kf[j][0], "graph"))
            errs.append((100 * trans / (travelled[j] - travelled[i]), rot))
        errs = np.array(errs)
        key = f"{span:.0f}m"
        out[f"drift_{key}_pct"] = float(np.median(errs[:, 0])) if len(errs) else float("nan")
        out[f"drift_{key}_pct_p90"] = float(np.percentile(errs[:, 0], 90)) if len(errs) else float("nan")
        out[f"drift_{key}_rot_deg"] = float(np.median(errs[:, 1])) if len(errs) else float("nan")

    align = umeyama(est, ref)
    ate = np.linalg.norm(est @ align[:3, :3].T + align[:3, 3] - ref, axis=1)
    out["final_ate_rmse"] = float(np.sqrt(np.mean(ate**2)))
    out["final_ate_max"] = float(ate.max())
    live = [(t, pose) for t, pose in sorted(run.live.items()) if run.ins.covers(t)]
    live_t = np.array([t for t, _ in live])
    if live:
        est_live = np.array([pose[:3, 3] for _, pose in live])
        ref_live = np.array([run.ins.at(t)[:3, 3] for t, _ in live])
        ate_live = np.linalg.norm(est_live @ align[:3, :3].T + align[:3, 3] - ref_live, axis=1)
        out["live_median"] = float(np.median(ate_live))
        out["live_p95"] = float(np.percentile(ate_live, 95))
        out["live_max"] = float(ate_live.max())
    origin = run.ins.t[0]  # the INS starts with the bag
    kf_t = np.array([t for t, _ in kf])
    for a, b in windows:
        inside = (live_t >= origin + a) & (live_t < origin + b) if live else np.zeros(0, bool)
        key = f"outage_{a:.0f}_{b:.0f}"
        if inside.any():
            out[f"{key}_live_max"] = float(ate_live[inside].max())
            out[f"{key}_live_end"] = float(ate_live[np.flatnonzero(inside)[-1]])
        kin = (kf_t >= origin + a) & (kf_t < origin + b)
        if kin.any():
            out[f"{key}_final_max"] = float(ate[kin].max())
    out["live_series"] = (
        [(float(t - origin), float(e)) for t, e in zip(live_t, ate_live)] if live else []
    )
    out["final_series"] = [(float(t - origin), float(e)) for t, e in zip(kf_t, ate)]
    return out


def ablation_report(runs, windows, json_path=None):
    for run in runs:
        run.calibrate()
    print("Reference: INS (/novatel/oem7/odom), corrected per run for")
    describe_runs(runs)
    results = [m for m in (ablation_metrics(run, windows) for run in runs) if m]
    fmt = lambda v, f="%.2f": "-" if v is None or v != v else f % v
    rows = [
        [
            m["run"],
            m["graph"],
            m["keyframes"],
            fmt(m["step_pct"], "%.1f") + " / " + fmt(m["step_rot_deg"]),
            fmt(m["drift_100m_pct"]) + " / " + fmt(m["drift_100m_rot_deg"]),
            fmt(m["drift_1000m_pct"]) + " / " + fmt(m["drift_1000m_rot_deg"]),
            fmt(m["final_ate_rmse"]) + " / " + fmt(m["final_ate_max"]),
            " / ".join(fmt(m.get(k)) for k in ("live_median", "live_p95", "live_max")),
        ]
        + [
            fmt(m.get(f"outage_{a:.0f}_{b:.0f}_live_max")) + " / " + fmt(m.get(f"outage_{a:.0f}_{b:.0f}_live_end"))
            for a, b in windows
        ]
        for m in results
    ]
    table(
        "Factor ablation: the graph as a pose source against the INS (absolute errors after one rigid "
        "alignment fitted on the final keyframes; drift = median over all stretches of that length)",
        [
            "run",
            "factors",
            "keyframes",
            "step % / deg",
            "100 m drift % / deg",
            "1 km drift % / deg",
            "final ATE m (RMSE / max)",
            "live m (median / p95 / max)",
        ]
        + [f"live in GPS outage {a:.0f}-{b:.0f} s (max / at end)" for a, b in windows],
        rows,
    )
    if json_path:
        import json

        with open(json_path, "w") as f:
            json.dump(results, f)
        print(f"\nwrote {json_path}")


# ---- relocalization ---------------------------------------------------------------------------------


def truth_in_map(map_run: Run, query_ins: Trajectory, t: float, radius: float = 60.0):
    """The vehicle's pose at t in the prior map's frame.

    The map's keyframes near the INS position at t are aligned rigidly to their own INS positions
    (map run), which maps the query run's INS pose into the map frame; using only nearby keyframes
    absorbs the map's own drift. The rotation offset between INS and body frame is the map run's.
    """
    here = query_ins.at(t)
    kf = [
        (kt, pose) for kt, pose in map_run.keyframes.values() if map_run.ins.covers(kt)
    ]
    ins_pos = np.array([map_run.ins.at(kt)[:3, 3] for kt, _ in kf])
    near = np.linalg.norm(ins_pos - here[:3, 3], axis=1) < radius
    if near.sum() < 5:
        return None, int(near.sum())
    map_pos = np.array([pose[:3, 3] for (kt, pose), n in zip(kf, near) if n])
    align = umeyama(ins_pos[near], map_pos)  # map <- INS world
    offset = map_run.offset.get("graph", np.eye(4))
    return align @ here @ offset, int(near.sum())


def relocalization_report(map_run: Run, runs):
    map_run.calibrate()
    rows = []
    for run in runs:
        if run.ins is None:
            rows.append([run.name, "-", "no INS recorded", "-", "-", "-"])
            continue
        run.ins.shift = map_run.ins.shift
        if not run.initial:
            rows.append([run.name, "-", "never started tracking", "-", "-", "-"])
            continue
        t0, pose0 = run.initial[-1]
        truth, n = truth_in_map(map_run, run.ins, t0)
        if truth is None:
            rows.append(
                [
                    run.name,
                    "%.1f" % (t0 - run.ins.t[0]),
                    f"place not in the map ({n} keyframes near)",
                    "-",
                    "-",
                    "-",
                ]
            )
            continue
        start = error(pose0, truth)
        dyaw = np.degrees(np.arctan2(*(invert(truth) @ pose0)[[1, 0], 0]))
        est = "-"
        if run.estimates:
            te, pe = run.estimates[-1]
            truth_e, _ = truth_in_map(map_run, run.ins, te)
            if truth_e is not None:
                est = "%.2f m / %.2f deg" % error(pe, truth_e)
        after = "-"
        if run.liso is not None:
            later = [t for t in run.liso.t if t > t0 + 5.0]
            if later:
                sample = later[:: max(1, len(later) // 30)]
                errs = []
                for t in sample:
                    truth_t, _ = truth_in_map(map_run, run.ins, t)
                    if truth_t is not None:
                        errs.append(error(run.liso.at(t), truth_t))
                if errs:
                    errs = np.array(errs)
                    after = "%.2f m / %.2f deg" % (
                        np.median(errs[:, 0]),
                        np.median(errs[:, 1]),
                    )
        rows.append(
            [
                run.name,
                "%.1f" % (t0 - run.ins.t[0]),
                "%.2f m / %.2f deg (yaw %+.2f)" % (start[0], start[1], dyaw),
                est,
                after,
                str(n),
            ]
        )
    table(
        f"5. Relocalization in the map of '{map_run.name}', against the INS mapped into that map",
        [
            "run",
            "s of bag until tracking",
            "start pose error",
            "visual estimate error (at its images)",
            "tracking error from 5 s after start (med)",
            "map keyframes used for truth",
        ],
        rows,
    )
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="command", required=True)
    slam = sub.add_parser("slam")
    slam.add_argument("runs", type=Path, nargs="+")
    reloc = sub.add_parser("relocalization")
    reloc.add_argument(
        "--map", type=Path, required=True, help="run that saved the prior map"
    )
    reloc.add_argument("runs", type=Path, nargs="+")
    abl = sub.add_parser("ablation")
    abl.add_argument(
        "--outage",
        default="",
        help="GPS outage windows of the runs, 'start:end[,start:end]' in s after the bag start",
    )
    abl.add_argument("--json", type=Path, help="also write the metrics and error series here")
    abl.add_argument("runs", type=Path, nargs="+")
    args = parser.parse_args(argv)
    if args.command == "slam":
        slam_report([Run(p) for p in args.runs])
        return 0
    if args.command == "ablation":
        windows = [tuple(float(v) for v in w.split(":")) for w in args.outage.split(",") if w]
        ablation_report([Run(p) for p in args.runs], windows, args.json)
        return 0
    return relocalization_report(Run(args.map), [Run(p) for p in args.runs])


if __name__ == "__main__":
    sys.exit(main())
