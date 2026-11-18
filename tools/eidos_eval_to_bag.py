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
"""Outputs of tools/eidos_visual_eval.sh runs as topics to view next to the drive (host tool).

For every run, under /eval/<run name>/, in the frame of a reference eidos run recorded in the drive
bag (default: its /world_modeling/slam/odometry, frame "map"), so that all of them overlay:

  odometry     nav_msgs/Odometry: the pose eidos published (the live estimate), as recorded
  live_path    nav_msgs/Path: those poses, one per keyframe, growing
  final_path   nav_msgs/Path: the final optimized keyframes (from the run's saved map), growing
  map          sensor_msgs/PointCloud2: the run's accumulated map (slam/visualization/map), as recorded

and once, under /eval/ins_path, the INS (/novatel/oem7/odom) at the first run's keyframes, also
in the reference frame. Each run is placed by one rigid transform fitted on its final keyframes
against the reference trajectory (printed).

    tools/eidos_eval_to_bag.py --drive <drive bag> --out <output dir> <run dir> [<run dir> ...]

Merge the output into the drive bag with `mcap merge`. Runs on the host with rosbags, numpy, scipy.
"""

import argparse
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation

TS = get_typestore(Stores.ROS2_JAZZY)
SLAM_ODOM = "/world_modeling/slam/odometry"
MAP = "/world_modeling/slam/visualization/map"
INS = "/novatel/oem7/odom"
INDEX_MASK = (1 << 56) - 1


def stamp_ns(header) -> int:
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def pose_matrix(pose) -> np.ndarray:
    q, p = pose.orientation, pose.position
    out = np.eye(4)
    out[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    out[:3, 3] = [p.x, p.y, p.z]
    return out


def umeyama(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """Rigid 4x4 T minimising sum |T src_i - dst_i|^2."""
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    u, _, vt = np.linalg.svd((src - cs).T @ (dst - cd))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    out = np.eye(4)
    out[:3, :3] = r
    out[:3, 3] = cd - r @ cs
    return out


def interpolate(times: np.ndarray, positions: np.ndarray, t: np.ndarray) -> np.ndarray:
    return np.stack([np.interp(t, times, positions[:, k]) for k in range(3)], axis=1)


def keyframes(map_file: Path):
    """(stamps ns, 4x4 poses) of a run's final keyframes, in time order."""
    db = sqlite3.connect(f"file:{map_file}?mode=ro&immutable=1", uri=True)
    rows = sorted(
        db.execute("SELECT time, x, y, z, roll, pitch, yaw FROM keyframes"),
        key=lambda r: r[0],
    )
    db.close()
    stamps, poses = [], []
    for t, x, y, z, roll, pitch, yaw in rows:
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
        pose[:3, 3] = [x, y, z]
        stamps.append(int(round(t * 1e9)))
        poses.append(pose)
    return np.array(stamps), np.array(poses)


def track(bag: Path, topic: str):
    """(stamps ns, 4x4 poses) of an Odometry topic."""
    stamps, poses = [], []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            stamps.append(stamp_ns(msg.header))
            poses.append(pose_matrix(msg.pose.pose))
    return np.array(stamps), np.array(poses)


# ---- messages ---------------------------------------------------------------------------------------


def header(ns: int, frame: str):
    return TS.types["std_msgs/msg/Header"](
        stamp=TS.types["builtin_interfaces/msg/Time"](
            sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000)
        ),
        frame_id=frame,
    )


def pose_msg(t: np.ndarray):
    q = Rotation.from_matrix(t[:3, :3]).as_quat()
    return TS.types["geometry_msgs/msg/Pose"](
        position=TS.types["geometry_msgs/msg/Point"](
            x=float(t[0, 3]), y=float(t[1, 3]), z=float(t[2, 3])
        ),
        orientation=TS.types["geometry_msgs/msg/Quaternion"](
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        ),
    )


def path_msg(ns: int, frame: str, stamps, poses):
    pose_stamped = TS.types["geometry_msgs/msg/PoseStamped"]
    return TS.types["nav_msgs/msg/Path"](
        header=header(ns, frame),
        poses=[
            pose_stamped(header=header(s, frame), pose=pose_msg(p))
            for s, p in zip(stamps, poses)
        ],
    )


def transform_cloud(msg, t: np.ndarray, frame: str):
    """The cloud with x, y, z (float32 at offsets 0, 4, 8) moved by t; other fields untouched."""
    offsets = {f.name: f.offset for f in msg.fields}
    if (offsets.get("x"), offsets.get("y"), offsets.get("z")) != (0, 4, 8):
        raise ValueError("expected float32 x, y, z at offsets 0, 4, 8")
    n = msg.width * msg.height
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, msg.point_step).copy()
    xyz = data[:, :12].copy().view("<f4").reshape(n, 3).astype(np.float64)
    data[:, :12] = (
        (xyz @ t[:3, :3].T + t[:3, 3]).astype("<f4").view(np.uint8).reshape(n, 12)
    )
    msg.data = data.reshape(-1)
    msg.header.frame_id = frame
    return msg


# ---- main -------------------------------------------------------------------------------------------


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--drive", type=Path, required=True, help="bag of the drive")
    parser.add_argument("--out", type=Path, required=True, help="output bag directory")
    parser.add_argument(
        "--reference",
        default=SLAM_ODOM,
        help="Odometry topic in the drive bag whose frame the runs are placed in",
    )
    parser.add_argument(
        "--no-ins-path",
        action="store_true",
        help="do not write /eval/ins_path (e.g. adding a run to a bag that has it)",
    )
    parser.add_argument("runs", type=Path, nargs="+")
    args = parser.parse_args(argv)

    ref_t, ref_poses = track(args.drive, args.reference)
    if len(ref_t) < 2:
        print(f"no {args.reference} in {args.drive}")
        return 1
    order = np.argsort(ref_t)
    ref_t, ref_pos = ref_t[order], ref_poses[order][:, :3, 3]
    frame = "map"
    with AnyReader([args.drive]) as reader:
        conn = next(c for c in reader.connections if c.topic == args.reference)
        _, _, raw = next(reader.messages(connections=[conn]))
        frame = reader.deserialize(raw, conn.msgtype).header.frame_id

    writer = Writer(args.out, version=8, storage_plugin=StoragePlugin.MCAP)
    writer.open()
    conns = {}

    def write(topic, msgtype, ns, msg):
        if topic not in conns:
            conns[topic] = writer.add_connection(topic, msgtype, typestore=TS)
        writer.write(conns[topic], ns, TS.serialize_cdr(msg, msgtype))

    ins_written = args.no_ins_path
    for run in args.runs:
        name = run.name
        kf_t, kf_poses = keyframes(run / "map.map")
        inside = (kf_t >= ref_t[0]) & (kf_t <= ref_t[-1])
        align = umeyama(
            kf_poses[inside][:, :3, 3],
            interpolate(ref_t, ref_pos, kf_t[inside]),
        )
        residual = np.linalg.norm(
            kf_poses[inside][:, :3, 3] @ align[:3, :3].T
            + align[:3, 3]
            - interpolate(ref_t, ref_pos, kf_t[inside]),
            axis=1,
        )
        print(
            f"{name}: {len(kf_t)} keyframes; placed by {np.linalg.norm(align[:3, 3]):.2f} m, "
            f"{np.degrees(Rotation.from_matrix(align[:3, :3]).magnitude()):.2f} deg; "
            f"then {np.sqrt(np.mean(residual**2)):.2f} m RMSE from {args.reference}"
        )
        final = np.array([align @ p for p in kf_poses])
        ns = f"/eval/{name}"

        live_t, live_poses = [], []
        with AnyReader([run / "out"]) as reader:
            wanted = [c for c in reader.connections if c.topic in (SLAM_ODOM, MAP, INS)]
            ins = {"t": [], "p": []}
            for conn, log_ns, raw in reader.messages(connections=wanted):
                msg = reader.deserialize(raw, conn.msgtype)
                if conn.topic == INS:
                    ins["t"].append(stamp_ns(msg.header))
                    ins["p"].append(pose_matrix(msg.pose.pose)[:3, 3])
                    continue
                if conn.topic == MAP:
                    write(
                        f"{ns}/map",
                        conn.msgtype,
                        log_ns,
                        transform_cloud(msg, align, frame),
                    )
                    continue
                pose = align @ pose_matrix(msg.pose.pose)
                t = stamp_ns(msg.header)
                msg.header.frame_id = frame
                msg.pose.pose = pose_msg(pose)
                write(f"{ns}/odometry", conn.msgtype, log_ns, msg)
                if not live_t or t > live_t[-1]:  # first message of each keyframe
                    live_t.append(t)
                    live_poses.append(pose)
                    write(
                        f"{ns}/live_path",
                        "nav_msgs/msg/Path",
                        t,
                        path_msg(t, frame, live_t, live_poses),
                    )
        for i, t in enumerate(kf_t):
            write(
                f"{ns}/final_path",
                "nav_msgs/msg/Path",
                int(t),
                path_msg(int(t), frame, kf_t[: i + 1], final[: i + 1]),
            )

        if not ins_written and len(ins["t"]) > 1:
            # The INS at this run's keyframes, placed by a rigid fit onto them
            ins_t = np.array(ins["t"])
            ins_p = np.array(ins["p"])
            covered = (kf_t >= ins_t[0]) & (kf_t <= ins_t[-1])
            ins_at = interpolate(ins_t, ins_p, kf_t[covered])
            fit = umeyama(ins_at, final[covered][:, :3, 3])
            poses = []
            for p in ins_at:
                pose = np.eye(4)
                pose[:3, 3] = fit[:3, :3] @ p + fit[:3, 3]
                poses.append(pose)
            stamps = kf_t[covered]
            for i, t in enumerate(stamps):
                write(
                    "/eval/ins_path",
                    "nav_msgs/msg/Path",
                    int(t),
                    path_msg(int(t), frame, stamps[: i + 1], poses[: i + 1]),
                )
            ins_written = True
    writer.close()
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
