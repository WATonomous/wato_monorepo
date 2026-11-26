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
"""Recorded replay (tracker output + ego pose) -> scenes/<slice>/NNNN.npz, one sample per 0.5 s.
usage (in the perception container):
  python3 make_scenes.py <run_mcap> <slice> <scenes_out_dir> [<preds_out_dir>] [--ego TOPIC]
--ego: nav_msgs/Odometry or geometry_msgs/PoseStamped giving the ego pose in a world-fixed frame; default
/novatel/oem7/odom (frame odom), as prediction_ml runs on the car. Tracks in another frame (lidar_cc on main) are
moved into it the way prediction_ml does: ego pose at the track stamp times the static mount from /tf_static.
The optional preds dir gets the MTR predictions prediction_ml published for exactly those sample times.
Self-check (no ROS needed): python3 make_scenes.py --selftest"""

import pathlib
import sys
from types import SimpleNamespace

import numpy as np

import pbscene

STRIDE_S, LEAD_S, TAIL_S = (
    0.5,
    1.0,
    2.0,
)  # first sample 1 s after the first tracker frame, last 2 s before the end
TRACKED, SEEDS, TF_STATIC = (
    "/perception/detections_3D_tracked",
    "/world_modeling/world_object_seeds",
    "/tf_static",
)
MAX_GAP_S = 0.2  # prediction_ml odometry_max_gap_s
POSE_CHILD = "base_footprint"  # a PoseStamped ego (/localization/pose) is the base_footprint pose
META = (
    "model=prediction_ml (live, as recorded); checkpoint=see run; train=see run; map=no"
)
IDENTITY = (np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))


def read(uri, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        t.name: get_message(t.type)
        for t in r.get_all_topics_and_types()
        if t.name in topics
    }
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while r.has_next():
        topic, data, _ = r.read_next()
        yield topic, deserialize_message(data, types[topic])


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


# A pose is (t [3], q [4] = x y z w), meaning parent_from_child.
def qmul(a, b):
    (ax, ay, az, aw), (bx, by, bz, bw) = a, b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def qrot(q, v):
    return qmul(qmul(q, [*v, 0.0]), q * [-1, -1, -1, 1])[:3]


def compose(a, b):
    return a[0] + qrot(a[1], b[0]), qmul(a[1], b[1])


def inverse(a):
    qi = a[1] * [-1, -1, -1, 1]
    return -qrot(qi, a[0]), qi


def static_from(edges, a, b):
    """edges: child -> (parent, parent_from_child). Returns a_from_b, or None when a and b share no root."""

    def root_from(f):
        pose, root = IDENTITY, f
        while root in edges:
            parent, p_from_c = edges[root]
            pose, root = compose(p_from_c, pose), parent
        return root, pose

    (ra, pa), (rb, pb) = root_from(a), root_from(b)
    return compose(inverse(pa), pb) if ra == rb else None


def pose_at(ts, P, Q, t):
    """Ego pose at t from sorted samples, as prediction_ml's odometryAt: linear position, normalized-lerp rotation,
    None when the nearer bracketing sample is more than MAX_GAP_S away."""
    i = int(np.searchsorted(ts, t))
    if i == 0 or i == len(ts):
        j = 0 if i == 0 else len(ts) - 1
        return (P[j], Q[j]) if abs(ts[j] - t) <= MAX_GAP_S else None
    ta, tb = ts[i - 1], ts[i]
    if min(t - ta, tb - t) > MAX_GAP_S:
        return None
    r = (t - ta) / (tb - ta)
    qb = Q[i] if np.dot(Q[i - 1], Q[i]) >= 0 else -Q[i]
    q = Q[i - 1] + r * (qb - Q[i - 1])
    return P[i - 1] + r * (P[i] - P[i - 1]), q / np.linalg.norm(q)


def selftest():
    yaw90 = np.array([0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    edges = {
        "base_link": (
            "base_footprint",
            (np.array([0, 0, 0.3]), np.array([0.0, 0, 0, 1])),
        ),
        "lidar_cc": ("base_link", (np.array([1.0, 0, 1.5]), yaw90)),
    }
    t, q = static_from(edges, "base_link", "lidar_cc")
    assert np.allclose(t, [1, 0, 1.5]) and np.allclose(q, yaw90)
    t, _ = static_from(edges, "lidar_cc", "base_footprint")  # the inverse direction
    assert np.allclose(t, [0, 1, -1.8])
    assert static_from(edges, "lidar_cc", "nowhere") is None
    ts, P, Q = (
        np.array([0.0, 0.1]),
        np.array([[0.0, 0, 0], [1.0, 0, 0]]),
        np.array([[0, 0, 0, 1.0], yaw90]),
    )
    p, q = pose_at(ts, P, Q, 0.05)
    assert np.allclose(p, [0.5, 0, 0])
    assert np.isclose(
        pbscene.quat_yaw(SimpleNamespace(**dict(zip("xyzw", q)))), np.pi / 4
    )
    assert pose_at(ts, P, Q, 0.35) is None and pose_at(ts, P, Q, 0.25) is not None
    box = compose(
        (np.array([10.0, 0, 0]), yaw90), (np.array([1.0, 0, 0]), IDENTITY[1])
    )  # 1 m ahead of an ego facing +y
    assert np.allclose(box[0], [10, 1, 0]) and np.allclose(box[1], yaw90)
    print("make_scenes self-check ok")


def main(args):
    ego_topic = "/novatel/oem7/odom"
    if "--ego" in args:
        i = args.index("--ego")
        ego_topic = args.pop(i + 1)
        args.pop(i)
    uri, slice_name, out = args[0], args[1], pathlib.Path(args[2])
    mtr_out = pathlib.Path(args[3]) if len(args) > 3 else None

    # Pass 1: ego poses and static mounts, so each track stamp is interpolated between samples on both sides.
    edges, ego, world, ego_child = {}, [], None, None
    for topic, m in read(uri, {ego_topic, TF_STATIC}):
        if topic == TF_STATIC:
            for tf in m.transforms:
                tt, rr = tf.transform.translation, tf.transform.rotation
                edges[tf.child_frame_id] = (
                    tf.header.frame_id,
                    (np.array([tt.x, tt.y, tt.z]), np.array([rr.x, rr.y, rr.z, rr.w])),
                )
        else:
            world, ego_child = (
                m.header.frame_id,
                getattr(m, "child_frame_id", "") or POSE_CHILD,
            )
            ego.append(
                (stamp(m.header), m.pose.pose if hasattr(m.pose, "pose") else m.pose)
            )
    if not ego:
        sys.exit(
            f"no {ego_topic} in {uri}: pass --ego with the bag's odometry or pose topic"
        )
    ego.sort(key=lambda e: e[0])
    ts = np.array([e[0] for e in ego])
    P = np.array([[e[1].position.x, e[1].position.y, e[1].position.z] for e in ego])
    Q = np.array(
        [
            [
                e[1].orientation.x,
                e[1].orientation.y,
                e[1].orientation.z,
                e[1].orientation.w,
            ]
            for e in ego
        ]
    )

    tr = pbscene.Tracks()
    for t, pose in ego:
        sec = int(t)
        header = SimpleNamespace(
            stamp=SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))
        )
        tr.add_ego(SimpleNamespace(header=header, pose=pose))

    mounts = {}

    def world_from(
        frame, t
    ):  # world_from_frame at t, or None without an ego pose near t
        if frame == world:
            return IDENTITY
        if frame not in mounts:
            mounts[frame] = static_from(edges, ego_child, frame)
            if mounts[frame] is None:
                sys.exit(f"/tf_static has no {ego_child} <- {frame} chain")
        e = pose_at(ts, P, Q, t)
        return None if e is None else compose(e, mounts[frame])

    # Pass 2: tracks and live predictions, moved into the ego's world frame.
    frames, mtr, dropped = (
        [],
        {},
        0,
    )  # mtr: (source stamp ms, track id) -> (traj [K,80,2], conf [K])
    for topic, m in read(uri, {TRACKED, SEEDS}):
        T = world_from(m.header.frame_id, stamp(m.header))
        if T is None:
            dropped += topic == TRACKED
            continue
        if topic == TRACKED:
            for d in m.detections:
                c = d.bbox.center
                p, q = compose(
                    T,
                    (
                        np.array([c.position.x, c.position.y, c.position.z]),
                        np.array(
                            [
                                c.orientation.x,
                                c.orientation.y,
                                c.orientation.z,
                                c.orientation.w,
                            ]
                        ),
                    ),
                )
                c.position.x, c.position.y, c.position.z = p
                c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w = q
            frames.append(tr.add_detections(m))
            continue
        for o in m.objects:
            p = o.predictions
            if len(p) < 2 or not all(
                x.poses for x in p
            ):  # 1 prediction = the node's fallback
                continue
            t0 = stamp(p[0].header)
            c = o.detection.bbox.center.position
            start = (T[0] + qrot(T[1], [c.x, c.y, c.z]))[
                :2
            ]  # the track now, so the 0.1 s step can be interpolated
            # Waypoints carry their own stamps (0.2 s apart on the car): resample to the scorer's 0.1 s grid.
            horizon = min(stamp(x.poses[-1].header) for x in p[:6]) - t0
            grid = 0.1 * np.arange(1, int(horizon * 10 + 1e-6) + 1)
            modes = []
            for x in p[:6]:
                tw = np.array([0.0] + [stamp(w.header) - t0 for w in x.poses])
                pw = [
                    T[0]
                    + qrot(
                        T[1], [w.pose.position.x, w.pose.position.y, w.pose.position.z]
                    )
                    for w in x.poses
                ]
                xy = np.vstack([start, np.array(pw)[:, :2]])
                modes.append(
                    np.stack(
                        [np.interp(grid, tw, xy[:, 0]), np.interp(grid, tw, xy[:, 1])],
                        1,
                    )
                )
            key = (round(t0 * 1e3), int(o.detection.id))
            mtr.setdefault(
                key,
                (
                    np.array(modes, np.float32),
                    np.array([x.conf for x in p[:6]], np.float32),
                ),
            )
    if not frames:
        sys.exit(f"no tracks with a {ego_topic} sample within {MAX_GAP_S} s")

    frames = np.array(sorted(set(frames)))
    grid = np.arange(frames[0] + LEAD_S, frames[-1] - TAIL_S, STRIDE_S)
    times = sorted(
        {frames[np.abs(frames - g).argmin()] for g in grid}
    )  # the tracker frame nearest each grid time
    out.mkdir(parents=True, exist_ok=True)
    n_agents = n_targets = n_mtr = 0
    for k, t in enumerate(times):
        s = tr.scene(t, slice_name)
        np.savez_compressed(out / f"{k:04d}.npz", **s)
        n_agents += len(s["track_ids"])
        n_targets += int(s["targets"].sum())
        got = [i for i in s["track_ids"] if (round(t * 1e3), int(i)) in mtr]
        if mtr_out is not None and got:
            mtr_out.mkdir(parents=True, exist_ok=True)
            K = max(len(mtr[(round(t * 1e3), int(i))][1]) for i in got)
            pad = lambda a: np.concatenate([a, np.repeat(a[-1:], K - len(a), 0)])  # noqa: E731  fewer modes: repeat the last
            np.savez_compressed(
                mtr_out / f"{k:04d}.npz",
                track_ids=np.array(got, np.int64),
                traj=np.stack([pad(mtr[(round(t * 1e3), int(i))][0]) for i in got]),
                probs=np.stack([pad(mtr[(round(t * 1e3), int(i))][1]) for i in got]),
                latency_ms=np.nan,
                meta=META,
            )
            n_mtr += len(set(got) & set(s["track_ids"][s["targets"]]))
    print(
        f"{slice_name} (frame {world}, ego {ego_topic}): {len(frames)} tracker frames over {frames[-1] - frames[0]:.1f} s, "
        f"{dropped} dropped (no ego pose within {MAX_GAP_S} s) -> {len(times)} samples, {n_agents} agents, "
        f"{n_targets} target samples, {len({i for i in tr.d if i != pbscene.EGO_ID})} track ids; "
        f"MTR predicted {n_mtr} of the target samples"
    )


if __name__ == "__main__":
    selftest() if sys.argv[1:] == ["--selftest"] else main(sys.argv[1:])
