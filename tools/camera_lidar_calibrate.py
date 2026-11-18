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
"""Camera rotations and the camera time offset against the lidar, on a recorded drive (host tool).

At instants while driving, MapAnything predicts every pixel's depth from the images alone. The lidar
scans around each image are moved to the image's time (per-firing times, vehicle motion from the
INS) and projected into the camera. Wrong camera rotations and a wrong time offset put lidar points
on the wrong pixels, where their depth disagrees with the predicted one. This fits:

  * one rotation correction per camera (in its optical frame), so base_link -> camera changes;
  * one time offset for all cameras: image taken at its stamp + offset (stamps are arrival times).

The lidar extrinsic is taken as given (from --tf), and so are the camera positions. MapAnything's
depth is only used for its shape: each view's overall scale is removed. Instants alternate between
fitting and testing; the errors reported are on the test instants.

    tools/camera_lidar_calibrate.py --bag <drive> --tf <tf_static.yaml> --out <overrides.yaml>

--tf is a static TF set as written by tools/tf_static_override.py extract; --out gets the corrected
base_link -> camera transforms in the same format, to add to the overrides. Needs torch, mapanything,
rosbags, opencv, scipy (the map_anything host venv).
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation, Slerp

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(REPO / "src/perception/map_anything"),
    str(REPO / "src/perception/visual_odometry"),
]
from map_anything.model import MapAnythingModel, centred_view  # noqa: E402

PANO8 = [f"camera_pano_{c}" for c in ("nn", "ne", "ee", "se", "ss", "sw", "ww", "nw")]
INS = "/novatel/oem7/odom"
SWEEP = 0.05  # s per lidar revolution (20 Hz)
MIN_DEPTH, MAX_DEPTH = 2.0, 40.0
SAME_SWEEP = 0.03  # s: a lidar point is used if fired this close to the image time


# ---- transforms -------------------------------------------------------------------------------------


def matrix(xyz, rpy) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    t[:3, 3] = xyz
    return t


def invert(t: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return out


class StaticTf:
    """A static TF tree (list of parent/child/xyz/rpy) with lookups between any two frames."""

    def __init__(self, entries):
        self.parent = {e["child"]: (e["parent"], matrix(e["xyz"], e["rpy"])) for e in entries}
        self.entries = {(e["parent"], e["child"]): e for e in entries}

    def _root_from(self, frame):
        t = np.eye(4)
        while frame in self.parent:
            parent, t_pc = self.parent[frame]
            t = t_pc @ t
            frame = parent
        return frame, t

    def lookup(self, target, source) -> np.ndarray:
        """T_target_source."""
        root_a, t_a = self._root_from(target)
        root_b, t_b = self._root_from(source)
        if root_a != root_b:
            raise ValueError(f"{target} and {source} are not connected")
        return invert(t_a) @ t_b


class Motion:
    """INS poses with interpolation: relative motion of the vehicle between two times."""

    def __init__(self, stamps, poses):
        order = np.argsort(stamps)
        self.t = np.asarray(stamps)[order]
        self.p = np.asarray(poses)[order]
        self.slerp = Slerp(self.t, Rotation.from_matrix(self.p[:, :3, :3]))

    def at(self, t):
        t = np.atleast_1d(t)
        out = np.repeat(np.eye(4)[None], len(t), axis=0)
        out[:, :3, :3] = self.slerp(np.clip(t, self.t[0], self.t[-1])).as_matrix()
        for k in range(3):
            out[:, k, 3] = np.interp(t, self.t, self.p[:, k, 3])
        return out

    def speed(self, t):
        a, b = self.at([t - 0.1, t + 0.1])
        return np.linalg.norm(b[:3, 3] - a[:3, 3]) / 0.2


# ---- data -------------------------------------------------------------------------------------------


def stamp(header) -> float:
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_motion(bag: Path) -> Motion:
    stamps, poses = [], []
    with AnyReader([bag]) as reader:
        conns = [c for c in reader.connections if c.topic == INS]
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            q, p = m.pose.pose.orientation, m.pose.pose.position
            t = np.eye(4)
            t[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            t[:3, 3] = [p.x, p.y, p.z]
            stamps.append(stamp(m.header))
            poses.append(t)
    return Motion(stamps, poses)


def read_instants(bag: Path, instants, cameras, lidar_topic):
    """Per instant: each camera's nearest image (stamp, jpeg) and the lidar scans around it."""
    want_img = {f"/{c}/image_rect_compressed": c for c in cameras}
    want_info = {f"/{c}/camera_info": c for c in cameras}
    images = [dict() for _ in instants]
    scans = [list() for _ in instants]
    intrinsics = {}
    instants = np.asarray(instants)
    with AnyReader([bag]) as reader:
        conns = [
            c
            for c in reader.connections
            if c.topic in want_img or c.topic in want_info or c.topic == lidar_topic
        ]
        for conn, log_ns, raw in reader.messages(connections=conns):
            t_log = log_ns * 1e-9
            near = np.where(np.abs(instants - t_log) < 0.3)[0]
            if conn.topic in want_info:
                cam = want_info[conn.topic]
                if cam not in intrinsics:
                    m = reader.deserialize(raw, conn.msgtype)
                    intrinsics[cam] = np.array(m.p, dtype=float).reshape(3, 4)[:, :3]
                continue
            if not len(near):
                continue
            m = reader.deserialize(raw, conn.msgtype)
            t = stamp(m.header)
            for i in near:
                if conn.topic == lidar_topic:
                    if -0.2 < t - instants[i] < 0.25:
                        n = m.width * m.height
                        a = np.frombuffer(bytes(m.data), np.uint8).reshape(n, m.point_step)
                        xyz = a[:, :12].copy().view("<f4").reshape(m.height, m.width, 3)
                        scans[i].append((t, xyz.astype(np.float64)))
                else:
                    cam = want_img[conn.topic]
                    best = images[i].get(cam)
                    if best is None or abs(t - instants[i]) < abs(best[0] - instants[i]):
                        images[i][cam] = (t, bytes(m.data))
    return images, scans, intrinsics


# ---- the measurement --------------------------------------------------------------------------------


class View:
    """One camera image at one instant: predicted depth and the lidar points to project into it."""

    def __init__(self, camera, image_stamp, depth, valid, k, cam_from_base, firing_points):
        self.camera = camera
        self.image_stamp = image_stamp
        self.depth = depth  # (h, w) predicted z
        self.valid = valid
        self.k = k  # model-resolution intrinsics
        self.cam_from_base = cam_from_base  # nominal (current) extrinsic
        self.firing_points = firing_points  # list of (firing times (N,), base-frame points at firing (N, 3))


def lidar_in_base(scans, base_from_lidar):
    """All scans' points in base_link at their own firing time: (times (M,), points (M, 3))."""
    times, points = [], []
    for scan_stamp, xyz in scans:
        firings = xyz.shape[0]
        t_firing = scan_stamp - SWEEP + SWEEP * (np.arange(firings) + 0.5) / firings
        p = xyz.reshape(-1, 3)
        t = np.repeat(t_firing, xyz.shape[1])
        ok = np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) > 1.0)
        times.append(t[ok])
        points.append(p[ok] @ base_from_lidar[:3, :3].T + base_from_lidar[:3, 3])
    return np.concatenate(times), np.concatenate(points)


_DESKEWED = {}


def deskewed(view: View, motion: Motion, offset: float):
    """The view's lidar points in base_link at the image's time (stamp + offset), cached."""
    key = (id(view), round(offset, 4))
    if key not in _DESKEWED:
        image_time = view.image_stamp + offset
        t, p = view.firing_points
        use = np.abs(t - image_time) < SAME_SWEEP
        t, p = t[use], p[use]
        # move each point from its firing time to the image time: base(image)^-1 * base(firing)
        uniq, inverse = np.unique(np.round(t, 4), return_inverse=True)
        rel = np.einsum("ij,njk->nik", invert(motion.at([image_time])[0]), motion.at(uniq))[inverse]
        _DESKEWED[key] = np.einsum("nij,nj->ni", rel[:, :3, :3], p) + rel[:, :3, 3]
    return _DESKEWED[key]


def residuals(view: View, motion: Motion, offset: float, correction: np.ndarray):
    """log(lidar depth) - log(predicted depth) at the lidar points' pixels, view scale removed."""
    p_now = deskewed(view, motion, offset)
    if len(p_now) < 50:
        return np.zeros(0)
    cam = (p_now @ view.cam_from_base[:3, :3].T + view.cam_from_base[:3, 3]) @ correction.T
    z = cam[:, 2]
    ok = (z > MIN_DEPTH) & (z < MAX_DEPTH)
    cam, z = cam[ok], z[ok]
    u = view.k[0, 0] * cam[:, 0] / z + view.k[0, 2]
    v = view.k[1, 1] * cam[:, 1] / z + view.k[1, 2]
    h, w = view.depth.shape
    ui, vi = np.floor(u).astype(int), np.floor(v).astype(int)
    inside = (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    ui, vi, z = ui[inside], vi[inside], z[inside]
    # nearest lidar point per pixel (occlusion from the slightly different viewpoint)
    order = np.argsort(z)
    pix = vi[order] * w + ui[order]
    _, first = np.unique(pix, return_index=True)
    keep = order[first]
    ui, vi, z = ui[keep], vi[keep], z[keep]
    good = view.valid[vi, ui] & (view.depth[vi, ui] > MIN_DEPTH)
    r = np.log(z[good]) - np.log(view.depth[vi, ui][good])
    return r - np.median(r) if len(r) else r


def cost(r: np.ndarray, clip: float = 0.15) -> float:
    """Mean truncated absolute log-depth error."""
    return float(np.mean(np.minimum(np.abs(r), clip))) if len(r) else np.nan


def correction_matrix(x):
    return Rotation.from_rotvec(np.radians(x)).as_matrix()


# ---- main -------------------------------------------------------------------------------------------


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--tf", type=Path, required=True, help="static TF set (YAML)")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--cameras", nargs="+", default=PANO8)
    parser.add_argument("--lidar-topic", default="/lidar_cc/velodyne_points")
    parser.add_argument("--lidar-frame", default="lidar_cc")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--instants", type=int, default=40)
    parser.add_argument("--min-speed", type=float, default=4.0, help="m/s")
    parser.add_argument("--checkpoint", default="facebook/map-anything")
    parser.add_argument("--crop-bottom", type=int, default=110)
    args = parser.parse_args(argv)

    tf = StaticTf(yaml.safe_load(args.tf.read_text()))
    base_from_lidar = tf.lookup(args.base_frame, args.lidar_frame)
    motion = read_motion(args.bag)
    span = np.linspace(motion.t[0] + 5, motion.t[-1] - 5, 2000)
    moving = span[[motion.speed(t) > args.min_speed for t in span]]
    instants = moving[np.linspace(0, len(moving) - 1, args.instants).astype(int)]
    print(f"{len(instants)} instants while moving (> {args.min_speed} m/s); reading the bag ...", flush=True)
    images, scans, intrinsics = read_instants(args.bag, instants, args.cameras, args.lidar_topic)

    model = MapAnythingModel(args.checkpoint, "cuda")
    views = {c: [] for c in args.cameras}  # camera -> [(instant index, View)]
    started = time.perf_counter()
    for i, t in enumerate(instants):
        if any(c not in images[i] for c in args.cameras) or not scans[i]:
            continue
        rgb_views = []
        for c in args.cameras:
            bgr = cv2.imdecode(np.frombuffer(images[i][c][1], np.uint8), cv2.IMREAD_COLOR)
            rgb_views.append(
                centred_view(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), intrinsics[c], args.crop_bottom)
            )
        dense = model.reconstruct(rgb_views)
        fired = lidar_in_base(scans[i], base_from_lidar)
        for c, d in zip(args.cameras, dense):
            camera_from_world = invert(d.world_from_camera)
            z = (d.points.reshape(-1, 3) @ camera_from_world[:3, :3].T + camera_from_world[:3, 3])[
                :, 2
            ].reshape(d.points.shape[:2])
            views[c].append(
                (
                    i,
                    View(
                        c,
                        images[i][c][0],
                        z,
                        d.valid,
                        d.intrinsics,
                        tf.lookup(c, args.base_frame),
                        fired,
                    ),
                )
            )
        print(f"  instant {i + 1}/{len(instants)} ({time.perf_counter() - started:.0f} s)", flush=True)

    def split(c, fit):
        return [v for i, v in views[c] if (i % 2 == 0) == fit]

    def total(offset, corrections, fit):
        r = [
            residuals(v, motion, offset, corrections[c])
            for c in args.cameras
            for v in split(c, fit)
        ]
        return cost(np.concatenate(r))

    identity = {c: np.eye(3) for c in args.cameras}
    offsets = np.arange(-0.15, 0.1501, 0.01)

    def best_offset(corrections):
        costs = [total(o, corrections, True) for o in offsets]
        fine = np.arange(offsets[np.argmin(costs)] - 0.01, offsets[np.argmin(costs)] + 0.0101, 0.002)
        fine_costs = [total(o, corrections, True) for o in fine]
        return float(fine[np.argmin(fine_costs)]), costs

    offset, coarse = best_offset(identity)
    print("time offset scan (fit instants, nominal rotations):")
    print("  " + "  ".join(f"{o:+.2f}:{c:.4f}" for o, c in zip(offsets[::3], coarse[::3])))
    print(f"  best {offset * 1e3:+.0f} ms")

    corrections = dict(identity)
    for _ in range(2):
        for c in args.cameras:
            fit_views = split(c, True)

            def f(x, c=c, fit_views=fit_views):
                m = correction_matrix(x)
                return cost(np.concatenate([residuals(v, motion, offset, m) for v in fit_views]))

            res = minimize(
                f,
                np.zeros(3),
                method="Nelder-Mead",
                options={"initial_simplex": np.vstack([np.zeros(3), 0.7 * np.eye(3)]), "xatol": 0.02, "fatol": 1e-5},
            )
            corrections[c] = correction_matrix(res.x)
        offset, _ = best_offset(corrections)
    print(f"time offset with fitted rotations: {offset * 1e3:+.0f} ms (image taken at stamp + offset)")

    print("\ntest instants: mean truncated |log depth error| and share within 5 %")
    print(
        f"  {'camera':<14} {'nominal, 0 ms':>16} {'offset only':>16} {'fitted':>16}   "
        "correction in base_link (roll, pitch, yaw deg)"
    )
    out = []
    for c in args.cameras:
        test = split(c, False)
        r0 = np.concatenate([residuals(v, motion, 0.0, np.eye(3)) for v in test])
        rt = np.concatenate([residuals(v, motion, offset, np.eye(3)) for v in test])
        r1 = np.concatenate([residuals(v, motion, offset, corrections[c]) for v in test])
        base_from_cam = invert(tf.lookup(c, args.base_frame))
        new = base_from_cam.copy()
        new[:3, :3] = base_from_cam[:3, :3] @ corrections[c].T
        delta = Rotation.from_matrix(new[:3, :3] @ base_from_cam[:3, :3].T).as_euler("xyz", degrees=True)
        print(
            f"  {c:<14} {cost(r0):.4f} ({100 * np.mean(np.abs(r0) < 0.05):3.0f} %)  "
            f"{cost(rt):.4f} ({100 * np.mean(np.abs(rt) < 0.05):3.0f} %)  "
            f"{cost(r1):.4f} ({100 * np.mean(np.abs(r1) < 0.05):3.0f} %)   "
            f"({delta[0]:+.2f}, {delta[1]:+.2f}, {delta[2]:+.2f})"
        )
        parent = tf.entries.get((args.base_frame, c))
        if parent is None:
            print(f"    {c} is not a child of {args.base_frame}; not written")
            continue
        out.append(
            {
                "parent": args.base_frame,
                "child": c,
                "xyz": [float(v) for v in parent["xyz"]],
                "rpy": [round(float(v), 6) for v in Rotation.from_matrix(new[:3, :3]).as_euler("xyz")],
            }
        )
    args.out.write_text(
        f"# Camera rotations fitted against {args.lidar_frame} on {args.bag.name}; camera time offset "
        f"{offset * 1e3:+.0f} ms (image at stamp + offset)\n"
        + yaml.safe_dump(out, default_flow_style=None, sort_keys=False)
    )
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
