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
"""Score MapAnything relative rig poses against a reference trajectory (host tool).

Pairs of instants (a, b) are sampled, the camera images of both go through MapAnything in one pass,
the rig motion is fitted from the predicted camera poses (map_anything.rig, both fits) and compared
with the reference's motion of the same frame.

Datasets:
  * a rosbag2 bag (``--bag``) with CompressedImage + CameraInfo per camera, TF extrinsics and a
    nav_msgs/Odometry reference. Defaults suit the WATO pano cameras with the Novatel INS; for the
    nuScenes bags use ``--cameras CAM_FRONT ... --ref-topic /odom --tf-topics /tf``.
  * a Boreas sequence directory (``--boreas``): one camera, post-processed Applanix camera poses.

Pair sampling (``--mode``):
  * ``consecutive``: an anchor every ``--anchor-spacing`` metres of travel, paired with the instant
    ``--baselines`` metres further along the path. Only needs a reference that is accurate over
    seconds (an INS without RTK qualifies).
  * ``revisit``: each anchor is paired with an instant at least ``--min-gap`` seconds away whose
    position is about a baseline away, i.e. what a loop closure would compare. Needs a globally
    accurate reference (Boreas); the heading difference of every pair is reported.

Cameras without hardware sync: every camera contributes its frame nearest to the first camera's
stamp. The reference motion is taken between the first camera's two stamps.

Needs a Python environment with torch, mapanything, rosbags, opencv and scipy (see the map_anything
package README). Reads the map_anything and visual_odometry sources from this checkout.

    tools/map_anything_pairs_eval.py --bag <sensor bag> [--csv pairs.csv]
    tools/map_anything_pairs_eval.py --boreas <sequence dir> --mode revisit
"""

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from scipy.spatial.transform import Rotation, Slerp

REPO = Path(__file__).resolve().parents[1]
for package in ("map_anything", "visual_odometry"):
    sys.path.insert(0, str(REPO / "src" / "perception" / package))

from map_anything.model import (  # noqa: E402
    DEFAULT_CHECKPOINT,
    MapAnythingModel,
    View,
    centred_view,
)
from map_anything.rig import (  # noqa: E402
    relative_rig_pose,
    relative_rig_pose_per_camera,
)
from visual_odometry.geometry import (  # noqa: E402
    invert,
    make_transform,
    rotation_angle,
)
from visual_odometry.pipeline import load_extrinsics  # noqa: E402
from visual_odometry.tf_tree import StaticTfTree  # noqa: E402

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
PANO = [f"camera_pano_{c}" for c in ("nn", "nw", "ne", "ss", "se", "sw")]
FITS = ("joint", "per_camera")


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class Reference:
    """Reference trajectory of the rig frame, interpolated in time."""

    def __init__(self, stamps_ns, poses):
        self.stamps = np.asarray(stamps_ns, dtype=np.int64)
        self.positions = np.array([p[:3, 3] for p in poses])
        # UTM is large: keep positions relative to the first, which pose() then omits
        self.origin = self.positions[0].copy()
        self.positions -= self.origin
        rotations = Rotation.from_matrix([p[:3, :3] for p in poses])
        self._slerp = Slerp(self.stamps * 1e-9, rotations)
        forward = rotations.as_matrix()[:, :2, 0]
        self.headings = np.arctan2(forward[:, 1], forward[:, 0])
        steps = np.linalg.norm(np.diff(self.positions, axis=0), axis=1)
        self.travelled = np.concatenate([[0.0], np.cumsum(steps)])

    def pose(self, t_ns: int) -> np.ndarray:
        out = np.eye(4)
        out[:3, :3] = self._slerp(t_ns * 1e-9).as_matrix()
        for axis in range(3):
            out[axis, 3] = np.interp(t_ns, self.stamps, self.positions[:, axis])
        return out

    def time_at_distance(self, distance: float) -> int:
        return int(np.interp(distance, self.travelled, self.stamps))

    def revisit_time(
        self, t_ns: int, baseline: float, min_gap_s: float, tolerance: float
    ) -> Optional[int]:
        """Stamp at least min_gap_s away whose position is nearest to ``baseline`` m from the pose at t_ns."""
        here = self.pose(t_ns)[:3, 3]
        distance = np.linalg.norm(self.positions - here, axis=1)
        miss = np.abs(distance - baseline)
        miss[np.abs(self.stamps - t_ns) < min_gap_s * 1e9] = np.inf
        best = int(np.argmin(miss))
        return int(self.stamps[best]) if miss[best] <= tolerance * baseline else None


@dataclass
class Frames:
    """Images of one instant. ``stamp_ns`` is the first camera's stamp."""

    stamp_ns: int
    images: Dict[str, np.ndarray]  # BGR


class BagSource:
    """A rosbag2 bag: random access to camera frame sets, calibration and the reference odometry."""

    def __init__(self, args):
        self.reader = AnyReader([args.bag], default_typestore=TYPESTORE)
        self.reader.open()
        self.cameras = args.cameras
        self.rig_frame = args.rig_frame
        self._image_conns = {
            cam: self._connections(args.image_topic.format(camera=cam))
            for cam in self.cameras
        }
        tf = StaticTfTree()
        for msg in self._read(args.tf_topics):
            for tr in msg.transforms:
                t, q = tr.transform.translation, tr.transform.rotation
                tf.add(
                    tr.header.frame_id,
                    tr.child_frame_id,
                    make_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]),
                )
        overrides = load_extrinsics(args.extrinsics, self.rig_frame)
        self.intrinsics, self.rig_from_camera = {}, {}
        for cam in self.cameras:
            info = next(self._read([args.info_topic.format(camera=cam)]))
            self.intrinsics[cam] = np.array(info.p, dtype=float).reshape(3, 4)[:, :3]
            self.rig_from_camera[cam] = (
                overrides[cam]
                if cam in overrides
                else tf.lookup(self.rig_frame, info.header.frame_id or cam)
            )
        self.reference = self._reference(args.ref_topic)
        self.description = (
            f"{args.bag.name}, reference {args.ref_topic}, extrinsics "
            f"{'TF + ' + args.extrinsics if overrides else 'TF'}"
        )

    def _connections(self, topic):
        conns = [c for c in self.reader.connections if c.topic == topic]
        if not conns:
            raise KeyError(f"topic {topic} is not in the bag")
        return conns

    def _read(self, topics, start=None, stop=None):
        conns = [c for t in topics for c in self._connections(t)]
        for conn, _, raw in self.reader.messages(conns, start=start, stop=stop):
            yield self.reader.deserialize(raw, conn.msgtype)

    def _reference(self, topic: str) -> Reference:
        stamps, poses, child = [], [], None
        for msg in self._read([topic]):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            stamps.append(stamp_ns(msg.header))
            poses.append(make_transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
            child = msg.child_frame_id
        if child != self.rig_frame:
            raise ValueError(
                f"{topic} moves {child!r}; expected the rig frame {self.rig_frame!r}"
            )
        return Reference(stamps, poses)

    def frames(self, t_ns: int, window_ns: int = 80_000_000) -> Optional[Frames]:
        """Frame set nearest to t_ns, or None if a camera has no frame within the window."""
        target, ref_stamp, images = t_ns, None, {}
        for cam in self.cameras:
            candidates = []
            for conn, _, raw in self.reader.messages(
                self._image_conns[cam], start=t_ns - window_ns, stop=t_ns + window_ns
            ):
                msg = self.reader.deserialize(raw, conn.msgtype)
                candidates.append((stamp_ns(msg.header), msg.data))
            if not candidates:
                return None
            stamp, data = min(candidates, key=lambda c: abs(c[0] - target))
            if ref_stamp is None:
                ref_stamp = target = stamp
            images[cam] = cv2.imdecode(
                np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR
            )
        return Frames(ref_stamp, images)


class BoreasSource:
    """A Boreas sequence directory: one rectified camera with post-processed Applanix poses.

    The rig frame is the camera itself. ``applanix/camera_poses.csv`` holds the camera pose in ENU
    at every image: C_enu_camera = (Rz(heading) Ry(pitch) Rx(roll))^T.
    """

    def __init__(self, args):
        root = args.boreas
        self.cameras = ["camera"]
        self.intrinsics = {
            "camera": np.loadtxt(root / "calib" / "P_camera.txt")[:3, :3]
        }
        self.rig_from_camera = {"camera": np.eye(4)}
        rows = np.loadtxt(
            root / "applanix" / "camera_poses.csv", delimiter=",", skiprows=1
        )
        self._stamps_us = rows[:, 0].astype(np.int64)
        self._dir = root / "camera"
        poses = []
        for row in rows:
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_euler("ZYX", row[[9, 8, 7]]).as_matrix().T
            pose[:3, 3] = row[1:4]
            poses.append(pose)
        self.reference = Reference(self._stamps_us * 1000, poses)
        self.description = f"Boreas {root.name}, reference applanix/camera_poses.csv"

    def frames(self, t_ns: int) -> Optional[Frames]:
        stamp_us = self._stamps_us[
            int(np.argmin(np.abs(self._stamps_us * 1000 - t_ns)))
        ]
        image = cv2.imread(str(self._dir / f"{stamp_us}.png"), cv2.IMREAD_COLOR)
        return (
            None if image is None else Frames(int(stamp_us) * 1000, {"camera": image})
        )


def add_source_arguments(parser: argparse.ArgumentParser) -> None:
    """Arguments that select the dataset; see open_source()."""
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path, help="rosbag2 directory or .mcap file")
    source.add_argument("--boreas", type=Path, help="Boreas sequence directory")
    parser.add_argument(
        "--cameras", nargs="+", default=PANO, help="first = stamp reference"
    )
    parser.add_argument("--image-topic", default="/{camera}/image_rect_compressed")
    parser.add_argument("--info-topic", default="/{camera}/camera_info")
    parser.add_argument("--tf-topics", nargs="+", default=["/tf_static"])
    parser.add_argument("--ref-topic", default="/novatel/oem7/odom")
    parser.add_argument("--rig-frame", default="base_link")
    parser.add_argument(
        "--extrinsics",
        default="",
        help="calibrate_pairs YAML overriding the bag's TF extrinsics (default: TF only)",
    )


def open_source(args):
    return BoreasSource(args) if args.boreas else BagSource(args)


def heading_class(diff_deg: float) -> str:
    return "same" if diff_deg < 45 else "opposite" if diff_deg > 135 else "crossing"


def summarize(rows: List[dict], fits) -> None:
    groups = sorted({(r["baseline"], r["heading"]) for r in rows})
    for fit in fits:
        print(
            f"\nfit: {fit}\n"
            "baseline heading   pairs  trans err m (med / p90)  % of dist  direction deg  "
            "rot err deg (med / p90)  scale (med, IQR)   spread m"
        )
        for baseline, heading in groups:
            sel = [
                r for r in rows if (r["baseline"], r["heading"]) == (baseline, heading)
            ]
            col = lambda k: np.array([r[k] for r in sel])  # noqa: E731
            trans, rot, scale = (
                col(f"{fit}_trans_err"),
                col(f"{fit}_rot_err_deg"),
                col(f"{fit}_scale"),
            )
            q25, q75 = np.percentile(scale, [25, 75])
            print(
                f"{baseline:6.1f}m  {heading:8s} {len(sel):5d}    {np.median(trans):7.3f} / "
                f"{np.percentile(trans, 90):7.3f}       {100 * np.median(trans / col('ref_dist')):6.1f}"
                f"       {np.median(col(f'{fit}_dir_err_deg')):6.2f}        {np.median(rot):6.2f} / "
                f"{np.percentile(rot, 90):6.2f}      {np.median(scale):5.3f} [{q25:.2f}, {q75:.2f}]"
                f"   {np.median(col(f'{fit}_trans_spread')):6.3f}"
            )
    print(
        f"\ninference: median {np.median([r['infer_ms'] for r in rows]):.0f} ms per pair"
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    add_source_arguments(parser)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument(
        "--mode", choices=("consecutive", "revisit"), default="consecutive"
    )
    parser.add_argument(
        "--baselines", type=float, nargs="+", default=[2, 5, 10, 20, 40]
    )
    parser.add_argument(
        "--anchor-spacing", type=float, default=100.0, help="m of travel"
    )
    parser.add_argument("--max-anchors", type=int, default=0, help="0 = all")
    parser.add_argument(
        "--min-gap", type=float, default=60.0, help="revisit: s between a and b"
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.3,
        help="revisit: accepted relative miss of the baseline distance",
    )
    parser.add_argument(
        "--crop-bottom",
        type=int,
        default=0,
        help="rows showing the car itself (roof rack / hood) to cut from the bottom of every image "
        "(images are always cropped so that the principal point is centred)",
    )
    parser.add_argument("--csv", type=Path, help="write one row per pair")
    args = parser.parse_args(argv)

    data = open_source(args)
    ref, cameras = data.reference, data.cameras
    extrinsics = [data.rig_from_camera[cam] for cam in cameras]
    fits = FITS if len(cameras) > 1 else ("joint",)
    print(
        f"{data.description}: {ref.travelled[-1]:.0f} m in "
        f"{(ref.stamps[-1] - ref.stamps[0]) * 1e-9:.0f} s; cameras {cameras}; "
        f"checkpoint {args.checkpoint}; mode {args.mode}"
    )
    model = MapAnythingModel(args.checkpoint)

    def views(frames: Frames) -> List[View]:
        return [
            centred_view(
                cv2.cvtColor(frames.images[cam], cv2.COLOR_BGR2RGB),
                data.intrinsics[cam],
                args.crop_bottom,
            )
            for cam in cameras
        ]

    margin = max(args.baselines) if args.mode == "consecutive" else 0.0
    anchors = np.arange(
        args.anchor_spacing, ref.travelled[-1] - margin, args.anchor_spacing
    )
    if args.max_anchors:
        anchors = anchors[: args.max_anchors]
    rows = []
    for anchor in anchors:
        t_a = ref.time_at_distance(anchor)
        frames_a = data.frames(t_a)
        if frames_a is None:
            continue
        views_a = views(frames_a)
        for baseline in args.baselines:
            if args.mode == "consecutive":
                t_b = ref.time_at_distance(anchor + baseline)
            else:
                t_b = ref.revisit_time(t_a, baseline, args.min_gap, args.tolerance)
            frames_b = None if t_b is None else data.frames(t_b)
            if frames_b is None:
                continue
            prediction = model.predict(views_a + views(frames_b))
            n = len(cameras)
            poses_a, poses_b = (
                prediction.world_from_cameras[:n],
                prediction.world_from_cameras[n:],
            )
            estimates = {
                "joint": relative_rig_pose(poses_a, extrinsics, poses_b, extrinsics),
                "per_camera": relative_rig_pose_per_camera(
                    poses_a, poses_b, extrinsics
                ),
            }
            truth = invert(ref.pose(frames_a.stamp_ns)) @ ref.pose(frames_b.stamp_ns)
            ref_dist = float(np.linalg.norm(truth[:3, 3]))
            row = {
                "anchor_m": float(anchor),
                "baseline": float(baseline),
                "dt": (frames_b.stamp_ns - frames_a.stamp_ns) * 1e-9,
                "ref_dist": ref_dist,
                "ref_rot_deg": float(np.degrees(rotation_angle(truth[:3, :3]))),
                "heading": heading_class(
                    float(np.degrees(rotation_angle(truth[:3, :3])))
                ),
                "confidence": float(np.mean(prediction.confidences)),
                "infer_ms": prediction.infer_ms,
            }
            for fit in fits:
                estimate = estimates[fit].rig_a_from_rig_b
                error = invert(truth) @ estimate
                est_dist = float(np.linalg.norm(estimate[:3, 3]))
                cos = estimate[:3, 3] @ truth[:3, 3] / max(est_dist * ref_dist, 1e-9)
                row.update(
                    {
                        f"{fit}_trans_err": float(np.linalg.norm(error[:3, 3])),
                        f"{fit}_dir_err_deg": float(
                            np.degrees(np.arccos(np.clip(cos, -1, 1)))
                        ),
                        f"{fit}_rot_err_deg": float(
                            np.degrees(rotation_angle(error[:3, :3]))
                        ),
                        f"{fit}_scale": est_dist / max(ref_dist, 1e-9),
                        f"{fit}_trans_spread": estimates[fit].trans_spread,
                    }
                )
            rows.append(row)
        print(f"anchor {anchor:6.0f} m: {len(rows)} pairs so far", flush=True)

    if not rows:
        print("no pairs evaluated")
        return 1
    summarize(rows, fits)
    if args.csv:
        with open(args.csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"wrote {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
