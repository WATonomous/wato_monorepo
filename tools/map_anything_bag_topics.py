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
"""MapAnything output for a recorded drive, as ROS topics in a separate mcap (host tool).

Every ``--period`` seconds of the drive, the camera images of that instant and of the instant
``--period`` earlier go through MapAnything in one pass (images and intrinsics only). From it:

  /map_anything/local_cloud   sensor_msgs/PointCloud2, rig frame: what the cameras see now, in 3D
  /map_anything/map_chunks    sensor_msgs/PointCloud2, map frame: the same, placed by the reference
                              odometry (in Foxglove, give it a decay time to watch the map grow)
  /map_anything/map           sensor_msgs/PointCloud2, map frame: all chunks, voxel-downsampled,
                              published once at the first instant
  /map_anything/odometry      nav_msgs/Odometry, map frame: MapAnything's own rig motion chained
                              from instant to instant, starting at the reference pose (its drift
                              is the gap to the reference)
  /map_anything/path          nav_msgs/Path of the same
  /map_anything/reference_path  nav_msgs/Path of the reference odometry at the same instants

All messages are stamped (and logged) at the first camera's image stamp of their instant, so the
output plays in sync with the input bag: open both in Foxglove. The reference odometry (default:
eidos LISO, map -> base_footprint) only places the map chunks and starts the chained path.

Needs a Python environment with torch, mapanything, rosbags, opencv and scipy (see the map_anything
package README). Reads the map_anything and visual_odometry sources from this checkout.

    tools/map_anything_bag_topics.py --bag <bag> --out <output dir> [--start 0 --duration 0]
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

import map_anything_pairs_eval as ev
from map_anything.model import DEFAULT_CHECKPOINT, MapAnythingModel, centred_view
from map_anything.rig import relative_rig_pose_per_camera, rig_pose
from visual_odometry.geometry import invert, split_transform

TS = get_typestore(Stores.ROS2_HUMBLE)
PANO8 = [f"camera_pano_{c}" for c in ("nn", "ne", "ee", "se", "ss", "sw", "ww", "nw")]
NS = "/map_anything"


# ---- messages ---------------------------------------------------------------------------------------


def header(stamp_ns: int, frame: str):
    Time = TS.types["builtin_interfaces/msg/Time"]
    Header = TS.types["std_msgs/msg/Header"]
    return Header(
        stamp=Time(
            sec=int(stamp_ns // 1_000_000_000), nanosec=int(stamp_ns % 1_000_000_000)
        ),
        frame_id=frame,
    )


def pose_msg(t: np.ndarray):
    Point = TS.types["geometry_msgs/msg/Point"]
    Quaternion = TS.types["geometry_msgs/msg/Quaternion"]
    Pose = TS.types["geometry_msgs/msg/Pose"]
    translation, q = split_transform(t)
    return Pose(
        position=Point(
            x=float(translation[0]), y=float(translation[1]), z=float(translation[2])
        ),
        orientation=Quaternion(
            x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
        ),
    )


def cloud_msg(stamp_ns: int, frame: str, points: np.ndarray, colors: np.ndarray):
    """PointCloud2 with x, y, z (float32) and rgb (packed in a float32, as RViz and Foxglove read it)."""
    PointField = TS.types["sensor_msgs/msg/PointField"]
    PointCloud2 = TS.types["sensor_msgs/msg/PointCloud2"]
    data = np.empty(len(points), dtype=[("xyz", "<f4", 3), ("rgb", "<u4")])
    data["xyz"] = points
    c = colors.astype(np.uint32)
    data["rgb"] = (c[:, 0] << 16) | (c[:, 1] << 8) | c[:, 2]
    fields = [
        PointField(name="x", offset=0, datatype=7, count=1),
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
        PointField(name="rgb", offset=12, datatype=7, count=1),
    ]
    return PointCloud2(
        header=header(stamp_ns, frame),
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * len(points),
        data=np.frombuffer(data.tobytes(), dtype=np.uint8),
        is_dense=True,
    )


def path_msg(stamp_ns: int, frame: str, stamps, poses):
    PoseStamped = TS.types["geometry_msgs/msg/PoseStamped"]
    Path_ = TS.types["nav_msgs/msg/Path"]
    return Path_(
        header=header(stamp_ns, frame),
        poses=[
            PoseStamped(header=header(s, frame), pose=pose_msg(p))
            for s, p in zip(stamps, poses)
        ],
    )


def odometry_msg(stamp_ns: int, frame: str, child: str, pose: np.ndarray):
    Odometry = TS.types["nav_msgs/msg/Odometry"]
    PoseWithCovariance = TS.types["geometry_msgs/msg/PoseWithCovariance"]
    TwistWithCovariance = TS.types["geometry_msgs/msg/TwistWithCovariance"]
    Twist = TS.types["geometry_msgs/msg/Twist"]
    Vector3 = TS.types["geometry_msgs/msg/Vector3"]
    zero = Vector3(x=0.0, y=0.0, z=0.0)
    return Odometry(
        header=header(stamp_ns, frame),
        child_frame_id=child,
        pose=PoseWithCovariance(pose=pose_msg(pose), covariance=np.zeros(36)),
        twist=TwistWithCovariance(
            twist=Twist(linear=zero, angular=zero), covariance=np.zeros(36)
        ),
    )


# ---- point clouds -----------------------------------------------------------------------------------


def voxel_downsample(points: np.ndarray, colors: np.ndarray, size: float):
    """One point per voxel (the first), keeping its colour."""
    if not len(points):
        return points, colors
    keys = np.floor(points / size).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return points[keep], colors[keep]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ev.add_source_arguments(parser)
    parser.set_defaults(
        cameras=PANO8,
        rig_frame="base_footprint",
        ref_topic="/world_modeling/liso/odometry",
    )
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--period", type=float, default=1.0, help="s between instants")
    parser.add_argument(
        "--start", type=float, default=0.0, help="s into the drive (reference time)"
    )
    parser.add_argument("--duration", type=float, default=0.0, help="s; 0 = to the end")
    parser.add_argument(
        "--crop-bottom", type=int, default=110, help="image rows of roof rack to cut"
    )
    parser.add_argument(
        "--max-depth", type=float, default=40.0, help="m from the camera"
    )
    parser.add_argument(
        "--local-voxel", type=float, default=0.1, help="m, local cloud downsampling"
    )
    parser.add_argument(
        "--chunk-voxel", type=float, default=0.2, help="m, map chunk downsampling"
    )
    parser.add_argument(
        "--map-voxel", type=float, default=0.25, help="m, accumulated map downsampling"
    )
    parser.add_argument("--map-frame", default="map")
    parser.add_argument(
        "--out", type=Path, required=True, help="output rosbag2 directory (mcap)"
    )
    args = parser.parse_args(argv)

    if args.out.exists():
        print(f"{args.out} exists; remove it or choose another name")
        return 1
    data = ev.open_source(args)
    ref, cameras = data.reference, data.cameras
    extrinsics = [data.rig_from_camera[cam] for cam in cameras]
    t0 = ref.stamps[0] + int(args.start * 1e9)
    t1 = (
        ref.stamps[-1]
        if args.duration <= 0
        else min(ref.stamps[-1], t0 + int(args.duration * 1e9))
    )
    period = int(args.period * 1e9)
    print(
        f"{data.description}: {len(cameras)} cameras, {(t1 - t0) * 1e-9:.0f} s every {args.period} s"
    )
    model = MapAnythingModel(args.checkpoint, memory_efficient=True)

    def views(frames):
        return [
            centred_view(
                cv2.cvtColor(frames.images[cam], cv2.COLOR_BGR2RGB),
                data.intrinsics[cam],
                args.crop_bottom,
            )
            for cam in cameras
        ]

    writer = Writer(args.out, version=8, storage_plugin=StoragePlugin.MCAP)
    writer.open()
    topics = {
        "local_cloud": "sensor_msgs/msg/PointCloud2",
        "map_chunks": "sensor_msgs/msg/PointCloud2",
        "map": "sensor_msgs/msg/PointCloud2",
        "odometry": "nav_msgs/msg/Odometry",
        "path": "nav_msgs/msg/Path",
        "reference_path": "nav_msgs/msg/Path",
    }
    conns = {
        name: writer.add_connection(f"{NS}/{name}", msgtype, typestore=TS)
        for name, msgtype in topics.items()
    }

    def write(name, stamp_ns, msg):
        writer.write(conns[name], stamp_ns, TS.serialize_cdr(msg, topics[name]))

    previous = None  # (frames, views) of the previous instant
    chained = None  # MapAnything's rig pose in the map frame
    path_stamps, path_poses, ref_poses = [], [], []
    map_points, map_colors = [], []
    first_stamp = None
    n = len(cameras)
    t = t0
    started = time.perf_counter()
    while t <= t1:
        frames = data.frames(t)
        t += period
        if frames is None or not ref.stamps[0] <= frames.stamp_ns <= ref.stamps[-1]:
            previous = None  # a camera missing, or no reference pose: do not difference across it
            continue
        current = views(frames)
        window = (previous[1] if previous else []) + current
        if len(window) < 2:
            previous = (frames, current)
            continue
        dense = model.reconstruct(window)
        now = dense[-n:]

        # Local cloud: the current instant's points in its rig frame
        world_from_rig = rig_pose(
            [v.world_from_camera for v in now], extrinsics
        ).world_from_rig
        rig_from_world = invert(world_from_rig)
        points, colors = [], []
        for v in now:
            keep = v.valid & (
                np.linalg.norm(v.points - v.world_from_camera[:3, 3], axis=-1)
                < args.max_depth
            )
            points.append(
                v.points[keep].astype(np.float64) @ rig_from_world[:3, :3].T
                + rig_from_world[:3, 3]
            )
            colors.append(v.rgb[keep])
        points, colors = voxel_downsample(
            np.concatenate(points), np.concatenate(colors), args.local_voxel
        )
        stamp = frames.stamp_ns
        first_stamp = first_stamp or stamp
        write(
            "local_cloud",
            stamp,
            cloud_msg(stamp, args.rig_frame, points.astype(np.float32), colors),
        )

        # Map chunk: placed by the reference odometry
        map_from_rig = ref.pose(stamp)
        map_from_rig[:3, 3] += (
            ref.origin
        )  # Reference.pose() is relative to its first position
        chunk, chunk_colors = voxel_downsample(points, colors, args.chunk_voxel)
        chunk = chunk @ map_from_rig[:3, :3].T + map_from_rig[:3, 3]
        write(
            "map_chunks",
            stamp,
            cloud_msg(stamp, args.map_frame, chunk.astype(np.float32), chunk_colors),
        )
        map_points.append(chunk)
        map_colors.append(chunk_colors)

        # MapAnything's own trajectory, chained from the reference pose at the first instant
        if chained is None or previous is None:
            chained = map_from_rig
        else:
            motion = relative_rig_pose_per_camera(
                [v.world_from_camera for v in dense[:n]],
                [v.world_from_camera for v in now],
                extrinsics,
            ).rig_a_from_rig_b
            chained = chained @ motion
        path_stamps.append(stamp)
        path_poses.append(chained.copy())
        ref_poses.append(map_from_rig)
        write(
            "odometry",
            stamp,
            odometry_msg(stamp, args.map_frame, args.rig_frame, chained),
        )
        write("path", stamp, path_msg(stamp, args.map_frame, path_stamps, path_poses))
        write(
            "reference_path",
            stamp,
            path_msg(stamp, args.map_frame, path_stamps, ref_poses),
        )

        previous = (frames, current)
        done = (stamp - t0) * 1e-9
        elapsed = time.perf_counter() - started
        print(
            f"{done:6.0f} s / {(t1 - t0) * 1e-9:.0f} s: {len(points)} local points; drift "
            f"{np.linalg.norm(chained[:3, 3] - map_from_rig[:3, 3]):.1f} m; {elapsed / 60:.1f} min",
            flush=True,
        )

    if map_points:
        points, colors = voxel_downsample(
            np.concatenate(map_points), np.concatenate(map_colors), args.map_voxel
        )
        write(
            "map",
            first_stamp,
            cloud_msg(first_stamp, args.map_frame, points.astype(np.float32), colors),
        )
        print(f"map: {len(points)} points")
    writer.close()
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
