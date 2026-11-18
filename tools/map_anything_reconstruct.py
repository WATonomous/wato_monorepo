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
"""Dense 3D reconstruction of a stretch of a drive with MapAnything (host tool).

Takes ``--instants`` camera frame sets, ``--spacing`` metres apart starting ``--start`` metres into
the drive, and runs all their images through MapAnything in a single feed-forward pass (images and
intrinsics only: no poses, no depth, no optimisation). The per-pixel 3D points the model predicts
are merged into one coloured point cloud.

The cloud is rigidly moved into a road frame for display: z = 0 on the road surface under the
drive (a plane fitted to the reconstructed points along the path), x along the initial direction
of travel, origin under the first instant. Nothing is rescaled or warped.

Outputs, next to ``--out``:
  * ``<out>.ply``: the point cloud (binary PLY: x y z, red green blue, confidence), for MeshLab,
    CloudCompare, Blender, Foxglove...
  * ``<out>.scene.txt``: the same cloud, quantised and sorted by confidence, with the predicted
    camera poses and the reference trajectory, for the web viewer (tools/map_anything_viewer.html).
    It is base64 text because that is what static page hosts serve; keep it under 16 MB
    (``--max-points`` 1.2 million).

Datasets and their arguments are those of tools/map_anything_pairs_eval.py.

    tools/map_anything_reconstruct.py --boreas <sequence dir> --start 600 --out scenes/boreas
"""

import argparse
import base64
import json
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import map_anything_pairs_eval as ev
from map_anything.model import DEFAULT_CHECKPOINT, MapAnythingModel, centred_view
from map_anything.rig import rig_pose
from visual_odometry.geometry import invert

# Rig frame (x forward, y left, z up) from an optical frame (x right, y down, z forward).
RIG_FROM_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


def road_frame(
    points: np.ndarray, path: np.ndarray, corridor: float = 3.0
) -> np.ndarray:
    """4x4 ``road_from_rig``: z = 0 on the plane fitted to the points along the path, x along the path's start.

    ``points`` (N, 3) and ``path`` (K, 3) are in a rig frame (x forward, y left, z up). The plane is
    fitted to the points within ``corridor`` m (in xy) of the path, trimming outliers (vehicles).
    """
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    travelled = np.concatenate([[0.0], np.cumsum(steps)])
    samples = np.arange(0.0, travelled[-1] + 1.0, 1.0)
    dense = np.stack(
        [np.interp(samples, travelled, path[:, axis]) for axis in (0, 1)], axis=1
    )
    near = np.zeros(len(points), dtype=bool)
    for xy in dense:
        near |= np.linalg.norm(points[:, :2] - xy, axis=1) < corridor
    road = points[near].astype(np.float64)
    road = road[road[:, 2] < path[:, 2].max()]  # below the rig
    normal, centre = np.array([0.0, 0.0, 1.0]), np.median(road, axis=0)
    for tolerance in (1.0, 0.5, 0.25, 0.25):
        inliers = road[np.abs((road - centre) @ normal) < tolerance]
        centre = inliers.mean(axis=0)
        normal = np.linalg.svd(inliers - centre, full_matrices=False)[2][2]
        normal = normal if normal[2] > 0 else -normal
    forward = path[min(1, len(path) - 1)] - path[0]
    forward = forward if np.linalg.norm(forward) > 1e-6 else np.array([1.0, 0.0, 0.0])
    x = forward - (forward @ normal) * normal
    x /= np.linalg.norm(x)
    origin = path[0] - ((path[0] - centre) @ normal) * normal  # under the first instant
    road_from_rig = np.eye(4)
    road_from_rig[:3, :3] = np.stack([x, np.cross(normal, x), normal])
    road_from_rig[:3, 3] = -road_from_rig[:3, :3] @ origin
    return road_from_rig


def write_ply(path: Path, points, colors, confidence) -> None:
    vertex = np.empty(
        len(points),
        dtype=[("xyz", "<f4", 3), ("rgb", "u1", 3), ("confidence", "u1")],
    )
    vertex["xyz"], vertex["rgb"], vertex["confidence"] = points, colors, confidence
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {len(points)}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "property uchar confidence\nend_header\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(vertex.tobytes())


def write_viewer_scene(path: Path, header: dict, points, colors) -> None:
    """Base64 of: ``MAPC`` + header length + JSON header + int16 xyz + uint8 rgb.

    Points are quantised to the cloud's bounding box (a few millimetres at these sizes).
    """
    center = (points.min(axis=0) + points.max(axis=0)) / 2
    extent = float(np.abs(points - center).max())
    quantised = np.round((points - center) / extent * 32767).astype("<i2")
    header = {
        **header,
        "points": len(points),
        "center": center.tolist(),
        "extent": extent,
    }
    blob = json.dumps(header, separators=(",", ":")).encode("utf-8")
    blob += b" " * (-len(blob) % 4)  # keep the int16 block aligned
    payload = b"MAPC" + struct.pack("<I", len(blob)) + blob + quantised.tobytes()
    payload += np.ascontiguousarray(colors, dtype=np.uint8).tobytes()
    path.write_bytes(base64.b64encode(payload))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ev.add_source_arguments(parser)
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT)
    parser.add_argument("--start", type=float, default=100.0, help="m into the drive")
    parser.add_argument("--spacing", type=float, default=3.0, help="m between instants")
    parser.add_argument("--instants", type=int, default=24)
    parser.add_argument(
        "--max-depth",
        type=float,
        default=50.0,
        help="drop points further than this from their camera, m",
    )
    parser.add_argument("--max-points", type=int, default=1_200_000)
    parser.add_argument(
        "--crop-bottom",
        type=int,
        default=0,
        help="rows showing the car itself (roof rack / hood) to cut from the bottom of every image "
        "(images are always cropped so that the principal point is centred)",
    )
    parser.add_argument("--title", default="", help="scene name shown in the viewer")
    parser.add_argument(
        "--note", default="", help="one line about the data, shown in the viewer"
    )
    parser.add_argument(
        "--out", type=Path, required=True, help="output path without extension"
    )
    args = parser.parse_args(argv)

    data = ev.open_source(args)
    ref, cameras = data.reference, data.cameras
    frame_sets = []
    for k in range(args.instants):
        frames = data.frames(ref.time_at_distance(args.start + k * args.spacing))
        if frames is None:
            print(f"instant {k}: a camera has no frame, skipped")
            continue
        frame_sets.append(frames)
    if len(frame_sets) * len(cameras) < 2:
        print("need at least two views")
        return 1
    views = [
        centred_view(
            cv2.cvtColor(frames.images[cam], cv2.COLOR_BGR2RGB),
            data.intrinsics[cam],
            args.crop_bottom,
        )
        for frames in frame_sets
        for cam in cameras
    ]
    print(
        f"{data.description}: {len(frame_sets)} instants x {len(cameras)} cameras = {len(views)} views"
    )

    model = MapAnythingModel(args.checkpoint, memory_efficient=True)
    start = time.perf_counter()
    dense = model.reconstruct(views)
    infer_s = time.perf_counter() - start

    # The model's frame is the first view's camera; its extrinsic gives the first instant's rig frame.
    rig_from_optical = RIG_FROM_OPTICAL if args.boreas else np.eye(4)
    extrinsics = [rig_from_optical @ data.rig_from_camera[cam] for cam in cameras]
    rig_from_world = extrinsics[0]

    n = len(cameras)
    predicted, reference = [], []
    ref0 = ref.pose(frame_sets[0].stamp_ns)
    for k, frames in enumerate(frame_sets):
        poses = [view.world_from_camera for view in dense[k * n : (k + 1) * n]]
        predicted.append(
            (rig_from_world @ rig_pose(poses, extrinsics).world_from_rig)[:3, 3]
        )
        motion = invert(ref0) @ ref.pose(frames.stamp_ns)  # of the dataset's rig frame
        reference.append(rig_from_optical[:3, :3] @ motion[:3, 3])
    predicted = np.array(predicted)
    reference = np.array(reference) + predicted[0]  # both paths start at the same place

    points, colors, confidence = [], [], []
    for view in dense:
        centre = view.world_from_camera[:3, 3]
        keep = view.valid & (
            np.linalg.norm(view.points - centre, axis=-1) < args.max_depth
        )
        local = view.points[keep].astype(np.float64)
        points.append(local @ rig_from_world[:3, :3].T + rig_from_world[:3, 3])
        colors.append(view.rgb[keep])
        confidence.append(view.confidence[keep])
    points, colors, confidence = (
        np.concatenate(points),
        np.concatenate(colors),
        np.concatenate(confidence),
    )
    total = len(points)
    if total > args.max_points:
        pick = np.random.default_rng(0).choice(total, args.max_points, replace=False)
        points, colors, confidence = points[pick], colors[pick], confidence[pick]
    order = np.argsort(
        -confidence, kind="stable"
    )  # most confident first: the viewer cuts the tail
    points, colors = points[order], colors[order]
    rank = np.round(
        255 * (1 - np.arange(len(points)) / max(len(points) - 1, 1))
    ).astype(np.uint8)

    road_from_rig = road_frame(points, predicted)
    move = lambda p: p @ road_from_rig[:3, :3].T + road_from_rig[:3, 3]  # noqa: E731
    points, predicted, reference = (
        move(points).astype(np.float32),
        move(predicted),
        move(reference),
    )
    camera_list = []
    for index, view in enumerate(dense):
        height, width = view.valid.shape
        camera_list.append(
            {
                "name": cameras[index % n],
                "instant": index // n,
                "pose": np.round(
                    road_from_rig @ rig_from_world @ view.world_from_camera, 5
                )
                .reshape(-1)
                .tolist(),
                # half extents of the image plane at unit depth
                "half_w": float(width / 2 / view.intrinsics[0, 0]),
                "half_h": float(height / 2 / view.intrinsics[1, 1]),
            }
        )
    path_length = lambda p: float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())  # noqa: E731

    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_ply(args.out.with_suffix(".ply"), points, colors, rank)
    header = {
        "title": args.title or args.out.name,
        "note": args.note,
        "source": data.description,
        "checkpoint": args.checkpoint,
        "views": len(views),
        "instants": len(frame_sets),
        "camera_names": cameras,
        "spacing_m": args.spacing,
        "infer_s": round(infer_s, 2),
        "points_predicted": int(total),
        "max_depth_m": args.max_depth,
        "reference_length_m": round(path_length(reference), 2),
        "predicted_length_m": round(path_length(predicted), 2),
        "end_gap_m": round(float(np.linalg.norm(predicted[-1] - reference[-1])), 2),
        "cameras": camera_list,
        "predicted_path": np.round(predicted, 3).tolist(),
        "reference_path": np.round(reference, 3).tolist(),
    }
    write_viewer_scene(args.out.with_suffix(".scene.txt"), header, points, colors)
    print(
        f"{len(views)} views in {infer_s:.1f} s -> {total} valid points, kept {len(points)}; "
        f"path length predicted {header['predicted_length_m']} m vs reference "
        f"{header['reference_length_m']} m, end points {header['end_gap_m']} m apart; "
        f"rig origin {predicted[0][2]:.2f} m above the road; wrote {args.out}.ply and .scene.txt"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
