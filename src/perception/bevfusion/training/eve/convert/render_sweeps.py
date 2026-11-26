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
"""Stack LIDAR_TOP sweeps of one sample with the devkit and save a top-down PNG.

Usage: python render_sweeps.py ROOT OUT.png [--scene NAME] [--index N] [--sweeps 10] [--min-z -1.5]
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud

from render_overlay import pick_sample

VIEWS_M = (60, 20)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    parser.add_argument("out")
    parser.add_argument("--scene")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--sweeps", type=int, default=10)
    parser.add_argument("--min-z", type=float, default=-1.5)
    args = parser.parse_args()

    nusc = NuScenes(version="v1.0-trainval", dataroot=args.root, verbose=False)
    scene, sample = pick_sample(nusc, args.scene, args.index)
    cloud, lag = LidarPointCloud.from_file_multisweep(
        nusc, sample, "LIDAR_TOP", "LIDAR_TOP", nsweeps=args.sweeps
    )
    data = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    newest = np.array(nusc.get("ego_pose", data["ego_pose_token"])["translation"])
    for _ in range(args.sweeps - 1):
        data = nusc.get("sample_data", data["prev"]) if data["prev"] else data
    travelled = np.linalg.norm(
        newest - np.array(nusc.get("ego_pose", data["ego_pose_token"])["translation"])
    )

    keep = cloud.points[2] > args.min_z
    x, y, lag = cloud.points[0, keep], cloud.points[1, keep], lag[0, keep]
    fig, axes = plt.subplots(1, 2, figsize=(24, 12.6), dpi=100)
    for ax, half in zip(axes, VIEWS_M):
        ax.scatter(x, y, c=lag, cmap="turbo", s=1 if half > 30 else 4, linewidths=0)
        ax.set_xlim(-half, half)
        ax.set_ylim(-half, half)
        ax.set_aspect("equal")
        ax.set_facecolor("black")
        ax.set_title(f"+/- {half} m", fontsize=12)
    fig.suptitle(
        f"{scene['name']} sample {args.index}: {len(np.unique(lag))} sweeps stacked with the dataset "
        f"tables, points above z = {args.min_z} m, colour = time lag, ego moved {travelled:.1f} m",
        fontsize=13,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(args.out)
    print("saved", args.out)


if __name__ == "__main__":
    main()
