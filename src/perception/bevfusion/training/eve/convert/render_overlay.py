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
"""Project LIDAR_TOP into all 8 cameras of one sample with the devkit and save a PNG grid.

Usage: python render_overlay.py ROOT OUT.png [--scene NAME] [--index N]
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from nuscenes.nuscenes import NuScenes

CHANNELS = [f"CAM_PANO_{c}" for c in ("NN", "NE", "EE", "SE", "SS", "SW", "WW", "NW")]
MAX_DEPTH_M = 25


def pick_sample(nusc, scene_name, index):
    scene = next(s for s in nusc.scene if scene_name in (None, s["name"]))
    sample = nusc.get("sample", scene["first_sample_token"])
    for _ in range(index):
        sample = nusc.get("sample", sample["next"])
    return scene, sample


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    parser.add_argument("out")
    parser.add_argument("--scene")
    parser.add_argument("--index", type=int, default=0)
    args = parser.parse_args()

    nusc = NuScenes(version="v1.0-trainval", dataroot=args.root, verbose=False)
    scene, sample = pick_sample(nusc, args.scene, args.index)
    lidar = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    fig, axes = plt.subplots(2, 4, figsize=(25.6, 11.2), dpi=200, layout="constrained")
    for ax, ch in zip(axes.ravel(), CHANNELS):
        camera = nusc.get("sample_data", sample["data"][ch])
        points, depth, image = nusc.explorer.map_pointcloud_to_image(
            lidar["token"], camera["token"]
        )
        ax.imshow(image)
        ax.scatter(
            points[0],
            points[1],
            c=np.clip(depth, 0, MAX_DEPTH_M),
            cmap="turbo",
            s=1.5,
            linewidths=0,
            vmin=0,
            vmax=MAX_DEPTH_M,
        )
        ax.set_title(
            f"{ch}   image - lidar = {(camera['timestamp'] - lidar['timestamp']) / 1000:+.0f} ms   "
            f"{points.shape[1]} points",
            fontsize=11,
        )
        ax.axis("off")
    fig.suptitle(
        f"{scene['name']} sample {args.index}: LIDAR_TOP projected with the dataset tables, "
        f"colour = depth 0-{MAX_DEPTH_M} m",
        fontsize=13,
    )
    fig.savefig(args.out)
    print("saved", args.out)


if __name__ == "__main__":
    main()
