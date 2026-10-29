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
"""Self-calibrate the relative rotations of the configured stereo pairs from a bag (VO-only extrinsics).

SIFT matches between the cameras of each pair are collected over many frame sets. For each pair, the
camera that is not yet fixed (anchors are fixed; see ``calibration_anchors``) is rotated so the
rectified pair is row-aligned; its rotation along the epipolar lines is set by the far-point
heuristic (see pair_calibration.py). Translations stay nominal. The URDF is not touched: the result
is a YAML file the VO config points at through ``extrinsics_file``.

A parked bag is best: the scene is static and both cameras see the same instant up to their ~3 ms
(nw/nn) or ~20 ms (ne, ss) free-running offsets.

    ros2 run visual_odometry calibrate_pairs --bag <bag> --config <all6.yaml> --out pano_extrinsics.yaml
    ros2 run visual_odometry calibrate_pairs --bag <bag> --config <all6.yaml> --check [--extrinsics <file>]
"""

import argparse
import datetime
import os
import sys
from typing import Dict, List, Tuple

import cv2
import numpy as np

from visual_odometry.bag_io import BagCameraReader
from visual_odometry.pair_calibration import calibrate_pair
from visual_odometry.pipeline import (
    dump_extrinsics,
    load_extrinsics,
    parse_pairs,
    physical_specs,
    resolve_path,
)
from visual_odometry.rectify import border_mask, rectified_disparity
from visual_odometry.ros_io import decode_gray, load_params
from visual_odometry.vo_core import CameraSpec


def collect_matches(
    source: BagCameraReader,
    pairs: List[List[str]],
    border: Tuple[int, int, int, int],
    num_sets: int,
    every: int,
) -> Dict[Tuple[str, str], Tuple[np.ndarray, np.ndarray]]:
    """SIFT + ratio-test matches for every pair, accumulated over ``num_sets`` frame sets."""
    sift = cv2.SIFT_create(nfeatures=4000)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    index = {c: i for i, c in enumerate(source.cameras)}
    needed = sorted({c for p in pairs for c in p}, key=index.get)
    matches = {tuple(p): ([], []) for p in pairs}
    used = seen = 0
    for frame_set in source.frame_sets():
        if not frame_set.complete:
            continue
        seen += 1
        if seen % every:
            continue
        features = {}
        for cam in needed:
            img = decode_gray(frame_set.frames[index[cam]][1])
            mask = border_mask(img.shape[0], img.shape[1], border)
            features[cam] = sift.detectAndCompute(img, mask)
        for (a, b), (pts_a, pts_b) in matches.items():
            (ka, da), (kb, db) = features[a], features[b]
            if da is None or db is None:
                continue
            for knn in matcher.knnMatch(da, db, k=2):
                if len(knn) == 2 and knn[0].distance < 0.75 * knn[1].distance:
                    pts_a.append(ka[knn[0].queryIdx].pt)
                    pts_b.append(kb[knn[0].trainIdx].pt)
        used += 1
        if used >= num_sets:
            break
    print(f"[calibrate_pairs] matched {used} frame sets (every {every}th complete set)")
    out = {}
    for key, (pts_a, pts_b) in matches.items():
        pts_a, pts_b = np.asarray(pts_a, float), np.asarray(pts_b, float)
        # Drop mismatches (repetitive texture such as hedges) independently of the extrinsics.
        _, keep = (
            cv2.findFundamentalMat(pts_a, pts_b, cv2.FM_RANSAC, 2.0, 0.999)
            if len(pts_a) >= 8
            else (None, None)
        )
        keep = np.zeros(len(pts_a), bool) if keep is None else keep.ravel().astype(bool)
        print(
            f"  {key[0]} / {key[1]}: {keep.sum()}/{len(pts_a)} matches pass a RANSAC epipolar check"
        )
        out[key] = (pts_a[keep], pts_b[keep])
    return out


def pair_report(
    a: CameraSpec, b: CameraSpec, pts_a: np.ndarray, pts_b: np.ndarray
) -> str:
    dx, dy = rectified_disparity(a, b, pts_a, pts_b)
    med = np.median(dy)
    good = np.abs(dy - med) < 3.0
    return (
        f"vertical disparity median {med:+.2f} px (MAD {np.median(np.abs(dy - med)):.2f}); "
        f"horizontal disparity p2/p50/p95 {np.round(np.percentile(dx[good], [2, 50, 95]), 1).tolist()} px "
        f"over {good.sum()}/{len(dy)} row-consistent matches"
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument(
        "--config", required=True, help="VO params YAML naming cameras and stereo_pairs"
    )
    parser.add_argument(
        "--out", default="", help="output extrinsics YAML (omit with --check)"
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="only report the alignment of the current extrinsics",
    )
    parser.add_argument(
        "--extrinsics", default=None, help="start from / check this extrinsics file"
    )
    parser.add_argument(
        "--start", type=float, default=0.0, help="seconds to skip from the bag start"
    )
    parser.add_argument("--sets", type=int, default=40, help="frame sets to match")
    parser.add_argument(
        "--every", type=int, default=10, help="use every Nth complete frame set"
    )
    parser.add_argument("--far-percentile", type=float, default=2.0)
    parser.add_argument(
        "--far-disparity",
        type=float,
        default=0.5,
        help="px assigned to the far percentile",
    )
    args = parser.parse_args(argv)
    if not args.check and not args.out:
        parser.error("--out is required unless --check")

    params = load_params(args.config)
    params["downscale"] = 1  # calibrate at full resolution
    pairs = parse_pairs(params["stereo_pairs"])
    if not pairs:
        parser.error(f"{args.config} has no stereo_pairs")
    anchors = list(params.get("calibration_anchors") or [params["cameras"][0]])
    extrinsics_file = args.extrinsics
    if extrinsics_file is None:
        extrinsics_file = resolve_path(
            params["extrinsics_file"], os.path.dirname(os.path.abspath(args.config))
        )
    overrides = {}
    if args.extrinsics or (
        args.check and extrinsics_file and os.path.exists(extrinsics_file)
    ):
        overrides = load_extrinsics(extrinsics_file, params["rig_frame"])

    source = BagCameraReader(args.bag, params, start=args.start)
    matches = collect_matches(
        source, pairs, tuple(params["border"]), args.sets, args.every
    )
    specs = {
        s.name: s
        for s in physical_specs(params, source.infos, source.tf.lookup, overrides)
    }
    print(
        f"[calibrate_pairs] extrinsics: {extrinsics_file if overrides else 'nominal (TF)'}"
    )

    for (a, b), (pts_a, pts_b) in matches.items():
        print(
            f"  {a} / {b}: {len(pts_a)} matches; {pair_report(specs[a], specs[b], pts_a, pts_b)}"
        )
    if args.check:
        return 0

    fixed = set(anchors)
    corrected: Dict[str, np.ndarray] = {}
    info: Dict[str, dict] = {}
    todo = list(matches.items())
    while todo:
        progress = False
        for item in list(todo):
            (a, b), (pts_a, pts_b) = item
            if a in fixed and b in fixed:
                todo.remove(item)
                progress = True
                continue
            if a not in fixed and b not in fixed:
                continue
            ref, other, pts_ref, pts_other = (
                (a, b, pts_a, pts_b) if a in fixed else (b, a, pts_b, pts_a)
            )
            fit = calibrate_pair(
                specs[ref],
                specs[other],
                pts_ref,
                pts_other,
                args.far_percentile,
                args.far_disparity,
            )
            specs[other] = fit.corrected
            corrected[other] = fit.corrected.rig_from_camera
            fixed.add(other)
            info[other] = {
                "reference": ref,
                "correction_deg_rect_xyz": [round(float(v), 4) for v in fit.delta_deg],
                "rotation_change_deg": round(fit.rotation_change_deg, 4),
                "matches": fit.num_matches,
                "inliers": fit.num_inliers,
                "vertical_disparity_px_before": [
                    round(v, 3) for v in fit.vertical_before
                ],
                "vertical_disparity_px_after": [
                    round(v, 3) for v in fit.vertical_after
                ],
                "far_disparity_px_before": round(fit.far_disparity_before, 3),
                "median_disparity_px_after": round(fit.median_disparity_after, 3),
            }
            print(
                f"[calibrate_pairs] {other} (vs {ref}): rotated {fit.rotation_change_deg:.3f} deg, "
                f"rect-axes xyz {np.round(fit.delta_deg, 3).tolist()} deg; vertical {fit.vertical_before[0]:+.2f} -> "
                f"{fit.vertical_after[0]:+.2f} px (MAD {fit.vertical_after[1]:.2f}), "
                f"{fit.num_inliers}/{fit.num_matches} inliers"
            )
            todo.remove(item)
            progress = True
        if not progress:
            print(
                f"[calibrate_pairs] pairs not reachable from anchors {anchors}: {[k for k, _ in todo]}",
                file=sys.stderr,
            )
            return 1

    for (a, b), (pts_a, pts_b) in matches.items():
        print(f"  after: {a} / {b}: {pair_report(specs[a], specs[b], pts_a, pts_b)}")
    header = (
        "# VO-only pano camera extrinsics, self-calibrated by visual_odometry calibrate_pairs.\n"
        f"# bag: {os.path.basename(os.path.normpath(args.bag))}, {datetime.date.today().isoformat()}; "
        f"anchors (kept nominal): {anchors}\n"
        "# Rotations about each pair's baseline/viewing axes are fitted to align rectified rows. The rotation\n"
        "# along the epipolar lines is NOT observable from these matches: it is set so the far points sit at\n"
        f"# {args.far_disparity} px disparity (p{args.far_percentile:g}). Translations are nominal (URDF).\n"
    )
    dump_extrinsics(args.out, params["rig_frame"], corrected, header, info)
    print(f"[calibrate_pairs] wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
