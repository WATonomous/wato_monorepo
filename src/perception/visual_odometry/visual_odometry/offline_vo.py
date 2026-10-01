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
"""Run cuVSLAM multicamera odometry over a bag, frame by frame, and write the result to a new bag.

Every frame set is tracked (nothing is dropped for being slow), so runs are reproducible. Output
messages are logged at their header stamp, so the output bag can be played next to the input bag
as if the odometry were a sensor.

    ros2 run visual_odometry offline_vo --bag <bag dir or .mcap> --config <params.yaml> --out <out dir>
"""

import argparse
import os
import sys
import time
from typing import Dict, List, Optional

import numpy as np
import rosbag2_py
from rclpy.serialization import serialize_message

from visual_odometry.bag_io import BagCameraReader, topic_metadata
from visual_odometry.geometry import invert, rotation_angle
from visual_odometry.grouping import FrameSet
from visual_odometry.pipeline import (
    VoPipeline,
    load_extrinsics,
    physical_specs,
    resolve_path,
)
from visual_odometry.ros_io import (
    OdometryBuilder,
    build_status,
    decode_gray,
    load_params,
)
from visual_odometry.vo_core import TrackResult

ODOM_TYPE = "nav_msgs/msg/Odometry"
STATUS_TYPE = "diagnostic_msgs/msg/DiagnosticArray"


class OfflineRunner:
    def __init__(
        self,
        params: Dict,
        source: BagCameraReader,
        writer: rosbag2_py.SequentialWriter,
        prefix: str,
    ):
        self.p = params
        self.source = source
        self.writer = writer
        self.pipeline: Optional[VoPipeline] = None
        self.overrides = load_extrinsics(params["extrinsics_file"], params["rig_frame"])
        self.builder = OdometryBuilder(
            params["odom_frame"], params["rig_frame"], params["max_twist_dt"]
        )
        self.odom_topic = f"{prefix}/{params['odom_topic']}".replace("//", "/")
        self.status_topic = f"{prefix}/{params['status_topic']}".replace("//", "/")
        self.counters = {
            "sets": 0,
            "tracked": 0,
            "lost": 0,
            "incomplete": 0,
            "skipped_no_calib": 0,
            "resets": 0,
        }
        self.track_ms: List[float] = []
        self.landmarks: List[int] = []
        self.session_start: Optional[TrackResult] = None
        self.last_valid: Optional[TrackResult] = None

    def _build_pipeline(self) -> bool:
        if not self.source.calibration_ready():
            return False
        try:
            specs = physical_specs(
                self.p, self.source.infos, self.source.tf.lookup, self.overrides
            )
        except LookupError:
            return False
        self.pipeline = VoPipeline(self.p, specs, decode_gray)
        for cam in (c for c in self.p["cameras"] if c in self.overrides):
            print(f"[offline_vo] {cam}: extrinsics from {self.p['extrinsics_file']}")
        for line in self.pipeline.describe():
            print(f"[offline_vo] {line}")
        return True

    def process(self, frame_set: FrameSet) -> None:
        self.counters["sets"] += 1
        if self.pipeline is None and not self._build_pipeline():
            self.counters["skipped_no_calib"] += 1
            return
        if not frame_set.complete:
            self.counters["incomplete"] += 1
            if not self.p["allow_incomplete"]:
                return
        try:
            result = self.pipeline.process(frame_set)
        except ValueError as e:
            print(
                f"[offline_vo] skipping set at {frame_set.stamp_ns}: {e}",
                file=sys.stderr,
            )
            return
        self.track_ms.append(result.track_ms)
        self.landmarks.append(result.num_landmarks)
        if result.valid:
            self.counters["tracked"] += 1
            if self.last_valid is None:
                if self.session_start is not None:
                    self.counters["resets"] += (
                        1  # cuVSLAM restarted its world frame after a loss
                    )
                self.session_start = result
            self.last_valid = result
            odom = self.builder.build(result)
            self.writer.write(self.odom_topic, serialize_message(odom), result.stamp_ns)
        else:
            self.counters["lost"] += 1
            self.last_valid = None
            self.builder.build(result)
        if self.counters["sets"] % 20 == 0:
            status = build_status(
                frame_set.stamp_ns,
                "visual_odometry",
                result,
                self.counters,
                self.source.grouper.offset_stats(),
            )
            self.writer.write(
                self.status_topic, serialize_message(status), frame_set.stamp_ns
            )

    def summary(self) -> str:
        c = self.counters
        lines = [
            f"sets={c['sets']} tracked={c['tracked']} lost={c['lost']} incomplete={c['incomplete']} "
            f"skipped_no_calib={c['skipped_no_calib']} resets={c['resets']}"
        ]
        if self.track_ms:
            ms, lm = np.asarray(self.track_ms), np.asarray(self.landmarks)
            lines.append(
                f"track_ms p50={np.percentile(ms, 50):.1f} p95={np.percentile(ms, 95):.1f} max={ms.max():.1f} | "
                f"landmarks p5={np.percentile(lm, 5):.0f} p50={np.percentile(lm, 50):.0f}"
            )
        for cam, s in self.source.grouper.offset_stats().items():
            lines.append(
                f"offset {cam} vs {self.source.cameras[0]}: mean {s['mean_ms']:+.2f} ms, max |.| {s['max_abs_ms']:.2f} ms"
            )
        if self.session_start is not None and self.last_valid is not None:
            delta = (
                invert(self.session_start.world_from_rig)
                @ self.last_valid.world_from_rig
            )
            dt = (self.last_valid.stamp_ns - self.session_start.stamp_ns) * 1e-9
            lines.append(
                f"net motion over last session ({dt:.1f} s): {np.linalg.norm(delta[:3, 3]):.3f} m, "
                f"{np.degrees(rotation_angle(delta[:3, :3])):.3f} deg"
            )
        return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--bag", required=True, help="input bag directory or .mcap file"
    )
    parser.add_argument(
        "--config", default="", help="ROS 2 params YAML (visual_odometry config)"
    )
    parser.add_argument("--out", required=True, help="output bag directory (mcap)")
    parser.add_argument(
        "--start", type=float, default=0.0, help="seconds to skip from the bag start"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0, help="seconds to process (0 = all)"
    )
    parser.add_argument(
        "--namespace", default="/perception", help="prefix for the output topics"
    )
    parser.add_argument(
        "--extrinsics", default=None, help="override extrinsics_file from the config"
    )
    parser.add_argument(
        "--async-sba",
        action="store_true",
        help="keep async bundle adjustment (non-deterministic)",
    )
    args = parser.parse_args(argv)

    params = load_params(args.config)
    if args.extrinsics is not None:
        params["extrinsics_file"] = args.extrinsics
    else:
        params["extrinsics_file"] = resolve_path(
            params["extrinsics_file"], os.path.dirname(os.path.abspath(args.config))
        )
    if not args.async_sba:
        params["async_sba"] = False

    try:
        source = BagCameraReader(
            args.bag, params, start=args.start, duration=args.duration
        )
    except KeyError as e:
        print(f"[offline_vo] {e}", file=sys.stderr)
        return 1
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.out, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    runner = OfflineRunner(params, source, writer, args.namespace)
    writer.create_topic(topic_metadata(0, runner.odom_topic, ODOM_TYPE))
    writer.create_topic(topic_metadata(1, runner.status_topic, STATUS_TYPE))

    wall0 = time.monotonic()
    next_report = 200
    for frame_set in source.frame_sets():
        runner.process(frame_set)
        if runner.counters["sets"] >= next_report:
            next_report += 200
            print(f"[offline_vo] t={source.elapsed:.1f}s {runner.counters}", flush=True)

    del writer  # flush and close the output bag
    print(f"[offline_vo] done in {time.monotonic() - wall0:.1f} s wall")
    print(runner.summary())
    return 0


if __name__ == "__main__":
    sys.exit(main())
