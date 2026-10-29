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
"""Live cuVSLAM multicamera visual odometry on the pano cameras.

Publishes nav_msgs/Odometry (``odom_frame`` -> ``rig_frame``) at the reference camera's stamps, and
nothing while tracking is lost: consumers (eidos OdometryFactor) treat a gap as a reset. No TF is
broadcast.
"""

import os
import threading
from collections import deque
from typing import Dict, Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage
from tf2_ros import Buffer, TransformException, TransformListener

from visual_odometry.grouping import FrameGrouper, FrameSet
from visual_odometry.pipeline import (
    VoPipeline,
    load_extrinsics,
    physical_specs,
    resolve_path,
)
from visual_odometry.ros_io import (
    DEFAULTS,
    OdometryBuilder,
    build_status,
    decode_gray,
    stamp_to_ns,
    transform_msg_to_matrix,
)

_STRING_ARRAYS = ("cameras", "stereo_pairs", "calibration_anchors")
_BACKWARDS_RESET_NS = (
    1_000_000_000  # a stamp this far behind the last one = bag loop / clock reset
)


class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__("visual_odometry")
        self.p: Dict = {}
        for name, default in DEFAULTS.items():
            if (
                name in _STRING_ARRAYS
            ):  # declared by type: an empty default cannot be type-inferred
                self.declare_parameter(name, Parameter.Type.STRING_ARRAY)
                try:
                    value = self.get_parameter(name).value
                except ParameterUninitializedException:
                    value = None
                self.p[name] = list(default) if value is None else list(value)
            else:
                self.p[name] = self.declare_parameter(name, default).value
        self.p["extrinsics_file"] = resolve_path(
            self.p["extrinsics_file"],
            os.path.join(get_package_share_directory("visual_odometry"), "config"),
        )
        self.overrides = load_extrinsics(self.p["extrinsics_file"], self.p["rig_frame"])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.infos: Dict[str, CameraInfo] = {}
        self.pipeline: Optional[VoPipeline] = None
        self.builder = OdometryBuilder(
            self.p["odom_frame"], self.p["rig_frame"], self.p["max_twist_dt"]
        )
        self.grouper = FrameGrouper(
            self.p["cameras"],
            slop_ns=int(self.p["sync_slop_ms"] * 1e6),
            max_wait_ns=int(self.p["max_wait_ms"] * 1e6),
        )
        self.counters = {
            "sets": 0,
            "tracked": 0,
            "lost": 0,
            "incomplete": 0,
            "dropped": 0,
            "resets": 0,
        }
        self._lock = threading.Lock()
        self._queue: deque = deque()
        self._wake = threading.Condition(self._lock)
        self._running = True
        self._last_stamp_ns: Optional[int] = None

        self.odom_pub = self.create_publisher(Odometry, self.p["odom_topic"], 10)
        self.status_pub = self.create_publisher(
            DiagnosticArray, self.p["status_topic"], 10
        )
        for cam in self.p["cameras"]:
            self.create_subscription(
                CompressedImage,
                self.p["image_topic"].format(camera=cam),
                lambda msg, cam=cam: self._on_image(cam, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                self.p["camera_info_topic"].format(camera=cam),
                lambda msg, cam=cam: self.infos.setdefault(cam, msg),
                qos_profile_sensor_data,
            )
        self.create_timer(1.0, self._try_build_pipeline)
        self._worker = threading.Thread(target=self._work, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"cameras={self.p['cameras']} pairs={self.p['stereo_pairs']} extrinsics={self.p['extrinsics_file'] or 'TF'}"
        )

    # ---- setup ----------------------------------------------------------------------------------

    def _lookup(self, target: str, source: str) -> np.ndarray:
        tf = self.tf_buffer.lookup_transform(target, source, Time())
        return transform_msg_to_matrix(tf.transform)

    def _try_build_pipeline(self) -> None:
        if self.pipeline is not None or len(self.infos) < len(self.p["cameras"]):
            return
        try:
            specs = physical_specs(self.p, self.infos, self._lookup, self.overrides)
        except TransformException as e:
            self.get_logger().warn(
                f"waiting for camera extrinsics: {e}", throttle_duration_sec=5.0
            )
            return
        pipeline = VoPipeline(self.p, specs, decode_gray)
        for line in pipeline.describe():
            self.get_logger().info(line)
        with self._lock:
            self.pipeline = pipeline

    # ---- input ----------------------------------------------------------------------------------

    def _on_image(self, cam: str, msg: CompressedImage) -> None:
        with self._lock:
            for frame_set in self.grouper.add(
                cam, stamp_to_ns(msg.header.stamp), msg.data
            ):
                if len(self._queue) >= self.p["queue_size"]:
                    self._queue.popleft()
                    self.counters["dropped"] += 1
                self._queue.append(frame_set)
            self._wake.notify()

    # ---- processing -----------------------------------------------------------------------------

    def _work(self) -> None:
        while True:
            with self._lock:
                while self._running and (not self._queue or self.pipeline is None):
                    self._wake.wait(timeout=0.5)
                if not self._running:
                    return
                frame_set = self._queue.popleft()
                pipeline = self.pipeline
            self._process(pipeline, frame_set)

    def _process(self, pipeline: VoPipeline, frame_set: FrameSet) -> None:
        self.counters["sets"] += 1
        if (
            self._last_stamp_ns is not None
            and frame_set.stamp_ns < self._last_stamp_ns - _BACKWARDS_RESET_NS
        ):
            self.get_logger().warn("time jumped backwards; restarting tracking")
            pipeline.reset()
            self.builder.reset()
            self.counters["resets"] += 1
        if not frame_set.complete:
            self.counters["incomplete"] += 1
            if not self.p["allow_incomplete"]:
                return
        try:
            result = pipeline.process(frame_set)
        except ValueError as e:
            self.get_logger().warn(
                f"skipping frame set: {e}", throttle_duration_sec=5.0
            )
            return
        self._last_stamp_ns = frame_set.stamp_ns
        if result.valid:
            self.counters["tracked"] += 1
            self.odom_pub.publish(self.builder.build(result))
        else:
            self.counters["lost"] += 1
            self.builder.build(result)
            self.get_logger().warn("tracking lost", throttle_duration_sec=2.0)
        if self.counters["sets"] % 20 == 0:
            with self._lock:
                offsets = self.grouper.offset_stats()
            self.status_pub.publish(
                build_status(
                    frame_set.stamp_ns,
                    self.get_name(),
                    result,
                    dict(self.counters),
                    offsets,
                )
            )

    def destroy_node(self):
        with self._lock:
            self._running = False
            self._wake.notify_all()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
