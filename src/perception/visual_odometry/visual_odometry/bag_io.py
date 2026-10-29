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
"""Reading camera frame sets, CameraInfo and /tf_static from a bag (rosbag2_py)."""

from typing import Dict, Iterator, Mapping

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from visual_odometry.grouping import FrameGrouper, FrameSet
from visual_odometry.ros_io import stamp_to_ns, transform_msg_to_matrix
from visual_odometry.tf_tree import StaticTfTree


def storage_options(uri: str) -> rosbag2_py.StorageOptions:
    return rosbag2_py.StorageOptions(
        uri=uri, storage_id="mcap" if uri.endswith(".mcap") else ""
    )


def topic_metadata(topic_id: int, name: str, msg_type: str) -> rosbag2_py.TopicMetadata:
    try:  # Jazzy: TopicMetadata(id, name, type, serialization_format, ...)
        return rosbag2_py.TopicMetadata(
            id=topic_id, name=name, type=msg_type, serialization_format="cdr"
        )
    except TypeError:  # Humble
        return rosbag2_py.TopicMetadata(
            name=name, type=msg_type, serialization_format="cdr"
        )


class BagCameraReader:
    """Streams grouped frame sets of ``params['cameras']`` from a bag, collecting CameraInfo and /tf_static.

    ``tf`` and ``infos`` fill up as the bag is read; /tf_static and camera_info are always read, image
    topics only inside [start, start + duration] (seconds from the first message).
    """

    def __init__(
        self, uri: str, params: Mapping, start: float = 0.0, duration: float = 0.0
    ):
        self.cameras = list(params["cameras"])
        self.image_topics = {
            params["image_topic"].format(camera=c): c for c in self.cameras
        }
        self.info_topics = {
            params["camera_info_topic"].format(camera=c): c for c in self.cameras
        }
        self.start, self.duration = start, duration
        self.tf = StaticTfTree()
        self.infos: Dict[str, object] = {}
        self.grouper = FrameGrouper(
            self.cameras,
            slop_ns=int(params["sync_slop_ms"] * 1e6),
            max_wait_ns=int(params["max_wait_ms"] * 1e6),
        )
        self.elapsed = 0.0  # seconds from the first message to the last one read

        self._reader = rosbag2_py.SequentialReader()
        self._reader.open(storage_options(uri), rosbag2_py.ConverterOptions("", ""))
        type_map = {t.name: t.type for t in self._reader.get_all_topics_and_types()}
        topics = ["/tf_static", *self.image_topics, *self.info_topics]
        missing = [t for t in topics if t not in type_map]
        if missing:
            raise KeyError(f"topics not in bag {uri}: {missing}")
        self._reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        self._types = {t: get_message(type_map[t]) for t in topics}

    def frame_sets(self) -> Iterator[FrameSet]:
        first_ns = None
        while self._reader.has_next():
            topic, data, log_ns = self._reader.read_next()
            first_ns = log_ns if first_ns is None else first_ns
            self.elapsed = (log_ns - first_ns) * 1e-9
            if self.duration > 0 and self.elapsed > self.start + self.duration:
                return
            if topic == "/tf_static":
                for tr in deserialize_message(data, self._types[topic]).transforms:
                    self.tf.add(
                        tr.header.frame_id,
                        tr.child_frame_id,
                        transform_msg_to_matrix(tr.transform),
                    )
            elif topic in self.info_topics:
                cam = self.info_topics[topic]
                if cam not in self.infos:
                    self.infos[cam] = deserialize_message(data, self._types[topic])
            elif self.elapsed >= self.start:
                msg = deserialize_message(data, self._types[topic])
                yield from self.grouper.add(
                    self.image_topics[topic], stamp_to_ns(msg.header.stamp), msg.data
                )

    def calibration_ready(self) -> bool:
        return len(self.infos) == len(self.cameras)
