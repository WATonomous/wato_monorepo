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
"""Finding, describing, reading and writing rosbag2 bags (MCAP or sqlite3)."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import rosbag2_py

# extract_segments.py drops this file in its output folder so later scans skip the
# cut bags, even when the output folder sits inside the folder being searched
OUTPUT_MARKER = ".lidar_cam_calib_output"


@dataclass
class BagInfo:
    path: Path
    start_ns: int
    duration_s: float
    message_count: int
    topics: dict[str, str]  # name -> type
    counts: dict[str, int]  # name -> message count

    @property
    def name(self) -> str:
        """Bag name without extension, used to name its segments."""
        return (
            self.path.name[: -len(".mcap")]
            if self.path.suffix == ".mcap"
            else self.path.name
        )


def find_bags(root: Path, exclude: list[Path] | None = None) -> list[Path]:
    """Every bag under root: folders holding metadata.yaml, plus loose .mcap files.

    Folders listed in exclude, and output folders of extract_segments.py, are skipped.
    """
    excluded = [p.resolve() for p in (exclude or [])]
    bags: list[Path] = []
    for dirpath, dirnames, filenames in os.walk(root, followlinks=True):
        here = Path(dirpath).resolve()
        if OUTPUT_MARKER in filenames or any(
            here == e or e in here.parents for e in excluded
        ):
            dirnames[:] = []
            continue
        if "metadata.yaml" in filenames:
            bags.append(Path(dirpath))
            dirnames[:] = []  # the bag's own files are not separate bags
            continue
        bags += [Path(dirpath) / f for f in filenames if f.endswith(".mcap")]
        dirnames.sort()
    return sorted(bags)


def read_info(path: Path) -> BagInfo:
    meta = rosbag2_py.Info().read_metadata(str(path), "")
    topics, counts = {}, {}
    for t in meta.topics_with_message_count:
        topics[t.topic_metadata.name] = t.topic_metadata.type
        counts[t.topic_metadata.name] = t.message_count
    return BagInfo(
        path=path,
        start_ns=meta.starting_time.nanoseconds,
        duration_s=meta.duration.nanoseconds * 1e-9,
        message_count=meta.message_count,
        topics=topics,
        counts=counts,
    )


def open_reader(
    path: Path, topics: list[str] | None = None, start_ns: int | None = None
) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    if start_ns is not None:
        reader.seek(start_ns)
    return reader


def topic_metadata(reader: rosbag2_py.SequentialReader) -> dict:
    """name -> TopicMetadata, reusable for SequentialWriter.create_topic."""
    return {t.name: t for t in reader.get_all_topics_and_types()}


def open_writer(path: Path) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return writer
