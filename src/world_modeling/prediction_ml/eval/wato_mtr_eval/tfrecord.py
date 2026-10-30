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
"""Minimal TFRecord reader.

TFRecord framing is: u64 length, u32 masked-crc32c(length), payload,
u32 masked-crc32c(payload). Parsing it directly avoids a TensorFlow
dependency for what amounts to twenty lines of struct unpacking. CRCs are
not verified; a corrupt shard surfaces as a protobuf parse failure instead.
"""

from __future__ import annotations

import pathlib
import struct
from collections.abc import Iterator

_LENGTH_SIZE = 8
_CRC_SIZE = 4


def read_records(path: pathlib.Path) -> Iterator[bytes]:
    """Yield each record payload from a TFRecord file."""
    with path.open("rb") as fh:
        while True:
            header = fh.read(_LENGTH_SIZE)
            if not header:
                return
            if len(header) < _LENGTH_SIZE:
                raise ValueError("truncated tfrecord: incomplete length header")
            (length,) = struct.unpack("<Q", header)

            if len(fh.read(_CRC_SIZE)) < _CRC_SIZE:
                raise ValueError("truncated tfrecord: incomplete length crc")

            payload = fh.read(length)
            if len(payload) < length:
                raise ValueError(
                    f"truncated tfrecord: wanted {length} payload bytes, got {len(payload)}"
                )
            if len(fh.read(_CRC_SIZE)) < _CRC_SIZE:
                raise ValueError("truncated tfrecord: incomplete payload crc")

            yield payload
