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

import struct

import pytest

from wato_mtr_eval import tfrecord


def _frame(payload: bytes) -> bytes:
    """Build one TFRecord entry with placeholder CRC fields."""
    return struct.pack("<Q", len(payload)) + b"\x00" * 4 + payload + b"\x00" * 4


def test_reads_multiple_records_in_order(tmp_path):
    path = tmp_path / "x.tfrecord"
    path.write_bytes(_frame(b"alpha") + _frame(b"bravo") + _frame(b"charlie"))
    assert list(tfrecord.read_records(path)) == [b"alpha", b"bravo", b"charlie"]


def test_empty_file_yields_nothing(tmp_path):
    path = tmp_path / "x.tfrecord"
    path.write_bytes(b"")
    assert list(tfrecord.read_records(path)) == []


def test_truncated_payload_raises(tmp_path):
    path = tmp_path / "x.tfrecord"
    path.write_bytes(struct.pack("<Q", 100) + b"\x00" * 4 + b"short")
    with pytest.raises(ValueError, match="truncated"):
        list(tfrecord.read_records(path))
