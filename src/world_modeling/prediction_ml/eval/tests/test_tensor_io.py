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
import gzip
import struct

import numpy as np
import pytest

from wato_mtr_eval import tensor_io


def _sample() -> dict[str, np.ndarray]:
    rng = np.random.default_rng(0)
    return {
        "obj_trajs": rng.standard_normal((2, 3, 4, 5), dtype=np.float32),
        "obj_trajs_mask": rng.integers(0, 2, (2, 3, 4), dtype=np.uint8),
        "track_index_to_predict": np.arange(2, dtype=np.int64),
    }


def test_round_trip_preserves_values_shapes_and_dtypes(tmp_path):
    original = _sample()
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, original)
    loaded = tensor_io.read_bundle(path)

    assert set(loaded) == set(original)
    for name, arr in original.items():
        assert loaded[name].dtype == arr.dtype, name
        assert loaded[name].shape == arr.shape, name
        np.testing.assert_array_equal(loaded[name], arr, err_msg=name)


def test_file_is_gzip_and_starts_with_magic(tmp_path):
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, _sample())
    with gzip.open(path, "rb") as fh:
        assert fh.read(4) == tensor_io.MAGIC


def test_rejects_unsupported_dtype(tmp_path):
    with pytest.raises(ValueError, match="unsupported dtype"):
        tensor_io.write_bundle(
            tmp_path / "b.bin.gz", {"x": np.zeros(3, dtype=np.float64)}
        )


def test_rejects_truncated_file(tmp_path):
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, _sample())
    with gzip.open(path, "rb") as fh:
        blob = fh.read()
    truncated = tmp_path / "t.bin.gz"
    with gzip.open(truncated, "wb") as fh:
        fh.write(blob[: len(blob) // 2])
    with pytest.raises(ValueError, match="truncated"):
        tensor_io.read_bundle(truncated)


def test_rejects_physically_truncated_gzip_stream(tmp_path):
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, _sample())
    blob = path.read_bytes()
    path.write_bytes(blob[:-8])
    with pytest.raises(ValueError, match="truncated"):
        tensor_io.read_bundle(path)


def test_rejects_bad_magic(tmp_path):
    path = tmp_path / "b.bin.gz"
    with gzip.open(path, "wb") as fh:
        fh.write(b"XXXX" + b"\x00" * 32)
    with pytest.raises(ValueError, match="magic"):
        tensor_io.read_bundle(path)


def test_rejects_unknown_dtype_code(tmp_path):
    path = tmp_path / "b.bin.gz"
    payload = (
        tensor_io.MAGIC
        + struct.pack("<II", tensor_io.VERSION, 1)
        + struct.pack("<I", 1)
        + b"x"
        + struct.pack("<BBI", 99, 1, 0)
    )
    with gzip.open(path, "wb") as fh:
        fh.write(payload)
    with pytest.raises(ValueError, match="dtype code"):
        tensor_io.read_bundle(path)


def test_sparse_padding_compresses_hard(tmp_path):
    """Fixtures are committable only because the buffers are mostly zeros."""
    big = {"map_polylines": np.zeros((8, 768, 20, 9), dtype=np.float32)}
    big["map_polylines"][:, :50] = 1.0  # ~6.5% occupancy, as in a real frame
    path = tmp_path / "b.bin.gz"
    tensor_io.write_bundle(path, big)
    raw = big["map_polylines"].nbytes
    assert path.stat().st_size < raw / 20, "expected >20x compression on padded buffers"
