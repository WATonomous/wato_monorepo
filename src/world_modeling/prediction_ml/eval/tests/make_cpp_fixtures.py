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
"""Write the small bundles read by the C++ tensor I/O tests."""

import gzip
import pathlib
import struct
import sys

import numpy as np

from wato_mtr_eval import tensor_io


def _normalize_gzip_header(path: pathlib.Path) -> None:
    """Remove gzip's wall-clock timestamp so committed fixtures are reproducible."""
    compressed = bytearray(path.read_bytes())
    compressed[4:8] = b"\x00" * 4
    path.write_bytes(compressed)


def _write_raw(path: pathlib.Path, payload: bytes) -> None:
    with gzip.open(path, "wb") as stream:
        stream.write(payload)
    _normalize_gzip_header(path)


def _tensor(name: str, dtype: int, dims: tuple[int, ...], data: bytes) -> bytes:
    encoded = name.encode("utf-8")
    return b"".join(
        (
            struct.pack("<I", len(encoded)),
            encoded,
            struct.pack("<BB", dtype, len(dims)),
            struct.pack(f"<{len(dims)}I", *dims),
            data,
        )
    )


def main(output_dir: pathlib.Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(0)
    tensor_io.write_bundle(
        output_dir / "tensor_io_sample.bin.gz",
        {
            "obj_trajs": rng.standard_normal((2, 3, 4, 5), dtype=np.float32),
            "obj_trajs_mask": rng.integers(0, 2, (2, 3, 4), dtype=np.uint8),
            "track_index_to_predict": np.arange(2, dtype=np.int64),
        },
    )
    _normalize_gzip_header(output_dir / "tensor_io_sample.bin.gz")
    tensor_io.write_bundle(
        output_dir / "tensor_io_ordered.bin.gz",
        {"ordered": np.arange(120, dtype=np.float32).reshape(2, 3, 4, 5)},
    )
    _normalize_gzip_header(output_dir / "tensor_io_ordered.bin.gz")
    tensor_io.write_bundle(
        output_dir / "tensor_io_scalar.bin.gz",
        {"scalar": np.array(42, dtype=np.int64)},
    )
    _normalize_gzip_header(output_dir / "tensor_io_scalar.bin.gz")

    _write_raw(output_dir / "tensor_io_bad_magic.bin.gz", b"XXXX" + b"\x00" * 32)
    _write_raw(
        output_dir / "tensor_io_bad_version.bin.gz",
        tensor_io.MAGIC + struct.pack("<II", tensor_io.VERSION + 1, 0),
    )

    header = tensor_io.MAGIC + struct.pack("<II", tensor_io.VERSION, 1)
    _write_raw(
        output_dir / "tensor_io_bad_dtype.bin.gz",
        header + _tensor("bad", 255, (), b""),
    )
    _write_raw(
        output_dir / "tensor_io_overflow.bin.gz",
        header + _tensor("huge", 0, (0xFFFFFFFF, 0xFFFFFFFF), b""),
    )

    duplicate = _tensor("same", 1, (1,), b"\x01")
    _write_raw(
        output_dir / "tensor_io_duplicate.bin.gz",
        tensor_io.MAGIC + struct.pack("<II", tensor_io.VERSION, 2) + duplicate + duplicate,
    )

    with gzip.open(output_dir / "tensor_io_sample.bin.gz", "rb") as stream:
        sample = stream.read()
    _write_raw(output_dir / "tensor_io_truncated.bin.gz", sample[: len(sample) // 2])
    _write_raw(output_dir / "tensor_io_trailing.bin.gz", sample + b"trailing")

    compressed = (output_dir / "tensor_io_sample.bin.gz").read_bytes()
    (output_dir / "tensor_io_truncated_gzip.bin.gz").write_bytes(compressed[:-8])


if __name__ == "__main__":
    main(pathlib.Path(sys.argv[1]))
