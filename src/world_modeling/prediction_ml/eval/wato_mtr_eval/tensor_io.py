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
"""Gzipped binary tensor bundle, readable from both Python and C++.

Deliberately not .npz: the C++ fixture reader then needs only zlib, which is
already in the ROS dependency tree, rather than a new npz dependency for a
test-only code path.
"""

from __future__ import annotations

import gzip
import pathlib
import struct
import zlib

import numpy as np

MAGIC = b"WMTR"
VERSION = 1

_DTYPE_CODE = {np.dtype(np.float32): 0, np.dtype(np.uint8): 1, np.dtype(np.int64): 2}
_CODE_DTYPE = {v: k for k, v in _DTYPE_CODE.items()}


def write_bundle(path: pathlib.Path, tensors: dict[str, np.ndarray]) -> None:
    """Write tensors to a gzipped bundle. Arrays are stored C-order."""
    parts = [MAGIC, struct.pack("<II", VERSION, len(tensors))]
    for name, arr in tensors.items():
        dtype = np.dtype(arr.dtype)
        if dtype not in _DTYPE_CODE:
            raise ValueError(f"unsupported dtype {dtype} for tensor {name!r}")
        arr = np.ascontiguousarray(arr)
        encoded = name.encode("utf-8")
        parts.append(struct.pack("<I", len(encoded)))
        parts.append(encoded)
        parts.append(struct.pack("<BB", _DTYPE_CODE[dtype], arr.ndim))
        parts.append(struct.pack(f"<{arr.ndim}I", *arr.shape))
        parts.append(arr.tobytes(order="C"))

    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, "wb") as fh:
        fh.write(b"".join(parts))


def read_bundle(path: pathlib.Path) -> dict[str, np.ndarray]:
    """Read a bundle written by write_bundle."""
    try:
        with gzip.open(path, "rb") as fh:
            blob = fh.read()
    except (EOFError, gzip.BadGzipFile, zlib.error) as error:
        raise ValueError("truncated or invalid gzip bundle") from error

    def need(offset: int, count: int) -> int:
        if offset + count > len(blob):
            raise ValueError(f"truncated bundle: need {count} bytes at {offset}")
        return offset + count

    if len(blob) < 12 or blob[:4] != MAGIC:
        raise ValueError(f"bad magic: expected {MAGIC!r}")
    version, count = struct.unpack_from("<II", blob, 4)
    if version != VERSION:
        raise ValueError(f"unsupported bundle version {version}")

    out: dict[str, np.ndarray] = {}
    off = 12
    for _ in range(count):
        off = need(off, 4)
        (name_len,) = struct.unpack_from("<I", blob, off - 4)
        off = need(off, name_len)
        name = blob[off - name_len : off].decode("utf-8")
        off = need(off, 2)
        dtype_code, ndim = struct.unpack_from("<BB", blob, off - 2)
        off = need(off, 4 * ndim)
        dims = struct.unpack_from(f"<{ndim}I", blob, off - 4 * ndim)

        try:
            dtype = _CODE_DTYPE[dtype_code]
        except KeyError as error:
            raise ValueError(f"unsupported dtype code {dtype_code}") from error
        nbytes = int(np.prod(dims, dtype=np.int64)) * dtype.itemsize
        off = need(off, nbytes)
        if name in out:
            raise ValueError(f"duplicate tensor name {name!r}")
        out[name] = (
            np.frombuffer(
                blob,
                dtype=dtype,
                count=int(np.prod(dims, dtype=np.int64)),
                offset=off - nbytes,
            )
            .reshape(dims)
            .copy()
        )

    if off != len(blob):
        raise ValueError(f"trailing data after tensor bundle: {len(blob) - off} bytes")
    return out
