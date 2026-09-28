#!/usr/bin/env python3
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
"""Read an eidos .map (SQLite) file without ROS. Usage: read_eidos_map.py {info,tum,export} ..."""

import argparse
import math
import sqlite3
import struct
import sys


def _doubles(n):
    def decode(b):
        if len(b) < 8 * n:
            raise ValueError(f"need {8 * n} bytes, got {len(b)}")
        return list(struct.unpack_from(f"<{n}d", b))

    return decode


def _pcl_xyzi(b):
    # pcl::PointXYZI is 32 bytes: x, y, z, pad, intensity, pad[3] (float32)
    return [
        (p[0], p[1], p[2], p[4])
        for p in struct.iter_unpack("<8f", b[: len(b) // 32 * 32])
    ]


def _small_gicp(b):
    (n,) = struct.unpack_from("<Q", b)
    if len(b) < 8 + 32 * n:
        raise ValueError("truncated small_gicp_binary blob")
    return [tuple(struct.unpack_from("<4d", b, 8 + 32 * i)) for i in range(n)]


DECODERS = {
    "pcl_pcd_binary": _pcl_xyzi,
    "small_gicp_binary": _small_gicp,
    "raw_double3": _doubles(3),
    "raw_double4_eigen": _doubles(4),
    "raw_double5": _doubles(5),
}


def decode(fmt, blob):
    """Return decoded payload, or None if the format is unknown."""
    dec = DECODERS.get(fmt)
    return dec(bytes(blob)) if dec else None


def gtsam_key_str(k):
    c, idx = (k >> 56) & 0xFF, k & ((1 << 56) - 1)
    return f"{chr(c)}{idx}" if 32 < c < 127 else str(k)


class EidosMap:
    def __init__(self, path):
        self.db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        q = self.db.execute
        self.metadata = dict(q("SELECT key, value FROM metadata"))
        self.keyframes = [
            dict(
                zip(
                    (
                        "id",
                        "key",
                        "x",
                        "y",
                        "z",
                        "roll",
                        "pitch",
                        "yaw",
                        "time",
                        "owner",
                    ),
                    r,
                )
            )
            for r in q(
                "SELECT id, gtsam_key, x, y, z, roll, pitch, yaw, time, owner FROM keyframes ORDER BY id"
            )
        ]
        self.edges = q("SELECT key_a, key_b, owner FROM edges").fetchall()
        self.formats = {
            dk: (fmt, scope)
            for dk, fmt, scope in q("SELECT data_key, format, scope FROM data_formats")
        }

    def keyframe_data(self, key, data_key):
        row = self.db.execute(
            "SELECT data FROM keyframe_data WHERE gtsam_key=? AND data_key=?",
            (key, data_key),
        ).fetchone()
        return decode(self.formats.get(data_key, ("?",))[0], row[0]) if row else None

    def global_data(self, data_key):
        row = self.db.execute(
            "SELECT data FROM global_data WHERE data_key=?", (data_key,)
        ).fetchone()
        return decode(self.formats.get(data_key, ("?",))[0], row[0]) if row else None


def rpy_to_quat(r, p, y):
    cr, sr, cp, sp, cy, sy = (f(a / 2) for a in (r, p, y) for f in (math.cos, math.sin))
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def cmd_info(m, _):
    print("metadata:", m.metadata)
    print(f"keyframes: {len(m.keyframes)}  edges: {len(m.edges)}")
    owners = {}
    for e in m.edges:
        owners[e[2] or "<none>"] = owners.get(e[2] or "<none>", 0) + 1
    print("edge owners:", owners)
    for dk, (fmt, scope) in sorted(m.formats.items()):
        tbl = "keyframe_data" if scope == "keyframe" else "global_data"
        rows = m.db.execute(
            f"SELECT data FROM {tbl} WHERE data_key=?", (dk,)
        ).fetchall()
        status = "ok"
        if fmt not in DECODERS:
            status = "UNKNOWN FORMAT (not decoded)"
        else:
            try:
                sizes = [len(decode(fmt, r[0])) for r in rows]
                status = f"ok, decoded elements min/max {min(sizes, default=0)}/{max(sizes, default=0)}"
            except (ValueError, struct.error) as ex:
                status = f"DECODE ERROR: {ex}"
        print(f"  [{scope}] {dk}: {fmt}, {len(rows)} blobs, {status}")


def cmd_tum(m, a):
    out = open(a.output, "w") if a.output else sys.stdout
    for k in m.keyframes:
        qx, qy, qz, qw = rpy_to_quat(k["roll"], k["pitch"], k["yaw"])
        out.write(
            f"{k['time']:.9f} {k['x']:.6f} {k['y']:.6f} {k['z']:.6f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
        )
    if a.output:
        out.close()


def cmd_export(m, a):
    key = m.keyframes[a.index]["key"] if a.index is not None else a.key
    pts = m.keyframe_data(key, a.data_key)
    if not pts or not isinstance(pts[0], tuple):
        sys.exit(
            f"no point cloud for key {gtsam_key_str(key)} / '{a.data_key}' (format unknown or missing)"
        )
    if a.output.endswith(".npy"):
        hdr = f"{{'descr': '<f8', 'fortran_order': False, 'shape': ({len(pts)}, {len(pts[0])}), }}"
        hdr += " " * (-(len(hdr) + 11) % 64) + "\n"
        with open(a.output, "wb") as f:
            f.write(b"\x93NUMPY\x01\x00" + struct.pack("<H", len(hdr)) + hdr.encode())
            f.write(b"".join(struct.pack(f"<{len(p)}d", *p) for p in pts))
    else:
        with open(a.output, "w") as f:
            f.write(
                f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\n"
                "property float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n"
            )
            f.writelines(" ".join(f"{v:.6f}" for v in p[:4]) + "\n" for p in pts)
    print(f"wrote {len(pts)} points from {gtsam_key_str(key)} to {a.output}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("map")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    t = sub.add_parser("tum", help="dump keyframe poses in TUM format")
    t.add_argument("-o", "--output")
    e = sub.add_parser("export", help="export a keyframe cloud to .npy or .ply")
    e.add_argument("data_key")
    g = e.add_mutually_exclusive_group(required=True)
    g.add_argument("--index", type=int, help="keyframe row index")
    g.add_argument("--key", type=int, help="raw gtsam key")
    e.add_argument("-o", "--output", required=True)
    a = ap.parse_args()
    {"info": cmd_info, "tum": cmd_tum, "export": cmd_export}[a.cmd](EidosMap(a.map), a)


if __name__ == "__main__":
    main()
