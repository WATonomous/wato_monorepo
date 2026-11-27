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
"""Save single camera frames at given times from an MCAP on Drive (to eyeball a scene).

Usage:
    grab_frames.py <drive_file_id> <out_dir> <topic> <t1> [t2 ...]

Times are seconds from bag start. Costs a few MB per frame.
"""

import os
import sys

from drive_mcap import DriveMcap
from mcap_ros2.decoder import DecoderFactory


def main():
    file_id, out_dir, topic, times = (
        sys.argv[1],
        sys.argv[2],
        sys.argv[3],
        [float(t) for t in sys.argv[4:]],
    )
    os.makedirs(out_dir, exist_ok=True)
    bag = DriveMcap(file_id)
    chan = bag.channel(topic)
    if chan is None:
        sys.exit(
            f"{topic} not in bag; topics: {sorted(c.topic for c in bag.summary.channels.values())}"
        )
    decode = DecoderFactory().decoder_for(
        chan.message_encoding, bag.summary.schemas[chan.schema_id]
    )
    t0 = bag.start_time()
    cands = sorted(
        (ci for ci in bag.summary.chunk_indexes if chan.id in ci.message_index_offsets),
        key=lambda c: c.message_start_time,
    )
    for t in times:
        target = t0 + t * 1e9
        ci = next((c for c in cands if c.message_end_time >= target), cands[-1])
        rec = min(
            bag.chunk_messages(ci, chan.id), key=lambda r: abs(r.log_time - target)
        )
        img = decode(rec.data)
        ext = "png" if "png" in img.format else "jpg"
        path = os.path.join(
            out_dir, f"{topic.strip('/').replace('/', '_')}_{t:06.1f}s.{ext}"
        )
        with open(path, "wb") as f:
            f.write(bytes(img.data))
        print(f"{path}  (actual {(rec.log_time - t0) / 1e9:.1f}s)")
    print(f"fetched {bag.fetched / 1e6:.0f} MB of {bag.size / 1e9:.1f} GB")


if __name__ == "__main__":
    main()
