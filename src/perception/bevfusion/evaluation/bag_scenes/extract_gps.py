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
"""Pull GPS fixes out of an MCAP on Google Drive without downloading it.

Reads only the MCAP summary (chunk index) plus one chunk every --every seconds, so a
20 GB bag costs ~150 MB of download and nothing on disk but a small CSV.

Usage:
    extract_gps.py <drive_file_id> <out.csv> [--every 2.0] [--topic /novatel/oem7/fix]
"""

import argparse
import csv
import sys
import time

from drive_mcap import DriveMcap
from mcap_ros2.decoder import DecoderFactory


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("file_id")
    ap.add_argument("out")
    ap.add_argument("--every", type=float, default=2.0)
    ap.add_argument("--topic", default="/novatel/oem7/fix")
    args = ap.parse_args()

    t_start = time.time()
    bag = DriveMcap(args.file_id)
    if bag.summary is None or not bag.summary.chunk_indexes:
        sys.exit(
            f"{args.out}: no chunk index in summary (unindexed/truncated bag); needs a full download"
        )
    chan = bag.channel(args.topic)
    if chan is None:
        sys.exit(f"{args.out}: topic {args.topic} not in bag")
    decode = DecoderFactory().decoder_for(
        chan.message_encoding, bag.summary.schemas[chan.schema_id]
    )

    # One chunk per --every seconds, only chunks that actually hold a GPS message.
    picked, next_t = [], 0
    for ci in sorted(bag.summary.chunk_indexes, key=lambda c: c.message_start_time):
        if chan.id in ci.message_index_offsets and ci.message_start_time >= next_t:
            picked.append(ci)
            next_t = ci.message_start_time + args.every * 1e9

    with open(args.out, "w", newline="") as out:
        w = csv.writer(out)
        w.writerow(["t", "lat", "lon"])
        for ci in picked:
            rec = bag.chunk_messages(ci, chan.id)[0]
            m = decode(rec.data)
            w.writerow(
                [f"{rec.log_time / 1e9:.2f}", f"{m.latitude:.7f}", f"{m.longitude:.7f}"]
            )
    print(
        f"{args.out}: {len(picked)} fixes from {len(bag.summary.chunk_indexes)} chunks, "
        f"{bag.fetched / 1e6:.0f} MB of {bag.size / 1e9:.1f} GB fetched in {time.time() - t_start:.0f}s",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
