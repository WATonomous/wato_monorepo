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
"""Random access to an MCAP in a public Google Drive folder via HTTP range requests.

MCAP ends with a summary listing every chunk's byte offset and time range, so we can read
just the summary and then the few chunks we need instead of downloading the whole file.
"""

import io
import time

import requests
from mcap.reader import SeekingReader, breakup_chunk
from mcap.records import Chunk, Message
from mcap.stream_reader import StreamReader


class DriveMcap:
    """Seekable file-like view of a Drive file, plus chunk-level MCAP helpers."""

    BLOCK = 1 << 20

    def __init__(self, file_id):
        self.url = f"https://drive.usercontent.google.com/download?id={file_id}&export=download&confirm=t"
        self.session = requests.Session()
        self.session.headers["User-Agent"] = "wato-bag-scene-finder/0.1"
        self.fetched = 0
        r = self.session.get(self.url, headers={"Range": "bytes=0-0"}, timeout=60)
        if r.status_code != 206:
            raise RuntimeError(f"no range support / throttled: HTTP {r.status_code}")
        self.size = int(r.headers["Content-Range"].split("/")[1])
        self.pos = 0
        self.cache = {}
        self.summary = SeekingReader(self).get_summary()

    def get_range(self, start, end):
        """Bytes [start, end) with retries; Drive sometimes answers with an HTML throttle page."""
        err = ""
        for attempt in range(8):
            try:
                r = self.session.get(
                    self.url, headers={"Range": f"bytes={start}-{end - 1}"}, timeout=60
                )
                if r.status_code == 206 and not r.content.startswith(b"<!DOCTYPE"):
                    self.fetched += len(r.content)
                    return r.content
                err = f"HTTP {r.status_code}"
            except requests.RequestException as e:
                err = str(e)
            time.sleep(min(60, 3 * 2**attempt))
        raise RuntimeError(f"range {start}-{end} failed: {err}")

    # --- file-like interface for mcap's SeekingReader ---
    def seekable(self):
        return True

    def seek(self, off, whence=0):
        self.pos = {0: off, 1: self.pos + off, 2: self.size + off}[whence]
        return self.pos

    def tell(self):
        return self.pos

    def read(self, n=-1):
        end = self.size if n is None or n < 0 else min(self.size, self.pos + n)
        out = bytearray()
        while self.pos < end:
            i = self.pos // self.BLOCK
            if i not in self.cache:
                if len(self.cache) > 64:
                    self.cache.clear()
                s = i * self.BLOCK
                self.cache[i] = self.get_range(s, min(self.size, s + self.BLOCK))
            b = self.cache[i]
            off = self.pos % self.BLOCK
            take = b[off : off + (end - self.pos)]
            out += take
            self.pos += len(take)
        return bytes(out)

    # --- MCAP helpers ---
    def channel(self, topic):
        return next(
            (c for c in self.summary.channels.values() if c.topic == topic), None
        )

    def chunk_messages(self, ci, channel_id):
        """All messages of channel_id inside one chunk index entry."""
        raw = self.get_range(
            ci.chunk_start_offset, ci.chunk_start_offset + ci.chunk_length
        )
        records = StreamReader(
            io.BytesIO(raw), skip_magic=True, emit_chunks=True
        ).records
        chunk = next(r for r in records if isinstance(r, Chunk))
        return [
            r
            for r in breakup_chunk(chunk, validate_crc=False)
            if isinstance(r, Message) and r.channel_id == channel_id
        ]

    def start_time(self):
        return min(ci.message_start_time for ci in self.summary.chunk_indexes)
