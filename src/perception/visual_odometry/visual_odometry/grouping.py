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
"""Software grouping of free-running (unsynchronized) camera frames into multicamera sets."""

import bisect
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

Frame = Tuple[int, Any]  # (stamp_ns, payload)


@dataclass
class FrameSet:
    """One multicamera set, stamped with the reference camera's stamp."""

    stamp_ns: int
    # same order as FrameGrouper.cameras; None = no frame within slop
    frames: List[Optional[Frame]]
    complete: bool


class FrameGrouper:
    """Group per-camera frames around the reference camera (the first in the list).

    For each reference frame, every other camera contributes its frame nearest in time, provided it
    lies within ``slop_ns``. A reference frame is finalized once every other camera has delivered a
    frame at or after it (so its nearest frame is known), or once it is ``max_wait_ns`` older than the
    newest frame seen (a camera stopped publishing). Per-camera offsets to the reference are kept so the
    lack of hardware sync is observable at runtime.
    """

    def __init__(
        self,
        cameras: Sequence[str],
        slop_ns: int,
        max_wait_ns: int = 300_000_000,
        max_buffer: int = 20,
        stats_window: int = 200,
    ):
        if len(cameras) < 1:
            raise ValueError("FrameGrouper needs at least one camera")
        self.cameras = list(cameras)
        self.slop_ns = int(slop_ns)
        self.max_wait_ns = int(max_wait_ns)
        self.max_buffer = int(max_buffer)
        self._buffers: Dict[str, List[Frame]] = {c: [] for c in self.cameras}
        self._offsets: Dict[str, deque] = {
            c: deque(maxlen=stats_window) for c in self.cameras[1:]
        }
        self._newest_ns: Optional[int] = None
        self._last_ref_ns: Optional[int] = (
            None  # stamp of the last finalized reference frame
        )
        self.num_complete = 0
        self.num_incomplete = 0

    @property
    def reference(self) -> str:
        return self.cameras[0]

    def reset(self) -> None:
        for buf in self._buffers.values():
            buf.clear()
        for off in self._offsets.values():
            off.clear()
        self._newest_ns = None
        self._last_ref_ns = None

    def add(self, camera: str, stamp_ns: int, payload: Any) -> List[FrameSet]:
        """Add one frame and return any frame sets that became final (oldest first)."""
        if (
            camera == self.reference
            and self._last_ref_ns is not None
            and stamp_ns <= self._last_ref_ns
        ):
            if self._last_ref_ns - stamp_ns <= self.max_wait_ns:
                return []  # duplicate or stale reference frame
            self.reset()  # time jumped backwards (bag loop / clock reset): start over
        buf = self._buffers[camera]
        stamps = [f[0] for f in buf]
        idx = bisect.bisect_left(stamps, stamp_ns)
        if idx < len(buf) and buf[idx][0] == stamp_ns:
            return []  # duplicate stamp from the same camera
        buf.insert(idx, (int(stamp_ns), payload))
        if len(buf) > self.max_buffer:
            del buf[0]
        self._newest_ns = (
            stamp_ns if self._newest_ns is None else max(self._newest_ns, stamp_ns)
        )
        return self._drain()

    def offset_stats(self) -> Dict[str, Dict[str, float]]:
        """Per-camera offset to the reference, in ms: mean, max |offset|, number of samples."""
        stats = {}
        for cam, offs in self._offsets.items():
            if not offs:
                continue
            arr = np.asarray(offs, dtype=float) / 1e6
            stats[cam] = {
                "mean_ms": float(arr.mean()),
                "max_abs_ms": float(np.abs(arr).max()),
                "n": len(arr),
            }
        return stats

    def _drain(self) -> List[FrameSet]:
        out: List[FrameSet] = []
        ref_buf = self._buffers[self.reference]
        others = self.cameras[1:]
        while ref_buf:
            ref_stamp, ref_payload = ref_buf[0]
            ready = all(
                self._buffers[c] and self._buffers[c][-1][0] >= ref_stamp
                for c in others
            )
            timed_out = self._newest_ns - ref_stamp > self.max_wait_ns
            if not (ready or timed_out):
                break
            frames: List[Optional[Frame]] = [(ref_stamp, ref_payload)]
            complete = True
            for cam in others:
                buf = self._buffers[cam]
                best = min(buf, key=lambda f: abs(f[0] - ref_stamp), default=None)
                if best is not None and abs(best[0] - ref_stamp) <= self.slop_ns:
                    frames.append(best)
                    self._offsets[cam].append(best[0] - ref_stamp)
                    # A frame is used at most once; anything older can no longer be nearest.
                    self._buffers[cam] = [f for f in buf if f[0] > best[0]]
                else:
                    frames.append(None)
                    complete = False
            ref_buf.pop(0)
            self._last_ref_ns = ref_stamp
            if complete:
                self.num_complete += 1
            else:
                self.num_incomplete += 1
            out.append(FrameSet(stamp_ns=ref_stamp, frames=frames, complete=complete))
        return out
