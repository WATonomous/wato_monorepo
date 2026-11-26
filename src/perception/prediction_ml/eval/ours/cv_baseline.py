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
"""Constant-velocity baselines over a scenes dir. CV_DEADBAND=1.0 gives cv_db (slower agents stand still), the one to beat.
usage: [CV_DEADBAND=1.0] python3 cv_baseline.py <scenes_dir> <preds_dir>"""

import os
import pathlib
import sys

import numpy as np

DEADBAND = float(os.environ.get("CV_DEADBAND", 0))  # m/s
META = f"model={'cv_db' if DEADBAND else 'cv'}; checkpoint=none; train=none; map=no"
STEPS = 0.1 * np.arange(1, 81, dtype=np.float32)  # first step = t + 0.1 s, 8 s at 10 Hz


def predict(scene):
    cur = scene["hist"][:, 49].astype(np.float64)  # [N,9]: x,y = 0:2, vx,vy = 4:6
    vel = cur[:, 4:6] * (np.hypot(cur[:, 4], cur[:, 5]) >= DEADBAND)[:, None]
    traj = (
        cur[:, None, None, 0:2] + vel[:, None, None, :] * STEPS[None, None, :, None]
    )  # [N,K=1,80,2]
    return dict(
        track_ids=scene["track_ids"],
        traj=traj.astype(np.float32),
        probs=np.ones((len(traj), 1), np.float32),
        latency_ms=0.0,
        meta=META,
    )


if __name__ == "__main__":
    src, dst = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    for f in sorted(src.rglob("*.npz")):
        out = dst / f.relative_to(src)
        out.parent.mkdir(parents=True, exist_ok=True)
        np.savez(out, **predict(dict(np.load(f, allow_pickle=True))))
