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
"""Recorded replay (tracker output in map + localization) -> scenes/<slice>/NNNN.npz, one sample per 0.5 s.
usage (in the perception container): python3 make_scenes.py <run_mcap> <slice> <scenes_out_dir> [<preds_out_dir>]
The optional dir gets the MTR predictions prediction_ml published for exactly those sample times."""

import pathlib
import sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import pbscene

STRIDE_S, LEAD_S, TAIL_S = (
    0.5,
    1.0,
    2.0,
)  # first sample 1 s after the first tracker frame, last one 2 s before the end
TRACKED, POSE, SEEDS = (
    "/perception/detections_3D_tracked",
    "/localization/pose",
    "/world_modeling/world_object_seeds",
)
META = (
    "model=prediction_ml (live, as recorded); checkpoint=see run; train=see run; map=no"
)


def read(uri, topics):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        t.name: get_message(t.type)
        for t in r.get_all_topics_and_types()
        if t.name in topics
    }
    r.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while r.has_next():
        topic, data, _ = r.read_next()
        yield topic, deserialize_message(data, types[topic])


if __name__ == "__main__":
    uri, slice_name, out = sys.argv[1], sys.argv[2], pathlib.Path(sys.argv[3])
    mtr_out = pathlib.Path(sys.argv[4]) if len(sys.argv) > 4 else None
    tr, frames, mtr = (
        pbscene.Tracks(),
        [],
        {},
    )  # mtr: (source stamp ms, track id) -> (traj [K,80,2], conf [K])
    for topic, m in read(uri, {TRACKED, POSE, SEEDS}):
        if topic == TRACKED:
            # ponytail: map frame only (ego from /localization/pose); odom slices need the ego from odometry instead
            if m.header.frame_id != "map":
                sys.exit(
                    f"tracks are in '{m.header.frame_id}': record the replay with tracking output_frame:=map"
                )
            frames.append(tr.add_detections(m))
        elif topic == POSE:
            tr.add_ego(m)
        else:
            for o in m.objects:
                p = o.predictions
                if len(p) >= 2 and all(
                    len(x.poses) == 80 for x in p
                ):  # 1 prediction = the node's constant-velocity fallback
                    key = (
                        round(
                            (p[0].header.stamp.sec + p[0].header.stamp.nanosec * 1e-9)
                            * 1e3
                        ),
                        int(o.detection.id),
                    )
                    mtr.setdefault(
                        key,
                        (
                            np.array(
                                [
                                    [
                                        (q.pose.position.x, q.pose.position.y)
                                        for q in x.poses
                                    ]
                                    for x in p[:6]
                                ],
                                np.float32,
                            ),
                            np.array([x.conf for x in p[:6]], np.float32),
                        ),
                    )
    frames = np.array(sorted(set(frames)))
    grid = np.arange(frames[0] + LEAD_S, frames[-1] - TAIL_S, STRIDE_S)
    times = sorted(
        {frames[np.abs(frames - g).argmin()] for g in grid}
    )  # the tracker frame nearest each grid time
    out.mkdir(parents=True, exist_ok=True)
    n_agents = n_targets = n_mtr = 0
    for k, t in enumerate(times):
        s = tr.scene(t, slice_name)
        np.savez_compressed(out / f"{k:04d}.npz", **s)
        n_agents += len(s["track_ids"])
        n_targets += int(s["targets"].sum())
        got = [i for i in s["track_ids"] if (round(t * 1e3), int(i)) in mtr]
        if mtr_out is not None and got:
            mtr_out.mkdir(parents=True, exist_ok=True)
            K = max(len(mtr[(round(t * 1e3), int(i))][1]) for i in got)
            pad = lambda a: np.concatenate([a, np.repeat(a[-1:], K - len(a), 0)])  # noqa: E731  fewer modes: repeat the last
            np.savez_compressed(
                mtr_out / f"{k:04d}.npz",
                track_ids=np.array(got, np.int64),
                traj=np.stack([pad(mtr[(round(t * 1e3), int(i))][0]) for i in got]),
                probs=np.stack([pad(mtr[(round(t * 1e3), int(i))][1]) for i in got]),
                latency_ms=np.nan,
                meta=META,
            )
            n_mtr += len(set(got) & set(s["track_ids"][s["targets"]]))
    print(
        f"{slice_name}: {len(frames)} tracker frames over {frames[-1] - frames[0]:.1f} s -> {len(times)} samples, {n_agents} agents, "
        f"{n_targets} target samples, {len({i for i in tr.d if i != pbscene.EGO_ID})} track ids; MTR predicted {n_mtr} of the target samples"
    )
