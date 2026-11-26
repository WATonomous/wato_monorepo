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
"""Tracker output -> scene dict (format in README.md). numpy only.
Used by make_scenes.py (history + pseudo-GT futures from the tracker). Self-check: python3 pbscene.py"""

import numpy as np

MAX_GAP_S = 0.25  # interpolate only between tracker frames this close
MAX_SPEED = (
    40.0  # m/s between consecutive frames = ID switch: nothing is used across it
)
EPS = 1e-3
EGO_ID, EGO_SIZE = -1, (4.0936, 1.3579, 1.4090)
NUSC_TYPE = {
    0: 0,
    1: 0,
    2: 0,
    3: 0,
    4: 0,
    6: 0,
    7: 2,
    8: 1,
}  # nuScenes class id -> contract type; barrier 5 / cone 9 dropped


def quat_yaw(q):
    return float(
        np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    )


def resample(ts, X, q, t_ref, hold=False):
    """ts [n] sorted frame times, X [n,7] = x y z yaw l w h, q query times -> ([len(q),7], valid).
    x y z are linear between the bracketing frames, yaw and size come from the nearer one. Valid only when the bracketing
    frames are <= MAX_GAP_S apart and in the same ID-switch-free segment as the frame at t_ref.
    hold: also hold the last frame for up to MAX_GAP_S (ego pose arriving slightly later than the tracker frame)."""
    n = len(ts)
    seg = np.concatenate(
        [
            [0],
            np.cumsum(
                np.linalg.norm(np.diff(X[:, :2], axis=0), axis=1)
                > MAX_SPEED * np.diff(ts)
            ),
        ]
    )

    def bracket(q):
        i1 = np.clip(np.searchsorted(ts, q - EPS), 0, n - 1)
        i0 = np.where(np.abs(ts[i1] - q) < EPS, i1, np.clip(i1 - 1, 0, n - 1))
        return i0, i1

    i0, i1 = bracket(np.asarray(q, float))
    dt = ts[i1] - ts[i0]
    w = np.where(dt > 0, (q - ts[i0]) / np.where(dt > 0, dt, 1), 0.0)
    late = q > ts[-1] + EPS
    valid = (
        (q >= ts[0] - EPS)
        & (dt <= MAX_GAP_S)
        & (~late | (hold & (q <= ts[-1] + MAX_GAP_S)))
    )
    w = np.where(late, 1.0, w)
    sref = seg[bracket(np.array([min(t_ref, ts[-1])]))[1][0]]
    valid &= (seg[i0] == sref) & (seg[i1] == sref)
    out = X[np.where(w < 0.5, i0, i1)].copy()
    out[:, :3] = X[i0, :3] + w[:, None] * (X[i1, :3] - X[i0, :3])
    return out, valid


class Tracks:
    def __init__(self):
        self.d = {}  # track id -> ([t], [x y z yaw l w h], [contract type or -1])

    def add(self, t, tid, typ, row):
        ts, X, ty = self.d.setdefault(tid, ([], [], []))
        if not ts or t > ts[-1] + EPS:
            ts.append(t)
            X.append(row)
            ty.append(typ)

    def add_detections(
        self, msg
    ):  # vision_msgs/Detection3DArray from the tracker, already in the world frame
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for d in msg.detections:
            cls = next(
                (
                    int(r.hypothesis.class_id)
                    for r in d.results
                    if r.hypothesis.class_id.isdigit()
                ),
                -1,
            )
            p, s = d.bbox.center.position, d.bbox.size
            self.add(
                t,
                int(d.id),
                NUSC_TYPE.get(cls, -1),
                (p.x, p.y, p.z, quat_yaw(d.bbox.center.orientation), s.x, s.y, s.z),
            )
        return t

    def add_ego(
        self, msg
    ):  # .header.stamp + .pose (geometry_msgs/Pose) in the world frame
        p = msg.pose.position
        self.add(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            EGO_ID,
            0,
            (p.x, p.y, p.z, quat_yaw(msg.pose.orientation), *EGO_SIZE),
        )

    def prune(self, before):  # live: forget everything older than `before`
        for tid in list(self.d):
            ts, X, ty = self.d[tid]
            k = int(np.searchsorted(ts, before))
            del ts[:k], X[:k], ty[:k]
            if not ts:
                del self.d[tid]

    def scene(self, t, slice_name="live", future=True):
        qh, qf = t - 0.1 * np.arange(49, -1, -1), t + 0.1 * np.arange(1, 81)
        ids, types, H, HV, F, FV = [], [], [], [], [], []
        for tid, (ts, X, ty) in self.d.items():
            typ = (
                np.bincount(np.array(ty) + 1).argmax() - 1
            )  # most frequent class of the track so far
            if typ < 0:
                continue
            ts, X = np.array(ts), np.array(X, float)
            h, hv = resample(ts, X, qh, t, hold=tid == EGO_ID)
            if not hv[49]:
                continue
            ids.append(tid)
            types.append(typ)
            H.append(h)
            HV.append(hv)
            if future:
                f, fv = resample(ts, X, qf, t)
                F.append(f[:, :2])
                FV.append(fv)
        n = len(ids)
        H, HV = np.array(H).reshape(n, 50, 7), np.array(HV, bool).reshape(n, 50)
        v, done = np.zeros((n, 50, 2)), np.zeros((n, 50), bool)
        for j in (
            5,
            4,
            3,
            2,
            1,
        ):  # finite difference over the longest valid lag <= 0.5 s
            ok = HV[:, j:] & HV[:, :-j] & ~done[:, j:]
            v[:, j:][ok] = (H[:, j:, :2][ok] - H[:, :-j, :2][ok]) / (0.1 * j)
            done[:, j:] |= ok
        hist = np.concatenate([H[:, :, :4], v, H[:, :, 4:]], axis=2) * HV[:, :, None]
        ids = np.array(ids, np.int64)
        s = dict(
            t=float(t),
            slice=slice_name,
            track_ids=ids,
            types=np.array(types, np.int8),
            hist=hist.astype(np.float32),
            hist_valid=HV,
            ego_idx=int(np.flatnonzero(ids == EGO_ID)[0]) if EGO_ID in ids else -1,
            lanes=np.empty(0, object),
            lane_types=np.zeros(0, np.int8),
            targets=HV[:, 39:].all(1) & (ids != EGO_ID),
        )
        if future:
            FV = np.array(FV, bool).reshape(n, 80)
            s.update(
                fut=(np.array(F).reshape(n, 80, 2) * FV[:, :, None]).astype(np.float32),
                fut_valid=FV,
            )
        return s


if __name__ == "__main__":
    tr = Tracks()
    for i in range(
        240
    ):  # 20 s at 12 Hz, 10 m/s along x; track 7 teleports 50 m at t = 12 s (an ID switch)
        t = 100 + i / 12
        tr.add(
            t,
            7,
            0,
            (10 * (t - 100) + (50 if t >= 112 - EPS else 0), 5.0, 0, 0.1, 4, 2, 1.5),
        )
        tr.add(
            t, 8, 1, (0.0, 1.0 * (t - 100), 0, 1.57, 1, 1, 2)
        ) if i < 60 or i > 66 else None  # 0.58 s gap from t = 105
        tr.add(t, 9, -1, (0, 0, 0, 0, 1, 1, 1))  # cone: dropped
        tr.add(t, EGO_ID, 0, (0, 0, 0, 0, *EGO_SIZE))
    s = tr.scene(100 + 120 / 12)  # t = 110
    a, b = list(s["track_ids"]).index(7), list(s["track_ids"]).index(8)
    assert set(s["track_ids"]) == {7, 8, EGO_ID} and s["ego_idx"] == list(
        s["track_ids"]
    ).index(EGO_ID)
    assert np.allclose(
        s["hist"][a, 49], [100, 5, 0, 0.1, 10, 0, 4, 2, 1.5], atol=1e-3
    ) and np.allclose(s["hist"][a, 0, 0], 51, atol=1e-3)
    assert s["hist_valid"][a].all() and np.allclose(s["fut"][a, 9], [110, 5], atol=1e-3)
    assert (
        s["fut_valid"][a, :19].all() and not s["fut_valid"][a, 19:].any()
    )  # future stops at the ID switch (t = 112)
    assert (
        s["hist_valid"][b, 49]
        and not s["hist_valid"][b, 0]
        and not s["hist_valid"][b, 49 - 46]
        and s["hist_valid"][b, 49 - 44]
    )
    assert np.allclose(s["hist"][b, 49, 4:6], [0, 1], atol=1e-3) and list(
        s["targets"][[a, b]]
    ) == [True, True]
    s2 = tr.scene(
        100 + 150 / 12
    )  # t = 112.5, after the switch: history before t = 112 is invalid
    a = list(s2["track_ids"]).index(7)
    assert (
        s2["hist_valid"][a, 45:].all()
        and not s2["hist_valid"][a, :44].any()
        and not s2["targets"][a]
    )
    print("pbscene self-check ok")
