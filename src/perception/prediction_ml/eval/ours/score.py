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
"""Prediction scorer (numpy only). Definitions: README.md "Scoring".
score.py <preds_dir> <scenes_dir>                      one model, its own coverage
score.py --table <scenes_root> <out.md> <preds/modelA> <preds/modelB> ...
                                                       all models on the SAME (agent, sample) set, per dataset dir
score.py --selftest"""

import pathlib
import sys
import numpy as np

H = {2: 19, 3: 29, 5: 49, 8: 79}  # horizon s -> step index (step k = t + (k+1)*0.1 s)
WMISS = {
    3: (1.0, 2.0),
    5: (1.8, 3.6),
    8: (3.0, 6.0),
}  # Waymo (lateral, longitudinal) m, scaled by initial speed
SUBSETS = ["all", "vehicle", "pedestrian", "cyclist", "moving"]


def score_agent(traj, fut, fut_valid, cur, fut_yaw=None):
    """traj [K,T,2], fut [80,2], cur = hist[49] -> [4 horizons, 5] = minADE, minFDE, miss (2 m), Waymo miss, moving.
    A horizon the prediction does not reach or without ground truth at that step is a nan row."""
    out = np.full((len(H), 5), np.nan)
    T = min(traj.shape[1], 80)
    traj = traj[:, :T].astype(np.float64)
    d = np.linalg.norm(traj - fut[None, :T], axis=-1)  # [K,T]
    for r, (h, s) in enumerate(H.items()):
        if s >= T or not fut_valid[s]:
            continue
        v = fut_valid[: s + 1]
        fde = d[:, s].min()
        out[r, :3] = (d[:, : s + 1] * v).sum(1).min() / v.sum(), fde, fde > 2.0
        if fut_yaw is not None and h in WMISS:
            lat_t, lon_t = np.array(WMISS[h]) * np.clip(
                0.5 + 0.5 * (np.hypot(cur[4], cur[5]) - 1.4) / (11.0 - 1.4), 0.5, 1.0
            )
            e, c, sn = traj[:, s] - fut[s], np.cos(fut_yaw[s]), np.sin(fut_yaw[s])
            out[r, 3] = not np.any(
                (np.abs(-e[:, 0] * sn + e[:, 1] * c) <= lat_t)
                & (np.abs(e[:, 0] * c + e[:, 1] * sn) <= lon_t)
            )
        out[r, 4] = np.linalg.norm(fut[s].astype(np.float64) - cur[:2]) >= 1.0 * h
    return out


def load(preds_dir, scenes_dir):
    """-> dict (relative path, track id) -> (type, (slice, track id), metrics [4,5]); meta string; targets total; predicted"""
    rows, meta, n_targets, n_pred = {}, "", 0, 0
    for f in sorted(scenes_dir.rglob("*.npz")):
        s = np.load(f, allow_pickle=True)
        tgt = np.flatnonzero(s["targets"])
        n_targets += len(tgt)
        p = preds_dir / f.relative_to(scenes_dir)
        if not p.exists():
            continue
        z = np.load(p, allow_pickle=True)
        meta = meta or str(z["meta"])
        idx = {int(t): j for j, t in enumerate(z["track_ids"])}
        traj, yaw = z["traj"], (s["fut_yaw"] if "fut_yaw" in s else None)
        hist, fut, fv, ids, types, sl = (
            s["hist"],
            s["fut"],
            s["fut_valid"],
            s["track_ids"],
            s["types"],
            str(s["slice"]),
        )
        for i in tgt:
            j = idx.get(int(ids[i]))
            if (
                j is None or not np.isfinite(traj[j]).all()
            ):  # not predicted (or garbage): shows up as lost coverage
                continue
            n_pred += 1
            rows[(str(f.relative_to(scenes_dir)), int(ids[i]))] = (
                int(types[i]),
                (sl, int(ids[i])),
                score_agent(
                    traj[j],
                    fut[i],
                    fv[i],
                    hist[i, 49].astype(np.float64),
                    None if yaw is None else yaw[i],
                ),
            )
    return rows, meta, n_targets, n_pred


def stats(rows, keys_per_h):
    """-> {(subset, h): (n samples, n agents, minADE, minFDE, miss, waymo miss)} over keys_per_h[h] (None = everything scored)"""
    out = {}
    for r, h in enumerate(H):
        sel = [
            (t, a, m[r])
            for k, (t, a, m) in rows.items()
            if np.isfinite(m[r, 1]) and (keys_per_h is None or k in keys_per_h[h])
        ]
        for sub in SUBSETS:
            pick = [
                x
                for x in sel
                if sub == "all"
                or (sub == "moving" and x[2][4] == 1)
                or SUBSETS.index(sub) - 1 == x[0]
            ]
            if pick:
                m = np.array([x[2] for x in pick])
                out[(sub, h)] = (
                    len(pick),
                    len({x[1] for x in pick}),
                    *m[:, :3].mean(0),
                    m[:, 3].mean() if np.isfinite(m[:, 3]).all() else np.nan,
                )
    return out


def cell(st):
    if st is None:
        return "-"
    n, na, ade, fde, mr, wmr = st
    return f"{ade:.3f} / {fde:.3f} / {100 * mr:.1f}%" + (
        "" if np.isnan(wmr) else f" / W {100 * wmr:.1f}%"
    )


def field(meta, key):
    return next(
        (
            p.split("=", 1)[1].strip()
            for p in meta.split(";")
            if p.strip().startswith(key + "=")
        ),
        "?",
    )


def single(preds_dir, scenes_dir):
    rows, meta, n_targets, n_pred = load(preds_dir, scenes_dir)
    print(
        f"{preds_dir} vs {scenes_dir}\nmeta: {meta}\ncoverage: {n_pred} of {n_targets} target samples predicted"
    )
    print(
        "subset      horizon  samples agents   minADE / minFDE / miss(>2 m) [/ W = Waymo speed-scaled miss]"
    )
    st = stats(rows, None)
    for sub in SUBSETS:
        for h in H:
            if (sub, h) in st:
                n, na = st[(sub, h)][:2]
                print(
                    f"{sub:11s} {h} s     {n:7d} {na:6d}   {cell(st[(sub, h)])}{'   (small N)' if na < 30 else ''}"
                )


def table(scenes_root, out_md, model_dirs):
    lines = []
    for ds in sorted(p for p in scenes_root.iterdir() if p.is_dir()):
        models = [
            (m.name, *load(m / ds.name, ds))
            for m in model_dirs
            if (m / ds.name).is_dir()
        ]
        if not models:
            continue
        keys = {}  # per horizon: samples every model that reaches the horizon has scored
        for r, h in enumerate(H):
            sets = [
                {k for k, v in rows.items() if np.isfinite(v[2][r, 1])}
                for _, rows, *_ in models
            ]
            sets = [s for s in sets if s]
            keys[h] = set.intersection(*sets) if sets else set()
        sts = [
            (name, stats(rows, keys), meta, nt, npred)
            for name, rows, meta, nt, npred in models
        ]
        lines += [
            f"## {ds.name}: {len(models)} models on the same (agent, sample) set",
            "",
            "Cells: minADE / minFDE / miss rate (minFDE > 2 m)"
            + (" / W = Waymo speed-scaled miss rate" if "womd" in ds.name else "")
            + ", metres, best of the model's K <= 6 modes. `-` = the model does not predict that far.",
            "",
        ]
        lines += [
            f"- `{name}`: {field(meta, 'model')}; train = **{field(meta, 'train')}**; map = {field(meta, 'map')}; "
            f"predicted {npred} of {nt} target samples"
            for name, _, meta, nt, npred in sts
        ] + [""]
        for sub in SUBSETS:
            ref = [
                next((st[(sub, h)] for _, st, *_ in sts if (sub, h) in st), None)
                for h in H
            ]
            if not any(ref):
                continue
            head = [
                f"{h} s (N = {x[0]} samples / {x[1]} agents{', small N' if x[1] < 30 else ''})"
                if x
                else f"{h} s"
                for h, x in zip(H, ref)
            ]
            lines += [
                f"### {ds.name} / {sub}",
                "",
                "| model | train | " + " | ".join(head) + " |",
                "|---|---|" + "---|" * len(H),
            ]
            lines += [
                f"| {name} | {field(meta, 'train')} | "
                + " | ".join(cell(st.get((sub, h))) for h in H)
                + " |"
                for name, st, meta, *_ in sts
            ]
            lines.append("")
    out_md.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def selftest():
    fut = np.stack([np.arange(1, 81) * 0.5, np.zeros(80)], 1)  # 5 m/s along x
    valid = np.ones(80, bool)
    valid[60:] = False
    cur = np.array([0, 0, 0, 0, 5.0, 0, 4, 2, 1.5])
    traj = np.stack([fut + [0, 1.0], fut + [3.0, 0]])[
        :, :50
    ]  # mode 0: 1 m lateral; mode 1: 3 m longitudinal; T = 50
    m = score_agent(traj, fut, valid, cur, np.zeros(80))
    assert np.allclose(m[0], [1, 1, 0, np.nan, 1], equal_nan=True) and np.allclose(
        m[1, :4], [1, 1, 0, 1]
    )  # 3 s: 1 m lat > 1.0*0.6875
    assert (
        np.allclose(m[2, :4], [1, 1, 0, 0]) and np.isnan(m[3]).all()
    )  # 5 s: within 1.8*0.6875; 8 s: beyond T and no GT
    print("selftest ok")


if __name__ == "__main__":
    a = sys.argv[1:]
    if a[0] == "--selftest":
        selftest()
    elif a[0] == "--table":
        table(pathlib.Path(a[1]), pathlib.Path(a[2]), [pathlib.Path(x) for x in a[3:]])
    else:
        single(pathlib.Path(a[0]), pathlib.Path(a[1]))
