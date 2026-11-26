# Eval set: our drives

Prediction eval on our own bags, separate from the Waymo parity tooling one level up. numpy only, except
`make_scenes.py` (needs ROS 2).

## What is here

| Slice | Samples | Source |
|---|---|---|
| `scenes/may30_t188-236` | 89 | may_30 drive, bag seconds 188–236 |
| `scenes/may30_t348-380` | 57 | may_30 drive, bag seconds 348–380 |

One `.npz` per 0.5 s of bag time. The ground truth is the tracker's own future positions, cut at ID switches
(pseudo ground truth, not labels). **Almost every agent is a parked car**: 932 target samples, but only 26 moving samples at 2 s, from 6 distinct agents.
Growing that number is the main job; see "Growing the set".

Baseline to beat: `cv_db` (constant velocity, agents under 1 m/s stand still), 3 s all: minADE 0.443 / minFDE 0.714.

## Scene format (one `.npz` per sample; frame `map`, metres, radians, 10 Hz)

- `t` bag time (s), `slice` name.
- `track_ids` `[N]` int64 (the ego is id `-1`), `types` `[N]` int8: 0 vehicle, 1 pedestrian, 2 cyclist.
- `hist` `[N,50,9]` float32: x, y, z, yaw, vx, vy, length, width, height. Index 49 = now, 5 s back.
  `hist_valid` `[N,50]`. Velocity = finite difference of box centres over the last 0.5 s or less.
- `ego_idx`, `lanes` / `lane_types` (empty: no map on our drives).
- `fut` `[N,80,2]`: position at `t + (k+1)*0.1` s, `fut_valid` `[N,80]`.
- `targets` `[N]`: scored agents = full last second of history, ego excluded.

Prediction files mirror the scene paths: `track_ids` `[M]`, `traj` `[M,K,T,2]` (K ≤ 6, T 30–80, first step
`t + 0.1` s), `probs` `[M,K]`, `latency_ms`, `meta` = `model=...; checkpoint=...; train=...; map=yes|no`.

## Scoring

```bash
python3 cv_baseline.py scenes preds/cv/scenes                    # constant velocity
CV_DEADBAND=1.0 python3 cv_baseline.py scenes preds/cv_db/scenes
python3 score.py preds/cv_db/scenes scenes                        # one model
python3 score.py --table . results.md preds/cv preds/cv_db preds/<model>   # same samples; each writes preds/<model>/scenes
```

minADE / minFDE / miss rate (minFDE > 2 m) over the best of K modes at 2, 3, 5 and 8 s. Rows are all, per class and
"moving" (ground truth moved at least 1 m/s × horizon). Every row prints samples **and distinct agents**, and flags
fewer than 30 agents as small N. Self-checks: `python3 score.py --selftest`, `python3 pbscene.py`.

## Adding a slice

1. Replay the bag through perception with tracking `output_frame:=map`, and record
   `/perception/detections_3D_tracked`, `/localization/pose` and `/world_modeling/world_object_seeds` to an mcap.
   `make_scenes.py` refuses tracks that are not in `map`.
2. `python3 make_scenes.py <run_mcap> <bag>_t<start>-<end> scenes/<bag>_t<start>-<end> preds/prediction_ml/scenes/<bag>_t<start>-<end>`
   (the last argument is optional: it saves what prediction_ml published live, so the node itself gets scored).
   Name slices by bag seconds, and leave 2 s either side of the part you care about: samples need 1 s of history
   and the last 2 s of a slice give no samples.
3. Check the printed agent and target counts, then score.

## Growing the set

- **Count distinct moving agents, not samples**: samples 0.5 s apart on the same car are near copies. Aim for 200+
  now, and 1,000–2,000 before trusting MTR on the car, with at least 250 per group.
- **Groups**: vehicle straight, vehicle turning, lead car braking, oncoming and merging traffic, pedestrians at
  crosswalks, cyclists, parking lots. Tag each slice with what it contains.
- **Track quality**: hand-check ID switches on moving agents.
- **Bags** need lidar, cameras, `/tf_static`, localization and `/novatel/oem7/odom`. New drives should also record
  `/interfacing/can_state_estimator/odom`.
- **Hold-out**: keep some drives as a test split that nobody tunes on.
- **Reporting**: give 95% confidence intervals from a bootstrap that resamples by agent, plus the number of agents
  behind each figure.
- **Data**: our bags or car-legal datasets (Argoverse 2, nuScenes) only, never Waymo-derived files.
