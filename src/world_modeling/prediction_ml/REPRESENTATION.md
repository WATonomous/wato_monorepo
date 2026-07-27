# Prediction ML Representation Contract

## Purpose

This is the information contract that rung 1's Python ablation and rung 2's
C++ `SceneBuilder` must implement identically. The harness attributes their
paired delta to packing only, so any difference in the intended representation
would make that attribution meaningless.

## Agent history

Each agent has 11 fixed history slots sampled at 10 Hz: ten past samples plus
the current sample. The first and last timestamps are 1.0 second apart. History
is resampled into those slots and a missing source sample leaves its slot masked
(`mtr_types.hpp:56-57` in the design snapshot).

Fixture replay calls `build()` for every tracks callback through the scenario's
`current_time_index`; this is required to accumulate synthetic ego history.
ROS fixture timestamps add a constant positive one-second transport epoch
because zero means "unset" to `SceneBuilder`. The offset never changes relative
or scalar time. For each slot, the nearest retained sample within half of the
10 Hz period is used; an exact tie selects the later sample.

Context ranking considers every track with retained history and uses its latest
retained sample, not only tracks valid at the current Waymo step. Rows are
globally ordered by `(ego-frame distance squared, detection_id)`. The nearest
128 are kept, except that every selected target is mandatory: a target outside
the budget replaces the farthest non-target row, then the retained rows are
sorted again by the same global key.

There is exactly one self-driving-car row. Waymo's raw SDC detection is omitted
from the tracks topic and pose/odometry produce the reserved synthetic
`__ego__` row. A designated Waymo SDC target maps to `__ego__` for packing while
the raw Waymo ID remains in ground truth and the fixture identity map.

## The 29 `obj_trajs` columns

From `scene_builder.cpp:763-781` in the design snapshot:

| Cols | Content |
|---|---|
| 0–5 | `rx, ry, rz` (target-centered position), `dx, dy, dz` (box dims) |
| 6–10 | one-hot: vehicle, pedestrian, cyclist, `is_center`, `is_sdc` |
| 11–21 | time one-hot over the 11 history steps |
| 22 | scalar time, `desired_times[s] - current_time` |
| 23–24 | `sin(rel_heading)`, `cos(rel_heading)` |
| 25–26 | `rel_vx`, `rel_vy` (target-frame velocity) |
| 27–28 | `acc_x`, `acc_y` |

## Capacities

The frozen contract provides 8 target slots, 128 context agents ranked by
ego-frame distance, and 768 map chunks with 20 points each. Overflow is
silently truncated by the deployment representation; the evaluation ledger
must count every such loss.

## Map representation

Every map point has nine columns:
`[x, y, z, dir_x, dir_y, dir_z, global_type, pre_x, pre_y]`
(`scene_builder.cpp:939` in the design snapshot). `global_type` is the same
constant for every packed polyline (`scene_builder.cpp:50`).

Direction vectors are computed over each complete scene-frame centerline before
it is divided into 20-point chunks, so the first point of a later chunk retains
the preceding point's direction. Non-finite centerlines and centerlines with
fewer than two points are skipped; direction norms at or below `1e-6` become
zero. For each target, all chunks are ranked by distance from their mean center
to a point 30 m ahead of the target heading, then the closest 768 are packed.

## What WATO discards

| Information | Loss |
|---|---|
| Map polyline types | Roughly 20 Waymo types collapse to one lane-centerline token. |
| Road edges | Present as `BOUNDARY_ROAD_EDGE` in `lanelet_msgs/Lanelet`; not packed. |
| Crosswalks | Present as `lanelet_type == "crosswalk"`; not packed. |
| Lane boundaries | Left/right boundary geometry is present; not packed. |
| Traffic-light state | Genuinely unavailable: lanelet messages contain no dynamic signal state. |
| Stop lines | Present as `RegulatoryElement.ref_lines`; not packed. |

Road edges, crosswalks, boundary types, and stop lines are available on the
vehicle in `lanelet_msgs/Lanelet` and are thrown away by `SceneBuilder`. Only
dynamic traffic-light state is missing before packing.

## Output

MTR produces 6 modes with 80 steps at 0.1-second cadence. The node downsamples
that output to 15 waypoints at 0.2-second cadence over 3.0 seconds
(`output_converter.cpp:135-148` in the design snapshot).
