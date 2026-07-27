# Road-edge fidelity spike

Date: 2026-07-27

This spike tests whether adding road edges is primarily a SceneBuilder change
or requires a new transport contract. It does not change the production map
schema.

## Scope

All 286 scenarios in the first Waymo Motion validation shard were scanned.
Three edge-focused scenarios were selected:

| Scenario | Selection | Road edges | Points |
|---|---|---:|---:|
| `e633aa95b934cb1e` | densest edge geometry | 154 | 15,847 |
| `d42aa9e7ae47c9a` | most median geometry | 65 | 8,978 |
| `9fe8b30f6884e198` | most internal discontinuities | 49 | 7,570 |

The comparison therefore covered 268 road-edge features and 32,395 points.

## Findings

### Raw Waymo to harness parser

The harness preserved feature ID, order, point count, type, and float64 XYZ
coordinates exactly. Waymo road-edge types `0/1/2` mapped to the MTR global
types `-1/15/16` (unknown/boundary/median) exactly.

### Raw Waymo to official MTR training representation

The pinned official MTR decoder preserved feature IDs, order, point counts, and
types. Its coordinates equal a direct float32 cast of the Waymo coordinates;
the largest observed quantization error was 0.000244141 m.

The spike also invoked MTR's executable
`WaymoDataset.generate_batch_polylines_from_map`. Reconstructed values, masks,
padding, and chunk counts matched it exactly.

MTR's batching semantics are not simply "20 points from each feature":

- 38 adjacent road-edge point pairs exceeded the 1.0 m break threshold and
  caused a split.
- Eight adjacent feature boundaries involving a road edge were at most 1.0 m,
  allowing a chunk to contain points from more than one feature ID.

### Availability after WATO-style queries

| Query | Retained edges | Retained points |
|---|---:|---:|
| Raw selected scenarios | 268 | 32,395 |
| 80 m radius | 83 | 14,062 |
| Approximate forward reachable | 50 | 8,423 |

The fixed 768-polyline capacity evicted 21--84 edge-touching chunks per target
from the radius-query candidates in these cases. The smaller reachable-query
candidate sets did not suffer capacity eviction.

### Current production transport

The current `LaneletAhead` path is not road-edge equivalent:

- left/right boundary geometry is attached to lanelets rather than carried as
  standalone typed features;
- there is no standalone boundary feature ID;
- the message cannot distinguish Waymo/MTR boundary type 15 from median type
  16;
- the Waymo replay converter currently leaves boundary arrays empty; and
- `prediction_ml::SceneBuilder` consumes centerlines only.

Parser fidelity therefore does not de-risk the production implementation by
itself.

## Decision and implementation impact

Road edges require an additive typed map-feature transport with stable feature
IDs, MTR semantic types, and XYZ point arrays. SceneBuilder must then reproduce
the pinned 1.0 m break and 20-point chunking semantics before applying the
768-chunk target-relative ranking budget.

Production implementation should proceed in this order:

1. Add and test the typed map-feature message contract.
2. Populate standalone road edges in both the vehicle world-model producer and
   Waymo replay converter.
3. Verify point/type/ID dumps against the training-format oracle used here.
4. Implement MTR-compatible split, merge, direction, ranking, and truncation in
   Python and C++.
5. Regenerate fixtures and rerun representation parity, checkpoint scoring,
   TensorRT parity, and ROS replay.

The full coordinate dump and detailed per-target eviction ledger remain local,
ignored build artifacts under `eval/build/road_edge_fidelity_spike/`.
