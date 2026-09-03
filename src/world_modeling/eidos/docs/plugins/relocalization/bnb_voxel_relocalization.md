# BnbVoxelRelocalization

**Class:** `eidos::BnbVoxelRelocalization`
**XML:** `relocalization_plugins.xml`

GPS-free global relocalization. Given only a prior `.map` file, one LiDAR scan, and IMU gravity, it recovers the map-frame pose with no positional prior. It builds a multi-resolution sparse-hash voxel pyramid from the prior map's keyframe clouds, runs a best-first branch-and-bound search over 4-DOF `(x, y, z, yaw)` with a small roll/pitch range, and refines the winner with `small_gicp` GICP for a full 6-DOF pose. It is the fallback for when `GpsIcpRelocalization` cannot run.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `lidar_topic` | string | `"/lidar/points"` | Input LiDAR PointCloud2 topic. |
| `imu_topic` | string | `"/imu/data"` | Input IMU topic (for roll/pitch). |
| `lidar_frame` | string | `"lidar"` | TF frame of the LiDAR sensor. |
| `imu_frame` | string | `"imu_link"` | TF frame of the IMU sensor. |
| `pointcloud_from` | string | `"liso_factor/cloud"` | MapManager data key prefix for retrieving prior map keyframe point clouds. |
| `min_voxel_size` | double | `1.0` | Finest voxel pyramid resolution (meters). |
| `num_levels` | int | `4` | Number of pyramid levels; coarsest level is `min_voxel_size * 2^(num_levels-1)`. |
| `score_mode` | string | `"distance_field"` | `"distance_field"` (new default) or `"occupancy"` (today's ternary hit/unknown/free score, retained to A/B the two scorers against the same map and scan). An unrecognised value WARNs and falls back to `"distance_field"` rather than throwing. See "The distance-field score" below. |
| `df_sigma` | double | `1.0` | Distance-field falloff sigma (meters); level-0 cell value is `round(255 * exp(-d^2 / (2*sigma^2)))`. Set to roughly the registration offset the field should absorb (measured ~1 m on `ring_road.map`). |
| `df_truncation_voxels` | int | `2` | Distance-field kernel radius, in level-0 voxels. Cells farther than `df_truncation_voxels * min_voxel_size` from the nearest map point are not stored (score 0). |
| `max_score_voxels` | int | `40000000` | Guard on the distance-field grid size. On overflow, the pyramid falls back to occupancy-equivalent scoring for the session and the plugin WARNs. |
| `min_height` | double | `0.6` | Lower bound of the **body-frame** height band applied identically to the prior map, the live scan, and (via `Config::free_min_height`) the free-space channel below. Removes ground returns, which match almost anywhere and otherwise make the score reflect map coverage rather than alignment. `0` disables. |
| `max_height` | double | `6.0` | Upper bound of the same body-frame height band, also reused as `Config::free_max_height`. `0` disables. |
| `max_voxels_per_level` | int | `8000000` | Above this voxel count, neighbourhoods are probed on the fly instead of dilating the level. |
| `max_query_range` | double | `40.0` | Discard query points beyond this horizontal range. Also caps the coarsest yaw-bin count, which controls how many seed nodes the search must expand. `<= 0` disables. |
| `target_query_points` | int | `400` | Live scan downsample target for the branch-and-bound query. |
| `rp_search_range` | double | `0.02` | Roll/pitch half-range searched around the IMU gravity estimate (radians). |
| `rp_search_steps` | int | `1` | Roll/pitch outer grid steps per axis. |
| `prefer_downsampled_source` | bool | `false` | Prefer the pre-downsampled `liso_factor/gicp_cloud` sibling key when rasterizing the prior map. Much faster to build and loses little at a 1 m finest voxel. |
| `root_prefilter_points` | int | `128` | Query subsample size used to cheaply score roots before full search. |
| `root_prefilter_keep` | int | `256` | Number of highest-scoring roots passed into full branch-and-bound. `<= 0` disables the prefilter. |
| `search_corridor` | double | `20.0` | Search radius (meters) around the prior map's driven trajectory. |
| `z_margin` | double | `5.0` | Vertical search margin about the trajectory height (meters). |
| `max_search_nodes` | int | `200000` | Total branch-and-bound node budget per search, split across concurrent tasks. Makes the search anytime so it terminates within `relocalization_timeout`; raise for a more exhaustive (slower) search. |
| `prune_slack` | double | `0.8` | Relaxes branch-and-bound pruning so spatially distinct runners-up survive for the uniqueness gate. |
| `nms_radius` | double | `5.0` | Minimum separation (meters) between hypotheses considered spatially distinct. |
| `min_match_score` | double | `0.45` | Minimum **normalized** branch-and-bound score (`raw / max_possible`, in both score modes) required to accept a hypothesis. Under `score_mode: occupancy` this is the ternary score (`max_possible = hit_weight * n`); an all-unknown pose floors at `1/hit_weight` (0.33 at the default) rather than 0 -- see the free-space parameters below. Under `score_mode: distance_field`, `normalized` is instead the **mean per-point cell value scaled to [0,1]** (`max_possible = 255 * n`), not a hit fraction -- the default value is unchanged but means something different; see `Hypothesis::hit_fraction`, logged alongside `normalized` everywhere, for the mode-independent hit-rate figure. |
| `min_score_ratio` | double | `1.20` | Winning hypothesis's normalized score must beat the best spatially distinct runner-up's by this factor, in either score mode. |
| `num_gicp_candidates` | int | `5` | Number of top branch-and-bound hypotheses handed to GICP refinement. |
| `min_inlier_ratio` | double | `0.30` | Minimum GICP inlier ratio required to accept the refined pose. |
| `scan_ds_resolution` | double | `0.5` | Voxel downsample resolution for the live LiDAR scan before GICP (meters). |
| `submap_radius` | double | `40.0` | Radius for assembling the prior map submap used in GICP refinement (meters). |
| `submap_leaf_size` | double | `0.4` | Voxel downsample leaf size for the GICP submap (meters). |
| `max_correspondence_distance` | double | `2.0` | GICP max correspondence distance (meters). |
| `max_icp_iterations` | int | `100` | GICP max iterations. |
| `num_threads` | int | `16` | Thread count for GICP and preprocessing. |
| `num_neighbors` | int | `10` | Number of neighbors for normal/covariance estimation. |
| `publish_debug_grid` | bool | `true` | Whether to publish a latched debug OccupancyGrid of the voxel pyramid. |
| `debug_grid_topic` | string | `"slam/visualization/reloc_voxel_grid"` | Topic for the published debug voxel grid. |
| `use_free_space` | bool | `false` | Master switch for the free-space scoring channel (see Status below). Only meaningful under `score_mode: occupancy`; under `score_mode: distance_field` the channel is never built or raycast at all, because a distance field already scores a point far from all structure at ~0, continuously, which is what this channel was approximating. `false` reproduces today's binary hit-count score exactly -- ranking is bit-identical. Off by default: the measurement that would validate it turned out to be inconclusive rather than negative -- see Status. |
| `free_rays_per_keyframe` | int | `2000` | Stride each keyframe's cloud down to about this many free-space rays. `<= 0` uses every point. |
| `free_max_range` | double | `40.0` | Clamp free-space ray length (meters); bounds raycast cost. |
| `free_end_margin` | double | `1.0` | Stop each ray this many metres short of its endpoint, so the surface itself is never marked free. |
| `free_clear_near_occupied` | bool | `true` | At pyramid finalize, delete every level-0 free voxel that is occupied or 26-adjacent to an occupied voxel, so registration jitter and vegetation near real structure are not penalised. |
| `max_free_voxels` | int | `20000000` | Budget guard on the free-voxel set. On overflow, free space is abandoned entirely for the session (all free sets cleared) -- sound, since it only loosens the branch-and-bound bound. |
| `free_ray_origin_height` | double | `2.0` | Fallback free-space ray origin height (meters) above `base_link`, used only until the `base_link<-lidar` TF has resolved. The real ray origin is the resolved TF's translation (the LiDAR sits well above `base_link`, which is `base_footprint` at ground level) -- see Status below. |
| `hit_weight` | int | `3` | Per-point weight `W` for an occupied hit in the ternary score. Raw score is in `[0, W*n]`; `unknown` contributes `1`, `known-free` contributes `0`. |

## Notes

- Runs on a background worker thread; `tryRelocalize()` returns `std::nullopt` until a result is ready and never blocks the SLAM loop.
- The pyramid is built once on the first attempt from clouds already resident in memory after `loadMap`, so there is no map schema change and existing `.map` files work unmodified. Build cost is seconds.
- **Pyramid memory is recovered, not just dropped, once the plugin is done with it** -- after a successful lock, on `deactivate()`, and in the destructor. `VoxelPyramid::releaseMemory()` is used rather than `clear()`: it additionally `malloc_trim()`s under glibc, because freeing the pyramid's buffers does not by itself guarantee the OS reclaims that memory (freed heap normally stays in the allocator's arena). Each release logs process RSS immediately before and after (`pyramid released (<context>): RSS X MB -> Y MB`), so the recovery is checkable in a field log rather than assumed -- this was an explicit condition of accepting the distance field's larger memory footprint (see below). Deactivating also resets the build flags, so a later `activate()` rebuilds a fresh pyramid rather than searching an emptied one.
- Requires the prior map to be gravity-aligned. Maps built by eidos satisfy this because LISO gravity-aligns during IMU warmup; imported maps may not.
- Search is restricted to a corridor around the prior map's driven trajectory (`search_corridor`), since the vehicle can only be near where the map was recorded.
- Level 0 of the pyramid is exact; coarser levels are dilated by their 26-neighbourhood to form a valid branch-and-bound upper bound (occupancy), or max-dilated with the distance-field kernel to form the same kind of upper bound (distance field) -- see "The distance-field score" below.
- When `use_free_space` is enabled (only meaningful under `score_mode: occupancy`), free-space rays are cast in the same serial pass as occupancy insertion, from the resolved `base_link<-lidar` TF translation (falling back to `free_ray_origin_height` above `base_link` if that TF has not resolved yet) -- see "The ternary score" under Status for why this exists and why it is only active at pyramid levels 0-1. Under `score_mode: distance_field` this raycast is skipped entirely at build time, not merely built and unused.
- Acceptance requires all of: GICP convergence, `min_inlier_ratio`, `min_match_score`, and the uniqueness gate `min_score_ratio` against the best spatially distinct runner-up -- both now evaluated on the normalized score, in either score mode.
- This is initial-lock only. Once the node reaches TRACKING the plugin is never polled again; steady-state localization is LISO scan-to-submap matching.
- Set `publish_debug_grid` and view `debug_grid_topic` in RViz overlaid on `slam/visualization/map` to confirm the pyramid aligns with the map.

## Known Limitations

- **Geometrically degenerate environments will not lock.** Long tunnels and highway straights (translation along the axis is unobservable), featureless open fields or large empty lots (no structure to score), and symmetric parking structures or warehouse aisles (many equally good hypotheses). No single-scan method resolves these. The plugin is designed to return `std::nullopt` and let `relocalization_timeout` expire rather than lock onto an arbitrary hypothesis; the `min_score_ratio` uniqueness gate is what enforces that. Note the deliberate trade: a timeout starts the graph at the origin, which is a visible failure, whereas a confident wrong lock is not recoverable because eidos has no `TRACKING -> RELOCALIZING` transition.
- **Requires reliable IMU gravity.** Roll/pitch error beyond `rp_search_range` prevents a lock.
- **Requires point clouds in the map.** A `.map` with poses but no `pointcloud_from` blobs cannot be rasterized.
- **Scale ceiling.** Validated in the literature to roughly 430 x 400 m; targeted here to about 1 km squared of driven area. Beyond that a place-recognition descriptor prefilter would be needed to generate candidates.
- **Long-term map change.** Construction, foliage seasonality, or site rearrangement degrades scores; `min_match_score` is the knob and a map rebuild is the fix.
- **Dynamic occlusion.** Large nearby vehicles occluding the scan reduce the score and increase search time; retrying on later scans covers the transient case.
- **Not currently expected to lock on vegetated or otherwise geometrically saturated routes under `score_mode: occupancy`.** Measured on `ring_road.map`: a randomly rotated scan already hits 76% of its query points at +/-1 voxel, because the 0.6-6.0 m height band is very nearly space-filling along that tree-lined route -- binary occupancy carries almost no alignment signal there (see Status). `score_mode: distance_field` was implemented as the candidate fix and MEASURED NOT TO HELP on this map (true-yaw rank 7-14 of 36 versus occupancy's 5 of 36; see Status). Before relying on the plugin on a new map, run `debug_probe_pose` there and check the active-score-mode chance baseline's true-yaw rank among 36 sampled yaws; a rank far from 1/36 means that mode is not discriminating poses on that map either.

- **The root prefilter is a heuristic.** To keep the search inside its time budget on large maps, candidate root cells are first scored cheaply (a `root_prefilter_points` subsample at the coarsest pyramid level) and only the best `root_prefilter_keep` are passed to the full branch-and-bound. This can in principle discard the true pose. It is mitigated because the coarse level is dilated, so the correct location scores generously, and because any surviving wrong candidate must still pass the GICP inlier ratio and the uniqueness gate. Set `root_prefilter_keep: 0` to disable the prefilter and search every root exhaustively, at substantially higher cost.
- **Search cost grows with map size.** Measured on a 1.4 km x 1.3 km map (992 keyframes): pyramid build tens of seconds, and the search does not complete within a 30 s budget. `relocalization_timeout` is set to 180 s in the example config for this reason. If relocalization times out, the node starts from the origin, which for prior-map localization is a silent failure -- watch `slam/status` to confirm a real lock. The levers, in order of effect, are `root_prefilter_keep`, `search_corridor`, `target_query_points`, and `min_voxel_size`.
- **Free space is approximate near occlusion boundaries.** It is derived from a subsampled raycast (`free_rays_per_keyframe`) per keyframe rather than a dense per-pixel scan, so thin gaps behind occluding structure can be missed or coarsely rounded off. `free_clear_near_occupied` mitigates the worst case (free voxels butting directly against real structure) but does not eliminate it.

## Status

**Distance-field scoring is IMPLEMENTED AND MEASURED NOT TO FIX THE LOCK on this map.**

Measured with `debug_probe_pose` at the verified ground-truth pose, 10 samples:

| Statistic | `occupancy` | `distance_field` |
|---|---|---|
| True yaw's rank among 36 sampled yaws | 5 / 36 | 7-14 / 36 |
| `true / mean` normalized score | 1.166 | 1.052 - 1.117 |
| Mean normalized score over 36 yaws | 0.328 (hit fraction) | 0.583 |

The distance field is slightly WORSE on both statistics. It saturates the same way binary occupancy
does: a randomly rotated scan already reaches 58% of the maximum possible score, because in this
scene almost every query point lands within 1-2 m of some structure regardless of pose. Grading the
match by distance rather than thresholding it did not recover a peak at the true pose.

Two independent scoring functions now fail identically on this map. That is evidence the limitation
is not the scoring function but the scene: within the searched height band this route does not carry
enough distinctive 3D structure for single-scan global localization at these resolutions. The
outstanding control experiment -- not yet run -- is to score a prior-map keyframe's OWN cloud at its
OWN pose. That query is drawn from the map itself, so it is free of calibration error, sensor
differences and dynamic objects. If its true-yaw rank is 1/36 the scorer is sound and the problem
lies in the live-query path; if it is still mid-pack, the map lacks the structure and no scoring
function will recover it.

**Memory** is recoverable, as required and as measured: the pyramid is 241 MB on this map
(level 0 distance field 5.34 M cells / 144 MB), and `releaseMemory()` on lock, `deactivate()` and
destruction returned RSS from 2040 MB to 1646 MB in a live run, confirmed independently by reading
`/proc/<pid>/status` rather than trusting the plugin's own log. The root-cause investigation below
found that binary occupancy carries almost no alignment signal on `ring_road.map`; `score_mode:
distance_field` (see "The distance-field score" below) is the identified fix, and it is now wired
in end to end -- the pyramid builds the distance field, the branch-and-bound bound is sound over it,
and the search, GICP, and gates all score against it by default. **No measurement exists yet with
this scorer.** Do not read the presence of this feature as evidence that the lock is fixed; the
occupancy measurements below remain the only validated numbers, and are exactly what
`score_mode: occupancy` exists to reproduce for a controlled before/after comparison once that
measurement is taken (`debug_probe_pose`'s active-score-mode chance baseline reports the true
yaw's rank among 36 sampled yaws for whichever mode is running -- under occupancy that rank was
measured at 5/36; a fixed lock should move it decisively toward 1/36).

**Not yet validated to produce a lock on a large map (as measured under occupancy scoring).** The full pipeline runs end to end -- the
pyramid builds from a real prior map, the corridor prune and prefilter work, the search completes
within budget, and the GICP and uniqueness gates evaluate candidates -- but on a 1.4 km x 1.3 km
ring-road map (992 keyframes) the correct pose has so far scored slightly *below* an incorrect one
under the old binary hit-count score, so the uniqueness gate correctly refused to commit and
relocalization timed out. Measured on that map with a known-good reference pose from
`GpsIcpRelocalization`:

| Configuration | Score at true pose | Best wrong pose |
|---|---|---|
| downsampled map source, 40 m query | 0.385 | 0.470 |
| full-resolution map source, 25 m query | 0.453 | 0.475 |
| full-resolution + ground band | 0.393 | 0.430 |

This was originally attributed to three contributing factors, in order of estimated impact: the
search truncating before reaching the true optimum on this map's weak bound; the occupancy-hit
score being weakly discriminative because a point landing in occupied space is the only signal, so
a query point with no map coverage nearby is scored identically to one that lands in a real gap; and
long-range returns matching poorly against a map rasterized from voxel-downsampled keyframe clouds
(`max_query_range` exists for this reason). The ternary score above was built to address the second
factor. A follow-up investigation, below, isolated the actual root cause, and it is upstream of that
change: the occupancy signal itself does not discriminate poses on this map, at any leaf tolerance.

### Root cause: occupancy has no alignment signal on this map

Measured on `ring_road.map` against the `may_30_ring_road_test_2` bag, at a ground-truth pose taken
from `GpsIcpRelocalization` (`(42.90, -511.55, -6.26)`, yaw `-150.64°`) and validated against the
vehicle's motion profile (it stays within 0.8 m of that pose for the first 30 s of the bag, so the
probe scan and the probe pose genuinely correspond).

**The map does contain the structure.** A leaf-test tolerance sweep at the true pose, consistent
across 7 independent scans:

| Leaf test | Overall | 0-10 m | 10-20 m | 20-30 m | 30-40 m |
|---|---|---|---|---|---|
| exact containment | 34-41% | 41-55% | 33-40% | 40-55% | 22-32% |
| +/-1 voxel | 80-83% | 92-100% | 75-78% | 92-95% | 71-79% |
| +/-2 voxels | 94-96% | 98-100% | 91-95% | 98-100% | 91-95% |

The query sits about one voxel off the map's structure, not off the map entirely -- this is a
rasterization/registration offset, not missing map coverage.

**But relaxing the leaf test does not create discrimination.** Scoring 36 yaws at 10° steps at the
true translation:

| Test | mean over 36 yaws | true yaw | true/mean | true yaw's rank |
|---|---|---|---|---|
| exact | 0.328 | 0.383 | 1.166 | 11 of 36 |
| +/-1 voxel | 0.763 | 0.820 | 1.075 | 5 of 36 |

Loosening the leaf test improves the true yaw's rank but *worsens* the true/mean ratio, and a yaw
70° from truth still wins outright (91% vs 82% at +/-1 voxel). Tolerance lifts every pose roughly
equally instead of producing a peak at the true pose.

**Why: the map is nearly space-filling in the query band.** A *randomly rotated* scan already hits
76% of its query points at +/-1 voxel. In the 0.6-6.0 m height band along this tree-lined route, map
occupancy at 1 m resolution is very nearly space-filling, so "does this point land on occupied
space" carries almost no information -- at any leaf tolerance, and at any resolution the ~1 m
query-to-map registration offset permits. This is the mean-over-yaws figure in the table above (76%
random vs. 82% true at +/-1 voxel): the true pose is barely above the noise floor set by a random
one.

The finer sweeps confirm it directly: the fine yaw sweep (+/-10° at 1° steps) is flat at 35-41%; the
x and y sweeps (+/-3 m at 0.5 m steps) are flat at 33-43%. There is no local maximum at the true pose
at all. This retroactively explains the score table above -- 0.453 vs. 0.475 was never measuring
alignment, it was measuring local map density, and the incorrect pose simply sits somewhere denser.

**What this means for the free-space channel.** The free-space channel could not have helped here:
it refines a statistic (occupancy hits) that has no alignment signal in it on this map. That makes
the earlier free-space measurement **inconclusive**, not negative -- it was never exercised under
conditions where the underlying score could discriminate poses in the first place. The channel ships
sound and tested (`use_free_space: false` reproduces the old ranking bit-for-bit; see "The ternary
score" below), but stays off by default until it can be evaluated against a score that discriminates.

**Implication for the plugin.** Binary occupancy matching is a good fit for structured environments
-- 3D-BBS, the method this plugin follows, validated it on buildings and urban scenes -- and a poor
fit for a vegetated open route. `ring_road.map` is the latter.

The path forward -- now implemented, see below -- replaces the binary occupancy score with a
**distance-field** score (a truncated `exp(-d^2/sigma^2)`-style grid), which is what lets GICP
succeed on this same data where occupancy does not. A distance field still admits a sound
branch-and-bound bound by max-pooling over children at coarse levels, the same way Cartographer
bounds its probability grid, so the search machinery, corridor pruning, uniqueness gate, and overall
plugin structure are all reusable -- it was only the scoring function that needed replacing.

### The distance-field score

`score_mode: distance_field` is the new default. Per query point, the level-0 cell value is
`round(255 * exp(-d^2 / (2*sigma^2)))`, where `d` is the distance from the cell centre to the
nearest map point, truncated: cells farther than `df_truncation_voxels * min_voxel_size` are not
stored (absent means score 0). A pose's raw score is the sum of per-query-point cell values, in
`[0, 255*n]`; `normalized = raw / (255*n)` is the mean per-point cell value scaled to `[0,1]` -- see
`min_match_score`/`min_score_ratio` above for why this changes what those thresholds mean without
changing their default values. Unlike the ternary occupancy score, this is a *graded* value: it
keeps the distinction between "landed exactly on structure" (`d ~ 0`) and "landed near structure but
not on it" (`d ~ 0.5-1.5 m`) that binary containment throws away -- exactly the distinction a
vegetated scene needs, per the Root cause section above, since almost every point there is "near
structure" under a binary test.

Computing `d` exactly per cell is unnecessary at this truncation. The field is built by
**max-dilation** with a precomputed kernel: for each occupied level-0 voxel, every offset within
`df_truncation_voxels` (Chebyshev distance) is max-inserted with `kernel[m] = round(255 *
exp(-(m*r0)^2 / (2*sigma^2)))`, `m` the Euclidean offset distance -- the same blurred-probability-grid
construction Cartographer uses. Coarser pyramid levels are built the same way occupancy always was:
each coarse cell is the max over its 8 children, then max-dilated over its 26-neighbourhood. **Why
the branch-and-bound bound stays sound:** a child node's representative pose displaces a query point
by at most one cell at that level (the existing cell-centre / bin-centre invariant that already
makes occupancy dilation sound -- see `bnbCellCentre()`), so the point always stays inside the
3x3x3 block a coarse cell's stored value maxes over; therefore the stored value upper-bounds every
descendant's exact level-0 value, and the sum over query points upper-bounds the descendant's total
score. This is the same argument as the occupancy dilation, with `max` replacing set union, and it
is strictly simpler because there is no eroded free-space channel to reason about.

**`score_mode: occupancy` is retained deliberately, not left over.** It reproduces today's ternary
score bit-for-bit (see "The ternary score" below) and exists specifically to A/B the two scorers
against the same map and the same scan -- which is exactly the evidence that was missing when the
occupancy score was originally assumed to work. Under `distance_field` the free-space channel
(`use_free_space`) is inactive: it is never built or raycast, because a distance field already
scores a point far from all structure at ~0, continuously, which is what the free-space channel was
approximating with a hard boundary.

**Memory.** The distance-field grid is larger than the occupancy set it augments (`max_score_voxels`
guards it, falling back to occupancy on overflow), so `VoxelPyramid::releaseMemory()` -- not
`clear()` -- is used everywhere the pyramid's job is done, and every release is logged with the
process RSS immediately before and after, so the memory is verifiably recovered rather than assumed
to be. See the Notes section above.

### The ternary score

The score is no longer a binary hit count. Per query point, with integer `hit_weight` `W` (default
`3`):

| Point lands in | contributes |
|---|---|
| occupied (level 0) / bound-occupied (level > 0) | `W` |
| known-free | `0` |
| unknown (neither) | `1` |

The raw score is `sum` over query points, always in `[0, W*n]`, and `normalized = raw / (W*n)` --
this is what `min_match_score` and `min_score_ratio` now gate on. A pose with zero map coverage
nearby (all points "unknown") no longer scores 0; it floors at `1/W` (0.33 at the default), which is
the point: previously that pose and a pose that lands cleanly in known-free space scored the same
(0), so the score could not tell "no data here" apart from "there is definitely nothing here." Now a
point in known-free space is explicitly negative evidence relative to unknown, which is what gives
the uniqueness gate something real to discriminate on. `Hypothesis::hit_fraction` (`hits / n`)
carries the old binary signal forward unchanged, and every place `normalized` is logged also logs
`hit_fraction` right next to it, so the two are never confused when reading a field log.

`use_free_space: false` disables the channel entirely and reproduces today's ranking bit-for-bit:
with no free space built, the score reduces to `W*h + (n-h) = (W-1)*h + n`, strictly monotone in the
hit count `h` for any `W > 1`.

### Why free space is only active at pyramid levels 0-1

The free-space band (`min_height`/`max_height`) makes the known-free volume only a few metres thick.
Building a coarse level's free set requires ALL 8 children of a cell to be fully free (this is what
keeps the branch-and-bound bound sound -- see the soundness argument in `bnb_search.hpp`), and a
fully free 4 m cell needs 64 stacked free 1 m voxels, which a thin height band essentially never
produces. So free space is only ever populated at level 0 (the exact leaf) and level 1. This is
expected and fine, not a bug to "fix" by weakening the fully-free test (that would break the bound):
the coarse levels' bound stays exactly as loose as it always was, and the discrimination this change
buys happens where it needs to -- at the leaf score and in the acceptance/uniqueness gates.

### Diagnosing which failure mode you're looking at

Use `debug_probe_pose: [x, y, z, yaw_deg]` to score a known pose with the exact scorer the search
uses. Every diagnostic it logs reports under whichever `score_mode` actually ran (see
`active_score_mode_`/`PROBE active score_mode=...`), so the numbers are directly comparable to the
occupancy measurements recorded above -- **except** the exact / +/-1 / +/-2 voxel occupancy
tolerance sweep and its accompanying rank/discrimination checks, which stay fixed to occupancy
regardless of `score_mode`, since those are the reference measurement that motivated this work. It
logs, at the probe pose: the full breakdown (`hits`/`unknown`/`free`/`raw`/`max_possible`/
`normalized`/`hit_fraction`/`mean_cell_score`); a hit-fraction table bucketed by query range (0-10 /
10-20 / 20-30 / 30-40 m), both with the height band on and with it disabled; yaw/x/y/z sweeps
reporting `normalized%/hit_fraction%` at each step; and a **36-yaw chance baseline under the active
score mode** reporting `mean_norm`/`max_norm`/`true_norm`/`true/mean` and, critically, the true
yaw's **rank** among the 36 sampled yaws (`true_yaw_rank=N/36`) -- this is the statistic that
decides whether the distance field actually fixed the lock: under occupancy it was measured at
5/36 (no peak at the true pose); a working fix should move it decisively toward 1/36. The
range-bucket table is the one that actually separates "the search cannot find the true pose" from
"the true pose does not score well" -- the two failure modes are indistinguishable from the outcome
alone and need opposite fixes. If near-range points hit at ~90% and far-range at ~20%, the cause is
map density falling off with query range; if the hit rate is flat across every bucket, the cause is
a systematic misalignment (frame, TF, or timing) instead. If the probe scores well above the
search's best, the search or prefilter is at fault; if it scores comparably, the scoring/frames are.
The height-band-disabled bucket table additionally shows whether `min_height`/`max_height` is what
is discarding the matching structure (query-side only -- see the free-space limitation above about
the map side already being rasterized with the band applied).

Until a lock is measured and validated under `score_mode: distance_field` (see Status above), keep
`GpsIcpRelocalization` listed first so GPS is used whenever available.

## Mapping vs Localization

This plugin is listed only in `config/example_localization.yaml`, not `example_slam.yaml`. Even if loaded in mapping mode it no-ops immediately because `hasPriorMap()` is false. In localization mode it is listed *after* `gps_icp_relocalization`, so GPS wins when available and this runs only when GPS returns nothing.
