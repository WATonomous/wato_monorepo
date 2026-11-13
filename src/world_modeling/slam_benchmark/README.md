# slam_benchmark

Tools for comparing Eidos against RTAB-Map on recorded bags, for the Eidos paper. RTAB-Map is pulled into the world_modeling image by rosdep through this package's `package.xml`, so nothing is installed on the host. This package is in `bloom/*.ignored` and is not released.

## Contents

| Path | Purpose |
|---|---|
| `launch/eidos_bench.launch.yaml` | Eidos mapping on a bag with the car config; `use_gps:=false` gives a LiDAR-inertial-only run |
| `scripts/to_tum.py` | Converts GNSS fixes, pose topics, and `eidos poses` CSVs to TUM trajectories |
| `scripts/evaluate.py` | ATE vs GNSS and distance to lane centerlines (`.osm`), results table and plots |
| `launch/boreas_eidos_bench.launch.yaml`, `config/boreas_eidos.yaml` | Eidos on a Boreas bag: the car config with Boreas topics, frames and IMU warmup on top |
| `scripts/boreas_to_tum.py` | Converts a Boreas sequence's Applanix ground truth to a TUM reference |
| `scripts/boreas_benchmark.sh` | Eidos vs RTAB-Map on a Boreas bag, scored against the Applanix ground truth |

RTAB-Map uses the stock `rtabmap_examples/lidar3d.launch.py`.

## Running

All outputs go in `maps/bench/` (git-ignored, mounted at `/opt/watonomous/maps/bench` in the container).

Inside the world_modeling dev container, run each system on its own while replaying only the sensor topics. The bag's recorded `/tf` holds the original run's transforms and must not be replayed:

```bash
ros2 launch slam_benchmark eidos_bench.launch.yaml use_gps:=false
```

```bash
ros2 launch rtabmap_examples lidar3d.launch.py use_sim_time:=true lidar_topic:=/lidar_cc/velodyne_points frame_id:=base_footprint imu_topic:=/novatel/oem7/imu/data voxel_size:=0.5
```

```bash
ros2 bag play /ws/bags/<bag>.mcap --clock --rate 0.5 --topics /lidar_cc/velodyne_points /novatel/oem7/imu/data /novatel/oem7/fix /tf_static
```

Export trajectories after the bag finishes:

```bash
ros2 service call /world_modeling/slam/save_map eidos_msgs/srv/SaveMap "{filepath: '/opt/watonomous/maps/bench/<run>.map'}"
eidos poses /opt/watonomous/maps/bench/<run>.map > /opt/watonomous/maps/bench/<run>.csv
```

```bash
cp ~/.ros/rtabmap.db /opt/watonomous/maps/bench/<run>.db
rtabmap-export --poses --poses_format 10 /opt/watonomous/maps/bench/<run>.db
```

`lidar3d.launch.py` deletes `~/.ros/rtabmap.db` on every start, so copy it before the next run.

## Evaluating

The scripts need `evo` and `pyproj`, plus `rosbag2_py` from a sourced ROS install for reading bags:

```bash
python3 -m venv --system-site-packages ~/slam-bench-venv
~/slam-bench-venv/bin/pip install evo pyproj
```

```bash
source /opt/ros/jazzy/setup.bash
~/slam-bench-venv/bin/python scripts/to_tum.py gnss <bag>.mcap -o gnss.tum
~/slam-bench-venv/bin/python scripts/to_tum.py eidos <run>.csv -o eidos.tum
~/slam-bench-venv/bin/python scripts/evaluate.py --ref gnss.tum --est "Eidos=eidos.tum" --est "RTAB-Map=<run>_poses.txt" --osm <lane_map>.osm --out results
```

`evaluate.py` crops every trajectory to the time window they share, aligns each to GNSS with the same SE(3) Umeyama fit, and writes `results.csv`, `trajectories.png` and `errors.png`. The `GNSS (lane floor)` row is the GNSS track's own distance to the lane centerlines; differences between methods smaller than that floor are not meaningful.

`to_tum.py gnss` prints the fix status counts. Status `0` is a single-point fix (about 1 m), not RTK, which limits how small an ATE difference the GNSS reference can resolve.

## Boreas

The Boreas bags (converted from the [Boreas dataset](https://www.boreas.utias.utoronto.ca/)) carry `/boreas/lidar/points`, `/boreas/imu/data` and `/tf_static`, and no GNSS. `boreas_benchmark.sh` therefore runs Eidos LiDAR-inertial only, and scores against the sequence's `applanix/gps_post_process.csv`: the Applanix post-processed GNSS/INS solution, a much tighter reference than the car's single-point GNSS.

`config/boreas_eidos.yaml` changes only the topics, the frames and the IMU warmup threshold (0.02 rad/s instead of 0.005: the Boreas gyro's resting noise sits on 0.005, and Eidos would never leave warmup). Everything else is the car's config.

The dev container only sees `<repo>/bags` (as `/ws/bags`). Mount the Boreas folder under it on the host **before** `./watod up`, since a mount made later doesn't reach a running container. The folder needs the bags and the raw sequence (for `applanix/`).

Each Eidos map of the full drive is about 10 GB on disk (every keyframe keeps its 128-beam cloud), and Eidos holds it in RAM while running. Put `maps/` on a disk with room for one map per repeat, for example by making it a symlink before `./watod up`:

```bash
mkdir -p /mnt/data/wato_maps && ln -s /mnt/data/wato_maps ~/wato_monorepo/maps
sudo mkdir -p ~/wato_monorepo/bags/boreas
sudo mount --bind /mnt/data/boreas ~/wato_monorepo/bags/boreas
./watod up
./watod exec world_modeling_bringup_dev bash /ws/src/world_modeling/slam_benchmark/scripts/boreas_benchmark.sh
```

Settings are environment variables (`./watod exec -e RUNS=1 ...`): `BAG` (default `/ws/bags/boreas/bags/boreas-2020-11-26-13-58-lidar-imu`), `SEQUENCE` (the raw sequence with `applanix/`, default `/ws/bags/boreas/boreas-2020-11-26-13-58`), `PREFIX`, `RUNS` (3), `RATE` (0.5), `METHODS` (`eidos rtabmap`) and `EVAL_ONLY`. The 18-minute drive takes about 36 minutes per run at rate 0.5. Outputs land in `maps/bench/` as for the car, with a `<PREFIX>_summary.md` over the repeats.

The reference is in the sequence's local east/north/up frame, not UTM, so there is no lane metric: UTM's grid scale shortens distances by about 0.02% in Toronto, which the scale-free alignment would count as error.
