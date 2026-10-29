# slam_benchmark

Tools for comparing Eidos against RTAB-Map on recorded bags, for the Eidos paper. RTAB-Map is pulled into the world_modeling image by rosdep through this package's `package.xml`, so nothing is installed on the host. This package is in `bloom/*.ignored` and is not released.

## Contents

| Path | Purpose |
|---|---|
| `launch/eidos_bench.launch.yaml` | Eidos mapping on a bag with the car config; `use_gps:=false` gives a LiDAR-inertial-only run |
| `scripts/to_tum.py` | Converts GNSS fixes, pose topics, and `eidos poses` CSVs to TUM trajectories |
| `scripts/evaluate.py` | ATE vs GNSS and distance to lane centerlines (`.osm`), results table and plots |

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
