# slam_benchmark

Tools for comparing Eidos against RTAB-Map on recorded bags, for the Eidos paper. RTAB-Map is pulled into the world_modeling image by rosdep through this package's `package.xml`, so nothing is installed on the host. This package is in `bloom/*.ignored` and is not released.

## Contents

| Path | Purpose |
|---|---|
| `launch/eidos_bench.launch.yaml` | Eidos mapping on a bag with the car config; `use_gps:=false` gives a LiDAR-inertial-only run |
| `scripts/inhouse_bag/to_tum.py` | Converts GNSS fixes, pose topics, and `eidos poses` CSVs to TUM trajectories |
| `scripts/evaluate.py` | ATE vs GNSS and distance to lane centerlines (`.osm`), results table and plots |
| `launch/boreas_eidos_bench.launch.yaml`, `config/boreas_eidos.yaml` | Eidos on a Boreas bag: the car config with Boreas topics, frames and IMU warmup on top |
| `scripts/boreas/boreas_to_tum.py` | Converts a Boreas sequence's Applanix ground truth to a TUM reference |
| `scripts/boreas/boreas_benchmark.sh` | Eidos vs RTAB-Map on a Boreas bag, scored against the Applanix ground truth |

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
~/slam-bench-venv/bin/python scripts/inhouse_bag/to_tum.py gnss <bag>.mcap -o gnss.tum
~/slam-bench-venv/bin/python scripts/inhouse_bag/to_tum.py eidos <run>.csv -o eidos.tum
~/slam-bench-venv/bin/python scripts/evaluate.py --ref gnss.tum --est "Eidos=eidos.tum" --est "RTAB-Map=<run>_poses.txt" --osm <lane_map>.osm --out results
```

`evaluate.py` crops every trajectory to the time window they share, aligns each to GNSS with the same SE(3) Umeyama fit, and writes `results.csv`, `trajectories.png` and `errors.png`. The `GNSS (lane floor)` row is the GNSS track's own distance to the lane centerlines; differences between methods smaller than that floor are not meaningful.

`to_tum.py gnss` prints the fix status counts. Status `0` is a single-point fix (about 1 m), not RTK, which limits how small an ATE difference the GNSS reference can resolve.

## Boreas

The Boreas bags (converted from the [Boreas dataset](https://www.boreas.utias.utoronto.ca/)) carry `/boreas/lidar/points`, `/boreas/imu/data` and `/tf_static`, and no GNSS. `scripts/boreas/boreas_benchmark.sh` therefore runs Eidos LiDAR-inertial only (no `eidos/` GPS test and no `car/` comparison), and scores against the sequence's `applanix/gps_post_process.csv`: the Applanix post-processed GNSS/INS solution, a much tighter reference than the car's single-point GNSS.

`config/boreas_eidos.yaml` changes only the topics, the frames and the IMU warmup threshold (0.02 rad/s instead of 0.005: the Boreas gyro's resting noise sits on 0.005, and Eidos would never leave warmup). Everything else is the car's config.

### Data on WATcloud

The container sees only `~/wato_monorepo/bags` (as `/ws/bags`). That is usually a symlink to your own folder on the drive (`ls -ld ~/wato_monorepo/bags` shows where it points, e.g. `/mnt/wato-drive2/$USER/bags`). Docker follows that symlink, but not symlinks inside the folder, so the Boreas data has to be in it for real. A hard-linked copy of the shared bag is instant and takes no extra space, since both folders are on the same drive:

```bash
cd /mnt/wato-drive2
cp -al bags/boreas-2020-11-26-13-58-lidar-imu $USER/bags/      # or cp -r (34 GB) if hard links aren't permitted
mkdir -p $USER/bags/boreas-2020-11-26-13-58/applanix
```

Then copy the ground truth into that `applanix/` folder. Only `gps_post_process.csv` (77 MB) is needed from the raw sequence. The defaults expect:

```text
~/wato_monorepo/bags/boreas-2020-11-26-13-58-lidar-imu/                      the bag
~/wato_monorepo/bags/boreas-2020-11-26-13-58/applanix/gps_post_process.csv   the ground truth
```

### Running

```bash
cd ~ && BENCH=boreas sbatch ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
```

or as a job array, one test per task (two tests per repeat, so `1-6` for three repeats):

```bash
cd ~ && JOB=$(BENCH=boreas sbatch --parsable --array=1-6%4 --time=01:30:00 ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch)
BENCH=boreas EVAL_ONLY=1 sbatch --dependency=afterany:$JOB --cpus-per-task=4 --mem=16G --time=01:00:00 ~/wato_monorepo/src/world_modeling/slam_benchmark/scripts/watcloud_bench.sbatch
```

The job checks the bag and ground truth on the login node before starting Docker. Settings passed through: `BAG`, `SEQUENCE` (folder holding `applanix/`), `DATASET` (default `boreas`), `RUNS` (3), `RATE` (0.5), `METHODS` (`eidos_nogps rtabmap`) and `EVAL_ONLY`. The 18-minute drive takes about 40 minutes per test at rate 0.5, and each Eidos map is about 10 GB on disk and in RAM.

Outputs land in `maps/bench/results/<DATASET>/`:

```text
test1/
  eidos_nogps/    eidos.map, poses.csv, poses.tum, launch/play/save/status logs
  rtabmap/        rtabmap.db, poses.txt, launch/play/export logs
  results.csv, trajectories.png, errors.png, eval.log
test2/  test3/    same as test1
reference/        gnss.tum (the Applanix ground truth) + gnss.log
logs/             build and Python setup logs
summary.md        mean ± std over the tests
```

The reference is in the sequence's local east/north/up frame, not UTM, so there is no lane metric: UTM's grid scale shortens distances by about 0.02% in Toronto, which the scale-free alignment would count as error.
