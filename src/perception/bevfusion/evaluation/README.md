# BEVFusion evaluation tools

Offline scripts for issue #439 (BEVFusion evaluation). They run on the host or in any container with Python 3.10+. They are not ROS nodes, and colcon ignores this folder because it has no `package.xml`.

```bash
pip install mcap mcap-ros2-support requests
```

## Track stability (`track_stability.py`)

Computes label-free stability metrics from a bag that recorded `/perception/detections_3D_tracked`. The bag can come from a live run, or you can record it while replaying a sensor bag through `perception.launch.yaml`.

```bash
./track_stability.py <bag_dir> [--start 245 --end 290] [--json out.json]
```

Results are reported per class and per distance band (0–20 m, 20–40 m, 40 m+):

| Metric | Meaning |
|---|---|
| `life_med_s`, `life_p90_s` | How long tracks live |
| `short_frac` | Share of tracks that live less than 0.5 s |
| `fragments_per_min` | A new track starts within 2 m of a same-class track that ended less than 1 s earlier. This usually means one object was split into two IDs. |
| `class_flips_per_track` | How often a track's class label changes |
| `pos_jitter_cm_*` | Size of the second difference of the box centre. Constant ego velocity cancels out. |
| `size_jitter_pct_med` | Frame-to-frame change in box size, as a percentage of size |
| `yaw_delta_deg_med`, `yaw_flips_per_100_frames` | Heading noise, and how often a box flips heading by about 180° |

Tracks are in the `lidar_cc` frame, which moves with the car, so the car's own turning shows up in `yaw_delta_deg_med`.

## Finding scenes in bags without downloading them (`bag_scenes/`)

The perception bags on Drive are about 21.5 GB per split file. These scripts use HTTP range requests and read only the MCAP chunk index plus the few chunks they need, so finding scenes in a 20 GB bag downloads about 150 MB.

1. Get each `.mcap` file's Drive ID. The folder must be shared publicly.

   ```bash
   python3 -c "import gdown; [print(f.path, f.id) for f in gdown.download_folder(id='<folder_id>', skip_download=True, quiet=True)]"
   ```

2. Extract GPS from each file (writes a small CSV):

   ```bash
   bag_scenes/extract_gps.py <file_id> gps/<bag>_<n>.csv
   ```

3. Match the routes against OpenStreetMap roundabouts, parking lots and traffic signals:

   ```bash
   bag_scenes/find_scenes.py gps/ scenes/
   ```

4. Look at a scene by fetching single camera frames at given times:

   ```bash
   bag_scenes/grab_frames.py <file_id> frames/ /camera_pano_nn/image_rect_compressed 146 268
   ```

5. Rank the scenes by traffic. This needs recorded BEVFusion detections on the same timeline:

   ```bash
   bag_scenes/scene_density.py scenes/scenes.json <bag> <detections_bag_dir>
   ```

## Scene catalog: `Bags_Perception_Specific` Drive folder (as of 2026-11-15)

Times are seconds from bag start. Roundabouts in the first bag were checked on camera; all other times come from GPS matched against OSM.

| Bag | Length | Roundabouts | Notes |
|---|---|---|---|
| `sensors_2026_07_07-23_01_24_2_with_bevfusion` | 5.1 min | 139–154, 190–209, **250–286** | **Recommended.** Includes the car's BEVFusion and tracker output. 250–286 s is the busiest scene found (16.5 detections per frame on average, peak 32). Low sun and glare. |
| `sensors_2026_07_08-01_44_32` | 17.5 min | 240–256, 276–284, 311–320, 327–335, 485–497 | 5 files |
| `sensors_2026_07_07-23_01_24` | 19.0 min | 682–697, 734–753, 793–830, 895–924 | The last split file is truncated (no index) |
| `pano_lidar_cc_20261114_133353` | 4.5 min | 21–38, 43–55 | Smallest file (11.9 GB) with roundabouts |
| `pano_lidar_cc_20261114_173233` | 7.2 min | 161–174 | 5 signalized intersections |
| `pano_lidar_cc_20261029_190612` | 10.8 min | – | Slow loop around UW campus, passing 15 parking lots |
| `pano_lidar_cc_20261114_171252` | 9.6 min | – | Mostly stationary (0.6 km) |
