# eidos CLI

Command-line tool for inspecting and exporting eidos `.map` files. Available as `eidos` after sourcing the workspace.

## Commands

### `eidos info <map_file>`

Print map metadata: version, keyframe count, edge count, spatial bounds, data formats, and decoded global data.

```bash
eidos info /opt/watonomous/maps/wrestrc.map
```

### `eidos poses <map_file>`

Dump keyframe poses as CSV to stdout.

```bash
eidos poses /opt/watonomous/maps/wrestrc.map > poses.csv
```

Output columns: `id, gtsam_key, x, y, z, roll, pitch, yaw, time, owner`

### `eidos map <map_file> --key <data_key> -o <output.pcd>`

Build a combined world-frame point cloud from a specified keyframe data key.

```bash
# Full resolution map from LISO clouds
eidos map /opt/watonomous/maps/wrestrc.map --key liso_factor/cloud -o map.pcd

# Downsampled map
eidos map /opt/watonomous/maps/wrestrc.map --key liso_factor/cloud -o map_ds.pcd --voxel 0.5

# Every 5th keyframe only
eidos map /opt/watonomous/maps/wrestrc.map --key liso_factor/cloud -o map_sparse.pcd --skip 5
```

Supports `pcl_pcd_binary` and `small_gicp_binary` point cloud formats. If `--key` is wrong, available keys are listed in the error output.

| Flag | Description |
|---|---|
| `--key <data_key>` | Required. Keyframe data key (e.g. `liso_factor/cloud`) |
| `-o <output.pcd>` | Required. Output PCD file path |
| `--voxel <size>` | Voxel downsample the merged cloud (meters) |
| `--skip <N>` | Use every Nth keyframe (default: 1) |

### `eidos graph <map_file>`

Print factor graph edges as CSV to stdout.

```bash
eidos graph /opt/watonomous/maps/wrestrc.map
```

Output columns: `key_a, key_b, owner`
