# Multi-Object Tracking — Developer Guide

## Applying patches (best for small changes)

1) Clone the base repo used in [`CMakeLists.txt`](./CMakeLists.txt)

2) Commit new changes and create patch files using `git format-patch`
3) Add the patch files to [`patches/`](./patches)

Then build with

```bash
colcon build --packages-select bytetrack_cpp_vendor --cmake_args -DBYTETRACK_PATCHES="x.patch;y.patch;z.patch"
```
