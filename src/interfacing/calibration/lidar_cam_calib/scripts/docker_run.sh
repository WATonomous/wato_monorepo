#!/bin/bash
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Run a command in the lidar_cam_calib image. Builds the image the first time.
# Mounts: the repo at /ws, BAG_DIR at /bags (read-only), OUT_DIR at /out.
#
#   scripts/docker_run.sh python3 /ws/src/interfacing/calibration/lidar_cam_calib/scripts/extract_segments.py /bags /out
#
# Settings (environment variables):
#   BAG_DIR  folder of bags to search       (default <repo>/bags)
#   OUT_DIR  segments and results           (default <repo>/bags/lidar_cam_calib)
#   IMAGE    image name                      (default lidar_cam_calib:latest)
# With DISPLAY set, X11 is forwarded so `viewer` and `initial_guess_manual` can open.
set -euo pipefail

PKG_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
MONO=$(cd "$PKG_DIR/../../../.." && pwd)
IMAGE=${IMAGE:-lidar_cam_calib:latest}
BAG_DIR=${BAG_DIR:-$MONO/bags}
OUT_DIR=${OUT_DIR:-$MONO/bags/lidar_cam_calib}
mkdir -p "$OUT_DIR"

if ! docker image inspect "$IMAGE" > /dev/null 2>&1; then
  echo "Building $IMAGE (first time only)"
  docker build -t "$IMAGE" "$PKG_DIR/docker"
fi

# No --net host: xvfb-run's displays are abstract sockets in the network namespace, so
# containers sharing the host's would collide when several jobs run on one node
ARGS=(--rm -v "$MONO:/ws" -v "$(realpath "$BAG_DIR"):/bags:ro" -v "$(realpath "$OUT_DIR"):/out")
if [ -t 0 ]; then
  ARGS+=(-it)
fi
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
  ARGS+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix)
fi
exec docker run "${ARGS[@]}" "$IMAGE" "$@"
