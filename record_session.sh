#!/usr/bin/env bash
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
# Brings up the sensor drivers, perception (cameras, BEVFusion, tracking) and world modeling
# (eidos SLAM, world model, MTR prediction) for a recording session, without recording.
# Only the sensor topics in the pano_cameras_lidar_cc recording profile are recorded; perception
# and world modeling outputs are not. /tf is recorded, so it also carries the map -> odom ->
# base_footprint transforms that eidos_transform broadcasts while world modeling runs.
#
#   ./record_session.sh          start containers, build and start everything (no recording)
#   ./record_session.sh status   check that the drivers, BEVFusion and prediction are publishing
#   ./record_session.sh record   record until Ctrl+C (can be run again for each take)
#   ./record_session.sh down     stop everything and the containers
#
# Bags go to $BAG_DIRECTORY (default /mnt/data/bags).

set -euo pipefail

MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATOD="$MONO_DIR/watod"
PROFILE="$MONO_DIR/src/perception/perception_bringup/config/bag_recorder/pano_cameras_lidar_cc.yaml"
MODULES=(-m interfacing:dev -m perception:dev -m world_modeling:dev)

# Same config watod reads, so the container names match
[[ -f "$MONO_DIR/watod-config.sh" ]] && source "$MONO_DIR/watod-config.sh"
[[ -f "$MONO_DIR/watod-config.local.sh" ]] && source "$MONO_DIR/watod-config.local.sh"
PROJECT="${COMPOSE_PROJECT_NAME:-watod_$USER}"
INTERFACING="$PROJECT-interfacing_bringup_dev-1"
PERCEPTION="$PROJECT-perception_bringup_dev-1"
WORLD_MODELING="$PROJECT-world_modeling_bringup_dev-1"
# Runs the Zenoh router and the Foxglove bridge
INFRASTRUCTURE="$PROJECT-infrastructure_bringup-1"
export BAG_DIRECTORY="${BAG_DIRECTORY:-/mnt/data/bags}"

# MTR prediction (Argoverse 2 model). The engine is machine-local: build it on this machine with
# the steps in src/world_modeling/prediction_ml/DEVELOPING.md. Paths are inside the world modeling
# dev container, which mounts $MONO_DIR/models at /ws/models.
MTR_ENGINE=/ws/models/prediction/mtr_av2_r2.engine

# Run a command inside a dev container with ROS and the workspace sourced.
# docker exec skips the entrypoint, so the Isaac ROS underlay (rectify, H.264 encoder) is sourced here.
in_container() {
  local container=$1
  shift
  docker exec "$container" bash -c "
    source /opt/ros/\$ROS_DISTRO/setup.bash
    [[ -n \"\${ISAAC_ROS_INSTALL:-}\" ]] && source \"\$ISAAC_ROS_INSTALL/local_setup.bash\"
    [[ -f /ws/install/setup.bash ]] && source /ws/install/setup.bash
    $*"
}

# Same, but detached; output goes to a log file inside the container
in_container_detached() {
  local container=$1 log=$2
  shift 2
  docker exec -d "$container" bash -c "
    source /opt/ros/\$ROS_DISTRO/setup.bash
    [[ -n \"\${ISAAC_ROS_INSTALL:-}\" ]] && source \"\$ISAAC_ROS_INSTALL/local_setup.bash\"
    source /ws/install/setup.bash
    { $*; } > $log 2>&1"
}

profile_topics() {
  python3 -c 'import sys, yaml
print("\n".join(yaml.safe_load(open(sys.argv[1]))["bag_recorder"]["ros__parameters"]["record"]["topics"]))' "$PROFILE"
}

preflight() {
  if pgrep -f "watod-compose.sh build" > /dev/null; then
    echo "A watod build is still running. Wait for it to finish, then rerun." >&2
    exit 1
  fi

  # The cameras and lidars only accept one driver at a time
  local others
  others=$(docker ps --format '{{.Names}}' | grep -E '(perception|interfacing)_bringup' | grep -v "^$PROJECT-" || true)
  if [[ -n "$others" ]]; then
    echo "Another stack is running sensor drivers. Ask its owner to stop it first:" >&2
    echo "$others" >&2
    exit 1
  fi

  if [[ ! -f "$MONO_DIR/models/prediction/$(basename "$MTR_ENGINE")" ]]; then
    echo "MTR engine not found at models/prediction/$(basename "$MTR_ENGINE")." >&2
    echo "Build it with the steps in src/world_modeling/prediction_ml/DEVELOPING.md." >&2
    exit 1
  fi
}

build_drivers() {
  # --base-paths src: /ws/bags is a bind mount of BAG_DIRECTORY and colcon must not crawl it
  # The whole workspace: interfacing_bringup does not declare every package its launch uses
  echo "==> Building interfacing (~2 min the first time, incremental after)"
  in_container "$INTERFACING" "cd /ws && colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

  # perception.launch.yaml needs the camera drivers, BEVFusion and everything perception_bringup
  # depends on (tracking, patchwork++, camera_sync, spatial_association, ...)
  echo "==> Building perception (cameras, BEVFusion, tracking)"
  in_container "$PERCEPTION" "cd /ws \
    && colcon build --base-paths src --packages-up-to perception_bringup camera_aravis2 bevfusion \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

  echo "==> Building world modeling (localization, world model, prediction)"
  in_container "$WORLD_MODELING" "cd /ws && colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
}

start_drivers() {
  # [x]yz patterns keep pgrep from matching the bash -c wrapper that runs it
  if in_container "$INTERFACING" "pgrep -f '[i]nterfacing_bringup interfacing.launch' > /dev/null"; then
    echo "==> Interfacing drivers already running"
  else
    echo "==> Starting interfacing drivers (GPS, lidars, CAN)"
    in_container_detached "$INTERFACING" /tmp/interfacing.log \
      "ros2 launch interfacing_bringup interfacing.launch.yaml"
  fi

  # The full perception launch: the camera drivers plus camera_sync, BEVFusion and tracking,
  # all in perception_container
  if in_container "$PERCEPTION" "pgrep -f '[c]amera_container' > /dev/null"; then
    # Left over from the old cameras-only session; the cameras accept one driver at a time
    echo "Cameras-only drivers from an older session are running. Run ./record_session.sh down first." >&2
    exit 1
  elif in_container "$PERCEPTION" "pgrep -f '[p]erception_bringup perception.launch' > /dev/null"; then
    echo "==> Perception already running"
  else
    echo "==> Starting perception (cameras, BEVFusion, tracking)"
    in_container_detached "$PERCEPTION" /tmp/perception.log \
      "ros2 launch perception_bringup perception.launch.yaml"
  fi

  if in_container "$WORLD_MODELING" "pgrep -f '[w]orld_modeling_bringup world_modeling.launch' > /dev/null"; then
    echo "==> World modeling already running"
  else
    # Eidos runs SLAM from wherever the car starts (no prior map): localization mode only works on
    # the test track. MTR needs its map frame and ego pose; the OSM lanelets are optional.
    # eidos_map_path is '' (quotes included): ros2 launch rejects an empty value, and launch_ros
    # parses the quotes as YAML into the empty path that skips loading a map.
    echo "==> Starting world modeling (eidos SLAM, world model, MTR prediction)"
    in_container_detached "$WORLD_MODELING" /tmp/world_modeling.log \
      "ros2 launch world_modeling_bringup world_modeling.launch.yaml \
        eidos_config:=\$(ros2 pkg prefix world_modeling_bringup)/share/world_modeling_bringup/config/eidos_slam.yaml \
        eidos_map_path:=\"''\" \
        mtr_mode:=tensorrt mtr_engine_path:=$MTR_ENGINE mtr_planar_inputs:=true \
        mtr_metadata_path:=\$(ros2 pkg prefix prediction_ml)/share/prediction_ml/config/contract_8_target_av2.yaml"
  fi
}

status() {
  echo "==> Publish rates over 5 s (blank = nothing received)"
  local topics=(
    /camera_pano_nn/image_rect_compressed /camera_pano_ne/image_rect_compressed
    /camera_pano_ee/image_rect_compressed /camera_pano_se/image_rect_compressed
    /camera_pano_ss/image_rect_compressed /camera_pano_sw/image_rect_compressed
    /camera_pano_ww/image_rect_compressed /camera_pano_nw/image_rect_compressed
    /lidar_cc/velodyne_points /novatel/oem7/fix /interfacing/can_state_estimator/body_velocity
    # Not recorded: BEVFusion, tracking and prediction outputs
    /perception/detections_3d_bev /perception/detections_3D_tracked /world_modeling/world_object_seeds
  )
  in_container "$PERCEPTION" "
    for t in ${topics[*]}; do
      ( rate=\$(timeout -s INT 5 ros2 topic hz \"\$t\" 2>/dev/null | grep -m1 -o 'average rate: [0-9.]*' | cut -d' ' -f3)
        printf '  %-48s %s Hz\n' \"\$t\" \"\${rate:--}\" ) &
    done
    wait"

  local port
  port=$(docker inspect -f '{{join .Config.Cmd " "}}' "$INFRASTRUCTURE" 2> /dev/null \
    | grep -o 'foxglove_port:=[0-9]*' | cut -d= -f2 || true)
  if [[ -n "$port" ]] && ss -ltn "sport = :$port" | grep -q LISTEN; then
    echo "  Foxglove bridge: ws://$(hostname):$port"
  else
    echo "  Foxglove bridge: not listening (is $INFRASTRUCTURE running?)"
  fi
}

case "${1:-up}" in
  up)
    preflight
    echo "==> Starting containers (interfacing + perception + world modeling dev, infrastructure)"
    "$WATOD" "${MODULES[@]}" up -d
    build_drivers
    start_drivers
    echo "==> Waiting 20 s for the drivers to come up"
    sleep 20
    status
    echo
    echo "Drivers, perception and world modeling are running; nothing is being recorded."
    echo "  BEVFusion loads its engines ~20 s after start and can take a minute or two more;"
    echo "  rerun ./record_session.sh status if its topics are still blank."
    echo "  Logs:   docker exec $PERCEPTION tail -f /tmp/perception.log"
    echo "          docker exec $WORLD_MODELING tail -f /tmp/world_modeling.log"
    echo "          docker exec $INTERFACING tail -f /tmp/interfacing.log"
    echo "  Record: ./record_session.sh record   (Ctrl+C to stop)"
    echo "  Foxglove can stay connected while recording; close image panels to spare CPU."
    echo "  Keep infrastructure running: its Zenoh router is how the recorder finds the topics."
    ;;
  status)
    status
    ;;
  record)
    # Only the profile's sensor topics: perception and world modeling outputs are left out
    mapfile -t topics < <(profile_topics)
    "$WATOD" "${MODULES[@]}" bag record -o "pano_lidar_cc_$(date +%Y%m%d_%H%M%S)" --topics "${topics[@]}"
    ;;
  down)
    "$WATOD" "${MODULES[@]}" down
    ;;
  *)
    echo "Usage: $0 [up|status|record|down]" >&2
    exit 1
    ;;
esac
