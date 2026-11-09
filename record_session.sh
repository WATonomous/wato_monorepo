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
# Brings up the sensor drivers for a recording session without recording.
# Topics come from the pano_cameras_lidar_cc recording profile.
#
#   ./record_session.sh          start containers, build and start the drivers (no recording)
#   ./record_session.sh status   check that the drivers are publishing
#   ./record_session.sh record   record until Ctrl+C (can be run again for each take)
#   ./record_session.sh down     stop the drivers and containers
#
# Bags go to $BAG_DIRECTORY (default /mnt/data/bags).

set -euo pipefail

MONO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WATOD="$MONO_DIR/watod"
PROFILE="$MONO_DIR/src/perception/perception_bringup/config/bag_recorder/pano_cameras_lidar_cc.yaml"
MODULES=(-m interfacing:dev -m perception:dev)

# Same config watod reads, so the container names match
[[ -f "$MONO_DIR/watod-config.sh" ]] && source "$MONO_DIR/watod-config.sh"
[[ -f "$MONO_DIR/watod-config.local.sh" ]] && source "$MONO_DIR/watod-config.local.sh"
PROJECT="${COMPOSE_PROJECT_NAME:-watod_$USER}"
INTERFACING="$PROJECT-interfacing_bringup_dev-1"
PERCEPTION="$PROJECT-perception_bringup_dev-1"
# Runs the Zenoh router and the Foxglove bridge
INFRASTRUCTURE="$PROJECT-infrastructure_bringup-1"
export BAG_DIRECTORY="${BAG_DIRECTORY:-/mnt/data/bags}"

# Run a command inside a dev container with ROS and the workspace sourced
in_container() {
  local container=$1
  shift
  docker exec "$container" bash -c "
    source /opt/ros/\$ROS_DISTRO/setup.bash
    [[ -f /opt/watonomous/setup.bash ]] && source /opt/watonomous/setup.bash
    [[ -f /ws/install/setup.bash ]] && source /ws/install/setup.bash
    $*"
}

# Same, but detached; output goes to a log file inside the container
in_container_detached() {
  local container=$1 log=$2
  shift 2
  docker exec -d "$container" bash -c "
    source /opt/ros/\$ROS_DISTRO/setup.bash
    [[ -f /opt/watonomous/setup.bash ]] && source /opt/watonomous/setup.bash
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
}

build_drivers() {
  # The whole workspace: interfacing_bringup does not declare every package its launch uses
  echo "==> Building interfacing (~2 min the first time, incremental after)"
  in_container "$INTERFACING" "cd /ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"

  # Only what the camera launch needs. Ignoring the rest of the workspace stops colcon from
  # requiring perception_bringup's other dependencies (detection, tracking, ...) to be built.
  echo "==> Building camera drivers"
  in_container "$PERCEPTION" "cd /ws \
    && others=\$(colcon list --names-only | grep -vxE 'perception_bringup|camera_aravis2|camera_aravis2_msgs|camera_calib') \
    && colcon build --packages-up-to perception_bringup camera_aravis2 --packages-ignore \$others \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
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

  # Cameras only: a component container plus all_cameras_composed, without the
  # detection/tracking nodes that perception.launch.yaml also starts
  if in_container "$PERCEPTION" "pgrep -f '[c]amera_container' > /dev/null"; then
    echo "==> Camera drivers already running"
  else
    echo "==> Starting camera drivers"
    in_container_detached "$PERCEPTION" /tmp/cameras.log \
      "ros2 run rclcpp_components component_container_mt --ros-args -r __node:=camera_container &
       ros2 launch perception_bringup all_cameras_composed.yaml container_name:=camera_container
       wait"
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
    echo "==> Starting containers (interfacing + perception dev, infrastructure)"
    "$WATOD" "${MODULES[@]}" up -d
    build_drivers
    start_drivers
    echo "==> Waiting 20 s for the drivers to come up"
    sleep 20
    status
    echo
    echo "Drivers are running; nothing is being recorded."
    echo "  Logs:   docker exec $PERCEPTION tail -f /tmp/cameras.log"
    echo "          docker exec $INTERFACING tail -f /tmp/interfacing.log"
    echo "  Record: ./record_session.sh record   (Ctrl+C to stop)"
    echo "  Foxglove can stay connected while recording; close image panels to spare CPU."
    echo "  Keep infrastructure running: its Zenoh router is how the recorder finds the topics."
    ;;
  status)
    status
    ;;
  record)
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
