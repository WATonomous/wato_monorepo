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
# watod-sil.sh - Run a Software-In-the-Loop (SIL) component scenario.
#
# Brings up the SIL CARLA bridge (in the simulation image) and the component-under-test
# harness (in the action image), both joined to the same ROS graph as a running CARLA
# server. The run records a fixed-duration MCAP bag and then analyzes it against the
# scenario's limits, exiting non-zero on any breach.
#
# Prerequisite: a CARLA server must already be running (e.g. `watod up` with simulation
# active starts carla_sim / carla_sim_no_gpu).
#
# Usage:
#   watod sil run <scenario.yaml> [--component motion_control] [--output-dir DIR] [--warmup SECONDS]

set -euo pipefail

usage() {
  cat <<EOF
Usage: watod sil run <scenario.yaml> [options]

Options
  --component NAME     Component under test (default: motion_control)
  --output-dir DIR     Host directory for the bag + report (default: \$MONO_DIR/sil_runs)
  --warmup SECONDS     Seconds to wait for CARLA to load the scenario before starting
                       the component harness (default: 20)

Examples
  watod sil run src/simulation/sil_testing/config/scenarios/straight_follow.yaml
  watod sil run my_scenario.yaml --component motion_control --warmup 30
EOF
}

[[ $# -eq 0 ]] && { usage; exit 1; }

SUBCMD="$1"; shift
if [[ "$SUBCMD" != "run" ]]; then
  echo "Unknown sil subcommand: $SUBCMD" >&2
  usage
  exit 1
fi

[[ $# -gt 0 ]] || { echo "Error: scenario file required" >&2; usage; exit 1; }
SCENARIO_HOST="$1"; shift

COMPONENT="motion_control"
OUTPUT_DIR="$MONO_DIR/sil_runs"
WARMUP=20

while [[ $# -gt 0 ]]; do
  case "$1" in
    --component) COMPONENT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --warmup) WARMUP="$2"; shift 2 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

# Map the component to (a) the sim-side SIL launch (simulation image: CARLA bridge +
# reference feeder + monitor + recorder) and (b) the controller-under-test, which is an
# action-module binary launched from the action image.
case "$COMPONENT" in
  motion_control)
    SIM_LAUNCH="motion_control_sil.launch.py"
    CUT_PKG="ackermann_pure_pursuit"
    CUT_EXEC="pure_pursuit_node"
    ;;
  *) echo "Error: unsupported component '$COMPONENT' (supported: motion_control)" >&2; exit 1 ;;
esac

if [[ ! -f "$SCENARIO_HOST" ]]; then
  echo "Error: scenario file not found: $SCENARIO_HOST" >&2
  exit 1
fi
SCENARIO_HOST="$(cd "$(dirname "$SCENARIO_HOST")" && pwd)/$(basename "$SCENARIO_HOST")"

# Verify a CARLA server is running (host networking exposes its RPC port on localhost)
if ! docker ps --format '{{.Names}}' | grep -qi carla; then
  echo "Error: no CARLA server container appears to be running." >&2
  echo "Start one first, e.g.:  watod up   (with simulation in ACTIVE_MODULES)" >&2
  exit 1
fi

timestamp=$(date +%Y%m%d_%H%M%S)
scenario_name="$(basename "${SCENARIO_HOST%.*}")"
run_dir="$OUTPUT_DIR/${scenario_name}_${timestamp}"
mkdir -p "$run_dir"

# In-container paths
SCENARIO_CT="/sil_scenario.yaml"
BAG_CT="/sil_out/bag"

SOURCE_WS="source /opt/watonomous/setup.bash"

# ROS graph join flags shared by every container (host net/ipc + middleware config)
graph_flags=(
  --rm
  --ulimit memlock=-1
  --ipc host
  --network host
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  -e "ZENOH_ROUTER_CONFIG_URI=${ZENOH_ROUTER_CONFIG_URI}"
  -e "ZENOH_SESSION_CONFIG_URI=${ZENOH_SESSION_CONFIG_URI}"
)

sim_container="${COMPOSE_PROJECT_NAME}-sil_sim"
cut_container="${COMPOSE_PROJECT_NAME}-sil_cut"

cleanup() {
  echo ""
  echo "Tearing down SIL run..."
  docker stop "$sim_container" >/dev/null 2>&1 || true
  docker stop "$cut_container" >/dev/null 2>&1 || true
}
trap cleanup SIGINT SIGTERM EXIT

echo "==> Scenario:  $SCENARIO_HOST"
echo "==> Component: $COMPONENT  (controller: $CUT_PKG/$CUT_EXEC from action image)"
echo "==> Output:    $run_dir"

# 1) Sim side (simulation image): CARLA bridge + reference feeder + monitor + recorder.
#    This is the run owner; it ends after the scenario duration and stops the recorder.
echo "==> Starting SIL sim side (bridge + feeder + monitor + recorder)..."
docker run -d --name "$sim_container" \
  "${graph_flags[@]}" \
  -e "SIL_SCENARIO_FILE=${SCENARIO_CT}" \
  -e "SIL_BAG_PATH=${BAG_CT}" \
  -v "${SCENARIO_HOST}:${SCENARIO_CT}:ro" \
  -v "${run_dir}:/sil_out" \
  "$SIMULATION_IMAGE:$TAG" \
  bash -lc "${SOURCE_WS} && ros2 launch sil_testing ${SIM_LAUNCH} bag_path:=${BAG_CT}" \
  >/dev/null

echo "==> Waiting ${WARMUP}s for CARLA to load the scenario..."
sleep "$WARMUP"

# 2) Controller under test (action image): the real controller node, fed by the sim-side
#    reference trajectory and publishing to the CARLA Ackermann bridge. Loads the
#    controller's own params, overriding only the SIL wiring, then is activated.
echo "==> Starting controller under test ($CUT_PKG/$CUT_EXEC)..."
cut_overrides="-r __ns:=/action \
  --params-file \$(ros2 pkg prefix ${CUT_PKG})/share/${CUT_PKG}/config/params.yaml \
  -p use_sim_time:=true \
  -p trajectory_topic:=/sil/reference_trajectory \
  -p ackermann_topic:=/carla/ackermann_control/command \
  -p odom_topic:=/ego/odom \
  -p max_speed:=15.0 \
  -p disable_standby:=true"
docker run -d --name "$cut_container" \
  "${graph_flags[@]}" \
  "$ACTION_IMAGE:$TAG" \
  bash -lc "${SOURCE_WS} && ros2 run ${CUT_PKG} ${CUT_EXEC} --ros-args ${cut_overrides}" \
  >/dev/null

# Activate the lifecycle controller (transitions can be driven from any container)
sleep 3
echo "==> Activating controller..."
docker exec "$cut_container" bash -lc \
  "${SOURCE_WS} && ros2 lifecycle set /action/${CUT_EXEC} configure && ros2 lifecycle set /action/${CUT_EXEC} activate" || true

# 3) Wait for the sim side to finish the timed run (scenario_runner -> Shutdown)
echo "==> Recording... (run ends after the scenario duration of sim time)"
docker wait "$sim_container" >/dev/null || true
docker stop "$cut_container" >/dev/null 2>&1 || true

# 4) Analyze the recorded bag against the scenario limits (simulation image has sil_testing)
echo "==> Analyzing run..."
set +e
docker run \
  "${graph_flags[@]}" \
  -e "SIL_SCENARIO_FILE=${SCENARIO_CT}" \
  -v "${SCENARIO_HOST}:${SCENARIO_CT}:ro" \
  -v "${run_dir}:/sil_out" \
  "$SIMULATION_IMAGE:$TAG" \
  bash -lc "${SOURCE_WS} && ros2 run sil_testing sil_analyzer --bag ${BAG_CT} --scenario ${SCENARIO_CT} --report /sil_out/report.json"
analyze_rc=$?
set -e

echo ""
echo "==> Bag:    $run_dir/bag"
echo "==> Report: $run_dir/report.json"
echo "==> Play it back in Foxglove with: src/simulation/sil_testing/config/foxglove/sil.json"

exit $analyze_rc
