#!/bin/bash
# Iterative LiDAR-IMU calibration refinement script
# Runs N successive skip_init calibrations, each using the previous result as initial guess.
# Results are logged to a CSV file for post-processing.

set -e

NUM_ITERATIONS=${1:-100}
BAG_DIR="/ws/bags/recording_20260224_011359"
BAG_FILES=(
    "$BAG_DIR/recording_20260224_011359_0.mcap"
)
TOPICS="/lidar_cc/velodyne_points /novatel/oem7/imu/data_raw"
YAML_FILE="/ws/install/lidar_imu_init/share/lidar_imu_init/config/velodyne.yaml"
RESULTS_CSV="/ws/scripts/calibration_results.csv"
LOG_DIR="/ws/scripts/calibration_logs"

mkdir -p "$LOG_DIR"

# Starting values (from last run — R_LI as rpy, T_LI as xyz, in imu->lidar direction)
RPY_R="0.005644"
RPY_P="0.030361"
RPY_Y="1.675885"
XYZ_X="-0.077942"
XYZ_Y="0.811530"
XYZ_Z="1.360580"
TIME_LAG="-0.050064"

# Write CSV header
echo "iteration,rot_roll_deg,rot_pitch_deg,rot_yaw_deg,trans_x,trans_y,trans_z,time_lag,urdf_xyz_x,urdf_xyz_y,urdf_xyz_z,urdf_rpy_r,urdf_rpy_p,urdf_rpy_y" > "$RESULTS_CSV"

source /ws/install/setup.bash

cleanup() {
    # Kill all li_init and rosbag2 processes
    pkill -f "li_init" 2>/dev/null || true
    pkill -f "ros2.bag.play" 2>/dev/null || true
    sleep 1
    # Force kill any survivors
    pkill -9 -f "li_init" 2>/dev/null || true
    pkill -9 -f "ros2.bag.play" 2>/dev/null || true
    sleep 1
}

trap cleanup EXIT

for i in $(seq 1 $NUM_ITERATIONS); do
    echo "============================================"
    echo "  Iteration $i / $NUM_ITERATIONS"
    echo "  Input RPY: $RPY_R $RPY_P $RPY_Y"
    echo "  Input XYZ: $XYZ_X $XYZ_Y $XYZ_Z"
    echo "============================================"

    # Update yaml with current values
    sed -i "s|init_rpy_LI: \[.*\]|init_rpy_LI: [ $RPY_R, $RPY_P, $RPY_Y ]|" "$YAML_FILE"
    sed -i "s|init_xyz_LI: \[.*\]|init_xyz_LI: [ $XYZ_X, $XYZ_Y, $XYZ_Z ]|" "$YAML_FILE"
    sed -i "s|init_time_lag: .*|init_time_lag: $TIME_LAG|" "$YAML_FILE"

    LOG_FILE="$LOG_DIR/iteration_${i}.log"

    # Start calibration node in background, truncate log to prevent disk fill
    ros2 run lidar_imu_init li_init \
        --ros-args \
        -p point_filter_num:=3 \
        -p max_iteration:=5 \
        -p cube_side_length:=1000.0 \
        --params-file "$YAML_FILE" \
        > "$LOG_FILE" 2>&1 &
    NODE_PID=$!

    # Wait for node to initialize
    sleep 3

    # Play all bag files at 0.75x speed so node can keep up
    for bag in "${BAG_FILES[@]}"; do
        ros2 bag play "$bag" --rate 0.75 --topics $TOPICS 2>/dev/null
    done

    # Wait for node to finish processing buffered data
    # Poll for "Final Result" in log, up to 300s
    echo "  Bag done, waiting for refinement to complete..."
    for w in $(seq 1 300); do
        if grep -q "Final Result" "$LOG_FILE" 2>/dev/null; then
            echo "  Results found after ${w}s"
            sleep 3  # Let it finish writing all URDF output
            break
        fi
        # Check if node is still alive
        if ! kill -0 $NODE_PID 2>/dev/null; then
            echo "  Node exited early after ${w}s"
            break
        fi
        sleep 1
    done

    # Kill the node
    cleanup
    wait $NODE_PID 2>/dev/null || true

    # Strip ANSI codes and extract only relevant lines to a small file
    CLEAN_FILE="${LOG_FILE}.clean"
    sed 's/\x1b\[[0-9;]*m//g' "$LOG_FILE" | grep -E "(Final Result|URDF|parent=)" > "$CLEAN_FILE" 2>/dev/null || true
    # Delete the large raw log to save disk
    rm -f "$LOG_FILE"

    # Parse results from cleaned log
    FINAL_ROT=$(grep "Final Result.*Rotation LiDAR to IMU" "$CLEAN_FILE" | tail -1 | sed 's/.*= *//' | sed 's/ *deg//')
    FINAL_TRANS=$(grep "Final Result.*Translation LiDAR to IMU" "$CLEAN_FILE" | tail -1 | sed 's/.*= *//' | sed 's/ *m//')
    FINAL_LAG=$(grep "Final Result.*Time Lag IMU to LiDAR" "$CLEAN_FILE" | tail -1 | sed 's/.*= *//' | sed 's/ *s.*//')

    # Parse URDF values (parent=lidar, child=imu)
    URDF_XYZ=$(grep -A1 "parent=lidar_link" "$CLEAN_FILE" | grep "xyz=" | tail -1 | sed 's/.*xyz="//' | sed 's/"//')
    URDF_RPY=$(grep -A2 "parent=lidar_link" "$CLEAN_FILE" | grep "rpy=" | tail -1 | sed 's/.*rpy="//' | sed 's/"//')

    # Parse URDF values (parent=imu, child=lidar) for next iteration's input
    NEW_RPY=$(grep -A2 "parent=imu_link" "$CLEAN_FILE" | grep "rpy=" | tail -1 | sed 's/.*rpy="//' | sed 's/"//')
    NEW_XYZ=$(grep -A1 "parent=imu_link" "$CLEAN_FILE" | grep "xyz=" | tail -1 | sed 's/.*xyz="//' | sed 's/"//')

    if [ -z "$FINAL_ROT" ] || [ -z "$FINAL_TRANS" ] || [ -z "$NEW_RPY" ] || [ -z "$NEW_XYZ" ]; then
        echo "WARNING: Iteration $i failed to parse results, dumping log tail:"
        tail -30 "$LOG_FILE"
        echo "  FINAL_ROT='$FINAL_ROT' FINAL_TRANS='$FINAL_TRANS'"
        echo "  NEW_RPY='$NEW_RPY' NEW_XYZ='$NEW_XYZ'"
        echo "Skipping..."
        continue
    fi

    # Extract individual values
    ROT_R=$(echo "$FINAL_ROT" | awk '{print $1}')
    ROT_P=$(echo "$FINAL_ROT" | awk '{print $2}')
    ROT_Y=$(echo "$FINAL_ROT" | awk '{print $3}')
    TRANS_X=$(echo "$FINAL_TRANS" | awk '{print $1}')
    TRANS_Y=$(echo "$FINAL_TRANS" | awk '{print $2}')
    TRANS_Z=$(echo "$FINAL_TRANS" | awk '{print $3}')

    U_XYZ_X=$(echo "$URDF_XYZ" | awk '{print $1}')
    U_XYZ_Y=$(echo "$URDF_XYZ" | awk '{print $2}')
    U_XYZ_Z=$(echo "$URDF_XYZ" | awk '{print $3}')
    U_RPY_R=$(echo "$URDF_RPY" | awk '{print $1}')
    U_RPY_P=$(echo "$URDF_RPY" | awk '{print $2}')
    U_RPY_Y=$(echo "$URDF_RPY" | awk '{print $3}')

    # Write to CSV
    echo "$i,$ROT_R,$ROT_P,$ROT_Y,$TRANS_X,$TRANS_Y,$TRANS_Z,$FINAL_LAG,$U_XYZ_X,$U_XYZ_Y,$U_XYZ_Z,$U_RPY_R,$U_RPY_P,$U_RPY_Y" >> "$RESULTS_CSV"

    echo "  Result: Rot=($ROT_R, $ROT_P, $ROT_Y) deg"
    echo "  Result: Trans=($TRANS_X, $TRANS_Y, $TRANS_Z) m"
    echo "  URDF (lidar->imu): xyz=($U_XYZ_X, $U_XYZ_Y, $U_XYZ_Z) rpy=($U_RPY_R, $U_RPY_P, $U_RPY_Y)"

    # Update values for next iteration
    RPY_R=$(echo "$NEW_RPY" | awk '{print $1}')
    RPY_P=$(echo "$NEW_RPY" | awk '{print $2}')
    RPY_Y=$(echo "$NEW_RPY" | awk '{print $3}')
    XYZ_X=$(echo "$NEW_XYZ" | awk '{print $1}')
    XYZ_Y=$(echo "$NEW_XYZ" | awk '{print $2}')
    XYZ_Z=$(echo "$NEW_XYZ" | awk '{print $3}')

    echo "  Next init: RPY=($RPY_R, $RPY_P, $RPY_Y) XYZ=($XYZ_X, $XYZ_Y, $XYZ_Z)"
    echo ""
done

echo ""
echo "============================================"
echo "  All $NUM_ITERATIONS iterations complete"
echo "  Results saved to: $RESULTS_CSV"
echo "  Logs saved to: $LOG_DIR/"
echo "============================================"

# Print summary statistics
echo ""
echo "Summary (URDF parent=lidar, child=imu):"
python3 -c "
import csv
import statistics

with open('$RESULTS_CSV') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

if not rows:
    print('No results')
    exit()

# Skip first 10 as burn-in
burn_in = min(10, len(rows) // 2)
rows = rows[burn_in:]

fields = {
    'Yaw (deg)': 'rot_yaw_deg',
    'Roll (deg)': 'rot_roll_deg',
    'Pitch (deg)': 'rot_pitch_deg',
    'URDF X (m)': 'urdf_xyz_x',
    'URDF Y (m)': 'urdf_xyz_y',
    'URDF Z (m)': 'urdf_xyz_z',
    'URDF Roll (rad)': 'urdf_rpy_r',
    'URDF Pitch (rad)': 'urdf_rpy_p',
    'URDF Yaw (rad)': 'urdf_rpy_y',
    'Time lag (s)': 'time_lag',
}

print(f'Using {len(rows)} iterations (after {burn_in} burn-in)')
print(f'{\"Parameter\":<20} {\"Mean\":>12} {\"Std\":>12} {\"Min\":>12} {\"Max\":>12}')
print('-' * 68)
for name, col in fields.items():
    vals = [float(r[col]) for r in rows if r[col]]
    if vals:
        print(f'{name:<20} {statistics.mean(vals):>12.6f} {statistics.stdev(vals) if len(vals)>1 else 0:>12.6f} {min(vals):>12.6f} {max(vals):>12.6f}')

print()
print('Recommended URDF (mean values, parent=lidar_link, child=imu_link):')
xyz_x = statistics.mean([float(r['urdf_xyz_x']) for r in rows])
xyz_y = statistics.mean([float(r['urdf_xyz_y']) for r in rows])
xyz_z = statistics.mean([float(r['urdf_xyz_z']) for r in rows])
rpy_r = statistics.mean([float(r['urdf_rpy_r']) for r in rows])
rpy_p = statistics.mean([float(r['urdf_rpy_p']) for r in rows])
rpy_y = statistics.mean([float(r['urdf_rpy_y']) for r in rows])
print(f'  <origin xyz=\"{xyz_x:.6f} {xyz_y:.6f} {xyz_z:.6f}\" rpy=\"{rpy_r:.6f} {rpy_p:.6f} {rpy_y:.6f}\"/>')
"
