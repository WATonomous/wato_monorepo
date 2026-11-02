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

# Regenerate the Waymo-derived replay fixtures in test/fixtures/<scenario_id>/ from a local
# Waymo Open Dataset download. The WOD terms (Section 2) allow distributing the dataset and
# modified copies only to registered WOD users, so these files are built locally and gitignored.
# Tests that need them skip when they are absent.
#
# usage: eval/regenerate_fixtures.sh <.../uncompressed_scenario_validation_validation.tfrecord-00000-of-00150>
# needs: a sourced ROS 2 workspace with the WATO messages and rosbag2 MCAP storage, numpy, and the
#        waymo-open-dataset protos (python3 -c 'from waymo_open_dataset.protos import scenario_pb2').
set -euo pipefail

shard=${1:?usage: $0 <Motion v1.3.1 validation shard 00000-of-00150>}
eval_dir=$(cd "$(dirname "$0")" && pwd)
fixtures=$eval_dir/../test/fixtures
expected=(262112a66742fe8d 923a0c6bb90f4127 9f62a2970e4e8d4c)  # dense, sparse, gappy in shard 0

# Check before deleting anything, and make the path absolute because of the cd below.
[ -f "$shard" ] || { echo "shard not found: $shard" >&2; exit 1; }
shard=$(realpath "$shard")

cd "$eval_dir"
export PYTHONPATH="$eval_dir${PYTHONPATH:+:$PYTHONPATH}"
# Only scenario fixtures are directories; the synthetic tensor_io_*.bin.gz files stay.
find "$fixtures" -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} +
python3 -m wato_mtr_eval.waymo_to_ros --shard "$shard" --out "$fixtures"
python3 tests/make_reference_fixtures.py --shard "$shard" --fixtures "$fixtures"

for id in "${expected[@]}"; do
  [ -f "$fixtures/$id/expected_tensors.bin.gz" ] || {
    echo "missing fixture $id: is $shard validation shard 00000-of-00150?" >&2
    exit 1
  }
done
echo "fixtures ready in $fixtures"
