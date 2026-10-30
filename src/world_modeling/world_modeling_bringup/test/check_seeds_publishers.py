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
"""Static check: prediction_ml is the only publisher of /world_modeling/world_object_seeds.

usage: check_seeds_publishers.py <launch.yaml>...  The files together form one bringup; across
them there must be exactly one publisher, prediction_ml. Missing files are skipped (perception's
launch file is not in the world_modeling image). Two publishers on one topic made world_model flip between
tracking's Kalman predictions and prediction_ml's every frame.
"""

import pathlib
import sys

import yaml

TOPIC = "/world_modeling/world_object_seeds"
# prediction(_ml) and tracking publish on these ports
PUBLISHER_PORTS = {"world_object_seeds", "output_predictions"}
SUBSCRIBER_PORTS = {"world_objects"}  # world_model_markers


def nodes(tree):
    if isinstance(tree, dict):
        if "pkg" in tree:
            yield tree
        for value in tree.values():
            yield from nodes(value)
    elif isinstance(tree, list):
        for item in tree:
            yield from nodes(item)


def resolve(name, namespace):
    namespace = str(namespace or "").strip("/")
    return name if name.startswith("/") or not namespace else f"/{namespace}/{name}"


def publishers(path):
    found = []
    for node in nodes(yaml.safe_load(path.read_text())):
        for remap in node.get("remap") or []:
            if resolve(str(remap["to"]), node.get("namespace")) != TOPIC:
                continue
            port = str(remap["from"])
            if port in PUBLISHER_PORTS:
                found.append(f"{node['pkg']}/{node.get('name', '?')}")
            elif port not in SUBSCRIBER_PORTS:
                raise SystemExit(f"{path}: unknown port {port!r} remapped onto {TOPIC}")
    return found


def main(paths):
    total = []
    for path in map(pathlib.Path, paths):
        if not path.is_file():
            print(f"skipped (absent): {path}")
            continue
        found = publishers(path)
        print(f"{path}: {found}")
        total += found
    if len(total) != 1 or not total[0].startswith("prediction_ml/"):
        raise SystemExit(
            f"want exactly one publisher (prediction_ml) on {TOPIC}, got {total}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
