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
"""Regenerate SceneBuilder parity tensor bundles from the local Waymo shard."""

from __future__ import annotations

import argparse
import pathlib

import numpy as np

from wato_mtr_eval import lanelet_query as lq
from wato_mtr_eval import reference_pack, scenario, tensor_io


def _verify_round_trip(
    path: pathlib.Path,
    expected: dict[str, np.ndarray],
) -> None:
    loaded = tensor_io.read_bundle(path)
    if loaded.keys() != expected.keys():
        raise AssertionError(
            f"{path} tensor keys differ: {loaded.keys()} != {expected.keys()}"
        )
    for name, array in expected.items():
        if loaded[name].dtype != array.dtype:
            raise AssertionError(f"{path} changed {name} dtype")
        if loaded[name].shape != array.shape:
            raise AssertionError(f"{path} changed {name} shape")
        if not np.array_equal(loaded[name], array):
            raise AssertionError(f"{path} changed {name} values")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shard", type=pathlib.Path, required=True)
    parser.add_argument("--fixtures", type=pathlib.Path, required=True)
    args = parser.parse_args()

    fixture_dirs = {
        path.name: path
        for path in args.fixtures.iterdir()
        if path.is_dir()
    }
    remaining = set(fixture_dirs)
    config = lq.QueryConfig(strategy="radius")
    for scen in scenario.load_scenarios(args.shard):
        if scen.scenario_id not in remaining:
            continue
        fixture_dir = fixture_dirs[scen.scenario_id]
        tensors = reference_pack.pack(
            scen, config, step=scen.current_time_index
        )
        bundle = fixture_dir / "expected_tensors.bin.gz"
        tensor_io.write_bundle(bundle, tensors)
        _verify_round_trip(bundle, tensors)
        reference_pack.write_replay_sidecars(fixture_dir, scen)
        print(
            f"wrote {scen.scenario_id}: step={scen.current_time_index}, "
            f"targets={','.join(reference_pack.target_ids(scen))}"
        )
        remaining.remove(scen.scenario_id)
        if not remaining:
            return 0

    raise RuntimeError(
        f"fixture scenarios absent from shard: {sorted(remaining)}"
    )


if __name__ == "__main__":
    raise SystemExit(main())
