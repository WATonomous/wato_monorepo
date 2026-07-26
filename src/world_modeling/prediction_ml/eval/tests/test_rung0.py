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

import dataclasses

import numpy as np
import pytest

pytest.importorskip("torch")
rung0 = pytest.importorskip("wato_mtr_eval.rung0")


def test_official_anchor_samples_two_hz_and_all_three_horizons():
    gt = np.zeros((1, 80, 2), dtype=np.float32)
    pred = np.zeros((1, 2, 80, 7), dtype=np.float32)
    valid = np.ones((1, 80), dtype=bool)

    ades, fdes = rung0._official_anchor_batch(pred, gt, valid)

    assert len(ades) == len(fdes) == 3
    for values in [*ades, *fdes]:
        np.testing.assert_array_equal(values, [0.0])


def test_report_only_opens_anchor_for_finite_full_results():
    passing = rung0.Rung0Report(
        num_scenarios=286,
        num_agents=1268,
        min_ade_native=rung0.PUBLISHED_MIN_ADE,
        min_fde_native=rung0.PUBLISHED_MIN_FDE,
        min_ade_deployed=0.5,
        min_fde_deployed=1.0,
        per_type={},
    )
    incomplete = dataclasses.replace(passing, min_ade_native=float("nan"))

    assert passing.anchor_ok()
    assert not incomplete.anchor_ok()
