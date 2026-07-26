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
import numpy as np
import pytest

from wato_mtr_eval import metrics, representation as rep


def test_min_ade_picks_the_best_mode():
    # One agent, two modes, three steps. Mode 1 is exact; mode 0 is off by 3.
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 2, 3, 2), dtype=np.float32)
    pred[0, 0, :, 0] = 3.0
    valid = np.ones((1, 3), dtype=bool)
    np.testing.assert_allclose(metrics.min_ade(pred, gt, valid), [0.0], atol=1e-6)


def test_min_ade_ignores_invalid_steps():
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 1, 3, 2), dtype=np.float32)
    pred[0, 0, 2, 0] = 30.0  # huge error, but that step is invalid
    valid = np.array([[True, True, False]])
    np.testing.assert_allclose(metrics.min_ade(pred, gt, valid), [0.0], atol=1e-6)


def test_min_fde_uses_the_last_valid_step_not_the_last_step():
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 1, 3, 2), dtype=np.float32)
    pred[0, 0, 1, 0] = 4.0  # last VALID step -> this is the FDE
    pred[0, 0, 2, 0] = 99.0  # invalid, must be ignored
    valid = np.array([[True, True, False]])
    np.testing.assert_allclose(metrics.min_fde(pred, gt, valid), [4.0], atol=1e-6)


def test_agent_with_no_valid_steps_yields_nan():
    gt = np.zeros((1, 3, 2), dtype=np.float32)
    pred = np.zeros((1, 1, 3, 2), dtype=np.float32)
    valid = np.zeros((1, 3), dtype=bool)
    assert np.isnan(metrics.min_ade(pred, gt, valid)[0])
    assert np.isnan(metrics.min_fde(pred, gt, valid)[0])


def test_empty_agent_batch_preserves_vector_result_shape():
    gt = np.zeros((0, 3, 2), dtype=np.float32)
    pred = np.zeros((0, 2, 3, 2), dtype=np.float32)
    valid = np.zeros((0, 3), dtype=bool)
    assert metrics.min_ade(pred, gt, valid).shape == (0,)
    assert metrics.min_fde(pred, gt, valid).shape == (0,)


def test_to_deployed_strides_and_truncates_to_15_steps():
    pred = np.arange(1 * 1 * 80 * 7, dtype=np.float32).reshape(1, 1, 80, 7)
    out = metrics.to_deployed(pred)
    assert out.shape == (1, 1, rep.DEPLOYED_STEPS, 2)
    # stride 2 over the first 30 model steps: indices 1, 3, 5, ...
    expected_first = pred[0, 0, 1, :2]
    np.testing.assert_allclose(out[0, 0, 0], expected_first)


def test_paired_delta_reports_distribution_not_just_mean():
    # 1200 agents at zero, 68 at 0.19: the failure mode an aggregate hides.
    a = np.zeros(1268, dtype=np.float64)
    b = np.zeros(1268, dtype=np.float64)
    b[:68] = 0.19
    ids = [f"agent{i}" for i in range(1268)]
    scen = ["s0"] * 1268

    d = metrics.paired_delta(b, a, ids, scen, n_worst=5)
    assert d.mean == pytest.approx(0.19 * 68 / 1268, abs=1e-9)
    assert d.median == pytest.approx(0.0)
    assert d.max == pytest.approx(0.19)
    assert d.near_zero_fraction == pytest.approx(1200 / 1268)
    assert len(d.worst) == 5
    assert all(row.delta == pytest.approx(0.19) for row in d.worst)


def test_paired_delta_distinguishes_uniform_loss_from_a_broken_path():
    ids = [f"a{i}" for i in range(1268)]
    scen = ["s0"] * 1268
    base = np.zeros(1268)

    uniform = metrics.paired_delta(np.full(1268, 0.01), base, ids, scen)
    spiky = np.zeros(1268)
    spiky[:68] = 0.19
    broken = metrics.paired_delta(spiky, base, ids, scen)

    # Means are close; only the distribution separates them.
    assert uniform.mean == pytest.approx(0.01, abs=1e-9)
    assert broken.mean == pytest.approx(0.0102, abs=1e-3)
    assert uniform.near_zero_fraction == pytest.approx(0.0)
    assert broken.near_zero_fraction == pytest.approx(1200 / 1268)


def test_paired_delta_drops_nan_pairs():
    a = np.array([1.0, np.nan, 3.0])
    b = np.array([1.0, 2.0, np.nan])
    d = metrics.paired_delta(a, b, ["x", "y", "z"], ["s"] * 3)
    assert d.count == 1


def test_paired_delta_rejects_mismatched_lengths():
    with pytest.raises(ValueError, match="same length"):
        metrics.paired_delta(np.zeros(3), np.zeros(4), ["a"] * 3, ["s"] * 3)


def test_format_paired_delta_includes_worst_rows_and_near_zero():
    ids = [f"a{i}" for i in range(10)]
    d = metrics.paired_delta(
        np.linspace(0, 1, 10), np.zeros(10), ids, ["s"] * 10, n_worst=3
    )
    text = metrics.format_paired_delta(d, "R1 -> R2 minFDE")
    assert "R1 -> R2 minFDE" in text
    assert "worst 3" in text
    assert "p90" in text
    assert "|delta| < 1e-3" in text
