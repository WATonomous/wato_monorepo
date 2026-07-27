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
from __future__ import annotations

import os
import pathlib

import numpy as np
import pytest

torch = pytest.importorskip("torch")
model = pytest.importorskip("wato_mtr_eval.model")
ops_dense = pytest.importorskip("wato_mtr_eval.ops_dense")
rung2 = pytest.importorskip("wato_mtr_eval.rung2")
tensor_io = pytest.importorskip("wato_mtr_eval.tensor_io")


MTR_ROOT = os.environ.get("MTR_ROOT")
MTR_CHECKPOINT = os.environ.get("MTR_CHECKPOINT")
HAS_MTR = bool(MTR_ROOT and pathlib.Path(MTR_ROOT).is_dir())
HAS_CHECKPOINT = bool(
    HAS_MTR and MTR_CHECKPOINT and pathlib.Path(MTR_CHECKPOINT).is_file()
)
needs_mtr_cuda = pytest.mark.skipif(
    not HAS_MTR or not torch.cuda.is_available(),
    reason="set MTR_ROOT to a built pinned checkout and provide CUDA",
)
needs_checkpoint_cuda = pytest.mark.skipif(
    not HAS_CHECKPOINT or not torch.cuda.is_available(),
    reason="set MTR_ROOT and MTR_CHECKPOINT and provide CUDA",
)
FIXTURE = (
    pathlib.Path(__file__).resolve().parents[2]
    / "test"
    / "fixtures"
    / "923a0c6bb90f4127"
)


@pytest.fixture(autouse=True)
def _restore_custom_ops():
    ops_dense.restore_mtr_ops()
    yield
    ops_dense.restore_mtr_ops()


def test_dense_knn_returns_batch_local_indices_and_minus_one_padding():
    query_xyz = torch.tensor(
        [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
        ]
    )
    xyz = torch.tensor(
        [
            [0.1, 0.0, 0.0],
            [1.8, 0.0, 0.0],
            [10.2, 0.0, 0.0],
        ]
    )
    batch_idxs = torch.tensor([0, 0, 1], dtype=torch.int32)
    query_offsets = torch.tensor([0, 2, 3], dtype=torch.int32)

    actual = ops_dense.dense_knn(xyz, query_xyz, batch_idxs, query_offsets, 2)

    assert actual.dtype == torch.int32
    assert actual.tolist() == [[0, 1], [1, 0], [0, -1]]


def test_dense_attention_uses_batch_local_indices_and_ignores_padding():
    query_counts = torch.tensor([1, 1], dtype=torch.int32)
    key_counts = torch.tensor([2, 2], dtype=torch.int32)
    pair_batches = torch.tensor([0, 1], dtype=torch.int32)
    index_pair = torch.tensor([[1, -1], [0, 1]], dtype=torch.int32)
    query = torch.tensor([[[2.0, 3.0]], [[5.0, 7.0]]])
    key = torch.tensor(
        [
            [[11.0, 13.0]],
            [[17.0, 19.0]],
            [[23.0, 29.0]],
            [[31.0, 37.0]],
        ]
    )
    value = torch.tensor(
        [
            [[1.0, 2.0]],
            [[3.0, 4.0]],
            [[5.0, 6.0]],
            [[7.0, 8.0]],
        ]
    )

    logits = ops_dense.dense_attention_weight(
        query_counts,
        key_counts,
        pair_batches,
        index_pair,
        query,
        key,
    )
    weights = torch.tensor([[[1.0], [9.0]], [[0.25], [0.75]]])
    output = ops_dense.dense_attention_value(
        query_counts,
        key_counts,
        pair_batches,
        index_pair,
        weights,
        value,
    )

    torch.testing.assert_close(
        logits[:, :, 0],
        torch.tensor([[91.0, 0.0], [318.0, 414.0]]),
    )
    torch.testing.assert_close(
        output[:, 0, :],
        torch.tensor([[3.0, 4.0], [6.5, 7.5]]),
    )


def _import_pinned_ops():
    model._import_mtr(pathlib.Path(MTR_ROOT))
    from mtr.ops import attention
    from mtr.ops.knn import knn_utils

    return attention, knn_utils


@needs_mtr_cuda
def test_dense_knn_matches_pinned_cuda_kernel_neighbor_sets():
    _, knn_utils = _import_pinned_ops()
    generator = torch.Generator(device="cuda").manual_seed(1729)
    query_counts = [11, 7]
    xyz_counts = [9, 6]
    query_xyz = torch.randn((sum(query_counts), 3), generator=generator, device="cuda")
    xyz = torch.randn((sum(xyz_counts), 3), generator=generator, device="cuda")
    batch_idxs = torch.repeat_interleave(
        torch.arange(2, device="cuda", dtype=torch.int32),
        torch.tensor(xyz_counts, device="cuda"),
    )
    query_offsets = torch.tensor(
        [0, query_counts[0], sum(query_counts)],
        device="cuda",
        dtype=torch.int32,
    )

    expected = knn_utils.knn_batch_mlogk(xyz, query_xyz, batch_idxs, query_offsets, 4)
    actual = ops_dense.dense_knn(xyz, query_xyz, batch_idxs, query_offsets, 4)

    # The CUDA heap does not promise nearest-neighbor ordering.
    torch.testing.assert_close(actual.sort(dim=1).values, expected.sort(dim=1).values)


@needs_mtr_cuda
@pytest.mark.parametrize("version", ["v1", "v2"])
def test_dense_attention_matches_each_pinned_cuda_kernel(version):
    attention, _ = _import_pinned_ops()
    implementation = attention.__all__[version]
    generator = torch.Generator(device="cuda").manual_seed(8128)
    query_counts = torch.tensor([4, 3], dtype=torch.int32, device="cuda")
    key_counts = torch.tensor([5, 4], dtype=torch.int32, device="cuda")
    pair_batches = torch.tensor([0, 0, 0, 0, 1, 1, 1], dtype=torch.int32, device="cuda")
    index_pair = torch.tensor(
        [
            [0, 2, 4, -1],
            [1, 3, 0, 2],
            [4, 2, -1, -1],
            [3, 0, 1, 4],
            [0, 1, 3, -1],
            [2, 3, 1, 0],
            [3, 0, -1, -1],
        ],
        dtype=torch.int32,
        device="cuda",
    )
    query = torch.randn((7, 2, 16), generator=generator, device="cuda")
    key = torch.randn((9, 2, 16), generator=generator, device="cuda")
    value = torch.randn((9, 2, 16), generator=generator, device="cuda")

    expected_logits = implementation.attention_weight_computation(
        query_counts,
        key_counts,
        pair_batches,
        index_pair,
        query,
        key,
    )
    actual_logits = ops_dense.dense_attention_weight(
        query_counts,
        key_counts,
        pair_batches,
        index_pair,
        query,
        key,
    )
    valid = index_pair >= 0
    normalized = torch.softmax(
        actual_logits.masked_fill(~valid[:, :, None], float("-inf")),
        dim=1,
    )
    expected_value = implementation.attention_value_computation(
        query_counts,
        key_counts,
        pair_batches,
        index_pair,
        normalized,
        value,
    )
    actual_value = ops_dense.dense_attention_value(
        query_counts,
        key_counts,
        pair_batches,
        index_pair,
        normalized,
        value,
    )

    torch.testing.assert_close(actual_logits, expected_logits)
    torch.testing.assert_close(actual_value, expected_value)


@needs_mtr_cuda
def test_patch_mtr_replaces_all_pinned_call_sites():
    attention, knn_utils = _import_pinned_ops()
    sentinel = type("Network", (), {})()

    patched = ops_dense.patch_mtr(sentinel)

    assert patched is sentinel
    assert knn_utils.knn_batch is ops_dense.dense_knn
    assert knn_utils.knn_batch_mlogk is ops_dense.dense_knn
    for implementation in attention.__all__.values():
        assert (
            implementation.attention_weight_computation
            is ops_dense.dense_attention_weight
        )
        assert (
            implementation.attention_value_computation
            is ops_dense.dense_attention_value
        )


@needs_checkpoint_cuda
def test_real_patched_model_matches_custom_cuda_end_to_end():
    assert FIXTURE.is_dir()
    checkpoint = pathlib.Path(MTR_CHECKPOINT)
    root = pathlib.Path(MTR_ROOT)
    cfg = root / "tools/cfgs/waymo/mtr+100_percent_data.yaml"
    with np.load(FIXTURE / "ground_truth.npz", allow_pickle=False) as data:
        targets = rung2._read_targets(FIXTURE, data["agent_ids"])
    tensors = tensor_io.read_bundle(FIXTURE / "expected_tensors.bin.gz")
    batch = rung2._model_batch(tensors, targets, FIXTURE.name)
    net = model.load_checkpoint(checkpoint, cfg, device="cuda", mtr_root=root)

    expected_scores, expected_trajs = model.forward_batch(net, batch)
    ops_dense.patch_mtr(net)
    actual_scores, actual_trajs = model.forward_batch(net, batch)

    np.testing.assert_allclose(actual_scores, expected_scores, rtol=2e-5, atol=2e-6)
    np.testing.assert_allclose(actual_trajs, expected_trajs, rtol=2e-5, atol=2e-5)
