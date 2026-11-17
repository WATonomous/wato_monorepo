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
"""Exportable PyTorch replacements for MTR's custom CUDA operators.

The public signatures mirror the pinned MTR checkout exactly:

* KNN consumes flattened points, per-point batch IDs, and query offsets, then
  returns batch-local ``int32`` neighbor indices.
* Attention weight/value computation consumes batch-local ``index_pair``
  tensors, where ``-1`` marks padding.

The dense forms are deliberately ordinary PyTorch tensor operations. They are
slower than the custom kernels, but can be represented by standard ONNX ops.
"""

from __future__ import annotations

from typing import Any, Callable

import torch

_ORIGINALS: dict[tuple[int, str], tuple[Any, str, Callable]] = {}


def _validate_knn_shapes(
    xyz: torch.Tensor,
    query_xyz: torch.Tensor,
    batch_idxs: torch.Tensor,
    query_batch_offsets: torch.Tensor,
    k: int,
) -> None:
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f"xyz must have shape [N,3], got {tuple(xyz.shape)}")
    if query_xyz.ndim != 2 or query_xyz.shape[1] != 3:
        raise ValueError(
            f"query_xyz must have shape [M,3], got {tuple(query_xyz.shape)}"
        )
    if batch_idxs.shape != (xyz.shape[0],):
        raise ValueError(
            f"batch_idxs must have shape [{xyz.shape[0]}], "
            f"got {tuple(batch_idxs.shape)}"
        )
    if query_batch_offsets.ndim != 1 or len(query_batch_offsets) < 2:
        raise ValueError("query_batch_offsets must have shape [B+1]")
    if not isinstance(k, int) or k <= 0:
        raise ValueError("k must be a positive integer")
    if k > query_xyz.shape[0]:
        raise ValueError(f"k={k} exceeds total query count {query_xyz.shape[0]}")
    if not xyz.dtype.is_floating_point or query_xyz.dtype != xyz.dtype:
        raise ValueError("xyz and query_xyz must share a floating dtype")


def dense_knn(
    xyz: torch.Tensor,
    query_xyz: torch.Tensor,
    batch_idxs: torch.Tensor,
    query_batch_offsets: torch.Tensor,
    k: int,
) -> torch.Tensor:
    """Exact-signature dense replacement for both pinned MTR KNN kernels.

    Returned indices are local to each query batch, matching ``knn_gpu.cu``.
    Batches with fewer than ``k`` candidates receive ``-1`` padding.
    """
    _validate_knn_shapes(xyz, query_xyz, batch_idxs, query_batch_offsets, k)
    offsets = query_batch_offsets.to(dtype=torch.long)
    batch_ids = batch_idxs.to(dtype=torch.long)
    # Derive each query's batch directly from the exclusive upper offsets.
    # ``repeat_interleave(counts)`` is equivalent, but the legacy ONNX
    # exporter lowers it through sequence-valued SplitToSequence/Loop nodes
    # that TensorRT cannot parse.
    query_positions = torch.arange(
        query_xyz.shape[0],
        device=query_xyz.device,
        dtype=torch.long,
    )
    query_batches = (query_positions[:, None] >= offsets[None, 1:]).sum(dim=1)

    # Squared Euclidean distance avoids a needless sqrt and matches CUDA.
    difference = xyz[:, None, :] - query_xyz[None, :, :]
    distance_sq = (difference * difference).sum(dim=-1)
    same_batch = batch_ids[:, None] == query_batches[None, :]
    distance_sq = distance_sq.masked_fill(~same_batch, float("inf"))
    distances, global_indices = torch.topk(
        distance_sq, k=k, dim=1, largest=False, sorted=True
    )
    local_indices = global_indices - offsets[batch_ids, None]
    local_indices = torch.where(
        torch.isfinite(distances),
        local_indices,
        torch.full_like(local_indices, -1),
    )
    return local_indices.to(dtype=torch.int32)


def _global_key_indices(
    key_batch_cnt: torch.Tensor,
    index_pair_batch: torch.Tensor,
    index_pair: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    key_counts = key_batch_cnt.to(dtype=torch.long)
    key_offsets = torch.cat(
        [
            torch.zeros(1, dtype=torch.long, device=key_batch_cnt.device),
            torch.cumsum(key_counts, dim=0),
        ]
    )
    batch = index_pair_batch.to(dtype=torch.long)
    valid = index_pair >= 0
    safe_local = index_pair.to(dtype=torch.long).clamp_min(0)
    global_indices = safe_local + key_offsets[batch, None]
    return global_indices, valid


def dense_attention_weight(
    query_batch_cnt: torch.Tensor,
    key_batch_cnt: torch.Tensor,
    index_pair_batch: torch.Tensor,
    index_pair: torch.Tensor,
    query_features: torch.Tensor,
    key_features: torch.Tensor,
) -> torch.Tensor:
    """Dense replacement for MTR attention weight computation v1 and v2."""
    del query_batch_cnt
    if query_features.ndim != 3 or key_features.ndim != 3:
        raise ValueError("query_features and key_features must have rank 3")
    if query_features.shape[1:] != key_features.shape[1:]:
        raise ValueError("query/key head count and head dimension must match")
    if index_pair.shape[0] != query_features.shape[0]:
        raise ValueError("index_pair rows must equal query feature rows")
    if index_pair_batch.shape != (query_features.shape[0],):
        raise ValueError("index_pair_batch must have one entry per query")

    global_indices, valid = _global_key_indices(
        key_batch_cnt, index_pair_batch, index_pair
    )
    gathered_keys = key_features[global_indices]
    weights = (query_features[:, None, :, :] * gathered_keys).sum(dim=-1)
    return weights.masked_fill(~valid[:, :, None], 0.0)


def dense_attention_value(
    query_batch_cnt: torch.Tensor,
    key_batch_cnt: torch.Tensor,
    index_pair_batch: torch.Tensor,
    index_pair: torch.Tensor,
    attention_weights: torch.Tensor,
    value_features: torch.Tensor,
) -> torch.Tensor:
    """Dense replacement for MTR attention value computation v1 and v2."""
    del query_batch_cnt
    if value_features.ndim != 3 or attention_weights.ndim != 3:
        raise ValueError("value_features and attention_weights must have rank 3")
    if index_pair.shape != attention_weights.shape[:2]:
        raise ValueError("index_pair must match attention weight query/local dims")
    if attention_weights.shape[2] != value_features.shape[1]:
        raise ValueError("attention/value head count must match")

    global_indices, valid = _global_key_indices(
        key_batch_cnt, index_pair_batch, index_pair
    )
    gathered_values = value_features[global_indices]
    weighted = (attention_weights[:, :, :, None] * gathered_values).masked_fill(
        ~valid[:, :, None, None], 0.0
    )
    return weighted.sum(dim=1)


def _replace(module: Any, attribute: str, replacement: Callable) -> None:
    key = (id(module), attribute)
    if key not in _ORIGINALS:
        _ORIGINALS[key] = (module, attribute, getattr(module, attribute))
    setattr(module, attribute, replacement)


def patch_mtr(net):
    """Patch every pinned-MTR custom-op call site, process-wide.

    MTR resolves these functions through imported module globals rather than
    storing operator modules on ``net``. Patching is therefore process-wide;
    ``restore_mtr_ops`` exists for tests that need both paths.
    """
    try:
        from mtr.ops import attention
        from mtr.ops.knn import knn_utils
    except ImportError as error:
        raise RuntimeError(
            "could not import pinned MTR operators; import/load MTR first"
        ) from error

    _replace(knn_utils, "knn_batch", dense_knn)
    _replace(knn_utils, "knn_batch_mlogk", dense_knn)
    versions = attention.__all__
    if set(versions) != {"v1", "v2"}:
        raise RuntimeError(f"unexpected MTR attention versions: {sorted(versions)}")
    for implementation in versions.values():
        _replace(
            implementation,
            "attention_weight_computation",
            dense_attention_weight,
        )
        _replace(
            implementation,
            "attention_value_computation",
            dense_attention_value,
        )
    return net


def restore_mtr_ops() -> None:
    """Restore custom MTR callables after a process-wide patch."""
    for module, attribute, original in _ORIGINALS.values():
        setattr(module, attribute, original)
    _ORIGINALS.clear()
