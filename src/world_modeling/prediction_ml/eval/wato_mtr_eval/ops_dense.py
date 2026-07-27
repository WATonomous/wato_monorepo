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

import argparse
import dataclasses
import pathlib
import time
from collections.abc import Sequence
from typing import Any, Callable

import numpy as np
import torch
from torch import nn

from . import model, rung2, tensor_io

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


class DenseAttention(nn.Module):
    """End-to-end scaled local attention composed from standard PyTorch ops."""

    def forward(
        self,
        query_batch_cnt: torch.Tensor,
        key_batch_cnt: torch.Tensor,
        index_pair_batch: torch.Tensor,
        index_pair: torch.Tensor,
        query_features: torch.Tensor,
        key_features: torch.Tensor,
        value_features: torch.Tensor,
    ) -> torch.Tensor:
        scale = float(query_features.shape[-1]) ** -0.5
        logits = dense_attention_weight(
            query_batch_cnt,
            key_batch_cnt,
            index_pair_batch,
            index_pair,
            query_features * scale,
            key_features,
        )
        valid = index_pair >= 0
        logits = logits.masked_fill(~valid[:, :, None], float("-inf"))
        weights = torch.softmax(logits, dim=1)
        return dense_attention_value(
            query_batch_cnt,
            key_batch_cnt,
            index_pair_batch,
            index_pair,
            weights,
            value_features,
        )


def _replace(module: Any, attribute: str, replacement: Callable) -> None:
    key = (id(module), attribute)
    if key not in _ORIGINALS:
        _ORIGINALS[key] = (module, attribute, getattr(module, attribute))
    setattr(module, attribute, replacement)


def patch_mtr(net):
    """Patch every pinned-MTR custom-op call site, process-wide.

    MTR resolves these functions through imported module globals rather than
    storing operator modules on ``net``. Patching is therefore process-wide;
    ``restore_mtr_ops`` exists for tests and benchmarks that need both paths.
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
    setattr(net, "_wato_dense_ops_patched", True)
    return net


def restore_mtr_ops() -> None:
    """Restore custom MTR callables after a process-wide patch."""
    for module, attribute, original in _ORIGINALS.values():
        setattr(module, attribute, original)
    _ORIGINALS.clear()


@dataclasses.dataclass(frozen=True)
class BenchmarkResult:
    custom_ms: float
    dense_ms: float
    slowdown: float
    iterations: int

    def format(self) -> str:
        return (
            f"MTR forward ({self.iterations} iterations)\n"
            f"  custom CUDA  {self.custom_ms:.3f} ms\n"
            f"  dense ops    {self.dense_ms:.3f} ms\n"
            f"  slowdown     {self.slowdown:.2f}x"
        )


def _timed_forward(net, batch: dict, iterations: int) -> float:
    if iterations <= 0:
        raise ValueError("iterations must be positive")
    with torch.no_grad():
        for _ in range(2):
            net(batch)
        torch.cuda.synchronize()
        started = time.perf_counter()
        for _ in range(iterations):
            net(batch)
        torch.cuda.synchronize()
    return (time.perf_counter() - started) * 1000.0 / iterations


def benchmark(
    checkpoint: pathlib.Path,
    mtr_root: pathlib.Path,
    fixture: pathlib.Path,
    *,
    iterations: int = 10,
    cfg_name: str = "mtr+100_percent_data.yaml",
) -> BenchmarkResult:
    """Benchmark custom and dense forward paths on one committed fixture."""
    if not torch.cuda.is_available():
        raise RuntimeError("benchmark requires CUDA")
    checkpoint = pathlib.Path(checkpoint).expanduser().resolve()
    mtr_root = pathlib.Path(mtr_root).expanduser().resolve()
    fixture = pathlib.Path(fixture).expanduser().resolve()
    cfg_path = mtr_root / "tools/cfgs/waymo" / cfg_name
    if not checkpoint.is_file() or not cfg_path.is_file():
        raise FileNotFoundError("checkpoint or official MTR config is missing")
    if not fixture.is_dir():
        raise FileNotFoundError(f"fixture does not exist: {fixture}")

    with np.load(fixture / "ground_truth.npz", allow_pickle=False) as data:
        targets = rung2._read_targets(fixture, data["agent_ids"])
    tensors = tensor_io.read_bundle(fixture / "expected_tensors.bin.gz")
    batch = rung2._model_batch(tensors, targets, fixture.name)
    net = model.load_checkpoint(
        checkpoint,
        cfg_path,
        device="cuda",
        mtr_root=mtr_root,
    )
    restore_mtr_ops()
    custom_ms = _timed_forward(net, batch, iterations)
    patch_mtr(net)
    dense_ms = _timed_forward(net, batch, iterations)
    return BenchmarkResult(
        custom_ms=custom_ms,
        dense_ms=dense_ms,
        slowdown=dense_ms / custom_ms,
        iterations=iterations,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--checkpoint", type=pathlib.Path)
    parser.add_argument("--mtr-root", type=pathlib.Path)
    parser.add_argument("--fixture", type=pathlib.Path)
    parser.add_argument("--iterations", type=int, default=10)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not args.benchmark:
        raise SystemExit("pass --benchmark to run the dense-op benchmark")
    if args.checkpoint is None or args.mtr_root is None or args.fixture is None:
        raise SystemExit("--benchmark requires --checkpoint, --mtr-root, and --fixture")
    result = benchmark(
        args.checkpoint,
        args.mtr_root,
        args.fixture,
        iterations=args.iterations,
    )
    print(result.format())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
