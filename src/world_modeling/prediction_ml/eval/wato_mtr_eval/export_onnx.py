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
"""Export the pinned MTR checkpoint with the frozen eight-target ONNX contract."""

from __future__ import annotations

import argparse
import dataclasses
import pathlib
import types
from collections.abc import Sequence
from typing import Any

import torch
import yaml
from torch import nn
from torch.nn import functional as F

from . import model, ops_dense

INPUT_NAMES = (
    "obj_trajs",
    "obj_trajs_mask",
    "obj_trajs_last_pos",
    "map_polylines",
    "map_polylines_mask",
    "map_polylines_center",
    "track_index_to_predict",
    "center_type_ids",
)
OUTPUT_NAMES = ("pred_scores", "pred_trajs")
_DTYPES = {
    "float32": torch.float32,
    "bool": torch.bool,
    "int64": torch.int64,
}
_CENTER_TYPES = (
    "TYPE_VEHICLE",
    "TYPE_PEDESTRIAN",
    "TYPE_CYCLIST",
)


@dataclasses.dataclass(frozen=True)
class Binding:
    name: str
    dtype: torch.dtype
    shape: tuple[int, ...]


@dataclasses.dataclass(frozen=True)
class Contract:
    inputs: tuple[Binding, ...]
    outputs: tuple[Binding, ...]


def _bindings(raw: Any, expected_names: tuple[str, ...]) -> tuple[Binding, ...]:
    if not isinstance(raw, list):
        raise ValueError("contract bindings must be lists")
    bindings = []
    for item in raw:
        if not isinstance(item, dict):
            raise ValueError("each contract binding must be a mapping")
        name = item.get("name")
        dtype_name = item.get("dtype")
        shape = item.get("shape")
        if dtype_name not in _DTYPES:
            raise ValueError(f"unsupported dtype for {name}: {dtype_name}")
        if (
            not isinstance(shape, list)
            or not shape
            or any(not isinstance(dim, int) or dim <= 0 for dim in shape)
        ):
            raise ValueError(f"{name} must have a fully static positive shape")
        bindings.append(
            Binding(
                name=str(name),
                dtype=_DTYPES[dtype_name],
                shape=tuple(shape),
            )
        )
    names = tuple(binding.name for binding in bindings)
    if names != expected_names:
        raise ValueError(
            f"contract binding names/order are {names}, expected {expected_names}"
        )
    return tuple(bindings)


def load_contract(path: pathlib.Path) -> Contract:
    """Load and strictly validate the static ONNX binding contract."""
    path = pathlib.Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"contract does not exist: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("contract root must be a mapping")
    return Contract(
        inputs=_bindings(raw.get("inputs"), INPUT_NAMES),
        outputs=_bindings(raw.get("outputs"), OUTPUT_NAMES),
    )


def dummy_inputs(
    contract: Contract, device: torch.device | str
) -> tuple[torch.Tensor, ...]:
    """Create a valid sparse sample at exactly the frozen binding shapes."""
    tensors = [
        torch.zeros(binding.shape, dtype=binding.dtype, device=device)
        for binding in contract.inputs
    ]
    by_name = dict(zip(INPUT_NAMES, tensors, strict=True))

    # Every target needs at least one valid object and one valid map polyline.
    # Sparse validity keeps export/validation memory bounded while the input
    # tensor dimensions remain the full deployment contract.
    by_name["obj_trajs_mask"][:, 0, :] = True
    by_name["map_polylines_mask"][:, 0, :] = True
    by_name["map_polylines_center"][:, 0, 0] = 1.0
    by_name["center_type_ids"].copy_(
        torch.arange(
            by_name["center_type_ids"].shape[0],
            device=device,
            dtype=torch.int64,
        )
        % len(_CENTER_TYPES)
    )
    return tuple(by_name[name] for name in INPUT_NAMES)


def _get_motion_query_from_type_ids(decoder, center_type_ids):
    """Tensor-only equivalent of MTRDecoder.get_motion_query."""
    point_bank = torch.stack(
        [decoder.intention_points[name] for name in _CENTER_TYPES], dim=0
    )
    intention_points = point_bank[center_type_ids.to(dtype=torch.long)]
    intention_points = intention_points.permute(1, 0, 2)
    intention_query = decoder.intention_query_mlps(
        decoder.position_encoding(intention_points, hidden_dim=decoder.d_model).view(
            -1, decoder.d_model
        )
    ).view(-1, center_type_ids.shape[0], decoder.d_model)
    return intention_query, intention_points


def _apply_dense_future_prediction_tensor(
    decoder,
    obj_feature,
    obj_mask,
    obj_pos,
):
    """Assignment-free equivalent of MTR's dense object prediction."""
    batch_size, num_objects, channels = obj_feature.shape
    obj_pos_xy = obj_pos[..., 0:2]
    pos_feature = decoder.obj_pos_encoding_layer(obj_pos_xy.reshape(-1, 2)).view(
        batch_size, num_objects, -1
    )
    fused = torch.cat((pos_feature, obj_feature), dim=-1)
    dense_trajs = decoder.dense_future_head(fused.reshape(-1, fused.shape[-1])).view(
        batch_size, num_objects, decoder.num_future_frames, 7
    )
    dense_trajs = torch.cat(
        (
            dense_trajs[..., 0:2] + obj_pos_xy[:, :, None, :],
            dense_trajs[..., 2:],
        ),
        dim=-1,
    )

    future_input = dense_trajs[..., [0, 1, -2, -1]].flatten(start_dim=2)
    future_feature = decoder.future_traj_mlps(
        future_input.reshape(-1, 4 * decoder.num_future_frames)
    ).view(batch_size, num_objects, channels)
    full_feature = torch.cat((obj_feature, future_feature), dim=-1)
    output = decoder.traj_fusion_mlps(
        full_feature.reshape(-1, full_feature.shape[-1])
    ).view(batch_size, num_objects, channels)

    output = output.masked_fill(~obj_mask[:, :, None], 0.0)
    dense_trajs = dense_trajs.masked_fill(~obj_mask[:, :, None, None], 0.0)
    decoder.forward_ret_dict["pred_dense_trajs"] = dense_trajs
    return output, dense_trajs


def _apply_dynamic_map_collection_tensor(
    decoder,
    map_pos,
    map_mask,
    pred_waypoints,
    base_region_offset,
    num_query,
    num_waypoint_polylines=128,
    num_base_polylines=256,
    base_map_idxs=None,
):
    """Assignment-free equivalent of MTR's dynamic map selection."""
    safe_map_pos = map_pos.masked_fill(~map_mask[:, :, None], 10_000_000.0)
    num_polylines = map_pos.shape[1]
    if base_map_idxs is None:
        base_points = map_pos.new_tensor(base_region_offset)
        base_dist = (safe_map_pos[:, :, 0:2] - base_points[None, None, :]).norm(dim=-1)
        base_topk_dist, base_map_idxs = base_dist.topk(
            k=min(num_polylines, num_base_polylines),
            dim=-1,
            largest=False,
        )
        base_map_idxs = torch.where(
            base_topk_dist > 10_000_000.0,
            torch.full_like(base_map_idxs, -1),
            base_map_idxs,
        )
        base_map_idxs = base_map_idxs[:, None, :].repeat(1, num_query, 1)
        if base_map_idxs.shape[-1] < num_base_polylines:
            base_map_idxs = F.pad(
                base_map_idxs,
                (0, num_base_polylines - base_map_idxs.shape[-1]),
                value=-1,
            )

    dynamic_dist = (
        pred_waypoints[:, :, None, :, 0:2] - safe_map_pos[:, None, :, None, 0:2]
    ).norm(dim=-1)
    dynamic_dist = dynamic_dist.min(dim=-1).values
    dynamic_topk_dist, dynamic_map_idxs = dynamic_dist.topk(
        k=min(num_polylines, num_waypoint_polylines),
        dim=-1,
        largest=False,
    )
    dynamic_map_idxs = torch.where(
        dynamic_topk_dist > 10_000_000.0,
        torch.full_like(dynamic_map_idxs, -1),
        dynamic_map_idxs,
    )
    if dynamic_map_idxs.shape[-1] < num_waypoint_polylines:
        dynamic_map_idxs = F.pad(
            dynamic_map_idxs,
            (0, num_waypoint_polylines - dynamic_map_idxs.shape[-1]),
            value=-1,
        )

    sorted_idxs = (
        torch.cat((base_map_idxs, dynamic_map_idxs), dim=-1).sort(dim=-1).values
    )
    duplicate_mask = torch.cat(
        (
            torch.ones_like(sorted_idxs[..., :1], dtype=torch.bool),
            sorted_idxs[..., 1:] != sorted_idxs[..., :-1],
        ),
        dim=-1,
    )
    sorted_idxs = torch.where(
        duplicate_mask, sorted_idxs, torch.full_like(sorted_idxs, -1)
    )
    return sorted_idxs.to(dtype=torch.int32), base_map_idxs


def _batch_nms_tensor(
    pred_trajs,
    pred_scores,
    dist_thresh,
    num_ret_modes=6,
):
    """Assignment-free equivalent of MTR's fixed-iteration batch NMS."""
    batch_size, num_modes, num_timestamps, num_features = pred_trajs.shape
    sorted_idxs = pred_scores.argsort(dim=-1, descending=True)
    sorted_scores = torch.gather(pred_scores, 1, sorted_idxs)
    traj_indices = sorted_idxs[:, :, None, None].expand(
        batch_size, num_modes, num_timestamps, num_features
    )
    sorted_trajs = torch.gather(pred_trajs, 1, traj_indices)
    goals = sorted_trajs[:, :, -1, 0:2]
    cover_mask = (goals[:, :, None, :] - goals[:, None, :, :]).norm(
        dim=-1
    ) < dist_thresh

    point_value = sorted_scores
    selected_penalty = torch.zeros_like(point_value)
    selected_indices = []
    selected_trajs = []
    selected_scores = []
    for _ in range(num_ret_modes):
        current = point_value.argmax(dim=-1)
        selected_indices.append(current)
        selected_trajs.append(
            torch.gather(
                sorted_trajs,
                1,
                current[:, None, None, None].expand(
                    batch_size, 1, num_timestamps, num_features
                ),
            ).squeeze(1)
        )
        selected_scores.append(
            torch.gather(sorted_scores, 1, current[:, None]).squeeze(1)
        )
        new_cover = torch.gather(
            cover_mask,
            1,
            current[:, None, None].expand(batch_size, 1, num_modes),
        ).squeeze(1)
        selected_penalty = torch.where(
            F.one_hot(current, num_modes).to(dtype=torch.bool),
            torch.full_like(selected_penalty, -1.0),
            selected_penalty,
        )
        point_value = (
            point_value * (~new_cover).to(dtype=point_value.dtype) + selected_penalty
        )

    local_indices = torch.stack(selected_indices, dim=1)
    original_indices = torch.gather(sorted_idxs, 1, local_indices)
    return (
        torch.stack(selected_trajs, dim=1),
        torch.stack(selected_scores, dim=1),
        original_indices,
    )


def _generate_final_prediction_tensor(decoder, pred_list, batch_dict):
    del batch_dict
    pred_scores, pred_trajs = pred_list[-1]
    pred_scores = torch.softmax(pred_scores, dim=-1)
    if decoder.num_motion_modes != pred_trajs.shape[1]:
        pred_trajs, pred_scores, _ = _batch_nms_tensor(
            pred_trajs,
            pred_scores,
            decoder.model_cfg.NMS_DIST_THRESH,
            decoder.num_motion_modes,
        )
    return pred_scores, pred_trajs


def _decoder_forward_tensor(decoder, batch_dict):
    """Exporter-safe equivalent of MTRDecoder.forward in eval mode."""
    input_dict = batch_dict["input_dict"]
    obj_feature = batch_dict["obj_feature"]
    obj_mask = batch_dict["obj_mask"]
    obj_pos = batch_dict["obj_pos"]
    map_feature = batch_dict["map_feature"]
    map_mask = batch_dict["map_mask"]
    map_pos = batch_dict["map_pos"]
    center_feature = decoder.in_proj_center_obj(batch_dict["center_objects_feature"])
    batch_size, num_objects, obj_channels = obj_feature.shape
    num_polylines = map_feature.shape[1]

    obj_feature = decoder.in_proj_obj(obj_feature.reshape(-1, obj_channels)).view(
        batch_size, num_objects, -1
    )
    obj_feature = obj_feature.masked_fill(~obj_mask[:, :, None], 0.0)
    map_feature = decoder.in_proj_map(
        map_feature.reshape(-1, map_feature.shape[-1])
    ).view(batch_size, num_polylines, -1)
    map_feature = map_feature.masked_fill(~map_mask[:, :, None], 0.0)

    obj_feature, _ = decoder.apply_dense_future_prediction(
        obj_feature, obj_mask, obj_pos
    )
    pred_list = decoder.apply_transformer_decoder(
        center_objects_feature=center_feature,
        center_objects_type=input_dict["center_objects_type"],
        obj_feature=obj_feature,
        obj_mask=obj_mask,
        obj_pos=obj_pos,
        map_feature=map_feature,
        map_mask=map_mask,
        map_pos=map_pos,
    )
    decoder.forward_ret_dict["pred_list"] = pred_list
    pred_scores, pred_trajs = decoder.generate_final_prediction(pred_list, batch_dict)
    batch_dict["pred_scores"] = pred_scores
    batch_dict["pred_trajs"] = pred_trajs
    return batch_dict


def _ensure_nonempty_modalities(
    obj_trajs,
    obj_trajs_mask,
    obj_trajs_last_pos,
    map_polylines,
    map_polylines_mask,
    map_polylines_center,
    track_index_to_predict,
):
    """Inject zero-valued sentinels into completely padded target rows.

    SceneBuilder fills unused slots in the static target-capacity dimension
    with masked zeros. Pinned MTR assumes every batch row has at least one
    valid token, so preserve real rows exactly and make only an all-empty
    modality's first token valid.
    """
    obj_empty = ~obj_trajs_mask.flatten(start_dim=1).any(dim=1)
    obj_slot_zero = (
        torch.arange(
            obj_trajs_mask.shape[1],
            device=obj_trajs_mask.device,
        )
        == 0
    )
    obj_time_zero = (
        torch.arange(
            obj_trajs_mask.shape[2],
            device=obj_trajs_mask.device,
        )
        == 0
    )
    obj_sentinel = (
        obj_empty[:, None, None]
        & obj_slot_zero[None, :, None]
        & obj_time_zero[None, None, :]
    )
    obj_trajs = obj_trajs.masked_fill(obj_sentinel[:, :, :, None], 0.0)
    obj_trajs_mask = obj_trajs_mask | obj_sentinel
    obj_center_sentinel = obj_empty[:, None] & obj_slot_zero[None, :]
    obj_trajs_last_pos = obj_trajs_last_pos.masked_fill(
        obj_center_sentinel[:, :, None], 0.0
    )
    track_index_to_predict = torch.where(
        obj_empty,
        torch.zeros_like(track_index_to_predict),
        track_index_to_predict,
    )

    map_empty = ~map_polylines_mask.flatten(start_dim=1).any(dim=1)
    map_slot_zero = (
        torch.arange(
            map_polylines_mask.shape[1],
            device=map_polylines_mask.device,
        )
        == 0
    )
    map_point_zero = (
        torch.arange(
            map_polylines_mask.shape[2],
            device=map_polylines_mask.device,
        )
        == 0
    )
    map_sentinel = (
        map_empty[:, None, None]
        & map_slot_zero[None, :, None]
        & map_point_zero[None, None, :]
    )
    map_polylines = map_polylines.masked_fill(map_sentinel[:, :, :, None], 0.0)
    map_polylines_mask = map_polylines_mask | map_sentinel
    map_center_sentinel = map_empty[:, None] & map_slot_zero[None, :]
    map_polylines_center = map_polylines_center.masked_fill(
        map_center_sentinel[:, :, None], 0.0
    )

    return (
        obj_trajs,
        obj_trajs_mask,
        obj_trajs_last_pos,
        map_polylines,
        map_polylines_mask,
        map_polylines_center,
        track_index_to_predict,
    )


class MtrOnnxWrapper(nn.Module):
    """Tensor-only public binding surface around MTR's dictionary API."""

    def __init__(self, net: nn.Module):
        super().__init__()
        self.net = net

    def forward(
        self,
        obj_trajs: torch.Tensor,
        obj_trajs_mask: torch.Tensor,
        obj_trajs_last_pos: torch.Tensor,
        map_polylines: torch.Tensor,
        map_polylines_mask: torch.Tensor,
        map_polylines_center: torch.Tensor,
        track_index_to_predict: torch.Tensor,
        center_type_ids: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        (
            obj_trajs,
            obj_trajs_mask,
            obj_trajs_last_pos,
            map_polylines,
            map_polylines_mask,
            map_polylines_center,
            track_index_to_predict,
        ) = _ensure_nonempty_modalities(
            obj_trajs,
            obj_trajs_mask,
            obj_trajs_last_pos,
            map_polylines,
            map_polylines_mask,
            map_polylines_center,
            track_index_to_predict,
        )
        result = self.net(
            {
                "input_dict": {
                    "obj_trajs": obj_trajs,
                    "obj_trajs_mask": obj_trajs_mask,
                    "obj_trajs_last_pos": obj_trajs_last_pos,
                    "map_polylines": map_polylines,
                    "map_polylines_mask": map_polylines_mask,
                    "map_polylines_center": map_polylines_center,
                    "track_index_to_predict": track_index_to_predict,
                    "center_objects_type": center_type_ids,
                }
            }
        )
        return result["pred_scores"], result["pred_trajs"]


def _patch_tensor_center_types(net: nn.Module) -> None:
    decoder = net.motion_decoder
    if tuple(decoder.object_type) != _CENTER_TYPES:
        raise ValueError(
            "checkpoint config object-type order does not match binding IDs: "
            f"{tuple(decoder.object_type)}"
        )
    from mtr.models.utils.transformer import position_encoding_utils

    decoder.position_encoding = position_encoding_utils.gen_sineembed_for_position
    decoder.get_motion_query = types.MethodType(
        _get_motion_query_from_type_ids, decoder
    )
    decoder.apply_dense_future_prediction = types.MethodType(
        _apply_dense_future_prediction_tensor, decoder
    )
    decoder.apply_dynamic_map_collection = types.MethodType(
        _apply_dynamic_map_collection_tensor, decoder
    )
    decoder.generate_final_prediction = types.MethodType(
        _generate_final_prediction_tensor, decoder
    )
    decoder.forward = types.MethodType(_decoder_forward_tensor, decoder)


def build_wrapper(
    checkpoint: pathlib.Path,
    cfg_path: pathlib.Path,
    *,
    device: torch.device | str = "cuda",
) -> MtrOnnxWrapper:
    """Load and patch the exact model used for ONNX export."""
    net = model.load_checkpoint(checkpoint, cfg_path, device=str(device))
    try:
        ops_dense.patch_mtr(net)
        _patch_tensor_center_types(net)
    except BaseException:
        # ``patch_mtr`` replaces process-wide module globals, and can itself
        # fail after replacing only a subset of them.
        ops_dense.restore_mtr_ops()
        raise
    return MtrOnnxWrapper(net).eval()


def export(
    checkpoint: pathlib.Path,
    cfg_path: pathlib.Path,
    out_path: pathlib.Path,
    contract_path: pathlib.Path,
) -> None:
    """Write and validate one static, standard-domain ONNX model."""
    if not torch.cuda.is_available():
        raise RuntimeError("MTR ONNX export requires CUDA")
    contract = load_contract(contract_path)
    try:
        wrapper = build_wrapper(checkpoint, cfg_path, device="cuda")
        inputs = dummy_inputs(contract, "cuda")
        out_path = pathlib.Path(out_path).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with torch.no_grad():
            torch.onnx.export(
                wrapper,
                inputs,
                str(out_path),
                export_params=True,
                opset_version=17,
                do_constant_folding=True,
                input_names=list(INPUT_NAMES),
                output_names=list(OUTPUT_NAMES),
                dynamic_axes=None,
                dynamo=False,
            )
    finally:
        ops_dense.restore_mtr_ops()

    import onnx

    exported = onnx.load(str(out_path))
    onnx.checker.check_model(exported)
    custom_domains = {
        node.domain
        for node in exported.graph.node
        if node.domain not in {"", "ai.onnx"}
    }
    if custom_domains:
        raise RuntimeError(
            f"export retained custom operator domains: {sorted(custom_domains)}"
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=pathlib.Path, required=True)
    parser.add_argument("--cfg", type=pathlib.Path, required=True)
    parser.add_argument("--contract", type=pathlib.Path, required=True)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    export(args.checkpoint, args.cfg, args.out, args.contract)
    print(f"exported static MTR ONNX model: {args.out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
