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

import functools
import os
import pathlib

import numpy as np
import pytest

onnx = pytest.importorskip("onnx")
torch = pytest.importorskip("torch")
export_onnx = pytest.importorskip("wato_mtr_eval.export_onnx")
model = pytest.importorskip("wato_mtr_eval.model")
ops_dense = pytest.importorskip("wato_mtr_eval.ops_dense")
rep = pytest.importorskip("wato_mtr_eval.representation")
rung2 = pytest.importorskip("wato_mtr_eval.rung2")
tensor_io = pytest.importorskip("wato_mtr_eval.tensor_io")

ONNX_PATH = pathlib.Path(os.environ.get("MTR_ONNX", "build/mtr.onnx"))
CONTRACT_PATH = (
    pathlib.Path(__file__).resolve().parents[2] / "config" / "contract_8_target.yaml"
)
FIXTURE = (
    pathlib.Path(__file__).resolve().parents[2]
    / "test"
    / "fixtures"
    / "923a0c6bb90f4127"
)
HAS_CHECKPOINT_CUDA = bool(
    os.environ.get("MTR_CHECKPOINT")
    and pathlib.Path(os.environ["MTR_CHECKPOINT"]).is_file()
    and os.environ.get("MTR_ROOT")
    and pathlib.Path(os.environ["MTR_ROOT"]).is_dir()
    and torch.cuda.is_available()
)
needs_onnx = pytest.mark.skipif(not ONNX_PATH.exists(), reason="run ONNX export first")
needs_checkpoint_cuda = pytest.mark.skipif(
    not HAS_CHECKPOINT_CUDA,
    reason="set MTR_CHECKPOINT and MTR_ROOT and provide CUDA",
)


@pytest.fixture(autouse=True)
def _restore_custom_ops():
    ops_dense.restore_mtr_ops()
    yield
    ops_dense.restore_mtr_ops()


@functools.cache
def _model_proto():
    return onnx.load(str(ONNX_PATH))


def _graph():
    return _model_proto().graph


def _shapes(values):
    return {
        value.name: [
            dimension.dim_value for dimension in value.type.tensor_type.shape.dim
        ]
        for value in values
    }


def test_dummy_inputs_match_every_frozen_contract_binding():
    contract = export_onnx.load_contract(CONTRACT_PATH)
    inputs = export_onnx.dummy_inputs(contract, "cpu")

    assert tuple(binding.name for binding in contract.inputs) == (
        export_onnx.INPUT_NAMES
    )
    for tensor, binding in zip(inputs, contract.inputs, strict=True):
        assert tensor.shape == binding.shape
        assert tensor.dtype == binding.dtype


def test_padding_sentinels_preserve_real_rows_and_fill_empty_modalities():
    contract = export_onnx.load_contract(CONTRACT_PATH)
    values = list(export_onnx.dummy_inputs(contract, "cpu"))
    by_name = dict(zip(export_onnx.INPUT_NAMES, values, strict=True))
    by_name["obj_trajs_mask"][1].zero_()
    by_name["map_polylines_mask"][2].zero_()
    by_name["track_index_to_predict"][1] = 37
    original_obj_mask = by_name["obj_trajs_mask"][0].clone()
    original_map_mask = by_name["map_polylines_mask"][0].clone()

    sanitized = export_onnx._ensure_nonempty_modalities(
        *(by_name[name] for name in export_onnx.INPUT_NAMES[:7])
    )

    assert torch.equal(sanitized[1][0], original_obj_mask)
    assert torch.equal(sanitized[4][0], original_map_mask)
    assert sanitized[1][1].sum().item() == 1
    assert sanitized[1][1, 0, 0]
    assert sanitized[4][2].sum().item() == 1
    assert sanitized[4][2, 0, 0]
    assert sanitized[6][1].item() == 0
    assert sanitized[0][1, 0, 0].count_nonzero().item() == 0
    assert sanitized[2][1, 0].count_nonzero().item() == 0
    assert sanitized[3][2, 0, 0].count_nonzero().item() == 0
    assert sanitized[5][2, 0].count_nonzero().item() == 0


def test_build_wrapper_restores_process_wide_ops_when_instance_patch_fails(
    monkeypatch,
):
    restored = False
    sentinel = object()

    monkeypatch.setattr(
        export_onnx.model,
        "load_checkpoint",
        lambda checkpoint, cfg_path, device: sentinel,
    )
    monkeypatch.setattr(export_onnx.ops_dense, "patch_mtr", lambda net: net)

    def fail_instance_patch(net):
        assert net is sentinel
        raise RuntimeError("bad pinned API")

    def record_restore():
        nonlocal restored
        restored = True

    monkeypatch.setattr(export_onnx, "_patch_tensor_center_types", fail_instance_patch)
    monkeypatch.setattr(export_onnx.ops_dense, "restore_mtr_ops", record_restore)

    with pytest.raises(RuntimeError, match="bad pinned API"):
        export_onnx.build_wrapper(
            pathlib.Path("checkpoint.pth"),
            pathlib.Path("model.yaml"),
        )
    assert restored


@needs_onnx
def test_input_names_and_shapes_match_the_frozen_contract():
    got = _shapes(_graph().input)
    assert got == {
        "obj_trajs": [
            rep.TARGET_CAPACITY,
            rep.CONTEXT_CAPACITY,
            rep.HISTORY_STEPS,
            rep.OBJ_FEATURE_DIM,
        ],
        "obj_trajs_mask": [
            rep.TARGET_CAPACITY,
            rep.CONTEXT_CAPACITY,
            rep.HISTORY_STEPS,
        ],
        "obj_trajs_last_pos": [
            rep.TARGET_CAPACITY,
            rep.CONTEXT_CAPACITY,
            3,
        ],
        "map_polylines": [
            rep.TARGET_CAPACITY,
            rep.NUM_SRC_POLYLINES,
            rep.POINTS_PER_POLYLINE,
            rep.MAP_FEATURE_DIM,
        ],
        "map_polylines_mask": [
            rep.TARGET_CAPACITY,
            rep.NUM_SRC_POLYLINES,
            rep.POINTS_PER_POLYLINE,
        ],
        "map_polylines_center": [
            rep.TARGET_CAPACITY,
            rep.NUM_SRC_POLYLINES,
            3,
        ],
        "track_index_to_predict": [rep.TARGET_CAPACITY],
        "center_type_ids": [rep.TARGET_CAPACITY],
    }


@needs_onnx
def test_output_names_and_shapes_match_the_frozen_contract():
    got = _shapes(_graph().output)
    assert got == {
        "pred_scores": [rep.TARGET_CAPACITY, rep.NUM_MODES],
        "pred_trajs": [
            rep.TARGET_CAPACITY,
            rep.NUM_MODES,
            rep.NUM_FUTURE_FRAMES,
            rep.TRAJ_CHANNELS,
        ],
    }


@needs_onnx
def test_index_inputs_are_int64_matching_engine_binding_contract():
    dtypes = {value.name: value.type.tensor_type.elem_type for value in _graph().input}
    assert dtypes["track_index_to_predict"] == onnx.TensorProto.INT64
    assert dtypes["center_type_ids"] == onnx.TensorProto.INT64


@needs_onnx
def test_all_binding_dtypes_match_the_frozen_contract():
    contract = export_onnx.load_contract(CONTRACT_PATH)
    torch_to_onnx = {
        torch.float32: onnx.TensorProto.FLOAT,
        torch.bool: onnx.TensorProto.BOOL,
        torch.int64: onnx.TensorProto.INT64,
    }
    graph = _graph()
    actual = {
        value.name: value.type.tensor_type.elem_type
        for value in list(graph.input) + list(graph.output)
    }
    expected = {
        binding.name: torch_to_onnx[binding.dtype]
        for binding in contract.inputs + contract.outputs
    }
    assert actual == expected


@needs_onnx
def test_all_public_shapes_are_static():
    graph = _graph()
    for tensor in list(graph.input) + list(graph.output):
        for dimension in tensor.type.tensor_type.shape.dim:
            assert dimension.dim_value > 0, f"{tensor.name} has a dynamic dimension"


@needs_onnx
def test_no_custom_operators_survive():
    model_proto = _model_proto()
    domains = {node.domain for node in model_proto.graph.node}
    assert domains <= {"", "ai.onnx"}


@needs_onnx
def test_no_onnx_sequence_or_loop_operators_survive():
    """TensorRT has no sequence-tensor binding for repeat_interleave lowering."""
    unsupported = {
        "ConcatFromSequence",
        "Loop",
        "SequenceAt",
        "SequenceConstruct",
        "SequenceEmpty",
        "SequenceErase",
        "SequenceInsert",
        "SequenceLength",
        "SplitToSequence",
    }
    model_proto = _model_proto()
    found = {node.op_type for node in model_proto.graph.node} & unsupported
    assert not found, f"TensorRT-incompatible ONNX operators present: {sorted(found)}"


@needs_onnx
def test_model_passes_onnx_checker():
    onnx.checker.check_model(_model_proto())


@needs_checkpoint_cuda
def test_tensor_only_wrapper_matches_official_mtr_on_real_fixture():
    checkpoint = pathlib.Path(os.environ["MTR_CHECKPOINT"])
    root = pathlib.Path(os.environ["MTR_ROOT"])
    cfg = root / "tools/cfgs/waymo/mtr+100_percent_data.yaml"
    tensors = tensor_io.read_bundle(FIXTURE / "expected_tensors.bin.gz")
    with np.load(FIXTURE / "ground_truth.npz", allow_pickle=False) as data:
        targets = rung2._read_targets(FIXTURE, data["agent_ids"])
    batch = rung2._model_batch(tensors, targets, FIXTURE.name)

    net = model.load_checkpoint(checkpoint, cfg, device="cuda", mtr_root=root)
    try:
        expected_scores, expected_trajs = model.forward_batch(net, batch)
    finally:
        del net
        torch.cuda.empty_cache()

    contract = export_onnx.load_contract(CONTRACT_PATH)
    inputs = []
    for binding in contract.inputs:
        value = tensors[binding.name]
        if binding.dtype == torch.bool:
            value = value.astype(bool)
        inputs.append(torch.from_numpy(value).to("cuda"))
    wrapper = export_onnx.build_wrapper(checkpoint, cfg, device="cuda")
    try:
        with torch.no_grad():
            actual_scores, actual_trajs = wrapper(*inputs)
    finally:
        ops_dense.restore_mtr_ops()

    count = len(targets)
    np.testing.assert_allclose(
        actual_scores[:count].cpu().numpy(),
        expected_scores,
        rtol=2e-5,
        atol=2e-6,
    )
    np.testing.assert_allclose(
        actual_trajs[:count].cpu().numpy(),
        expected_trajs,
        rtol=2e-5,
        atol=2e-5,
    )


@needs_onnx
@pytest.mark.skipif(
    not os.environ.get("MTR_CHECKPOINT") or not os.environ.get("MTR_ROOT"),
    reason="set MTR_CHECKPOINT and MTR_ROOT for numerical parity",
)
def test_onnxruntime_matches_patched_pytorch():
    ort = pytest.importorskip("onnxruntime")
    contract = export_onnx.load_contract(CONTRACT_PATH)
    inputs_cpu = list(export_onnx.dummy_inputs(contract, "cpu"))
    generator = torch.Generator().manual_seed(20260726)
    for index, binding in enumerate(contract.inputs):
        if binding.dtype == torch.float32:
            inputs_cpu[index].copy_(
                torch.randn(binding.shape, generator=generator) * 0.1
            )
    by_name = dict(zip(export_onnx.INPUT_NAMES, inputs_cpu, strict=True))
    by_name["obj_trajs_mask"].zero_()
    by_name["map_polylines_mask"].zero_()
    for target in range(rep.TARGET_CAPACITY):
        by_name["obj_trajs_mask"][target, : 1 + target % 3, :] = True
        by_name["map_polylines_mask"][target, : 1 + target % 2, :] = True
    by_name["track_index_to_predict"].zero_()
    by_name["center_type_ids"].copy_(torch.arange(rep.TARGET_CAPACITY - 1, -1, -1) % 3)
    inputs_cpu = tuple(by_name[name] for name in export_onnx.INPUT_NAMES)
    inputs = tuple(tensor.cuda() for tensor in inputs_cpu)
    root = pathlib.Path(os.environ["MTR_ROOT"])
    wrapper = export_onnx.build_wrapper(
        pathlib.Path(os.environ["MTR_CHECKPOINT"]),
        root / "tools/cfgs/waymo/mtr+100_percent_data.yaml",
        device="cuda",
    )
    try:
        with torch.no_grad():
            expected = [value.cpu().numpy() for value in wrapper(*inputs)]
    finally:
        ops_dense.restore_mtr_ops()
    session = ort.InferenceSession(str(ONNX_PATH), providers=["CPUExecutionProvider"])
    feed = {
        name: tensor.numpy()
        for name, tensor in zip(export_onnx.INPUT_NAMES, inputs_cpu, strict=True)
    }
    actual = session.run(list(export_onnx.OUTPUT_NAMES), feed)

    np.testing.assert_allclose(actual[0], expected[0], rtol=2e-4, atol=2e-5)
    np.testing.assert_allclose(actual[1], expected[1], rtol=2e-4, atol=2e-4)
