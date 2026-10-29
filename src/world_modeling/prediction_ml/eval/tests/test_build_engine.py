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
"""Tests for static TensorRT engine construction without requiring TensorRT."""

from __future__ import annotations

import importlib
import json
import pathlib
import sys
import types

import pytest
import yaml

from wato_mtr_eval import build_engine

PREDICTION_ML = pathlib.Path(__file__).resolve().parents[2]
CONFIG = PREDICTION_ML / "config" / "engine_build.yaml"
FP16_EXPERIMENTAL_CONFIG = (
    PREDICTION_ML / "config" / "engine_build_fp16_experimental.yaml"
)


def _write_config(path: pathlib.Path, **changes: object) -> None:
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw.update(changes)
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")


def test_import_does_not_require_tensorrt():
    sys.modules.pop("tensorrt", None)
    assert importlib.reload(build_engine)
    assert "tensorrt" not in sys.modules


def test_checked_in_config_is_static_and_valid():
    cfg = build_engine.load_config(CONFIG)
    assert cfg.precision == "fp32"
    assert cfg.allow_tf32 is False
    assert cfg.workspace_size_mb == 4096
    assert cfg.optimization_profiles == ()
    assert cfg.tactic_sources == (
        "EDGE_MASK_CONVOLUTIONS",
        "JIT_CONVOLUTIONS",
    )
    assert cfg.builder_optimization_level == 3


def test_checked_in_experimental_fp16_config_is_static_and_valid():
    cfg = build_engine.load_config(FP16_EXPERIMENTAL_CONFIG)
    assert cfg.precision == "fp16"
    assert cfg.allow_tf32 is False
    assert cfg.workspace_size_mb == 4096
    assert cfg.optimization_profiles == ()
    assert cfg.tactic_sources == (
        "EDGE_MASK_CONVOLUTIONS",
        "JIT_CONVOLUTIONS",
    )
    assert cfg.builder_optimization_level == 3


@pytest.mark.parametrize("allow_tf32", [0, 1, "false", None])
def test_allow_tf32_must_be_a_boolean(tmp_path, allow_tf32):
    path = tmp_path / "engine_build.yaml"
    _write_config(path, allow_tf32=allow_tf32)
    with pytest.raises(TypeError, match="allow_tf32 must be a boolean"):
        build_engine.load_config(path)


def test_allow_tf32_is_required(tmp_path):
    path = tmp_path / "engine_build.yaml"
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    del raw["allow_tf32"]
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="missing.*allow_tf32"):
        build_engine.load_config(path)


def test_dynamic_optimization_profiles_are_rejected(tmp_path):
    path = tmp_path / "engine_build.yaml"
    _write_config(
        path,
        optimization_profiles=[
            {"obj_trajs": {"min": [1, 128, 11, 29], "max": [8, 128, 11, 29]}}
        ],
    )
    with pytest.raises(ValueError, match="must be empty"):
        build_engine.load_config(path)


def _write_onnx(path: pathlib.Path, *, dynamic: bool = False) -> None:
    onnx = pytest.importorskip("onnx")
    helper = onnx.helper
    first_dim: int | str = "batch" if dynamic else 8
    x = helper.make_tensor_value_info(
        "obj_trajs", onnx.TensorProto.FLOAT, [first_dim, 128, 11, 29]
    )
    y = helper.make_tensor_value_info(
        "pred_scores", onnx.TensorProto.FLOAT, [first_dim, 128, 11, 29]
    )
    graph = helper.make_graph(
        [helper.make_node("Identity", ["obj_trajs"], ["pred_scores"])],
        "static_test",
        [x],
        [y],
    )
    onnx.save(helper.make_model(graph), str(path))


def test_dynamic_onnx_binding_is_rejected(tmp_path):
    path = tmp_path / "dynamic.onnx"
    _write_onnx(path, dynamic=True)
    with pytest.raises(ValueError, match="is dynamic"):
        build_engine._validate_onnx(path)


class _FakeBuilderConfig:
    def __init__(self):
        self.workspace = None
        self.builder_optimization_level = None
        self.flags = set()
        self.cleared_flags = []
        self.tactic_mask = None

    def set_memory_pool_limit(self, pool, size):
        self.workspace = (pool, size)

    def set_flag(self, flag):
        self.flags.add(flag)

    def clear_flag(self, flag):
        self.flags.discard(flag)
        self.cleared_flags.append(flag)

    def set_tactic_sources(self, mask):
        self.tactic_mask = mask
        return True


class _FakeParser:
    def __init__(self, network, logger):
        del network, logger
        self.num_errors = 0

    def parse_from_file(self, path):
        return pathlib.Path(path).is_file()


class _FakeBuilder:
    platform_has_fast_fp16 = True
    last_config = None

    def __init__(self, logger):
        del logger

    def create_network(self, flags):
        assert flags == 2
        return object()

    def create_builder_config(self):
        self.last_config = _FakeBuilderConfig()
        return self.last_config

    def build_serialized_network(self, network, config):
        del network
        assert config is self.last_config
        return b"serialized-engine"


def _fake_trt():
    return types.SimpleNamespace(
        __version__="8.6.1",
        Logger=type("Logger", (), {"WARNING": 2, "__init__": lambda self, level: None}),
        Builder=_FakeBuilder,
        OnnxParser=_FakeParser,
        NetworkDefinitionCreationFlag=types.SimpleNamespace(EXPLICIT_BATCH=1),
        MemoryPoolType=types.SimpleNamespace(WORKSPACE=0),
        BuilderFlag=types.SimpleNamespace(FP16=1, TF32=2),
        TacticSource=types.SimpleNamespace(
            EDGE_MASK_CONVOLUTIONS=0,
            JIT_CONVOLUTIONS=1,
        ),
    )


def test_build_writes_atomically_and_returns_engine_info(tmp_path, monkeypatch):
    onnx_path = tmp_path / "mtr.onnx"
    _write_onnx(onnx_path)
    out_path = tmp_path / "mtr.engine"
    out_path.write_bytes(b"old-engine")
    monkeypatch.setattr(build_engine, "_import_tensorrt", _fake_trt)
    monkeypatch.setattr(build_engine, "_detect_sm_arch", lambda: "sm_86")
    monkeypatch.setattr(build_engine.time, "perf_counter", iter([10.0, 12.5]).__next__)

    info = build_engine.build(onnx_path, CONFIG, out_path)

    assert out_path.read_bytes() == b"serialized-engine"
    assert info == build_engine.EngineInfo("sm_86", "8.6.1", 2.5)
    leftovers = list(tmp_path.glob(".mtr.engine.*"))
    assert leftovers == []


@pytest.mark.parametrize(
    ("allow_tf32", "expect_set"),
    [(False, False), (True, True)],
)
def test_configure_builder_explicitly_controls_tf32(tmp_path, allow_tf32, expect_set):
    path = tmp_path / "engine_build.yaml"
    _write_config(path, precision="fp32", allow_tf32=allow_tf32)
    cfg = build_engine.load_config(path)
    trt = _fake_trt()
    builder_config = _FakeBuilderConfig()

    build_engine._configure_builder(
        trt,
        _FakeBuilder(None),
        builder_config,
        cfg,
    )

    assert (trt.BuilderFlag.TF32 in builder_config.flags) is expect_set
    assert (trt.BuilderFlag.TF32 in builder_config.cleared_flags) is (not expect_set)


def test_trt11_fp16_requires_a_preconverted_onnx():
    trt = _fake_trt()
    trt.__version__ = "11.1.0.106"
    trt.BuilderFlag = types.SimpleNamespace(TF32=2)
    cfg = build_engine.load_config(FP16_EXPERIMENTAL_CONFIG)

    with pytest.raises(RuntimeError, match="ModelOpt AutoCast"):
        build_engine._configure_builder(
            trt,
            _FakeBuilder(None),
            _FakeBuilderConfig(),
            cfg,
        )


def test_failed_build_preserves_existing_output(tmp_path, monkeypatch):
    onnx_path = tmp_path / "mtr.onnx"
    _write_onnx(onnx_path)
    out_path = tmp_path / "mtr.engine"
    out_path.write_bytes(b"known-good")
    trt = _fake_trt()
    monkeypatch.setattr(
        trt.Builder,
        "build_serialized_network",
        lambda self, network, config: None,
    )
    monkeypatch.setattr(build_engine, "_import_tensorrt", lambda: trt)
    monkeypatch.setattr(build_engine, "_detect_sm_arch", lambda: "sm_86")

    with pytest.raises(RuntimeError, match="failed without"):
        build_engine.build(onnx_path, CONFIG, out_path)
    assert out_path.read_bytes() == b"known-good"


def test_main_prints_machine_readable_engine_info(tmp_path, monkeypatch, capsys):
    out_path = tmp_path / "mtr.engine"
    expected = build_engine.EngineInfo("sm_89", "10.13.2", 42.25)
    monkeypatch.setattr(build_engine, "build", lambda *args: expected)

    assert (
        build_engine.main(
            [
                "--onnx",
                str(tmp_path / "mtr.onnx"),
                "--config",
                str(CONFIG),
                "--out",
                str(out_path),
            ]
        )
        == 0
    )
    assert json.loads(capsys.readouterr().out) == {
        "build_seconds": 42.25,
        "sm_arch": "sm_89",
        "trt_version": "10.13.2",
    }
