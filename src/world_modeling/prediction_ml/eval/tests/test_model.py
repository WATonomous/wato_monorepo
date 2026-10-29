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
import os
import pathlib

import pytest

torch = pytest.importorskip("torch")
model = pytest.importorskip("wato_mtr_eval.model")

CKPT = os.environ.get("MTR_CHECKPOINT")
MTR_ROOT = os.environ.get("MTR_ROOT")
needs_ckpt = pytest.mark.skipif(
    not (
        CKPT
        and MTR_ROOT
        and pathlib.Path(CKPT).is_file()
        and pathlib.Path(MTR_ROOT).is_dir()
    ),
    reason="set MTR_CHECKPOINT and MTR_ROOT to existing paths to run",
)


@needs_ckpt
def test_checkpoint_contains_model_weights_not_only_optimizer_state():
    raw = torch.load(CKPT, map_location="cpu", weights_only=False)
    assert "model_state" in raw, f"expected 'model_state', found {sorted(raw)}"
    assert len(raw["model_state"]) > 100, "suspiciously few parameter tensors"


@needs_ckpt
def test_checkpoint_reports_epoch_30():
    raw = torch.load(CKPT, map_location="cpu", weights_only=False)
    assert raw.get("epoch") == 30


@needs_ckpt
def test_checkpoint_loads_with_no_missing_or_unexpected_keys():
    """A silent key mismatch is how a wrong-config load produces garbage output."""
    net, report = model.load_checkpoint(
        pathlib.Path(CKPT),
        pathlib.Path(MTR_ROOT) / "tools/cfgs/waymo/mtr+100_percent_data.yaml",
        device="cpu",
        return_report=True,
    )
    assert net is not None
    assert report.missing_keys == [], f"missing: {report.missing_keys[:5]}"
    assert report.unexpected_keys == [], f"unexpected: {report.unexpected_keys[:5]}"


@needs_ckpt
def test_num_motion_modes_and_future_frames_match_the_contract():
    from wato_mtr_eval import representation as rep

    net = model.load_checkpoint(
        pathlib.Path(CKPT),
        pathlib.Path(MTR_ROOT) / "tools/cfgs/waymo/mtr+100_percent_data.yaml",
        device="cpu",
    )
    assert net.motion_decoder.num_motion_modes == rep.NUM_MODES
    assert net.motion_decoder.num_future_frames == rep.NUM_FUTURE_FRAMES
