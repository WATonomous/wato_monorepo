#!/usr/bin/env python3
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
"""Score an INT8 PTQ model (from qat/ptq.py or scripts/export.sh) on nuScenes val.

Same metric as scripts/eval.sh (mAP/NDS), but loads the pickled PTQ model with its
quantizers on, so the score shows the INT8 accuracy cost in PyTorch.
Usage (in the training container, after sourcing scripts/env.sh):
  cd $CB && python $TRAINING_DIR/tools/eval_ptq.py --ckpt <ptq.pth>"""

import argparse
import os
import sys

import torch
from mmcv import Config
from mmcv.parallel import MMDataParallel
from mmdet3d.apis import single_gpu_test
from mmdet3d.datasets import build_dataloader, build_dataset
from mmdet3d.utils import recursive_eval
from torchpack.utils.config import configs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=os.environ["R50_FUSION_CFG"])
    parser.add_argument("--ckpt", required=True)
    args = parser.parse_args()

    sys.path.insert(0, os.getcwd())
    sys.path.insert(0, os.path.join(os.getcwd(), "qat"))
    import lean.quantize as quantize

    quantize.initialize()
    configs.load(args.config, recursive=True)
    cfg = Config(recursive_eval(configs), filename=args.config)
    cfg.data.test.test_mode = True
    dataset = build_dataset(cfg.data.test)
    loader = build_dataloader(
        dataset,
        samples_per_gpu=1,
        workers_per_gpu=cfg.data.workers_per_gpu,
        dist=False,
        shuffle=False,
    )
    model = torch.load(args.ckpt)
    if not isinstance(model, MMDataParallel):
        model = MMDataParallel(model, device_ids=[0])
    model.eval()
    outputs = single_gpu_test(model, loader)
    print(dataset.evaluate(outputs, metric="bbox"))


if __name__ == "__main__":
    main()
