"""Run upstream bevfusion tools/train.py with all layers frozen except those named in TRAINABLE.

Usage (from $BF): TRAINABLE=heads.object torchpack dist-run -np 1 \
    python /workspace/src/perception/bevfusion/training/tools/train_frozen.py <config> [upstream args]
"""
import importlib.util
import os
import time

import torch
from mmdet3d.datasets.custom_3d import Custom3DDataset

spec = importlib.util.spec_from_file_location("bf_train", os.path.join(os.environ["BF"], "tools/train.py"))
bf_train = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bf_train)

# A parameter is trained if its name starts with one of these layer names, e.g. "heads.object."
TRAINABLE = tuple(p.rstrip(".") + "." for p in os.environ["TRAINABLE"].split(",") if p)
upstream_train_model = bf_train.train_model


def freeze(model):
    names = [name for name, _ in model.named_parameters()]
    # A misspelled layer name would freeze everything it was meant to train, so fail early instead.
    unmatched = [p for p in TRAINABLE if not any(n.startswith(p) for n in names)]
    if not TRAINABLE or unmatched:
        raise ValueError(f"TRAINABLE layers not found in the model: {unmatched or 'none given'}")
    for name, param in model.named_parameters():
        param.requires_grad = name.startswith(TRAINABLE)
    # Frozen BatchNorm layers must stay in eval mode, otherwise they normalize with batch statistics
    # and update their running stats. The runner calls model.train() every epoch, so override it per layer.
    for name, module in model.named_modules():
        if isinstance(module, torch.nn.modules.batchnorm._BatchNorm) and not name.startswith(TRAINABLE):
            module.eval()
            module.train = lambda mode=True, m=module: m
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total = sum(p.numel() for p in model.parameters())
    bf_train.get_root_logger().info(f"Trainable parameters: {trainable:,} / {total:,} ({TRAINABLE})")


def train_model(model, *args, **kwargs):
    freeze(model)
    return upstream_train_model(model, *args, **kwargs)


bf_train.train_model = train_model

# nuScenes is read from NFS (wato-drive2), which can briefly return "Stale file handle" (errno 116).
# One failed read in a dataloader worker otherwise kills a multi-hour run, so reload the sample.
LOAD_RETRIES = 5
upstream_getitem = Custom3DDataset.__getitem__


def getitem_with_retry(self, idx):
    for attempt in range(1, LOAD_RETRIES + 1):
        try:
            return upstream_getitem(self, idx)
        except OSError as e:
            if attempt == LOAD_RETRIES:
                raise
            print(f"train_frozen: OSError loading sample {idx} (attempt {attempt}/{LOAD_RETRIES}): {e}",
                  flush=True)
            time.sleep(10 * attempt)


Custom3DDataset.__getitem__ = getitem_with_retry

if __name__ == "__main__":
    bf_train.main()
