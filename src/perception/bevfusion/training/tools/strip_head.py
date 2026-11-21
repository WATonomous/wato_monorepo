"""Usage: python strip_head.py IN.pth OUT.pth [PREFIX]   (default PREFIX: heads.object.)"""
import sys

import torch

src, dst = sys.argv[1], sys.argv[2]
prefix = sys.argv[3] if len(sys.argv) > 3 else "heads.object."
ckpt = torch.load(src, map_location="cpu")
state = ckpt.get("state_dict", ckpt)
kept = {k: v for k, v in state.items() if not k.startswith(prefix)}
if len(kept) == len(state):
    sys.exit(f"no tensors with prefix {prefix} in {src}")
print(f"removed {len(state) - len(kept)} tensors with prefix {prefix}")
torch.save({"state_dict": kept}, dst)
