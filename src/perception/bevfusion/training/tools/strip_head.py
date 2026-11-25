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
"""Remove detection-head weights so training starts the head from scratch (head-reinit run).

Usage: python strip_head.py IN.pth OUT.pth [PREFIX]   (default PREFIX: heads.object.)"""

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
