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
"""Minimal static transform tree, for resolving extrinsics from a bag's /tf_static offline."""

from typing import Dict, List, Tuple

import numpy as np

from visual_odometry.geometry import invert


class StaticTfTree:
    """Stores parent->child transforms and answers ``lookup(target, source)`` like tf2.

    ``lookup(target, source)`` returns the 4x4 transform mapping points in ``source`` into ``target``
    (i.e. T_target_source), the same convention as tf2's lookupTransform.
    """

    def __init__(self):
        # child -> (parent, T_parent_child)
        self._parent: Dict[str, Tuple[str, np.ndarray]] = {}

    def add(self, parent: str, child: str, t_parent_child: np.ndarray) -> None:
        self._parent[child.lstrip("/")] = (
            parent.lstrip("/"),
            np.asarray(t_parent_child, dtype=float),
        )

    def __contains__(self, frame: str) -> bool:
        frame = frame.lstrip("/")
        return frame in self._parent or any(
            p == frame for p, _ in self._parent.values()
        )

    def _chain_to_root(self, frame: str) -> List[Tuple[str, np.ndarray]]:
        """Return [(frame, T_root_frame)...] walking up from ``frame`` to its root."""
        chain = [(frame, np.eye(4))]
        seen = {frame}
        t_frame_to_current_root = np.eye(4)
        current = frame
        while current in self._parent:
            parent, t_parent_current = self._parent[current]
            if parent in seen:
                raise ValueError(f"TF cycle at {parent}")
            seen.add(parent)
            t_frame_to_current_root = t_parent_current @ t_frame_to_current_root
            chain.append((parent, t_frame_to_current_root.copy()))
            current = parent
        return chain

    def lookup(self, target: str, source: str) -> np.ndarray:
        target, source = target.lstrip("/"), source.lstrip("/")
        # (ancestor, T_ancestor_source) walking up from source; {ancestor: T_ancestor_target}
        source_chain = self._chain_to_root(source)
        target_chain = dict(self._chain_to_root(target))
        for ancestor, t_ancestor_source in source_chain:
            if ancestor in target_chain:
                return invert(target_chain[ancestor]) @ t_ancestor_source
        raise LookupError(f"no static transform between {target} and {source}")
