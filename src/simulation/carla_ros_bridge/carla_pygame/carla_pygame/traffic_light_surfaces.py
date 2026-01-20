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
"""Traffic light surface rendering for CARLA pygame visualization."""

try:
    import pygame
except ImportError:
    pygame = None

try:
    from carla import TrafficLightState as tls
except ImportError:
    tls = None

from carla_pygame.constants import (
    COLOR_ALUMINIUM_4,
    COLOR_ALUMINIUM_5,
    COLOR_BUTTER_0,
    COLOR_CHAMELEON_0,
    COLOR_ORANGE_2,
    COLOR_SCARLET_RED_0,
)


class TrafficLightSurfaces:
    """Manages pre-rendered surfaces for traffic light states."""

    def __init__(self):
        def make_surface(tl_state):
            w = 40
            surface = pygame.Surface((w, 3 * w), pygame.SRCALPHA)
            surface.fill(COLOR_ALUMINIUM_5 if tl_state != "h" else COLOR_ORANGE_2)
            if tl_state != "h":
                hw = int(w / 2)
                off = COLOR_ALUMINIUM_4
                red = COLOR_SCARLET_RED_0
                yellow = COLOR_BUTTER_0
                green = COLOR_CHAMELEON_0
                pygame.draw.circle(
                    surface, red if tl_state == tls.Red else off, (hw, hw), int(0.4 * w)
                )
                pygame.draw.circle(
                    surface,
                    yellow if tl_state == tls.Yellow else off,
                    (hw, w + hw),
                    int(0.4 * w),
                )
                pygame.draw.circle(
                    surface,
                    green if tl_state == tls.Green else off,
                    (hw, 2 * w + hw),
                    int(0.4 * w),
                )
            return pygame.transform.smoothscale(
                surface, (15, 45) if tl_state != "h" else (19, 49)
            )

        self._original_surfaces = {
            "h": make_surface("h"),
            tls.Red: make_surface(tls.Red),
            tls.Yellow: make_surface(tls.Yellow),
            tls.Green: make_surface(tls.Green),
            tls.Off: make_surface(tls.Off),
            tls.Unknown: make_surface(tls.Unknown),
        }
        self.surfaces = dict(self._original_surfaces)

    def rotozoom(self, angle, scale):
        """Apply rotation and zoom to all traffic light surfaces."""
        for key, surface in self._original_surfaces.items():
            self.surfaces[key] = pygame.transform.rotozoom(surface, angle, scale)
