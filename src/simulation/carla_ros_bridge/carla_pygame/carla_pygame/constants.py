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
"""Color constants and map scaling constants for CARLA pygame visualization."""

try:
    import pygame
except ImportError:
    pygame = None

# Tango Desktop Project color palette
COLOR_BUTTER_0 = pygame.Color(252, 233, 79) if pygame else None
COLOR_BUTTER_1 = pygame.Color(237, 212, 0) if pygame else None
COLOR_ORANGE_0 = pygame.Color(252, 175, 62) if pygame else None
COLOR_ORANGE_1 = pygame.Color(245, 121, 0) if pygame else None
COLOR_ORANGE_2 = pygame.Color(209, 92, 0) if pygame else None
COLOR_CHOCOLATE_0 = pygame.Color(233, 185, 110) if pygame else None
COLOR_CHOCOLATE_1 = pygame.Color(193, 125, 17) if pygame else None
COLOR_CHAMELEON_0 = pygame.Color(138, 226, 52) if pygame else None
COLOR_SKY_BLUE_0 = pygame.Color(114, 159, 207) if pygame else None
COLOR_PLUM_0 = pygame.Color(173, 127, 168) if pygame else None
COLOR_PLUM_2 = pygame.Color(92, 53, 102) if pygame else None
COLOR_MAGENTA = pygame.Color(255, 0, 255) if pygame else None
COLOR_SCARLET_RED_0 = pygame.Color(239, 41, 41) if pygame else None
COLOR_SCARLET_RED_1 = pygame.Color(204, 0, 0) if pygame else None
COLOR_ALUMINIUM_0 = pygame.Color(238, 238, 236) if pygame else None
COLOR_ALUMINIUM_2 = pygame.Color(186, 189, 182) if pygame else None
COLOR_ALUMINIUM_3 = pygame.Color(136, 138, 133) if pygame else None
COLOR_ALUMINIUM_4 = pygame.Color(85, 87, 83) if pygame else None
COLOR_ALUMINIUM_4_5 = pygame.Color(66, 62, 64) if pygame else None
COLOR_ALUMINIUM_5 = pygame.Color(46, 52, 54) if pygame else None
COLOR_WHITE = pygame.Color(255, 255, 255) if pygame else None
COLOR_BLACK = pygame.Color(0, 0, 0) if pygame else None

# Map scaling constants
PIXELS_PER_METER = 12
MAP_DEFAULT_SCALE = 0.1
