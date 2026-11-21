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
"""Target-less LiDAR-camera extrinsic calibration helpers for Eve.

Wraps koide3/direct_visual_lidar_calibration (Koide et al., ICRA 2023) with
bag scanning, rest/creep segment detection, bag cutting, repeated calibration
trials and result summaries.
"""
