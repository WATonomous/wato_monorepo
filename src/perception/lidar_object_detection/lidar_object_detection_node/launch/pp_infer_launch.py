# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='pp_infer',
            executable='pp_infer',
            # parameters=[{
            #     'nms_iou_thresh': 0.01,
            #     'pre_nms_top_n': 4096,
            #     'class_names': ['Vehicle', 'Pedestrian', 'Cyclist'],
            #     'model_path': '', 
            #     'engine_path': '/model/pointpillars_model/trt.engine',
            #     'data_type': 'fp32',
            #     'intensity_scale': 100.0,
            # }],
            # remappings=[('/point_cloud', '/LIDAR_TOP')],
        )
    ])
