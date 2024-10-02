# Copyright 2023 WATonomous
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

import rclpy 
from rclpy.node import Node

import carla
import casadi as ca
import numpy as np
import datetime
import os
import shutil

from action.model_predictive_control.model_predictive_control.boxconstraint import BoxConstraint
from sample_msgs.msg import Unfilterered, Filitered, FilteredArray 
from transformer.transformer_core import TransformerCore

SIM_DURATION = 500  # Simulation duration in time steps

# somehow send a message to sim container to init carla

class carla_com(Node):
    def __init__(self):
        super().__init__('python_transformer')

        
