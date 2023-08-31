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

import time

import rclpy
from aggregator.aggregator import Aggregator


def test_calc_freq():
    rclpy.init()

    aggregator_node = Aggregator()

    thresh = 100
    start_time = aggregator_node.node_start_time
    test_count = 5

    test_freq = test_count / (time.time() - start_time)

    assert abs(aggregator_node.calc_freq(5) - test_freq) < thresh
