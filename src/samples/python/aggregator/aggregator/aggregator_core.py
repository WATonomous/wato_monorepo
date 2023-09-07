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


class AggregatorCore():

    def __init__(self, start_time):
        # Init member variables to calculate msg frequency
        self.num_unfiltered_msgs = 0
        self.num_filtered_msgs = 0

        self.raw_freq = 0
        self.filtered_freq = 0

        self.__start_time = start_time

    def update_raw_freq(self):
        self.num_unfiltered_msgs += 1
        self.raw_freq = self.calc_freq(self.num_unfiltered_msgs)

    def update_filtered_freq(self):
        self.num_filtered_msgs += 1
        self.filtered_freq = self.calc_freq(self.num_filtered_msgs)

    def calc_freq(self, count):
        return count / (time.time() - self.__start_time)
