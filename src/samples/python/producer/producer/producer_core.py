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

import math


class ProducerCore():

    def __init__(self, pos_x, pos_y, pos_z, vel):
        # Init member variables for serialization
        self.__pos_x = pos_x
        self.__pos_y = pos_y
        self.__pos_z = pos_z
        self.__velocity = vel

    def update_position(self):
        # velocity in 3D delta_x = delta_y = delta_z
        self.__pos_x += self.__velocity / math.sqrt(3)
        self.__pos_y += self.__velocity / math.sqrt(3)
        self.__pos_z += self.__velocity / math.sqrt(3)

    def serialize_data(self):
        return "x:" + str(self.__pos_x) + ";y:" + \
            str(self.__pos_y) + ";z:" + str(self.__pos_z) + ";"
