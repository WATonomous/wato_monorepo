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

class TransformerCore():

    def deserialize_data(self, msg_data):
        unfiltered_array = msg_data
        unfiltered_array.split(";")

        pos_x = float(unfiltered_array[unfiltered_array.find("x") + 2])
        pos_y = float(unfiltered_array[unfiltered_array.find("y") + 2])
        pos_z = float(unfiltered_array[unfiltered_array.find("z") + 2])

        return pos_x, pos_y, pos_z
