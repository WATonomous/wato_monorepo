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
import carla
import os

## SETUP ##
client_name = os.environ.get("CLIENT_NAME", "DOES NOT EXIST")
if client_name == "DOES NOT EXIST":
    raise Exception(
        "The environment variable for the container name of the carla server has not been set"
    )

# Connect to the client and retrieve the world object
client = carla.Client(client_name, 2000)
world = client.get_world()
spectator = world.get_spectator()
