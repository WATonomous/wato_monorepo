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
"""CARLA client utilities."""

from typing import Optional

try:
    import carla
except ImportError:
    carla = None


def connect_carla(
    host: str = "localhost",
    port: int = 2000,
    timeout: float = 10.0,
) -> "carla.Client":
    """
    Connect to CARLA server.

    Args:
        host: CARLA server hostname
        port: CARLA server port
        timeout: Connection timeout in seconds

    Returns:
        Connected carla.Client

    Raises:
        RuntimeError: If CARLA Python API is not available
        Exception: If connection fails
    """
    if carla is None:
        raise RuntimeError("CARLA Python API not available")

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def find_ego_vehicle(
    world: "carla.World",
    role_name: str = "ego_vehicle",
) -> Optional["carla.Vehicle"]:
    """
    Find ego vehicle by role_name attribute.

    Args:
        world: CARLA world instance
        role_name: Role name to search for

    Returns:
        Vehicle actor if found, None otherwise
    """
    vehicles = world.get_actors().filter("vehicle.*")
    ego_vehicles = [v for v in vehicles if v.attributes.get("role_name") == role_name]
    return ego_vehicles[0] if ego_vehicles else None
