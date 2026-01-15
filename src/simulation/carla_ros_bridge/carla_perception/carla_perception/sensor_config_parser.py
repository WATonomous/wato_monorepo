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
"""Parse robot_description URDF for sensor placement."""

from typing import Dict, Optional
from geometry_msgs.msg import Transform, Vector3, Quaternion
import xml.etree.ElementTree as ET


class SensorConfigParser:
    """Parse URDF to extract sensor link transforms."""

    def __init__(self, robot_description: str):
        """
        Initialize parser with URDF string.

        Args:
            robot_description: URDF XML string
        """
        try:
            self.urdf = ET.fromstring(robot_description)
        except ET.ParseError as e:
            raise ValueError(f"Invalid URDF XML: {e}")

    def find_sensor_links(self) -> Dict[str, str]:
        """
        Find all sensor links in URDF.

        Returns:
            Dict mapping link_name to parent_link
        """
        sensor_links = {}

        # Find all joints and identify sensor links
        # Typically sensor links are connected to base_link via fixed joints
        for joint in self.urdf.findall(".//joint"):
            joint_type = joint.get("type", "")
            child_link = joint.find("child")

            if child_link is not None:
                child_name = child_link.get("link", "")
                # Common sensor link naming patterns
                if any(
                    sensor in child_name.lower()
                    for sensor in ["camera", "lidar", "laser", "depth", "rgb"]
                ):
                    parent_link = joint.find("parent")
                    parent_name = (
                        parent_link.get("link", "") if parent_link is not None else ""
                    )
                    sensor_links[child_name] = parent_name

        return sensor_links

    def get_sensor_transform(self, sensor_link: str) -> Optional[Transform]:
        """
        Get transform from base_link to sensor_link.

        Args:
            sensor_link: Name of the sensor link

        Returns:
            Transform message or None if not found
        """
        # Find the joint connecting to this sensor link
        for joint in self.urdf.findall(".//joint"):
            child = joint.find("child")
            if child is not None and child.get("link") == sensor_link:
                # Found the joint, now extract the origin/transform
                origin = joint.find("origin")
                if origin is not None:
                    return self._parse_origin(origin)
                else:
                    # No explicit origin, return identity transform
                    return self._identity_transform()

        return None

    def _parse_origin(self, origin_element: ET.Element) -> Transform:
        """Parse origin element to Transform message."""
        transform = Transform()

        # Parse xyz (translation)
        xyz_str = origin_element.get("xyz", "0 0 0")
        xyz = [float(x) for x in xyz_str.split()]
        transform.translation = Vector3(x=xyz[0], y=xyz[1], z=xyz[2])

        # Parse rpy (rotation) and convert to quaternion
        rpy_str = origin_element.get("rpy", "0 0 0")
        rpy = [float(x) for x in rpy_str.split()]
        transform.rotation = self._rpy_to_quaternion(rpy[0], rpy[1], rpy[2])

        return transform

    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert roll-pitch-yaw to quaternion."""
        import math

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def _identity_transform(self) -> Transform:
        """Return identity transform."""
        transform = Transform()
        transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
        transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return transform
