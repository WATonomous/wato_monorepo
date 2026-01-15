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
"""Base class for CARLA scenario plugins."""

from abc import ABC, abstractmethod
from typing import Optional

try:
    import carla
except ImportError:
    carla = None  # Will be checked at runtime


class ScenarioBase(ABC):
    """Abstract base class for scenario plugins."""

    def __init__(self):
        self.client: Optional["carla.Client"] = None
        self.world: Optional["carla.World"] = None

    @abstractmethod
    def get_name(self) -> str:
        """Return scenario name."""
        pass

    @abstractmethod
    def get_description(self) -> str:
        """Return scenario description."""
        pass

    @abstractmethod
    def initialize(self, client: "carla.Client") -> bool:
        """
        Initialize scenario with CARLA client.

        Args:
            client: CARLA client instance

        Returns:
            True if initialization successful, False otherwise
        """
        pass

    @abstractmethod
    def setup(self) -> bool:
        """
        Set up CARLA world for this scenario.

        Returns:
            True if setup successful, False otherwise
        """
        pass

    @abstractmethod
    def execute(self) -> None:
        """Execute scenario logic (called periodically)."""
        pass

    @abstractmethod
    def cleanup(self) -> None:
        """Clean up scenario resources."""
        pass
