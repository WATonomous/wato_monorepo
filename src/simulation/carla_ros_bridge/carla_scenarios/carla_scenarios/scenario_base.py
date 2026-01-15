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
        self.client: Optional['carla.Client'] = None
        self.world: Optional['carla.World'] = None

    @abstractmethod
    def get_name(self) -> str:
        """Return scenario name."""
        pass

    @abstractmethod
    def get_description(self) -> str:
        """Return scenario description."""
        pass

    @abstractmethod
    def initialize(self, client: 'carla.Client') -> bool:
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
