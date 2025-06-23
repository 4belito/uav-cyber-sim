"""Typed wrappers and interfaces for MAVLink waypoint loading and manipulation."""

from typing import Any, Protocol, runtime_checkable

from pymavlink.dialects.v20.ardupilotmega import MAVLink_mission_item_message as ItemMsg
from pymavlink.mavwp import MAVWPLoader


@runtime_checkable
class LoaderInterface(Protocol):
    """Protocol for MAVLink waypoint loader interface."""

    wpoints: list[ItemMsg]
    target_system: int
    target_component: int

    def count(self) -> int: ...  # noqa: D102
    def wp(self, i: int) -> Any: ...  # noqa: D102
    def item(self, i: int) -> Any: ...  # noqa: D102
    def add(self, w: Any, comment: str = "") -> None: ...  # noqa: D102
    def insert(self, idx: int, w: Any, comment: str = "") -> None: ...  # noqa: D102
    def reindex(self) -> None: ...  # noqa: D102
    def set(self, w: Any, idx: int) -> None: ...  # noqa: D102
    def remove(self, w: Any) -> None: ...  # noqa: D102
    def clear(self) -> None: ...  # noqa: D102
    def load(self, filename: str) -> int: ...  # noqa: D102
    def save(self, filename: str) -> None: ...  # noqa: D102


class MissionLoader:
    """Typed wrapper around MAVWPLoader with full method coverage and docstrings."""

    _loader: LoaderInterface

    def __init__(self, target_system: int = 0, target_component: int = 0):
        self._loader = MAVWPLoader(target_system, target_component)

    def count(self) -> int:
        """Return number of waypoints."""
        return self._loader.count()

    def wp(self, i: int) -> ItemMsg:
        """Alias for backwards compatibility."""
        return self._loader.wp(i)

    def item(self, i: int) -> ItemMsg:
        """Return an item."""
        return self._loader.item(i)

    def add(self, w: ItemMsg, comment: str = "") -> None:
        """Add a waypoint."""
        self._loader.add(w, comment)

    def insert(self, idx: int, w: ItemMsg, comment: str = "") -> None:
        """Insert a waypoint."""
        self._loader.insert(idx, w, comment)

    def reindex(self) -> None:
        """Reindex waypoints."""
        self._loader.reindex()

    def set(self, w: ItemMsg, idx: int) -> None:
        """Set a waypoint."""
        self._loader.set(w, idx)

    def remove(self, w: ItemMsg) -> None:
        """Remove a waypoint."""
        self._loader.remove(w)

    def clear(self) -> None:
        """Clear waypoint list."""
        self._loader.clear()

    def load(self, filename: str) -> int:
        """Load waypoints from a file. Returns number of waypoints loaded."""
        return self._loader.load(filename)

    def save(self, filename: str) -> None:
        """Save waypoints to a file."""
        self._loader.save(filename)

    def items(self) -> list[ItemMsg]:
        """Return internal list of waypoints."""
        return self._loader.wpoints
