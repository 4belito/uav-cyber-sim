"""Simulator GCS configuration entity."""

from dataclasses import dataclass, field


@dataclass
class SimGCS:
    """Simulator GCS configuration."""

    name: str
    sysids: list[int] = field(default_factory=lambda: list())
    port_offset: int | None = None

    @property
    def port_offset_required(self) -> int:
        """Return the port offset for the GCS, or raise an error if not set."""
        if self.port_offset is None:
            raise RuntimeError(f"GCS {self.name} port_offset has not been assigned")
        return self.port_offset
