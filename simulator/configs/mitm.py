"""TypedDict schema for man-in-the-middle configuration."""

from typing import NotRequired, TypedDict


class MITMConfig(TypedDict):
    """Man-in-the-middle proxy configuration.

    ``strategy`` selects a registered
    :class:`~simulator.runtime.mitm.strategies.MITMStrategy` by name.
    ``params`` carries strategy-specific settings (e.g. the hijack trigger and
    target coordinates) and is forwarded to the strategy at construction.
    """

    strategy: str
    params: NotRequired[dict[str, float]]
