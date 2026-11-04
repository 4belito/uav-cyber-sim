"""
Cooperative shutdown for long-running simulator processes.

Every simulator entity that runs as its own OS process (GCS, Logic, ...) is
torn down by the orchestrator at the end of a run — and on a timeout that
teardown arrives unannounced, while the entity is still in its work loop. A
process that is hard-killed there never gets to flush what it has buffered
(a GCS loses the trajectories it recorded, for example).

`SignalShutdown` is the standard lifecycle hook for that case. An entity mixes
it in, installs the handlers once at startup, checks `stopping` in its work
loops, and overrides `close` to flush. A `SIGTERM`/`SIGINT` from the
orchestrator then flips `stopping`, the loops wind down, and `close` runs on
the way out — the same `close` that runs on a normal, self-completed exit, so
there is a single cleanup path regardless of how the process ends.

The orchestrator must still deliver `SIGTERM` *before* any `SIGKILL`: a hard
kill cannot be caught, so `close` would never run. See
`simulator.helpers.cleanup.graceful_terminate`.
"""

from __future__ import annotations

import logging
import signal
import threading
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from types import FrameType


class SignalShutdown:
    """
    Mixin giving a runtime process a cooperative, signal-driven shutdown.

    Subclasses call `install_signal_handlers` once from the process's main
    thread, guard their work loops with `if self.stopping: break`, and override
    `close` with their flush/cleanup. `close` is run at most once, whether it is
    reached through a normal exit or a caught signal.
    """

    def __init__(self) -> None:
        self._shutdown = threading.Event()
        self._closed = False

    @property
    def stopping(self) -> bool:
        """True once a shutdown signal has been received (or `request_stop`)."""
        return self._shutdown.is_set()

    def request_stop(self) -> None:
        """Ask the work loops to wind down, as a caught signal would."""
        self._shutdown.set()

    def install_signal_handlers(self) -> None:
        """
        Route shutdown signals to a cooperative stop. Main thread only.

        `SIGHUP` is included so the process keeps running long enough to flush
        when its launching terminal (xterm/bash wrapper) is torn down alongside
        it, rather than dying on the default hangup action.
        """
        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            signal.signal(sig, self._on_signal)

    def _on_signal(self, signum: int, _frame: FrameType | None) -> None:
        """Flip `stopping` so the work loops exit and `close` can flush."""
        logging.info(f"Received signal {signum}; shutting down gracefully")
        self._shutdown.set()

    def close(self) -> None:
        """Flush and release resources. Override; the default does nothing."""

    def run_close_once(self) -> None:
        """Run `close` exactly once, swallowing nothing it does not raise."""
        if self._closed:
            return
        self._closed = True
        self.close()
