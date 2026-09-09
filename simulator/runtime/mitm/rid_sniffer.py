"""Passive Remote ID eavesdropper for the man-in-the-middle.

The MITM shares the victim vehicle's ``port_offset``, so it can attach its own
SUB socket to the *same* ``RID_DOWN`` endpoint the victim's logic already
listens on (the Oracle binds that port as a PUB, so extra subscribers are free).
This models the attacker overhearing the Remote ID broadcasts the compromised
drone receives — no cooperation from the Oracle is required.

The sniffer caches the latest :class:`~simulator.entities.riddata.RIDData` per
``sysid`` so a pursuit strategy can look up a moving target's position on demand,
mirroring :meth:`~simulator.runtime.vehicle.rid_manager.RIDManager.get_latest`.
"""

from __future__ import annotations

import logging
import threading
import time

import zmq

from simulator.config import BasePort
from simulator.entities.riddata import RIDData
from simulator.helpers.connections import create_zmq_socket


class RIDSniffer:
    """Eavesdrop the victim's ``RID_DOWN`` feed and cache the latest RID per sysid."""

    def __init__(self, port_offset: int) -> None:
        self._ctx = zmq.Context()
        self._sock = create_zmq_socket(
            self._ctx, zmq.SUB, BasePort.RID_DOWN, port_offset
        )
        self._latest: dict[int, RIDData] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        """Begin receiving RID broadcasts in the background."""
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                rid: RIDData = self._sock.recv_pyobj()  # type: ignore[assignment]
            except zmq.Again:
                time.sleep(0.001)
                continue
            except Exception as exc:  # pragma: no cover - defensive
                logging.error("MITM RID sniffer error: %s", exc)
                continue
            with self._lock:
                self._latest[rid.sysid] = rid

    def latest(self, sysid: int) -> RIDData | None:
        """Return the most recently overheard RID for ``sysid`` (or ``None``)."""
        with self._lock:
            return self._latest.get(sysid)

    def stop(self) -> None:
        """Stop the receive loop and release the socket and context."""
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._sock.close(linger=0)
        self._ctx.term()
