"""Man-in-the-middle proxy interposed on a vehicle's GCS<->Logic MAVLink links.

```text
telemetry (downlink primary):  Logic --MITM_TELEM--> [MITM] --GCS--> GCS
commands  (uplink primary):    GCS   --MITM_CMD-->   [MITM] --GCS_CMD--> Logic
```

Each link is proxied **bidirectionally** so request/ack handshakes (e.g. the
``LOGIC_DONE`` -> ``COMMAND_ACK`` exchange that rides the telemetry channel)
survive the interposition. By default every message is forwarded byte-for-byte;
a :class:`~simulator.runtime.mitm.strategies.MITMStrategy` may inspect, modify,
or drop messages on the two primary directions.

The MITM listens on per-vehicle ``MITM_TELEM`` / ``MITM_CMD`` ports while the GCS
and Logic keep their original receivers (the GCS telemetry window / ``GCS_CMD``).
Only the two
*senders* (Logic telemetry, GCS commands) are retargeted at the MITM, so the
endpoints are otherwise unaware of the interposition.

When several GCSs monitor the same vehicle, each has its own telemetry port and
the downlink is fanned out to all of them; the uplink needs no fan-out since
they all send commands to the one ``MITM_CMD`` listener.
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
from collections.abc import Callable, Sequence

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.config import GCS_TELEM_WINDOW, LOGS_PATH, VehPort
from simulator.helpers.connections import MAVConnection, create_udp_conn
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.math import connection_id
from simulator.runtime.mitm.strategies import MITMContext, MITMStrategy, get_strategy

MAVMsg = mavlink.MAVLink_message
Hook = Callable[[MAVMsg], "MAVMsg | None"]


class _Relay(threading.Thread):
    """Forward MAVLink messages from ``src`` to every ``dsts`` in one direction.

    An optional ``hook`` may transform or drop each message. Messages are
    forwarded as raw bytes when possible (transparent passthrough); a hook that
    synthesises a new message has it re-packed with this relay's own encoder so
    the destination connection's parser is never touched from two threads.

    More than one destination is only used on the downlink, where the same
    telemetry stream feeds every GCS monitoring the vehicle.
    """

    def __init__(
        self,
        name: str,
        src: MAVConnection,
        dsts: Sequence[MAVConnection],
        src_sysid: int,
        hook: Hook | None = None,
    ) -> None:
        super().__init__(daemon=True)
        self._name = name
        self._src = src
        self._dsts = list(dsts)
        self._hook = hook
        self._encoder = mavlink.MAVLink(None, srcSystem=src_sysid, srcComponent=200)
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.is_set():
            try:
                msg = self._src.recv_match(blocking=True, timeout=0.2)
                if msg is None or msg.get_type() == "BAD_DATA":
                    continue
                if self._hook is not None:
                    hooked = self._hook(msg)
                    if hooked is None:
                        logging.info("MITM %s: dropped %s", self._name, msg.get_type())
                        continue
                    msg = hooked
                buf = msg.get_msgbuf()
                if not buf:
                    buf = msg.pack(self._encoder)
                for dst in self._dsts:
                    dst.write(buf)
            except Exception as exc:
                logging.error("MITM relay %s error: %s", self._name, exc)

    def stop(self) -> None:
        """Signal the relay loop to exit."""
        self._stop_event.set()


class MITMProxy:
    """Bidirectional MAVLink proxy for one vehicle's GCS<->Logic links."""

    def __init__(
        self,
        sysid: int,
        port_offset: int,
        strategy: MITMStrategy,
        gcs_telem_ports: Sequence[int] = (),
    ) -> None:
        self.sysid = sysid
        self.strategy = strategy
        veh_sysid = connection_id(sysid)
        # Default to the first slot of this vehicle's GCS telemetry window.
        telem_ports = list(gcs_telem_ports) or [GCS_TELEM_WINDOW + port_offset]

        # --- telemetry link: Logic <-> GCS (primary downlink) ---
        # Logic-facing: receives telemetry from Logic, sends acks back to Logic.
        self.logic_telem = create_udp_conn(
            base_port=VehPort.MITM_TELEM,
            offset=port_offset,
            mode="receiver",
            src_sysid=veh_sysid,
            src_compid=200,
            wait_hb=False,
        )
        # GCS-facing: sends telemetry to each GCS, receives acks back from them.
        self.gcs_telems = [
            create_udp_conn(
                base_port=port,
                offset=0,
                mode="sender",
                src_sysid=veh_sysid,
                src_compid=200,
            )
            for port in telem_ports
        ]

        # --- command link: GCS <-> Logic (primary uplink) ---
        # GCS-facing: receives commands from GCS, sends replies back to GCS.
        self.gcs_cmd = create_udp_conn(
            base_port=VehPort.MITM_CMD,
            offset=port_offset,
            mode="receiver",
            src_sysid=255,
            src_compid=200,
            wait_hb=False,
        )
        # Logic-facing: sends commands to Logic, receives replies from Logic.
        self.logic_cmd = create_udp_conn(
            base_port=VehPort.GCS_CMD,
            offset=port_offset,
            mode="sender",
            src_sysid=255,
            src_compid=200,
        )

        # Let the strategy inject its own traffic (command injection, spoofing).
        # Injected traffic goes to the first GCS, the one owning the vehicle.
        strategy.bind(
            MITMContext(
                sysid=sysid, to_logic=self.logic_cmd, to_gcs=self.gcs_telems[0]
            )
        )

        self.relays: list[_Relay] = [
            # Primary directions carry the strategy hooks.
            _Relay(
                "downlink", self.logic_telem, self.gcs_telems,
                src_sysid=veh_sysid, hook=strategy.on_downlink,
            ),
            _Relay(
                "uplink", self.gcs_cmd, [self.logic_cmd],
                src_sysid=255, hook=strategy.on_uplink,
            ),
            # Backflow directions are transparent (acks, replies). Each GCS acks
            # on its own telemetry link, so each needs its own backflow relay.
            *(
                _Relay(
                    f"telem-ack-{i}", gcs_telem, [self.logic_telem],
                    src_sysid=veh_sysid,
                )
                for i, gcs_telem in enumerate(self.gcs_telems)
            ),
            _Relay("cmd-reply", self.logic_cmd, [self.gcs_cmd], src_sysid=255),
        ]

    def run_forever(self) -> None:
        """Start all relays and block until interrupted/terminated."""
        for relay in self.relays:
            relay.start()
        logging.info(
            "MITM proxy active for vehicle %s (strategy=%s)",
            self.sysid,
            type(self.strategy).__name__,
        )
        try:
            threading.Event().wait()  # block forever; killed via SIGTERM
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self) -> None:
        """Stop all relays and close connections."""
        for relay in self.relays:
            relay.stop()
        # Join before closing so no relay touches a socket after it is closed.
        for relay in self.relays:
            relay.join(timeout=1.0)
        conns = [self.logic_telem, self.gcs_cmd, self.logic_cmd, *self.gcs_telems]
        for conn in conns:
            try:
                conn.close()
            except Exception:
                pass
        logging.info("MITM proxy for vehicle %s stopped", self.sysid)


def parse_arguments() -> tuple[int, int, str, dict[str, float], list[int], int]:
    """Parse MITM proxy CLI arguments."""
    parser = argparse.ArgumentParser(description="Man-in-the-middle MAVLink proxy")
    parser.add_argument("--sysid", type=int, required=True)
    parser.add_argument("--port-offset", type=int, required=True)
    parser.add_argument("--strategy", type=str, default="passthrough")
    parser.add_argument(
        "--params",
        type=str,
        default="{}",
        help="JSON-encoded strategy parameters",
    )
    parser.add_argument(
        "--telem-ports",
        type=str,
        default="",
        help="Comma-separated UDP ports of the GCSs monitoring this vehicle",
    )
    parser.add_argument("--verbose", type=int, default=1)
    args = parser.parse_args()
    params: dict[str, float] = json.loads(args.params)
    telem_ports = [int(p) for p in args.telem_ports.split(",") if p]
    return (
        args.sysid,
        args.port_offset,
        args.strategy,
        params,
        telem_ports,
        args.verbose,
    )


def main() -> None:
    """Entry point for a single-vehicle MITM proxy process."""
    sysid, port_offset, strategy_name, params, telem_ports, verbose = parse_arguments()
    setup_logging(
        LOGS_PATH / "mitm" / f"mitm_{sysid}.log",
        verbose=verbose or 1,
        console_output=True,
    )
    proxy = MITMProxy(
        sysid, port_offset, get_strategy(strategy_name, params), telem_ports
    )
    proxy.run_forever()


if __name__ == "__main__":
    main()
