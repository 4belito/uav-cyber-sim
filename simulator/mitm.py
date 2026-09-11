"""
Man-in-the-middle proxy interposed on a vehicle's GCS<->Logic MAVLink links.

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
from typing import TypeAlias, cast

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from simulator.config import GCS_TELEM_WINDOW, LOGS_PATH, VehPort
from simulator.helpers.connections import MAVConnection, create_udp_conn
from simulator.helpers.coordinates import GRA
from simulator.helpers.logging.setup_log import setup_logging
from simulator.helpers.math import connection_id
from simulator.runtime.mitm.strategies import MITMContext, MITMSpec, MITMStrategy

MAVMsg: TypeAlias = mavlink.MAVLink_message
Hook: TypeAlias = Callable[[MAVMsg], MAVMsg | None]


def _describe(msg: MAVMsg) -> str:
    """Key field values for a message, so the debug log shows data, not just a type."""
    if msg.get_type() == "GLOBAL_POSITION_INT":
        pos = cast("mavlink.MAVLink_global_position_int_message", msg)
        lat, lon, alt = pos.lat / 1e7, pos.lon / 1e7, pos.alt / 1000
        return f" (lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m)"
    return ""


class _Relay(threading.Thread):
    """
    Forward MAVLink messages from ``src`` to every ``dsts`` in one direction.

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
                hook = self._hook
                if hook is not None:
                    hooked = hook(msg)
                    if hooked is None:
                        logging.info("MITM %s: dropped %s", self._name, msg.get_type())
                        continue
                    msg = hooked
                buf = msg.get_msgbuf()
                if not buf:
                    buf = msg.pack(self._encoder)
                logging.debug(
                    "MITM %s: forwarding %s%s",
                    self._name,
                    msg.get_type(),
                    _describe(msg),
                )
                for dst in self._dsts:
                    dst.write(bytes(buf))
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
        gra_origin: GRA = GRA(0.0, 0.0, 0.0),
    ) -> None:
        self.sysid = sysid
        self.strategy = strategy
        veh_sysid = connection_id(sysid)
        telem_ports = list(gcs_telem_ports) or [GCS_TELEM_WINDOW + port_offset]

        # Telemetry link (Logic <-> GCS, primary downlink)
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

        # Command link (GCS <-> Logic, primary uplink)
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

        # `to_gcs` is index-aligned with `veh.gcss` (0 = owner).
        strategy.bind(
            MITMContext(
                sysid=sysid,
                to_logic=self.logic_cmd,
                to_gcs=self.gcs_telems,
                gra_origin=gra_origin,
            )
        )

        self.relays: list[_Relay] = [
            # Primary directions: strategy hooks.
            _Relay(
                "downlink",
                self.logic_telem,
                self.gcs_telems,
                src_sysid=veh_sysid,
                hook=strategy.on_downlink,
            ),
            _Relay(
                "uplink",
                self.gcs_cmd,
                [self.logic_cmd],
                src_sysid=255,
                hook=strategy.on_uplink,
            ),
            # Backflow: transparent acks/replies, one per GCS telemetry link.
            *(
                _Relay(
                    f"telem-ack-{i}",
                    gcs_telem,
                    [self.logic_telem],
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
        self.strategy.close()
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


def parse_arguments() -> tuple[int, int, MITMSpec, list[int], int, GRA]:
    """Parse MITM proxy CLI arguments."""
    parser = argparse.ArgumentParser(description="Man-in-the-middle MAVLink proxy")
    parser.add_argument("--sysid", type=int, required=True)
    parser.add_argument("--port-offset", type=int, required=True)
    parser.add_argument(
        "--spec",
        type=str,
        default='{"strategy_class": "passthrough", "kwargs": {}}',
        help="JSON-encoded MITMSpec (strategy_class + kwargs)",
    )
    parser.add_argument(
        "--telem-ports",
        type=str,
        default="",
        help="Comma-separated UDP ports of the GCSs monitoring this vehicle",
    )
    parser.add_argument(
        "--gra-origin",
        type=str,
        default='{"lat": 0.0, "lon": 0.0, "alt": 0.0}',
        help="JSON-encoded run origin {lat, lon, alt}, for InterventionStrategy",
    )
    parser.add_argument("--verbose", type=int, default=1)
    args = parser.parse_args()
    spec_data = json.loads(args.spec)
    spec = MITMSpec(
        strategy_class=spec_data["strategy_class"], kwargs=spec_data["kwargs"]
    )
    telem_ports = [int(p) for p in args.telem_ports.split(",") if p]
    origin_data = json.loads(args.gra_origin)
    gra_origin = GRA(origin_data["lat"], origin_data["lon"], origin_data["alt"])
    return (args.sysid, args.port_offset, spec, telem_ports, args.verbose, gra_origin)


def main() -> None:
    """Entry point for a single-vehicle MITM proxy process."""
    sysid, port_offset, spec, telem_ports, verbose, gra_origin = parse_arguments()
    setup_logging(
        LOGS_PATH / "mitm" / f"mitm_{sysid}.log",
        verbose=verbose or 1,
        console_output=True,
    )
    proxy = MITMProxy(
        sysid,
        port_offset,
        MITMStrategy.build(spec),
        telem_ports,
        gra_origin=gra_origin,
    )
    proxy.run_forever()


if __name__ == "__main__":
    main()
