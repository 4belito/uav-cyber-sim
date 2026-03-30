#!/usr/bin/env python3
"""
ADS-B Injector for ArduPilot SITL (device adapter).

This module behaves like a real ADS-B receiver:
- subscribes to ADSBBeacon objects from Oracle (ZMQ)
- converts them to MAVLink ADSB_VEHICLE messages
- injects them into ArduPilot via a serial port

It does NOT generate traffic and does NOT know about Remote ID.
"""

from __future__ import annotations

import argparse
import logging
import time

import zmq

from simulator.config import BasePort
from simulator.entities.adsb import ADSBBeacon
from simulator.helpers.connections import create_zmq_socket
from simulator.helpers.connections.mavlink.conn import connect, send_heartbeat
from simulator.helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from simulator.helpers.connections.mavlink.enums import Autopilot, State, Type
from simulator.helpers.logging.setup_log import setup_logging

# =============================================================================
# MAVLink ADS-B constants
# =============================================================================

ADSB_ALTITUDE_TYPE_GEOMETRIC = 1
ADSB_EMITTER_TYPE_UAV = 14

ADSB_FLAGS_VALID_COORDS = 1
ADSB_FLAGS_VALID_ALTITUDE = 2
ADSB_FLAGS_VALID_HEADING = 4
ADSB_FLAGS_VALID_VELOCITY = 8
ADSB_FLAGS_VALID_CALLSIGN = 16
ADSB_FLAGS_SIMULATED = 64
ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128


# =============================================================================
# ADS-B Injector
# =============================================================================


class ADSBInjector:
    """ADS-B device adapter: ZMQ ADSBBeacon -> MAVLink ADSB_VEHICLE -> serial."""

    def __init__(self, sysid: int, baudrate: int, port_offset: int):
        self.uart = f"/tmp/adsb_{sysid}_injector"
        self.baudrate = baudrate
        self.port_offset = port_offset

        self.conn: MAVConnection | None = None
        self.mav = None

        # ZMQ (Oracle -> injector)
        self.ctx = zmq.Context()
        self.sub = create_zmq_socket(
            self.ctx,
            zmq.SUB,
            BasePort.ADSB_DOWN,
            port_offset,
        )

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Open serial connection to ArduPilot."""
        logging.debug(f"[ADSB] Connecting to {self.uart} @ {self.baudrate} baud")

        self.conn = connect(
            self.uart,
            baud=self.baudrate,
            src_sysid=1,
            src_compid=156,  # MAV_COMP_ID_ADSB
        )
        self.mav = self.conn.mav
        logging.debug("[ADSB] Connected")

    def close(self) -> None:
        """Clean up connections."""
        if self.conn:
            self.conn.close()
        self.sub.close(linger=0)
        self.ctx.term()

    # -------------------------------------------------------------------------
    # MAVLink output
    # -------------------------------------------------------------------------

    def send_heartbeat(self) -> None:
        """Identify as an ADS-B peripheral."""
        assert self.conn is not None
        send_heartbeat(
            self.conn,
            sys_type=Type.ADSB,
            autopilot=Autopilot.INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=State.ACTIVE,
        )

    def send_adsb_vehicle(self, beacon: ADSBBeacon) -> None:
        """Convert ADSBBeacon -> MAVLink ADSB_VEHICLE."""
        assert self.mav is not None

        self.mav.adsb_vehicle_send(
            ICAO_address=beacon.icao,
            lat=int(beacon.lat_deg * 1e7),
            lon=int(beacon.lon_deg * 1e7),
            altitude_type=ADSB_ALTITUDE_TYPE_GEOMETRIC,
            altitude=int(beacon.alt_m * 1000),
            heading=int(beacon.heading_deg * 100) % 36000,
            hor_velocity=int(beacon.hor_speed_mps * 100),
            ver_velocity=int(beacon.ver_speed_mps * 100),
            callsign=beacon.callsign[:8].ljust(8).encode("ascii"),
            emitter_type=ADSB_EMITTER_TYPE_UAV,
            tslc=0,
            flags=(
                ADSB_FLAGS_VALID_COORDS
                | ADSB_FLAGS_VALID_ALTITUDE
                | ADSB_FLAGS_VALID_HEADING
                | ADSB_FLAGS_VALID_VELOCITY
                | ADSB_FLAGS_VALID_CALLSIGN
                | ADSB_FLAGS_SIMULATED
                | ADSB_FLAGS_VERTICAL_VELOCITY_VALID
            ),
            squawk=1200,
        )


# =============================================================================
# Main
# =============================================================================


def main() -> None:
    """Run the ADS-B injector."""
    parser = argparse.ArgumentParser(description="ADS-B injector (Oracle-fed)")
    parser.add_argument(
        "--sysid",
        type=int,
        required=True,
        help="System ID for the ADS-B injector",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=57600,
        help="Serial baudrate (default: 57600)",
    )
    parser.add_argument(
        "--port-offset",
        type=int,
        required=True,
        help="Port offset used for ZMQ ADSB_DOWN socket",
    )

    parser.add_argument(
        "--verbose",
        type=int,
        required=False,
        help="verbosity level (e.g. 0,1,2,3)",
    )

    args = parser.parse_args()

    setup_logging(
        f"adsb_injector_{args.sysid}", verbose=args.verbose, console_output=True
    )

    injector = ADSBInjector(
        sysid=args.sysid,
        baudrate=args.baud,
        port_offset=args.port_offset,
    )

    try:
        injector.connect()

        last_heartbeat = 0.0

        while True:
            now = time.time()

            if now - last_heartbeat > 1.0:
                injector.send_heartbeat()
                last_heartbeat = now

            try:
                beacon: ADSBBeacon = injector.sub.recv_pyobj()
                injector.send_adsb_vehicle(beacon)
            except zmq.Again:
                pass

    except KeyboardInterrupt:
        logging.debug("\n[ADSB] Stopping")

    finally:
        injector.close()


if __name__ == "__main__":
    main()
