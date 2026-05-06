"""Helpers for MAVLink message streams."""

from collections.abc import Mapping, Sequence
from typing import TypeAlias, cast

import pymavlink.dialects.v20.ardupilotmega as mavlink
import pymavlink.dialects.v20.development as development_mavlink

from simulator.helpers.connections.mavlink.customtypes.mavconn import MAVConnection
from simulator.helpers.connections.mavlink.enums import CmdSet, DataStream


class DummyWriter:
    def write(self, data: bytes) -> int:
        return len(data)


secondary_decoder = development_mavlink.MAVLink(DummyWriter())

JSONType: TypeAlias = (
    dict[str, "JSONType"] | list["JSONType"] | str | int | float | bool | None
)


def ask_msg(
    conn: MAVConnection,
    msg_id: int,
    interval: int = 1_000_000,
) -> mavlink.MAVLink_command_long_message:
    """Request periodic sending of a MAVLink message (1 Hz)."""
    msg = conn.mav.command_long_encode(
        conn.target_system,
        conn.target_component,
        CmdSet.MESSAGE_INTERVAL,
        0,
        msg_id,
        interval,  # microseconds
        0,
        0,
        0,
        0,
        0,
    )
    return msg


def stop_msg(conn: MAVConnection, msg_id: int) -> mavlink.MAVLink_command_long_message:
    """Stop sending a specific MAVLink message."""
    msg = conn.mav.command_long_encode(
        conn.target_system,
        conn.target_component,
        CmdSet.MESSAGE_INTERVAL,
        0,
        msg_id,
        -1,  # Stop
        0,
        0,
        0,
        0,
        0,
    )
    return msg


def request_sensor_streams(
    conn: MAVConnection,
    stream_ids: list[DataStream],
    rate_hz: int = 5,
) -> dict[str, mavlink.MAVLink_request_data_stream_message]:
    """Request sensor messages from ArduPilot."""
    msgs: dict[str, mavlink.MAVLink_request_data_stream_message] = {}
    for stream_id in stream_ids:
        msgs[stream_id.name] = conn.mav.request_data_stream_encode(
            target_system=conn.target_system,
            target_component=conn.target_component,
            req_stream_id=stream_id,
            req_message_rate=rate_hz,
            start_stop=1,
        )
    return msgs


# Secondary MAVLink decoder (used to decode UNKNOWN_* messages)
# secondary_decoder = mavlink.MAVLink(None)


# def decode_unknown_message(msg: mavlink.MAVLink_message) -> mavlink.MAVLink_message:
#     """Attempt to decode an UNKNOWN_* message using a secondary MAVLink parser."""
#     try:
#         decoded = secondary_decoder.parse_char(msg.get_msgbuf())
#         if decoded:
#             return decoded
#     except Exception:
#         pass
#     return msg


def decode_unknown_message(msg: mavlink.MAVLink_message) -> mavlink.MAVLink_message:
    """Attempt to decode an UNKNOWN_* message using a secondary MAVLink parser."""
    try:
        buf = msg.get_msgbuf()
        decoded = None
        for byte in buf:
            decoded = secondary_decoder.parse_char(bytes([byte]))
        if decoded is not None:
            return cast(mavlink.MAVLink_message, decoded)
    except Exception:
        pass
    return msg


def make_json_safe(obj: object) -> JSONType:
    """
    Recursively convert an object to a JSON-serializable structure, decoding bytes
    and converting unknown types to strings.
    """
    if isinstance(obj, (bytes, bytearray)):
        return obj.decode("utf-8", errors="ignore").strip("\x00")

    elif isinstance(obj, Mapping):
        obj_map = cast(Mapping[object, object], obj)
        return {str(k): make_json_safe(v) for k, v in obj_map.items()}

    elif isinstance(obj, Sequence) and not isinstance(obj, (str, bytes, bytearray)):
        obj_seq = cast(Sequence[object], obj)
        return [make_json_safe(v) for v in obj_seq]

    elif isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj

    else:
        # fallback for unknown objects (e.g. enums, custom classes)
        return str(obj)
