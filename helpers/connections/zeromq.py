"""Helper functions for creating ZeroMQ sockets."""

from typing import Any

import zmq

from config import BasePort


def create_zmq_sockets(
    zmq_ctx: zmq.Context[Any],
    base_port: BasePort,
    sockets_type: int,
    offsets: dict[Any, int],
    timeout: int = 100,
):
    """Create ZMQ sockets for UAV communication."""
    socks = dict[Any, zmq.Socket[bytes]]()
    for sysid, offset in offsets.items():
        socks[sysid] = create_zmq_socket(
            base_port=base_port,
            zmq_ctx=zmq_ctx,
            sockets_type=sockets_type,
            offset=offset,
            timeout=timeout,
        )
    return socks


def create_zmq_socket(
    zmq_ctx: zmq.Context[Any],
    base_port: BasePort,
    sockets_type: int,
    offset: int,
    timeout: int = 100,
) -> zmq.Socket[bytes]:
    """Create a single ZMQ socket for UAV communication."""
    socket = zmq_ctx.socket(sockets_type)
    socket.connect(f"tcp://127.0.0.1:{base_port + offset}")
    if sockets_type == zmq.SUB:
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.setsockopt(zmq.RCVTIMEO, timeout)
    return socket
