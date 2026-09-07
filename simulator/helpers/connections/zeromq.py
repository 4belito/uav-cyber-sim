"""Helper functions for creating ZeroMQ sockets."""

import zmq

from simulator.config import SimPort, VehPort


def create_zmq_sockets(
    zmq_ctx: zmq.Context[zmq.Socket[bytes]],
    base_port: SimPort | VehPort,
    sockets_type: int,
    offsets: dict[int, int],
    timeout: int = 100,
) -> dict[int, zmq.Socket[bytes]]:
    """Create ZMQ sockets for UAV communication."""
    socks = dict[int, zmq.Socket[bytes]]()
    for sysid, offset in offsets.items():
        socks[sysid] = create_zmq_socket(
            zmq_ctx=zmq_ctx,
            sockets_type=sockets_type,
            base_port=base_port,
            offset=offset,
            timeout=timeout,
        )
    return socks


def create_zmq_socket(
    zmq_ctx: zmq.Context[zmq.Socket[bytes]],
    sockets_type: int,
    base_port: SimPort | VehPort,
    offset: int,
    timeout: int = 100,
    subscribe: bytes = b"",
    identity: bytes | None = None,
) -> zmq.Socket[bytes]:
    """Create a single ZMQ socket for UAV communication."""
    socket = zmq_ctx.socket(sockets_type)
    endpoint = f"tcp://127.0.0.1:{base_port + offset}"
    if identity is not None:
        socket.setsockopt(zmq.IDENTITY, identity)

    if sockets_type == zmq.PUB:
        socket.bind(endpoint)
        socket.setsockopt(zmq.SNDTIMEO, timeout)

    elif sockets_type == zmq.SUB:
        socket.connect(endpoint)
        socket.setsockopt(zmq.SUBSCRIBE, subscribe)
        socket.setsockopt(zmq.RCVTIMEO, timeout)

    elif sockets_type == zmq.ROUTER:
        socket.bind(endpoint)
        socket.setsockopt(zmq.RCVTIMEO, timeout)
        socket.setsockopt(zmq.SNDTIMEO, timeout)

    elif sockets_type == zmq.DEALER:
        socket.connect(endpoint)
        socket.setsockopt(zmq.RCVTIMEO, timeout)
        socket.setsockopt(zmq.SNDTIMEO, timeout)

    else:
        raise ValueError(f"Invalid socket type: {sockets_type}")

    return socket
