"""UDP transport layer (Student 2 - network communication).

Every drone owns a **real UDP socket** bound to a local port on
``127.0.0.1``.  Messages are serialised to JSON, sent with ``sendto`` and
received with ``recvfrom``, exactly like a lightweight wireless datagram
network.

Message format (one JSON object per datagram)::

    {
      "message_id": "MSG001",
      "sender_id": "D1",
      "receiver_id": "D2",
      "message_type": "POSITION_UPDATE",
      "sequence_number": 15,
      "created_at": 12.5,
      "payload": {"x": 120, "y": 250, "status": "ACTIVE"},
      "route": ["D1", "D2"],
      "hop": 0,
      "next_hop": "D2"
    }

The ``net`` sub-object is added by the transport layer and carries the
modelled one-way link delay used for the latency measurement.
"""

from __future__ import annotations

import json
import random
import select
import socket
from typing import Any, Dict, List, Optional, Tuple

REQUIRED_FIELDS = (
    "message_id",
    "sender_id",
    "receiver_id",
    "message_type",
    "sequence_number",
    "payload",
)


class ProtocolError(Exception):
    """Raised when a datagram does not follow the message format."""


def encode_packet(packet: Dict[str, Any]) -> bytes:
    """Serialise a packet to JSON bytes."""
    return json.dumps(packet).encode("utf-8")


def decode_packet(data: bytes) -> Dict[str, Any]:
    """Parse and validate a received datagram."""
    try:
        packet = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"malformed datagram: {exc}") from exc
    if not isinstance(packet, dict):
        raise ProtocolError("datagram is not a JSON object")
    missing = [field for field in REQUIRED_FIELDS if field not in packet]
    if missing:
        raise ProtocolError(f"missing fields: {', '.join(missing)}")
    return packet


class Endpoint:
    """A drone's network endpoint (logical identity + UDP socket)."""

    def __init__(self, drone_id: str, host: str, port: int, sock: socket.socket) -> None:
        self.drone_id = drone_id
        self.host = host
        self.port = port
        self.sock = sock
        self.bytes_sent = 0
        self.bytes_received = 0
        self.packets_sent = 0
        self.packets_received = 0

    @property
    def address(self) -> str:
        return f"{self.host}:{self.port}"

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class NetworkStack:
    """Manages every drone socket and moves datagrams between them."""

    def __init__(self, host: str = "127.0.0.1", base_port: int = 45600,
                 rng: Optional[random.Random] = None) -> None:
        self.host = host
        self.base_port = base_port
        self.rng = rng or random.Random()
        self.endpoints: Dict[str, Endpoint] = {}
        self.last_sequence: Dict[str, int] = {}
        self.seen_messages: Dict[str, set] = {}
        self.protocol_errors = 0
        self.duplicates = 0
        self.out_of_order = 0

    # ------------------------------------------------------------------
    # endpoint management
    # ------------------------------------------------------------------
    def add_endpoint(self, drone_id: str, preferred_port: Optional[int] = None) -> Endpoint:
        """Create and bind a UDP socket for ``drone_id``."""
        port = self.base_port if preferred_port is None else preferred_port
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # SO_REUSEADDR is deliberately not set: on Windows it would allow
            # a second process to bind the same UDP port and datagrams would
            # be delivered to an arbitrary socket.
            sock.bind((self.host, port))
        except OSError:
            # fall back to an ephemeral port when the preferred one is taken
            sock.close()
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind((self.host, 0))
        sock.setblocking(False)
        endpoint = Endpoint(drone_id, self.host, sock.getsockname()[1], sock)
        self.endpoints[drone_id] = endpoint
        return endpoint

    def remove_endpoint(self, drone_id: str) -> None:
        endpoint = self.endpoints.pop(drone_id, None)
        if endpoint is not None:
            endpoint.close()

    def close(self) -> None:
        for drone_id in list(self.endpoints):
            self.remove_endpoint(drone_id)

    # ------------------------------------------------------------------
    # transmission
    # ------------------------------------------------------------------
    def send(self, src_id: str, dst_id: str, packet: Dict[str, Any],
             sent_at: float, delay_s: float) -> bool:
        """Send one datagram over the real UDP socket.

        ``sent_at`` (simulated time) and ``delay_s`` (modelled link delay)
        are attached to the datagram so the receiver can compute the
        end-to-end latency.
        """
        source = self.endpoints.get(src_id)
        target = self.endpoints.get(dst_id)
        if source is None or target is None:
            return False

        wire = dict(packet)
        wire["net"] = {"sent_at": float(sent_at), "delay": float(delay_s)}
        data = encode_packet(wire)
        try:
            source.sock.sendto(data, (target.host, target.port))
        except OSError:
            return False
        source.bytes_sent += len(data)
        source.packets_sent += 1
        return True

    # ------------------------------------------------------------------
    # reception
    # ------------------------------------------------------------------
    def poll(self, timeout: float = 0.0) -> List[Tuple[str, Dict[str, Any]]]:
        """Read every pending datagram.

        Returns a list of ``(receiver_drone_id, packet)`` tuples.  Sequence
        numbers are checked here so that duplicated or reordered datagrams
        can be discarded (UDP does not guarantee ordering).
        """
        if not self.endpoints:
            return []

        sockets = [ep.sock for ep in self.endpoints.values()]
        if timeout > 0:
            try:
                select.select(sockets, [], [], timeout)
            except (OSError, ValueError):
                pass

        received: List[Tuple[str, Dict[str, Any]]] = []
        for drone_id, endpoint in self.endpoints.items():
            while True:
                try:
                    data, _addr = endpoint.sock.recvfrom(65535)
                except (BlockingIOError, ConnectionResetError):
                    break
                except OSError:
                    break
                endpoint.bytes_received += len(data)
                endpoint.packets_received += 1
                try:
                    packet = decode_packet(data)
                except ProtocolError:
                    self.protocol_errors += 1
                    continue
                # The datagram must be addressed to the endpoint that received
                # it.  ``next_hop`` names the immediate receiver (the final
                # destination for the last hop), ``receiver_id`` always names
                # the end-to-end destination.
                addressed_to = packet.get("next_hop") or packet["receiver_id"]
                if addressed_to != drone_id:
                    self.protocol_errors += 1
                    continue

                # A message must be processed only once per receiver.  The
                # identifier is used instead of the sequence number, because
                # the same message is forwarded by every intermediate drone
                # and therefore reaches several different endpoints.
                message_id = packet["message_id"]
                seen = self.seen_messages.setdefault(drone_id, set())
                if message_id in seen:
                    self.duplicates += 1
                    continue
                seen.add(message_id)
                if len(seen) > 20000:
                    seen.clear()

                # The sequence number is used to detect reordering, which UDP
                # does not guarantee.
                sequence = int(packet["sequence_number"])
                last = self.last_sequence.get(packet["sender_id"])
                if last is not None and sequence < last:
                    self.out_of_order += 1
                self.last_sequence[packet["sender_id"]] = max(last or 0, sequence)
                received.append((drone_id, packet))
        return received
