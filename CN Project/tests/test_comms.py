"""Tests for the UDP communication module (real sockets on localhost)."""

import socket

import pytest

from drone_swarm.comms import (Endpoint, NetworkStack, ProtocolError,
                               decode_packet, encode_packet)

PACKET = {
    "message_id": "MSG0001",
    "sender_id": "D1",
    "receiver_id": "D2",
    "message_type": "POSITION_UPDATE",
    "sequence_number": 1,
    "created_at": 1.0,
    "payload": {"x": 10, "y": 20, "status": "ACTIVE"},
    "route": ["D1", "D2"],
    "hop": 0,
    "next_hop": "D2",
}


def test_encode_and_decode_round_trip():
    data = encode_packet(PACKET)
    assert isinstance(data, bytes)
    decoded = decode_packet(data)
    assert decoded["message_id"] == "MSG0001"
    assert decoded["payload"]["x"] == 10


def test_decode_rejects_malformed_datagrams():
    with pytest.raises(ProtocolError):
        decode_packet(b"not json")
    with pytest.raises(ProtocolError):
        decode_packet(b'{"message_id": "X"}')


def test_udp_datagram_is_received_by_the_other_endpoint():
    stack = NetworkStack(base_port=0)
    try:
        stack.add_endpoint("D1")
        stack.add_endpoint("D2")
        assert stack.send("D1", "D2", PACKET, sent_at=0.0, delay_s=0.01)

        received = []
        for _ in range(20):
            received = stack.poll(timeout=0.05)
            if received:
                break
        assert received
        receiver_id, packet = received[0]
        assert receiver_id == "D2"
        assert packet["sender_id"] == "D1"
        assert packet["net"]["delay"] == pytest.approx(0.01)
    finally:
        stack.close()


def test_each_drone_gets_its_own_udp_port():
    stack = NetworkStack(base_port=0)
    try:
        first = stack.add_endpoint("D1")
        second = stack.add_endpoint("D2")
        assert first.port != second.port
        assert first.sock.getsockname()[0] == "127.0.0.1"
    finally:
        stack.close()


def test_duplicated_sequence_numbers_are_discarded():
    stack = NetworkStack(base_port=0)
    try:
        stack.add_endpoint("D1")
        stack.add_endpoint("D2")
        stack.send("D1", "D2", PACKET, sent_at=0.0, delay_s=0.0)
        for _ in range(20):
            first = stack.poll(timeout=0.05)
            if first:
                break
        assert len(first) == 1

        # the same datagram a second time must be treated as a duplicate
        stack.send("D1", "D2", PACKET, sent_at=0.0, delay_s=0.0)
        second = stack.poll(timeout=0.05)
        assert second == []
        assert stack.duplicates == 1
    finally:
        stack.close()


def test_sending_to_an_unknown_drone_fails():
    stack = NetworkStack(base_port=0)
    try:
        stack.add_endpoint("D1")
        assert stack.send("D1", "D99", PACKET, sent_at=0.0, delay_s=0.0) is False
    finally:
        stack.close()
