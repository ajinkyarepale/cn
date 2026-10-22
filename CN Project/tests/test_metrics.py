"""Tests for the performance metrics and the failure module."""

import pytest

from drone_swarm.failures import FailureManager
from drone_swarm.metrics import MetricsCollector


def make_packet(message_id, payload_bytes=40):
    return {
        "message_id": message_id,
        "sender_id": "D1",
        "receiver_id": "D2",
        "message_type": "POSITION_UPDATE",
        "sequence_number": 1,
        "created_at": 0.0,
        "payload": {"x": 1, "y": 2},
        "route": ["D1", "D2"],
        "payload_bytes": payload_bytes,
    }


def test_pdr_and_loss_percentage():
    metrics = MetricsCollector()
    for index in range(10):
        metrics.record_sent(make_packet(f"MSG{index}"))
    for index in range(8):
        metrics.record_delivered(make_packet(f"MSG{index}"), latency=0.05)
    metrics.record_dropped(make_packet("MSG8"), "packet loss")
    metrics.record_undeliverable(make_packet("MSG9"), "no route")

    assert metrics.total_sent == 10
    assert metrics.total_delivered == 8
    assert metrics.pdr == pytest.approx(80.0)
    assert metrics.loss_percentage == pytest.approx(20.0)
    assert metrics.simulated_drop_percentage == pytest.approx(10.0)
    assert metrics.unreachable_percentage == pytest.approx(10.0)


def test_latency_statistics():
    metrics = MetricsCollector()
    for index in range(100):
        metrics.record_sent(make_packet(f"MSG{index}"))
        metrics.record_delivered(make_packet(f"MSG{index}"), latency=index / 1000.0)

    assert metrics.min_latency == pytest.approx(0.0)
    assert metrics.max_latency == pytest.approx(0.099)
    assert metrics.avg_latency == pytest.approx(sum(range(100)) / 100 / 1000.0)
    assert 0.09 <= metrics.p95_latency <= 0.1


def test_throughput_uses_delivered_payload_bytes():
    metrics = MetricsCollector()
    for index in range(10):
        packet = make_packet(f"MSG{index}", payload_bytes=100)
        metrics.record_sent(packet)
        metrics.record_delivered(packet, latency=0.01)
    assert metrics.throughput(2.0) == pytest.approx(1000 / 2.0)


def test_failure_manager_disables_and_restores_links():
    failures = FailureManager()
    failures.disable_link("D2", "D1", 1.0)
    assert failures.link_disabled("D1", "D2")
    assert failures.link_disabled("D2", "D1")

    failures.restore_link("D1", "D2", 3.0)
    assert not failures.link_disabled("D1", "D2")
    assert failures.events[-1].kind == "LINK_UP"


def test_disconnect_and_reconnect_record_the_recovery_time():
    failures = FailureManager()
    event = failures.disconnect("D3", 10.0)
    assert failures.is_disconnected("D3")
    failures.reconnect("D3", 15.0)
    assert not failures.is_disconnected("D3")
    assert event.repaired
    assert event.recovery_time == pytest.approx(5.0)
    assert event.repair_latency == pytest.approx(5.0)
