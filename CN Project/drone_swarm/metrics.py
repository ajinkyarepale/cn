"""Performance monitoring module (Student 4).

Collects every communication event and derives the network performance
indicators that are defined in the project report:

* Packet Delivery Ratio (PDR)
* Packet loss percentage (split into simulated drops and unreachable messages)
* End-to-end latency (average / min / max / 95th percentile)
* Throughput (successfully delivered payload bytes per second)
* Network connectivity (nodes, links, components, isolated drones)
* Recovery time after a fault
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional


class MetricsCollector:
    """Aggregates communication events into performance metrics."""

    def __init__(self) -> None:
        self.sent: Dict[str, Dict[str, Any]] = {}
        self.delivered: Dict[str, float] = {}
        self.dropped: Dict[str, str] = {}
        self.undeliverable: Dict[str, str] = {}
        self.latencies: List[float] = []
        self.delivered_bytes = 0
        self.generated_bytes = 0
        self.duplicates = 0

    # ------------------------------------------------------------------
    # event recording
    # ------------------------------------------------------------------
    def record_sent(self, packet: Dict[str, Any]) -> None:
        self.sent[packet["message_id"]] = packet
        self.generated_bytes += int(packet.get("payload_bytes", 0))

    def record_delivered(self, packet: Dict[str, Any], latency: float) -> None:
        message_id = packet["message_id"]
        if message_id in self.delivered:
            self.duplicates += 1
            return
        self.delivered[message_id] = latency
        self.latencies.append(latency)
        self.delivered_bytes += int(packet.get("payload_bytes", 0))

    def record_dropped(self, packet: Dict[str, Any], reason: str) -> None:
        """A transmission was attempted but the datagram never arrived."""
        self.dropped.setdefault(packet["message_id"], reason)

    def record_undeliverable(self, packet: Dict[str, Any], reason: str) -> None:
        """No usable route to the destination existed."""
        self.undeliverable.setdefault(packet["message_id"], reason)

    # ------------------------------------------------------------------
    # derived metrics
    # ------------------------------------------------------------------
    @property
    def total_sent(self) -> int:
        return len(self.sent)

    @property
    def total_delivered(self) -> int:
        return len(self.delivered)

    @property
    def total_dropped(self) -> int:
        return len(self.dropped)

    @property
    def total_undeliverable(self) -> int:
        return len(self.undeliverable)

    @property
    def pdr(self) -> float:
        """Packet Delivery Ratio in percent."""
        if not self.sent:
            return 0.0
        return 100.0 * self.total_delivered / self.total_sent

    @property
    def loss_percentage(self) -> float:
        """Percentage of eligible messages that were not delivered."""
        if not self.sent:
            return 0.0
        return 100.0 * (self.total_sent - self.total_delivered) / self.total_sent

    @property
    def simulated_drop_percentage(self) -> float:
        """Part of the loss caused by the simulated channel (packet loss)."""
        if not self.sent:
            return 0.0
        return 100.0 * self.total_dropped / self.total_sent

    @property
    def unreachable_percentage(self) -> float:
        """Part of the loss caused by the absence of a route."""
        if not self.sent:
            return 0.0
        return 100.0 * self.total_undeliverable / self.total_sent

    @property
    def avg_latency(self) -> float:
        return sum(self.latencies) / len(self.latencies) if self.latencies else 0.0

    @property
    def min_latency(self) -> float:
        return min(self.latencies) if self.latencies else 0.0

    @property
    def max_latency(self) -> float:
        return max(self.latencies) if self.latencies else 0.0

    @property
    def p95_latency(self) -> float:
        if not self.latencies:
            return 0.0
        ordered = sorted(self.latencies)
        index = int(math.ceil(0.95 * len(ordered)) - 1)
        return ordered[max(0, min(index, len(ordered) - 1))]

    def throughput(self, elapsed: float) -> float:
        """Successfully delivered payload bytes per second."""
        if elapsed <= 0:
            return 0.0
        return self.delivered_bytes / elapsed

    # ------------------------------------------------------------------
    # reporting
    # ------------------------------------------------------------------
    def snapshot(self, elapsed: float = 0.0) -> Dict[str, Any]:
        return {
            "messages_sent": self.total_sent,
            "messages_delivered": self.total_delivered,
            "messages_dropped": self.total_dropped,
            "messages_undeliverable": self.total_undeliverable,
            "pdr_percent": round(self.pdr, 2),
            "loss_percent": round(self.loss_percentage, 2),
            "simulated_drop_percent": round(self.simulated_drop_percentage, 2),
            "unreachable_percent": round(self.unreachable_percentage, 2),
            "avg_latency_ms": round(self.avg_latency * 1000, 2),
            "min_latency_ms": round(self.min_latency * 1000, 2),
            "max_latency_ms": round(self.max_latency * 1000, 2),
            "p95_latency_ms": round(self.p95_latency * 1000, 2),
            "throughput_bytes_s": round(self.throughput(elapsed), 2),
            "delivered_bytes": self.delivered_bytes,
            "duplicates": self.duplicates,
        }

    def reset(self) -> None:
        self.sent.clear()
        self.delivered.clear()
        self.dropped.clear()
        self.undeliverable.clear()
        self.latencies.clear()
        self.delivered_bytes = 0
        self.generated_bytes = 0
        self.duplicates = 0
