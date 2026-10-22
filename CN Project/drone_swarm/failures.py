"""Network failure simulation module.

The user can inject controlled faults into the running simulation:

* disconnect / reconnect a drone,
* disable / restore a communication link,
* raise the simulated packet loss,
* raise the artificial communication delay.

Every fault is recorded as an event so that the dashboard can display the
failure history and the experiments module can measure recovery time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


@dataclass
class FailureEvent:
    """A single injected fault."""

    id: int
    kind: str                     # DISCONNECT | LINK_DOWN | RECONNECT | LINK_UP
    target: str                   # drone id or "A-B"
    time: float                   # simulated time of injection
    repaired: bool = False
    repair_time: Optional[float] = None
    recovery_time: Optional[float] = None
    affected_pairs: int = 0
    detail: str = ""

    @property
    def repair_latency(self) -> Optional[float]:
        """Time between fault injection and the repair of the fault."""
        if self.repair_time is None:
            return None
        return self.repair_time - self.time


class FailureManager:
    """Holds the current network conditions and the failure history."""

    def __init__(self) -> None:
        self.disconnected: Set[str] = set()
        self.disabled_links: Set[Tuple[str, str]] = set()
        self.packet_loss: float = 0.0
        self.delay_ms: float = 20.0
        self.events: List[FailureEvent] = []
        self.open_events: Dict[int, FailureEvent] = {}
        self._next_id = 1

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def normalise(a: str, b: str) -> Tuple[str, str]:
        return (a, b) if a <= b else (b, a)

    def link_disabled(self, a: str, b: str) -> bool:
        return self.normalise(a, b) in self.disabled_links

    def is_disconnected(self, drone_id: str) -> bool:
        return drone_id in self.disconnected

    # ------------------------------------------------------------------
    # fault injection
    # ------------------------------------------------------------------
    def disconnect(self, drone_id: str, now: float) -> FailureEvent:
        self.disconnected.add(drone_id)
        event = FailureEvent(self._next_id, "DISCONNECT", drone_id, now)
        self._next_id += 1
        self.events.append(event)
        self.open_events[event.id] = event
        return event

    def reconnect(self, drone_id: str, now: float) -> FailureEvent:
        self.disconnected.discard(drone_id)
        event = FailureEvent(self._next_id, "RECONNECT", drone_id, now)
        self._next_id += 1
        self.events.append(event)
        for open_event in list(self.open_events.values()):
            if open_event.kind == "DISCONNECT" and open_event.target == drone_id:
                self.close_event(open_event, now)
        return event

    def disable_link(self, a: str, b: str, now: float) -> FailureEvent:
        self.disabled_links.add(self.normalise(a, b))
        event = FailureEvent(self._next_id, "LINK_DOWN", f"{a}-{b}", now)
        self._next_id += 1
        self.events.append(event)
        self.open_events[event.id] = event
        return event

    def restore_link(self, a: str, b: str, now: float) -> FailureEvent:
        self.disabled_links.discard(self.normalise(a, b))
        event = FailureEvent(self._next_id, "LINK_UP", f"{a}-{b}", now)
        self._next_id += 1
        self.events.append(event)
        for open_event in list(self.open_events.values()):
            if open_event.kind == "LINK_DOWN" and open_event.target == f"{a}-{b}":
                self.close_event(open_event, now)
        return event

    # ------------------------------------------------------------------
    # recovery handling
    # ------------------------------------------------------------------
    def close_event(self, event: FailureEvent, now: float) -> None:
        event.repaired = True
        event.repair_time = now
        event.recovery_time = now - event.time
        self.open_events.pop(event.id, None)

    def update_recovery(self, now: float, connected_pairs: Set[frozenset]) -> None:
        """Close open faults once the network connectivity is restored.

        ``connected_pairs`` is the set of currently working links.  A fault
        is considered recovered when the number of working links is back to
        the level it had before the fault was injected.
        """
        for event in list(self.open_events.values()):
            if event.repaired:
                continue
            event.affected_pairs = max(0, event.affected_pairs)
            if event.affected_pairs == 0:
                self.close_event(event, now)

    @property
    def last_recovery_time(self) -> Optional[float]:
        for event in reversed(self.events):
            if event.recovery_time is not None:
                return event.recovery_time
        return None
