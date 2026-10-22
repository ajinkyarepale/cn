"""Simulation engine.

Glues the four modules together and owns the main loop:

``drone`` -> movement, ``comms`` -> UDP transport, ``topology`` -> graph and
routing, ``failures`` -> fault injection, ``metrics`` -> performance data.

The engine is completely free of any GUI dependency, so the same class is
driven by the interactive dashboard (GUI) and by the headless experiment
runner (CLI).
"""

from __future__ import annotations

import json
import math
import random
from collections import deque
from typing import Any, Callable, Dict, List, Optional, Tuple

from .comms import NetworkStack
from .config import SimConfig
from .drone import Drone, Obstacle
from .failures import FailureManager
from .metrics import MetricsCollector
from .topology import Topology

MESSAGE_TYPES = ("POSITION_UPDATE", "STATUS_UPDATE", "ALERT")


class Simulation:
    """The drone swarm and its communication network."""

    def __init__(self, config: Optional[SimConfig] = None) -> None:
        self.config = config or SimConfig()
        self.rng = random.Random(self.config.seed)
        self.metrics = MetricsCollector()
        self.failures = FailureManager()
        self.failures.packet_loss = self.config.packet_loss
        self.failures.delay_ms = self.config.delay_ms
        self.topology = Topology(self.config)
        self.stack = NetworkStack(self.config.host, self.config.base_port, self.rng)

        self.drones: Dict[str, Drone] = {}
        self.obstacles: List[Obstacle] = []
        self.time = 0.0
        self.running = False
        self.events: deque = deque(maxlen=self.config.max_log)
        self._message_counter = 0
        self._sequence = 0
        self._next_message_at = 0.5
        self._sent_this_step = False
        self._event_index = 0
        self._pairs_before_failure: Dict[int, Set[frozenset]] = {}

    # ------------------------------------------------------------------
    # setup / teardown
    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Rebuild the swarm, the sockets and all counters."""
        self.stack.close()
        self.drones.clear()
        self.obstacles.clear()
        self.metrics.reset()
        self.failures = FailureManager()
        self.failures.packet_loss = self.config.packet_loss
        self.failures.delay_ms = self.config.delay_ms
        self.topology = Topology(self.config)
        self.time = 0.0
        self.running = False
        self.events.clear()
        self._message_counter = 0
        self._sequence = 0
        self._next_message_at = 0.5
        self._pairs_before_failure.clear()

        self._create_obstacles()
        self._create_drones()
        self.topology.rebuild(self.drones, self.failures)

    def _create_obstacles(self) -> None:
        """Place two obstacles inside the area the swarm operates in."""
        cx = self.config.world_width / 2.0
        cy = self.config.world_height / 2.0
        for _ in range(2):
            for _attempt in range(30):
                radius = self.rng.uniform(40, 65)
                angle = self.rng.uniform(0, 6.283)
                dist = self.rng.uniform(60, self.config.swarm_radius * 0.8)
                x = cx + math.cos(angle) * dist
                y = cy + math.sin(angle) * dist
                if not (radius + 10 <= x <= self.config.world_width - radius - 10):
                    continue
                if not (radius + 10 <= y <= self.config.world_height - radius - 10):
                    continue
                if all(
                    (x - ob.x) ** 2 + (y - ob.y) ** 2 > (radius + ob.radius + 40) ** 2
                    for ob in self.obstacles
                ):
                    self.obstacles.append(Obstacle(x, y, radius))
                    break

    def _create_drones(self) -> None:
        count = max(2, int(self.config.num_drones))
        for index in range(count):
            drone_id = f"D{index + 1}"
            x, y = self._free_position()
            drone = Drone(drone_id, x, y, self.config, self.rng, self.obstacles)
            endpoint = self.stack.add_endpoint(drone_id, self.config.base_port + index)
            drone.port = endpoint.port
            self.drones[drone_id] = drone

    def _free_position(self) -> Tuple[float, float]:
        """Random position inside the swarm area, not inside an obstacle."""
        cx = self.config.world_width / 2.0
        cy = self.config.world_height / 2.0
        for _ in range(80):
            angle = self.rng.uniform(0, math.tau)
            distance = self.config.swarm_radius * math.sqrt(self.rng.uniform(0.0, 1.0))
            x = cx + math.cos(angle) * distance
            y = cy + math.sin(angle) * distance
            if not (25 <= x <= self.config.world_width - 25):
                continue
            if not (25 <= y <= self.config.world_height - 25):
                continue
            if all(
                (x - ob.x) ** 2 + (y - ob.y) ** 2 > (ob.radius + 25) ** 2
                for ob in self.obstacles
            ):
                return x, y
        return cx, cy

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def start(self) -> None:
        self.running = True

    def pause(self) -> None:
        self.running = False

    def toggle(self) -> bool:
        self.running = not self.running
        return self.running

    def step(self, dt: float) -> None:
        """Advance the simulation by ``dt`` simulated seconds."""
        if dt <= 0:
            return
        self.time += dt
        self._sent_this_step = False

        # 1. autonomous movement -------------------------------------
        for drone in self.drones.values():
            drone.update(dt)

        # 2. dynamic topology ----------------------------------------
        self.topology.rebuild(self.drones, self.failures)

        # 3. traffic generation --------------------------------------
        self._generate_messages(dt)

        # 4. transport + routing -------------------------------------
        self._process_network()

        # 5. fault recovery analysis ---------------------------------
        self._update_recovery()

    def _generate_messages(self, dt: float) -> None:
        self._next_message_at -= dt
        if self._next_message_at > 0:
            return
        self._next_message_at += max(0.05, self.config.message_interval)

        active = [d.drone_id for d in self.drones.values() if d.status == "ACTIVE"]
        if len(active) < 2:
            return
        source = self.rng.choice(active)
        destination = self.rng.choice([a for a in active if a != source])
        message_type = self.rng.choices(MESSAGE_TYPES, weights=(6, 3, 1))[0]
        self.send_message(source, destination, message_type)

    # ------------------------------------------------------------------
    # messaging
    # ------------------------------------------------------------------
    def _build_payload(self, drone: Drone, message_type: str) -> Dict[str, Any]:
        if message_type == "POSITION_UPDATE":
            return {
                "x": round(drone.x, 1),
                "y": round(drone.y, 1),
                "status": drone.status,
                "speed": round(drone.speed, 1),
            }
        if message_type == "STATUS_UPDATE":
            return {
                "battery": round(self.rng.uniform(35.0, 100.0), 1),
                "status": drone.status,
                "port": drone.port,
            }
        return {
            "alert": self.rng.choice(
                ["OBSTACLE", "LOW_BATTERY", "GEOFENCE", "WIND_GUST", "LOST_LINK"]
            ),
            "severity": self.rng.choice(["LOW", "MEDIUM", "HIGH"]),
        }

    def send_message(self, source_id: str, destination_id: str,
                     message_type: str = "POSITION_UPDATE",
                     payload: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Create and transmit a message, routing it if necessary."""
        source = self.drones.get(source_id)
        if source is None:
            raise KeyError(f"unknown drone {source_id}")

        self._sequence += 1
        self._message_counter += 1
        message_id = f"MSG{self._message_counter:04d}"
        if payload is None:
            payload = self._build_payload(source, message_type)

        packet: Dict[str, Any] = {
            "message_id": message_id,
            "sender_id": source_id,
            "receiver_id": destination_id,
            "message_type": message_type,
            "sequence_number": self._sequence,
            "created_at": round(self.time, 4),
            "payload": payload,
            "route": [],
            "hop": 0,
            "next_hop": None,
            "payload_bytes": len(json.dumps(payload).encode("utf-8")),
        }
        self.metrics.record_sent(packet)
        source.messages_sent += 1

        destination = self.drones.get(destination_id)
        if destination is None or destination.status != "ACTIVE":
            self.metrics.record_undeliverable(packet, "destination offline")
            self.log_event("UNREACHABLE", source_id, destination_id, message_type, message_id,
                           "destination offline")
            return packet

        if self.topology.has_link(source_id, destination_id):
            route = [source_id, destination_id]
        else:
            route = self.topology.find_route(source_id, destination_id)

        if not route:
            self.metrics.record_undeliverable(packet, "no route")
            self.log_event("UNREACHABLE", source_id, destination_id, message_type, message_id,
                           "no route")
            return packet

        packet["route"] = route
        self._transmit(packet, 0)
        return packet

    def _transmit(self, packet: Dict[str, Any], holder_index: int) -> bool:
        """Send a packet from ``route[holder_index]`` to the next hop."""
        route = packet["route"]
        if holder_index + 1 >= len(route):
            return False
        holder = route[holder_index]
        next_hop = route[holder_index + 1]

        if not self.topology.has_link(holder, next_hop):
            self.metrics.record_dropped(packet, "link failed")
            self.log_event("DROPPED", holder, next_hop, packet["message_type"],
                           packet["message_id"], "link failed")
            if holder in self.drones:
                self.drones[holder].messages_dropped += 1
            return False

        if self.rng.random() < self.failures.packet_loss:
            self.metrics.record_dropped(packet, "packet loss")
            self.log_event("DROPPED", holder, next_hop, packet["message_type"],
                           packet["message_id"], "packet loss")
            if holder in self.drones:
                self.drones[holder].messages_dropped += 1
            return False

        delay_s = (self.failures.delay_ms * self.rng.uniform(1.0, 1.25)) / 1000.0
        wire = dict(packet)
        wire["hop"] = holder_index
        wire["next_hop"] = next_hop
        wire["last_hop"] = holder

        sent = self.stack.send(holder, next_hop, wire, sent_at=self.time, delay_s=delay_s)
        if not sent:
            self.metrics.record_dropped(packet, "socket error")
            self.log_event("DROPPED", holder, next_hop, packet["message_type"],
                           packet["message_id"], "socket error")
            return False
        self._sent_this_step = True
        return True

    def _process_network(self) -> None:
        """Read the sockets and deliver / forward the received datagrams."""
        timeout = 0.002 if self._sent_this_step else 0.0
        for receiver_id, packet in self.stack.poll(timeout=timeout):
            if packet["receiver_id"] == receiver_id:
                net = packet.get("net", {})
                arrival = float(net.get("sent_at", self.time)) + float(net.get("delay", 0.0))
                latency = max(0.0, arrival - float(packet["created_at"]))
                self.metrics.record_delivered(packet, latency)
                if receiver_id in self.drones:
                    self.drones[receiver_id].messages_received += 1
                self.log_event("DELIVERED", packet["sender_id"], receiver_id,
                               packet["message_type"], packet["message_id"],
                               f"{latency * 1000:.0f} ms")
            else:
                self._forward(packet, receiver_id)

    def _forward(self, packet: Dict[str, Any], receiver_id: str) -> None:
        """Multi-hop forwarding by an intermediate drone."""
        route = packet.get("route") or []
        holder_index = int(packet.get("hop", 0)) + 1
        if holder_index + 1 >= len(route):
            self.metrics.record_dropped(packet, "no next hop")
            self.log_event("DROPPED", receiver_id, packet.get("receiver_id", "?"),
                           packet["message_type"], packet["message_id"], "no next hop")
            return

        forwarded = dict(packet)
        forwarded["hop"] = holder_index
        forwarded["next_hop"] = route[holder_index + 1]
        forwarded["last_hop"] = receiver_id

        if self._transmit(forwarded, holder_index):
            if receiver_id in self.drones:
                self.drones[receiver_id].messages_forwarded += 1
            self.log_event("FORWARDED", packet["sender_id"], receiver_id,
                           packet["message_type"], packet["message_id"],
                           f"hop {holder_index}/{len(route) - 1}")
        elif receiver_id in self.drones:
            self.drones[receiver_id].messages_dropped += 1

    # ------------------------------------------------------------------
    # failure handling
    # ------------------------------------------------------------------
    def disconnect_drone(self, drone_id: str) -> bool:
        if drone_id not in self.drones or drone_id in self.failures.disconnected:
            return False
        event = self.failures.disconnect(drone_id, self.time)
        self._pairs_before_failure[event.id] = self.topology.connected_pairs()
        drone = self.drones[drone_id]
        drone.status = "DISCONNECTED"
        self.stack.remove_endpoint(drone_id)
        self.topology.rebuild(self.drones, self.failures)
        self.log_event("DISCONNECT", drone_id, "-", "FAILURE", "-", "node removed from network")
        return True

    def reconnect_drone(self, drone_id: str) -> bool:
        if drone_id not in self.drones or drone_id not in self.failures.disconnected:
            return False
        self.failures.reconnect(drone_id, self.time)
        drone = self.drones[drone_id]
        drone.status = "ACTIVE"
        endpoint = self.stack.add_endpoint(drone_id)
        drone.port = endpoint.port
        self.topology.rebuild(self.drones, self.failures)
        self.log_event("RECONNECT", drone_id, "-", "FAILURE", "-", "node back online")
        return True

    def disable_link(self, a: str, b: str) -> bool:
        if a == b or not self.topology.has_link(a, b):
            return False
        event = self.failures.disable_link(a, b, self.time)
        self._pairs_before_failure[event.id] = self.topology.connected_pairs()
        self.topology.rebuild(self.drones, self.failures)
        self.log_event("LINK_DOWN", a, b, "FAILURE", "-", "link disabled")
        return True

    def restore_link(self, a: str, b: str) -> bool:
        if not self.failures.link_disabled(a, b):
            return False
        self.failures.restore_link(a, b, self.time)
        self.topology.rebuild(self.drones, self.failures)
        self.log_event("LINK_UP", a, b, "FAILURE", "-", "link restored")
        return True

    def _update_recovery(self) -> None:
        """Close a fault once the lost connectivity has been restored."""
        current_pairs = self.topology.connected_pairs()
        for event in list(self.failures.open_events.values()):
            if event.repaired:
                continue
            before = self._pairs_before_failure.get(event.id, set())
            missing = before - current_pairs
            event.affected_pairs = len(missing)
            if not missing and before:
                self.failures.close_event(event, self.time)
                self.log_event("RECOVERED", event.target, "-", "FAILURE", "-",
                               f"{event.recovery_time:.1f} s after failure")

    # ------------------------------------------------------------------
    # logging
    # ------------------------------------------------------------------
    def log_event(self, status: str, source: str, destination: str,
                  message_type: str, message_id: str = "-", detail: str = "") -> None:
        self.events.append({
            "index": self._event_index,
            "time": self.time,
            "source": source,
            "destination": destination,
            "type": message_type,
            "status": status,
            "message_id": message_id,
            "detail": detail,
        })
        self._event_index += 1

    # ------------------------------------------------------------------
    # reporting
    # ------------------------------------------------------------------
    def connectivity_snapshot(self) -> Dict[str, Any]:
        components = self.topology.connected_components()
        return {
            "active_drones": sum(1 for d in self.drones.values() if d.status == "ACTIVE"),
            "disconnected_drones": len(self.failures.disconnected),
            "active_links": self.topology.link_count(),
            "components": len(components),
            "isolated_drones": len(self.topology.isolated_drones()),
            "largest_component": max((len(c) for c in components), default=0),
        }

    def snapshot(self) -> Dict[str, Any]:
        data = self.metrics.snapshot(self.time)
        data.update(self.connectivity_snapshot())
        data["sim_time"] = round(self.time, 2)
        return data
