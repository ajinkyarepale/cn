"""Drone agents: identity, autonomous movement, boundary and obstacle handling.

Each drone is a node of the swarm.  It owns a logical identity (``drone_id``),
a position in the 2D world, a communication range and the local statistics
that are displayed on the dashboard.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

from .config import SimConfig


@dataclass
class Obstacle:
    """A circular no-fly region that drones have to steer around."""

    x: float
    y: float
    radius: float


class Drone:
    """A single UAV in the simulation.

    Movement uses a random-waypoint model: the drone continuously steers
    towards a randomly chosen waypoint and picks a new one when it arrives.
    Waypoints are kept inside the world bounds and the drone slides around
    obstacles instead of passing through them.
    """

    def __init__(
        self,
        drone_id: str,
        x: float,
        y: float,
        config: SimConfig,
        rng,
        obstacles: Optional[List[Obstacle]] = None,
    ) -> None:
        self.drone_id = drone_id
        self.x = float(x)
        self.y = float(y)
        self.config = config
        self.rng = rng
        self.obstacles = obstacles or []

        self.comm_range = float(config.comm_range)
        self.speed = float(config.drone_speed)
        self.status = "ACTIVE"          # ACTIVE | DISCONNECTED
        self.port = 0                   # UDP port assigned by the comms layer

        self.target_x = x
        self.target_y = y

        # Local statistics
        self.messages_sent = 0
        self.messages_received = 0
        self.messages_forwarded = 0
        self.messages_dropped = 0

        self._pick_target()

    # ------------------------------------------------------------------
    # geometry helpers
    # ------------------------------------------------------------------
    def distance_to(self, other: "Drone") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def in_range(self, other: "Drone") -> bool:
        """True when the two drones are directly reachable (one hop)."""
        return self.distance_to(other) <= self.comm_range

    # ------------------------------------------------------------------
    # autonomous movement
    # ------------------------------------------------------------------
    def _pick_target(self) -> None:
        """Choose the next waypoint.

        A real swarm operates inside a common area of interest, so the
        waypoints are drawn from a disc around the centre of the world with
        a small chance of a long range leg.
        """
        cx = self.config.world_width / 2.0
        cy = self.config.world_height / 2.0
        radius = self.config.swarm_radius
        for _ in range(12):
            angle = self.rng.uniform(0, math.tau)
            distance = radius * math.sqrt(self.rng.uniform(0.02, 1.0))
            if self.rng.random() < 0.15:
                distance = radius * self.rng.uniform(1.0, 1.25)
            cand_x = cx + math.cos(angle) * distance
            cand_y = cy + math.sin(angle) * distance
            if (10 <= cand_x <= self.config.world_width - 10
                    and 10 <= cand_y <= self.config.world_height - 10):
                self.target_x, self.target_y = cand_x, cand_y
                return
        self.target_x = self.rng.uniform(10, self.config.world_width - 10)
        self.target_y = self.rng.uniform(10, self.config.world_height - 10)

    def _blocked(self, nx: float, ny: float) -> bool:
        for obstacle in self.obstacles:
            if math.hypot(nx - obstacle.x, ny - obstacle.y) < obstacle.radius + 8:
                return True
        return False

    def update(self, dt: float) -> None:
        """Advance the drone by ``dt`` seconds of simulated time."""
        if self.status != "ACTIVE" or dt <= 0:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dist = math.hypot(dx, dy)
        if dist < 8:
            self._pick_target()
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            dist = math.hypot(dx, dy) or 1.0

        step = self.speed * dt
        nx = self.x + dx / dist * step
        ny = self.y + dy / dist * step

        if self._blocked(nx, ny):
            # Try to slide tangentially around the obstacle, else re-target.
            ax, ay = dx / dist, dy / dist
            for angle in (0.7, -0.7, 1.4, -1.4):
                rx = ax * math.cos(angle) - ay * math.sin(angle)
                ry = ax * math.sin(angle) + ay * math.cos(angle)
                cand_x = self.x + rx * step
                cand_y = self.y + ry * step
                if not self._blocked(cand_x, cand_y):
                    nx, ny = cand_x, cand_y
                    break
            else:
                self._pick_target()
                nx, ny = self.x, self.y

        self.x = min(max(nx, 4.0), self.config.world_width - 4.0)
        self.y = min(max(ny, 4.0), self.config.world_height - 4.0)
