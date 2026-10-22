"""Tkinter rendering of the swarm and its network topology."""

from __future__ import annotations

import math
import tkinter as tk

from ..simulation import Simulation

COLORS = {
    "bg": "#0b1220",
    "grid": "#16213a",
    "obstacle": "#2a3550",
    "obstacle_edge": "#3d4a6b",
    "link": "#2e6f8f",
    "link_failed": "#c0503f",
    "route": "#f2c14e",
    "drone": "#3fb6d8",
    "drone_off": "#7a8396",
    "drone_selected": "#f2c14e",
    "text": "#e8eefc",
    "dim": "#8fa0bd",
    "range": "#3fb6d8",
}


class SwarmCanvas(tk.Canvas):
    """Draws the drones, the wireless links, the obstacles and the routes."""

    def __init__(self, master, sim: Simulation, **kwargs) -> None:
        kwargs.setdefault("bg", COLORS["bg"])
        kwargs.setdefault("highlightthickness", 0)
        super().__init__(master, **kwargs)
        self.sim = sim
        self.selected: str | None = None
        self.route: list[str] = []
        self._scale = 1.0
        self._offset = (0.0, 0.0)
        self.bind("<Configure>", lambda _event: self.redraw())
        self.bind("<Button-1>", self._on_click)

    # ------------------------------------------------------------------
    # coordinate helpers
    # ------------------------------------------------------------------
    def _transform(self) -> None:
        width = max(1, self.winfo_width())
        height = max(1, self.winfo_height())
        world_w = self.sim.config.world_width
        world_h = self.sim.config.world_height
        self._scale = min(width / world_w, height / world_h)
        self._offset = ((width - world_w * self._scale) / 2.0,
                        (height - world_h * self._scale) / 2.0)

    def _to_canvas(self, x: float, y: float) -> tuple[float, float]:
        return (x * self._scale + self._offset[0],
                y * self._scale + self._offset[1])

    def _to_world(self, x: float, y: float) -> tuple[float, float]:
        return ((x - self._offset[0]) / self._scale,
                (y - self._offset[1]) / self._scale)

    # ------------------------------------------------------------------
    # interaction
    # ------------------------------------------------------------------
    def _on_click(self, event) -> None:
        wx, wy = self._to_world(event.x, event.y)
        best = None
        best_distance = 24.0
        for drone_id, drone in self.sim.drones.items():
            distance = math.hypot(drone.x - wx, drone.y - wy)
            if distance < best_distance:
                best = drone_id
                best_distance = distance
        if best != self.selected:
            self.selected = best
            self.route = []
            self.redraw()
            self.master.on_selection_changed(best)

    def show_route(self, route: list[str]) -> None:
        self.route = list(route)
        self.redraw()

    # ------------------------------------------------------------------
    # rendering
    # ------------------------------------------------------------------
    def redraw(self) -> None:
        self.delete("all")
        self._transform()
        self._draw_grid()
        self._draw_obstacles()
        self._draw_links()
        self._draw_route()
        self._draw_drones()

    def _draw_grid(self) -> None:
        world_w = self.sim.config.world_width
        world_h = self.sim.config.world_height
        step = 40
        for x in range(0, world_w + 1, step):
            x0, y0 = self._to_canvas(x, 0)
            x1, y1 = self._to_canvas(x, world_h)
            self.create_line(x0, y0, x1, y1, fill=COLORS["grid"])
        for y in range(0, world_h + 1, step):
            x0, y0 = self._to_canvas(0, y)
            x1, y1 = self._to_canvas(world_w, y)
            self.create_line(x0, y0, x1, y1, fill=COLORS["grid"])

    def _draw_obstacles(self) -> None:
        for obstacle in self.sim.obstacles:
            x0, y0 = self._to_canvas(obstacle.x - obstacle.radius, obstacle.y - obstacle.radius)
            x1, y1 = self._to_canvas(obstacle.x + obstacle.radius, obstacle.y + obstacle.radius)
            self.create_oval(x0, y0, x1, y1, fill=COLORS["obstacle"],
                             outline=COLORS["obstacle_edge"], width=1)

    def _draw_links(self) -> None:
        graph = self.sim.topology.graph
        for a, b in graph.edges():
            drone_a = self.sim.drones.get(a)
            drone_b = self.sim.drones.get(b)
            if not drone_a or not drone_b:
                continue
            x0, y0 = self._to_canvas(drone_a.x, drone_a.y)
            x1, y1 = self._to_canvas(drone_b.x, drone_b.y)
            self.create_line(x0, y0, x1, y1, fill=COLORS["link"], width=1)

        # links that were switched off by the failure module
        for a, b in self.sim.failures.disabled_links:
            drone_a = self.sim.drones.get(a)
            drone_b = self.sim.drones.get(b)
            if not drone_a or not drone_b:
                continue
            if drone_a.distance_to(drone_b) > self.sim.config.comm_range:
                continue
            x0, y0 = self._to_canvas(drone_a.x, drone_a.y)
            x1, y1 = self._to_canvas(drone_b.x, drone_b.y)
            self.create_line(x0, y0, x1, y1, fill=COLORS["link_failed"],
                             width=2, dash=(6, 4))

    def _draw_route(self) -> None:
        if len(self.route) < 2:
            return
        for index in range(len(self.route) - 1):
            a = self.sim.drones.get(self.route[index])
            b = self.sim.drones.get(self.route[index + 1])
            if not a or not b:
                continue
            x0, y0 = self._to_canvas(a.x, a.y)
            x1, y1 = self._to_canvas(b.x, b.y)
            self.create_line(x0, y0, x1, y1, fill=COLORS["route"], width=3)
            mid_x, mid_y = (x0 + x1) / 2, (y0 + y1) / 2
            self.create_text(mid_x, mid_y, text=f"hop {index + 1}", fill=COLORS["route"],
                             font=("Segoe UI", 8))

    def _draw_drones(self) -> None:
        radius = 9
        for drone_id, drone in self.sim.drones.items():
            x, y = self._to_canvas(drone.x, drone.y)
            if drone_id == self.selected:
                r = self.sim.config.comm_range * self._scale
                self.create_oval(x - r, y - r, x + r, y + r, outline=COLORS["range"],
                                 width=1, dash=(4, 6))
            colour = COLORS["drone"] if drone.status == "ACTIVE" else COLORS["drone_off"]
            outline = COLORS["drone_selected"] if drone_id == self.selected else colour
            self.create_oval(x - radius, y - radius, x + radius, y + radius,
                             fill=colour, outline=outline, width=2)
            self.create_text(x, y - radius - 8, text=drone_id, fill=COLORS["text"],
                             font=("Segoe UI", 9, "bold"))
            if drone.status != "ACTIVE":
                self.create_text(x, y + radius + 8, text="OFFLINE", fill=COLORS["link_failed"],
                                 font=("Segoe UI", 7))

    # ------------------------------------------------------------------
    def draw_legend(self, x: int, y: int) -> None:
        entries = [
            ("drone", "active drone"),
            ("drone_off", "disconnected drone"),
            ("link", "wireless link"),
            ("link_failed", "failed link"),
            ("route", "selected route"),
        ]
        for offset, (key, label) in enumerate(entries):
            self.create_oval(x, y + offset * 16 - 4, x + 8, y + offset * 16 + 4,
                             fill=COLORS[key], outline="")
            self.create_text(x + 14, y + offset * 16, text=label, fill=COLORS["dim"],
                             anchor="w", font=("Segoe UI", 8))
