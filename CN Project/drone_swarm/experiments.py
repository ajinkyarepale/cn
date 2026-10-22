"""Experiment runner and results export (Student 4).

The same :class:`~drone_swarm.simulation.Simulation` class that drives the
interactive dashboard is executed here **head-less**, so every number in the
project report comes from a real simulation run instead of an assumption.

Five experiments are defined in the project plan:

1. Normal network operation (baseline).
2. Effect of packet loss (10 %).
3. Drone disconnection and recovery.
4. Increasing the number of drones (5 / 10 / 20).
5. Effect of the communication range.
"""

from __future__ import annotations

import csv
import os
import re
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

from .config import SimConfig
from .simulation import Simulation

DT = 0.05  # simulated seconds per step


@dataclass
class ExperimentResult:
    """Outcome of a single experiment run."""

    name: str
    group: str
    config: Dict[str, Any]
    metrics: Dict[str, Any]
    duration: float

    def as_row(self) -> Dict[str, Any]:
        row = {"experiment": self.name, "group": self.group}
        row.update(self.config)
        row.update(self.metrics)
        row["duration_s"] = self.duration
        return row


@dataclass
class FaultAction:
    """A fault that is injected at a fixed time of the simulation."""

    at: float
    action: str          # DISCONNECT | RECONNECT | DISABLE_LINK | RESTORE_LINK
    target: str
    second: str = ""

    def apply(self, sim: Simulation) -> None:
        if self.action == "DISCONNECT":
            sim.disconnect_drone(self.target)
        elif self.action == "RECONNECT":
            sim.reconnect_drone(self.target)
        elif self.action == "DISABLE_LINK":
            sim.disable_link(self.target, self.second)
        elif self.action == "RESTORE_LINK":
            sim.restore_link(self.target, self.second)


def run_headless(config: SimConfig, duration: float,
                 faults: Optional[List[FaultAction]] = None,
                 progress: Optional[Callable[[float], None]] = None) -> ExperimentResult:
    """Run the simulation for ``duration`` simulated seconds without a GUI."""
    sim = Simulation(config)
    sim.reset()
    faults = faults or []
    pending = list(faults)

    steps = int(duration / DT)
    for step_index in range(steps):
        now = sim.time
        while pending and pending[0].at <= now:
            pending.pop(0).apply(sim)
        sim.step(DT)
        if progress and step_index % 100 == 0:
            progress(sim.time / duration if duration else 1.0)

    metrics = sim.snapshot()
    recovery = [event.recovery_time for event in sim.failures.events
                if event.recovery_time is not None]
    metrics["recovery_time_s"] = round(max(recovery), 2) if recovery else 0.0
    metrics["faults_injected"] = len([e for e in sim.failures.events
                                      if e.kind in ("DISCONNECT", "LINK_DOWN")])
    metrics["messages_per_second"] = round(metrics["messages_sent"] / duration, 3) if duration else 0.0
    metrics["mean_hops"] = round(_mean_hops(sim), 2)

    config_dict = {
        "drones": config.num_drones,
        "comm_range": config.comm_range,
        "packet_loss": config.packet_loss,
        "delay_ms": config.delay_ms,
        "message_interval": config.message_interval,
        "seed": config.seed,
    }
    sim.stack.close()
    return ExperimentResult("", "", config_dict, metrics, duration)


def _mean_hops(sim: Simulation) -> float:
    hops = []
    for packet in sim.metrics.sent.values():
        route = packet.get("route") or []
        if route:
            hops.append(len(route) - 1)
    return sum(hops) / len(hops) if hops else 0.0


# ----------------------------------------------------------------------
# experiment catalogue
# ----------------------------------------------------------------------
def experiment_plan(duration: float = 90.0) -> List[Tuple[str, str, SimConfig, List[FaultAction]]]:
    """Return ``(name, group, config, faults)`` for every planned run."""
    base = SimConfig()
    plan: List[Tuple[str, str, SimConfig, List[FaultAction]]] = []

    # Experiment 1 - baseline
    plan.append(("Exp1 Baseline (10 drones)", "Baseline",
                 base.copy(num_drones=10, packet_loss=0.0, seed=11), []))

    # Experiment 2 - packet loss
    plan.append(("Exp2 Packet loss 10% (10 drones)", "Packet loss",
                 base.copy(num_drones=10, packet_loss=0.10, seed=12), []))

    # Experiment 3 - drone disconnection (30 s outage in the middle)
    plan.append(("Exp3 Drone disconnection (10 drones)", "Disconnection",
                 base.copy(num_drones=10, packet_loss=0.0, seed=13),
                 [FaultAction(duration * 0.33, "DISCONNECT", "D5"),
                  FaultAction(duration * 0.66, "RECONNECT", "D5")]))

    # Experiment 4 - swarm size (all other parameters identical)
    for size in (5, 10, 20):
        plan.append((f"Exp4 Swarm size {size}", "Swarm size",
                     base.copy(num_drones=size, packet_loss=0.0, seed=14 + size), []))

    # Experiment 5 - communication range
    for radius in (80, 120, 160, 200, 240):
        plan.append((f"Exp5 Range {radius}px (10 drones)", "Range",
                     base.copy(num_drones=10, comm_range=float(radius),
                               packet_loss=0.0, seed=30 + radius), []))

    return plan


def run_all_experiments(duration: float = 90.0,
                         outdir: str = "results",
                         base_port: Optional[int] = None,
                         progress: Optional[Callable[[str, float], None]] = None) -> List[ExperimentResult]:
    """Execute every planned experiment and store the results."""
    os.makedirs(outdir, exist_ok=True)
    results: List[ExperimentResult] = []
    plan = experiment_plan(duration)
    for index, (name, group, config, faults) in enumerate(plan):
        if progress:
            progress(name, index / len(plan))
        if base_port is not None:
            config = config.copy(base_port=base_port)
        result = run_headless(config, duration, faults)
        result.name = name
        result.group = group
        results.append(result)
    if progress:
        progress("done", 1.0)
    return results


# ----------------------------------------------------------------------
# export helpers
# ----------------------------------------------------------------------
FIELD_ORDER = [
    "experiment", "group", "drones", "comm_range", "packet_loss", "delay_ms",
    "message_interval", "seed", "duration_s", "messages_sent", "messages_delivered",
    "messages_dropped", "messages_undeliverable", "pdr_percent", "loss_percent",
    "simulated_drop_percent", "unreachable_percent", "avg_latency_ms", "min_latency_ms",
    "max_latency_ms", "p95_latency_ms", "throughput_bytes_s", "delivered_bytes",
    "active_drones", "disconnected_drones", "active_links", "components",
    "isolated_drones", "largest_component", "mean_hops", "messages_per_second",
    "recovery_time_s", "faults_injected",
]


def export_csv(results: List[ExperimentResult], path: str) -> str:
    """Write every metric of every experiment into a CSV file."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    rows = [result.as_row() for result in results]
    fields = [field for field in FIELD_ORDER if any(field in row for row in rows)]
    extra = sorted({key for row in rows for key in row} - set(fields))
    fields += extra
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def export_dataframe(results: List[ExperimentResult]):
    """Return the results as a pandas DataFrame (optional dependency)."""
    import pandas as pd

    return pd.DataFrame([result.as_row() for result in results])


def short_label(name: str) -> str:
    """Short, unique label for the chart axis."""
    match = re.match(r"Exp(\d+)\s+Swarm size\s+(\d+)", name)
    if match:
        return f"Exp{match.group(1)}\nn={match.group(2)}"
    match = re.match(r"Exp(\d+)\s+Range\s+(\d+)px", name)
    if match:
        return f"Exp{match.group(1)}\nr={match.group(2)}"
    if name.startswith("Exp1"):
        return "Exp1\\nBase"
    if name.startswith("Exp2"):
        return "Exp2\\nLoss"
    if name.startswith("Exp3"):
        return "Exp3\\nDisc"
    return name.split(" ")[0]


def plot_results(results: List[ExperimentResult], outdir: str = "results") -> List[str]:
    """Create the comparison charts of the report."""
    from matplotlib.figure import Figure

    os.makedirs(outdir, exist_ok=True)
    pdr = [r.metrics["pdr_percent"] for r in results]
    latency = [r.metrics["avg_latency_ms"] for r in results]
    throughput = [r.metrics["throughput_bytes_s"] for r in results]
    unreachable = [r.metrics["unreachable_percent"] for r in results]

    paths = []
    short = [short_label(r.name) for r in results]

    figure = Figure(figsize=(15, 9))
    axes = figure.subplots(2, 2)
    figure.suptitle("Drone swarm network performance - experiment comparison")

    axes[0][0].bar(short, pdr, color="#2f9e8f")
    axes[0][0].set_title("Packet Delivery Ratio (%)")
    axes[0][0].set_ylim(0, 105)
    axes[0][0].grid(axis="y", alpha=0.3)

    axes[0][1].bar(short, latency, color="#e08b3c")
    axes[0][1].set_title("Average end-to-end latency (ms)")
    axes[0][1].grid(axis="y", alpha=0.3)

    axes[1][0].bar(short, throughput, color="#4a6fd4")
    axes[1][0].set_title("Throughput (payload bytes / s)")
    axes[1][0].grid(axis="y", alpha=0.3)

    axes[1][1].bar(short, unreachable, color="#b4523f")
    axes[1][1].set_title("Unreachable messages (%)")
    axes[1][1].grid(axis="y", alpha=0.3)

    for axis in axes.flat:
        axis.set_xlabel("experiment")
    figure.tight_layout()
    path = os.path.join(outdir, "experiment_comparison.png")
    figure.savefig(path, dpi=130)
    paths.append(path)

    # connectivity chart for the swarm-size / range experiments
    groups: Dict[str, List[ExperimentResult]] = {}
    for result in results:
        groups.setdefault(result.group, []).append(result)
    for group, items in groups.items():
        if len(items) < 2:
            continue
        labels = [short_label(i.name) for i in items]
        figure = Figure(figsize=(8, 5))
        axis = figure.add_subplot(111)
        axis.plot(labels, [i.metrics["active_links"] for i in items], marker="o", label="active links")
        axis.plot(labels, [i.metrics["components"] for i in items], marker="s", label="components")
        axis.set_title(f"{group} - topology and connectivity")
        axis.legend()
        axis.grid(alpha=0.3)
        figure.tight_layout()
        path = os.path.join(outdir, f"connectivity_{group.replace(' ', '_').lower()}.png")
        figure.savefig(path, dpi=130)
        paths.append(path)

    return paths
