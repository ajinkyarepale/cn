"""Central configuration for the drone swarm simulation.

Every tunable parameter of the experiment lives here so that the GUI and the
headless experiment runner can control exactly the same variables.
"""

from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Any, Dict


@dataclass
class SimConfig:
    """Tunable simulation parameters."""

    num_drones: int = 10
    world_width: int = 860
    world_height: int = 580

    # Wireless link parameters
    comm_range: float = 220.0      # pixels
    packet_loss: float = 0.0       # 0.0 - 1.0 (probability per transmission)
    delay_ms: float = 20.0         # modelled one-way link delay

    # Swarm behaviour
    drone_speed: float = 42.0      # pixels per simulated second
    sim_speed: float = 1.0         # simulation / wall-clock time multiplier
    swarm_radius: float = 210.0    # radius of the area the swarm operates in

    # Traffic generation (global rate, so swarm size does not change the load)
    message_interval: float = 1.0  # seconds between two generated messages

    # Networking
    base_port: int = 45600         # first UDP port used by the drones
    host: str = "127.0.0.1"

    # Misc
    seed: int = 7
    max_log: int = 600

    def copy(self, **changes: Any) -> "SimConfig":
        data = asdict(self)
        data.update(changes)
        return SimConfig(**data)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
