"""Dynamic network topology and shortest-hop (BFS) route discovery.

The swarm is represented as an undirected graph:

* every operational drone is a **node**,
* every usable wireless link between two drones is an **edge**.

The graph is rebuilt from scratch on every simulation tick, because the
drones keep moving and links can be switched off by the failure module.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Set, Tuple

import networkx as nx

from .config import SimConfig
from .drone import Drone


class Topology:
    """Maintains the live communication graph of the swarm."""

    def __init__(self, config: SimConfig) -> None:
        self.config = config
        self.graph = nx.Graph()
        self.failed_links: Set[Tuple[str, str]] = set()

    # ------------------------------------------------------------------
    # graph construction
    # ------------------------------------------------------------------
    def rebuild(self, drones: Dict[str, Drone], failures=None) -> nx.Graph:
        """Recompute the topology from the current drone positions.

        A link exists when the two drones are within the communication range
        of each other and the link has not been disabled by the failure
        module.  Disconnected drones are removed from the graph entirely.
        """
        graph = nx.Graph()
        active = [d for d in drones.values() if d.status == "ACTIVE"]

        for drone in active:
            graph.add_node(drone.drone_id, x=drone.x, y=drone.y, port=drone.port)

        for i in range(len(active)):
            for j in range(i + 1, len(active)):
                a, b = active[i], active[j]
                if not a.in_range(b):
                    continue
                if failures is not None and failures.link_disabled(a.drone_id, b.drone_id):
                    continue
                graph.add_edge(a.drone_id, b.drone_id, distance=a.distance_to(b))

        self.graph = graph
        return graph

    # ------------------------------------------------------------------
    # queries
    # ------------------------------------------------------------------
    def has_link(self, src: str, dst: str) -> bool:
        return self.graph.has_edge(src, dst)

    def neighbours(self, drone_id: str) -> List[str]:
        return list(self.graph.neighbors(drone_id)) if drone_id in self.graph else []

    def find_route(self, src: str, dst: str) -> Optional[List[str]]:
        """Return the fewest-hop path from ``src`` to ``dst`` (BFS).

        Returns ``None`` when the destination is unreachable.
        """
        if src not in self.graph or dst not in self.graph:
            return None
        try:
            # networkx uses breadth-first search for unweighted graphs.
            return list(nx.shortest_path(self.graph, src, dst))
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None

    def connected_components(self) -> List[Set[str]]:
        return [set(component) for component in nx.connected_components(self.graph)]

    def isolated_drones(self) -> Set[str]:
        return {node for node in self.graph.nodes if self.graph.degree(node) == 0}

    def link_count(self) -> int:
        return self.graph.number_of_edges()

    def node_count(self) -> int:
        return self.graph.number_of_nodes()

    def connected_pairs(self) -> Set[frozenset]:
        """All directly connected drone pairs (used by the recovery analysis)."""
        return {frozenset(edge) for edge in self.graph.edges()}

    def reachable(self, src: str, dst: str) -> bool:
        return self.find_route(src, dst) is not None
