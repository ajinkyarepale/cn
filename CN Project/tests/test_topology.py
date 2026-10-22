"""Tests for the topology and the BFS route discovery."""

import pytest

from drone_swarm.config import SimConfig
from drone_swarm.drone import Drone
from drone_swarm.failures import FailureManager
from drone_swarm.topology import Topology

import random


def build_drones(layout, comm_range=100.0):
    config = SimConfig(comm_range=comm_range)
    drones = {}
    for index, (drone_id, position) in enumerate(layout.items()):
        drones[drone_id] = Drone(drone_id, position[0], position[1], config,
                                 random.Random(index))
    return drones


def test_link_is_created_inside_the_range():
    drones = build_drones({"D1": (0, 0), "D2": (50, 0), "D3": (400, 0)})
    topology = Topology(SimConfig(comm_range=100.0))
    topology.rebuild(drones)
    assert topology.has_link("D1", "D2")
    assert not topology.has_link("D1", "D3")
    assert topology.link_count() == 1


def test_bfs_route_finds_the_shortest_path():
    # D1 - D2 - D3 and D1 - D4 - D5
    drones = build_drones({
        "D1": (0, 0), "D2": (80, 0), "D3": (160, 0),
        "D4": (0, 80), "D5": (80, 80),
    })
    topology = Topology(SimConfig(comm_range=100.0))
    topology.rebuild(drones)
    route = topology.find_route("D1", "D3")
    assert route == ["D1", "D2", "D3"]
    assert len(route) == 3


def test_route_is_none_when_the_network_is_split():
    drones = build_drones({"D1": (0, 0), "D2": (1000, 0)})
    topology = Topology(SimConfig(comm_range=100.0))
    topology.rebuild(drones)
    assert topology.find_route("D1", "D2") is None
    assert len(topology.connected_components()) == 2


def test_disconnected_drone_is_removed_from_the_graph():
    drones = build_drones({"D1": (0, 0), "D2": (50, 0)})
    drones["D2"].status = "DISCONNECTED"
    topology = Topology(SimConfig(comm_range=100.0))
    topology.rebuild(drones)
    assert topology.node_count() == 1
    assert not topology.has_link("D1", "D2")


def test_disabled_link_is_removed_from_the_graph():
    # D1 - D2 - D3  and  D1 - D4 - D3  (two alternative routes)
    drones = build_drones({"D1": (0, 0), "D2": (60, 0), "D3": (120, 0),
                           "D4": (60, 70)})
    failures = FailureManager()
    failures.disable_link("D1", "D2", 0.0)
    topology = Topology(SimConfig(comm_range=100.0))
    topology.rebuild(drones, failures)
    assert not topology.has_link("D1", "D2")
    assert topology.has_link("D2", "D3")
    # the alternative route over D4 is used instead
    assert topology.find_route("D1", "D3") == ["D1", "D4", "D3"]


def test_isolated_drones_are_detected():
    drones = build_drones({"D1": (0, 0), "D2": (500, 0)})
    topology = Topology(SimConfig(comm_range=100.0))
    topology.rebuild(drones)
    assert topology.isolated_drones() == {"D1", "D2"}
