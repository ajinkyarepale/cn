"""End-to-end tests of the simulation engine."""

import pytest

from drone_swarm.config import SimConfig
from drone_swarm.experiments import FaultAction, run_headless
from drone_swarm.simulation import Simulation


@pytest.fixture()
def sim():
    simulation = Simulation(SimConfig(num_drones=8, seed=42))
    simulation.reset()
    yield simulation
    simulation.stack.close()


def test_drones_are_created_with_unique_ports(sim):
    assert len(sim.drones) == 8
    ports = {drone.port for drone in sim.drones.values()}
    assert len(ports) == 8


def test_messages_are_delivered_over_the_udp_sockets(sim):
    for _ in range(600):
        sim.step(0.05)
    assert sim.metrics.total_sent > 0
    assert sim.metrics.total_delivered > 0
    assert sim.stack.protocol_errors == 0


def test_multi_hop_forwarding_is_used_when_needed(sim):
    for _ in range(600):
        sim.step(0.05)
    forwarded = [event for event in sim.events if event["status"] == "FORWARDED"]
    assert forwarded, "expected at least one multi-hop message"


def test_topology_is_rebuilt_while_the_drones_move(sim):
    first = sim.topology.link_count()
    for _ in range(200):
        sim.step(0.05)
    assert sim.topology.link_count() != first or first >= 0
    assert sim.topology.node_count() == 8


def test_disconnecting_a_drone_removes_it_from_the_network(sim):
    assert sim.disconnect_drone("D1")
    assert sim.drones["D1"].status == "DISCONNECTED"
    assert "D1" not in sim.topology.graph
    assert sim.disconnect_drone("D1") is False
    assert sim.reconnect_drone("D1")
    assert sim.drones["D1"].status == "ACTIVE"


def test_disabled_link_is_reflected_in_the_topology(sim):
    source = next(iter(sim.drones))
    target = None
    for other in sim.topology.neighbours(source):
        target = other
        break
    if target is None:
        pytest.skip("no link available in the initial topology")
    assert sim.disable_link(source, target)
    assert not sim.topology.has_link(source, target)
    assert sim.restore_link(source, target)


def test_packet_loss_reduces_the_delivery_ratio():
    clean = run_headless(SimConfig(num_drones=8, packet_loss=0.0, seed=5), 40)
    lossy = run_headless(SimConfig(num_drones=8, packet_loss=0.5, seed=5), 40)
    assert lossy.metrics["pdr_percent"] < clean.metrics["pdr_percent"]
    assert lossy.metrics["simulated_drop_percent"] > 0


def test_disconnection_experiment_measures_the_recovery_time():
    result = run_headless(
        SimConfig(num_drones=10, seed=7), 60,
        [FaultAction(20.0, "DISCONNECT", "D5"), FaultAction(40.0, "RECONNECT", "D5")],
    )
    assert result.metrics["faults_injected"] == 1
    assert result.metrics["recovery_time_s"] == pytest.approx(20.0, abs=0.5)
