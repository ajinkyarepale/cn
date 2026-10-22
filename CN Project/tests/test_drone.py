"""Tests for the drone simulation module."""

import math

import pytest

from drone_swarm.config import SimConfig
from drone_swarm.drone import Drone, Obstacle


def make_drone(drone_id="D1", x=100.0, y=100.0, config=None, rng=None, obstacles=None):
    import random

    config = config or SimConfig()
    rng = rng or random.Random(1)
    return Drone(drone_id, x, y, config, rng, obstacles or [])


def test_distance_and_range():
    config = SimConfig(comm_range=100.0)
    a = make_drone("D1", 0, 0, config)
    b = make_drone("D2", 80, 0, config)
    c = make_drone("D3", 150, 0, config)
    assert a.distance_to(b) == pytest.approx(80.0)
    assert a.in_range(b)
    assert not a.in_range(c)


def test_drone_stays_inside_the_world():
    config = SimConfig(world_width=400, world_height=300)
    drone = make_drone(config=config)
    for _ in range(400):
        drone.update(0.05)
        assert 0 <= drone.x <= config.world_width
        assert 0 <= drone.y <= config.world_height


def test_drone_moves_towards_its_target():
    drone = make_drone()
    start = (drone.x, drone.y)
    drone.update(0.5)
    assert math.hypot(drone.x - start[0], drone.y - start[1]) > 0


def test_drone_avoids_obstacles():
    obstacle = Obstacle(x=200.0, y=200.0, radius=60.0)
    drone = make_drone(x=200.0, y=120.0, obstacles=[obstacle])
    drone.target_x, drone.target_y = 200.0, 320.0
    for _ in range(120):
        drone.update(0.05)
        assert math.hypot(drone.x - obstacle.x, drone.y - obstacle.y) > obstacle.radius


def test_disconnected_drone_does_not_move():
    drone = make_drone()
    drone.status = "DISCONNECTED"
    position = (drone.x, drone.y)
    drone.update(1.0)
    assert (drone.x, drone.y) == position
