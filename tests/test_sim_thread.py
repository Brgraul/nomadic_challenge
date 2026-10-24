"""Real sim thread: commands round-trip, and a bad command never kills the loop."""

from __future__ import annotations

import time

import pytest

from grid_ops.grid import load_network
from grid_ops.sim import RegisterSite, Simulation
from grid_ops.world import CommandRejected


@pytest.fixture
def running():
    sim = Simulation(load_network(), minutes_per_second=30)
    sim.start()
    yield sim
    sim.stop()


def test_register_site_through_the_thread(running):
    msg = running.submit(RegisterSite("LINE_024", 13_382_000, "operator")).result(timeout=5)
    assert "LINE_024@13382000mm" in msg
    deadline = time.monotonic() + 3
    while "LINE_024@13382000mm" not in running.snapshot.sites and time.monotonic() < deadline:
        time.sleep(0.1)
    assert "LINE_024@13382000mm" in running.snapshot.sites


def test_bad_commands_do_not_kill_the_loop(running):
    with pytest.raises(CommandRejected):
        running.submit(RegisterSite("LINE_024", 999_999_999, "operator")).result(timeout=5)
    with pytest.raises(CommandRejected):
        running.submit(object()).result(timeout=5)  # type: ignore[arg-type]
    clock = running.snapshot.clock
    time.sleep(1.2)
    assert running.snapshot.clock > clock
