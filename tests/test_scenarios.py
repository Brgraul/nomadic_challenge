"""Seeded hidden episodes: deterministic construction, public context and paired public views.

The episode and its noise seed are private; only the public fixture seed selects them. These
tests hold the P1 guarantees in episode form: the same spec rebuilds the same hidden episode,
the public view carries no private handle, and two simulations that differ only in their private
episode construction are publicly indistinguishable before any measurement.
"""

from __future__ import annotations

import dataclasses

import pytest
from fixtures_three_method import tiny_network
from helpers_p2 import tiny_sim

import grid_ops.sim as sim_module
from grid_ops.circuits import build_registry
from grid_ops.geometry import Geometry, Interval, LinePoint
from grid_ops.grid import Network, Node
from grid_ops.scenario import ScenarioInfeasible
from grid_ops.sensing import ExecutedAcquisition, LidarPass, ScanPose, ScanTrajectory
from grid_ops.truth import EpisodeSpec, build_episode
from grid_ops.truth.build import choose_sources


@pytest.fixture(scope="module")
def env():
    net = tiny_network()
    return net, build_registry(net), Geometry(net)


def corridor_acquisition(geo, episode, acquisition_id="ACQ1") -> ExecutedAcquisition:
    """A full-segment lidar pass over the faulted segment, as a drone would fly it."""
    seg = episode.fault.circuit_id.split("/")[0]
    start, end = 0, geo.length_mm(seg)
    poses = tuple(
        ScanPose(geo.position_of(LinePoint(seg, mm)), 50.0, 0.0, -60.0) for mm in (start, end)
    )
    acq = LidarPass(Interval(seg, start, end), ScanTrajectory(poses, max(1, (end - start) // 10)))
    return ExecutedAcquisition(acquisition_id, acq, 0, acq.trajectory.duration_s, "DRONE_1")


def test_the_same_spec_rebuilds_the_same_fault_and_protection(env):
    net, registry, geometry = env
    first = build_episode(net, registry, geometry, EpisodeSpec(11, 5)).manifest()
    again = build_episode(net, registry, geometry, EpisodeSpec(11, 5)).manifest()
    assert first == again
    assert set(first) >= {"episode_spec", "fault", "protection", "newly_unsupplied_buses"}
    fault_segment = first["fault"]["circuit_id"].split("/")[0]
    assert fault_segment in net.segments
    assert {c.split("/")[0] for c in first["protection"]["circuit_ids"]} == {fault_segment}
    assert first["newly_unsupplied_buses"]  # the protection action actually islands load


def test_a_different_spec_builds_a_different_valid_episode(env):
    net, registry, geometry = env
    base = build_episode(net, registry, geometry, EpisodeSpec(11, 5))
    other = build_episode(net, registry, geometry, EpisodeSpec(12, 5)).manifest()
    assert other["episode_spec"]["episode_seed"] == 12
    assert other["fault"]["offset_mm"] != base.manifest()["fault"]["offset_mm"]
    assert other["newly_unsupplied_buses"]


def test_acquire_is_stable_for_the_same_public_inputs_and_noise_seed(env):
    net, registry, geometry = env
    ep1 = build_episode(net, registry, geometry, EpisodeSpec(11, 5))
    ep2 = build_episode(net, registry, geometry, EpisodeSpec(11, 5))
    executed = corridor_acquisition(geometry, ep1)
    first = ep1.acquire(executed)
    assert ep1.acquire(executed) == first  # a repeat is the same realisation
    assert ep2.acquire(executed) == first  # a rebuilt episode agrees
    other_noise = build_episode(net, registry, geometry, EpisodeSpec(11, 6)).acquire(executed)
    assert other_noise.payload != first.payload  # the noise seed is private and effective


def test_the_public_episode_view_carries_no_private_fields(env):
    net, registry, geometry = env
    episode = build_episode(net, registry, geometry, EpisodeSpec(11, 5))
    fields = dataclasses.asdict(episode.public)
    assert set(fields) == {
        "run_id",
        "fixture_id",
        "fixture_version",
        "policy_seed",
        "capabilities",
        "assumptions",
    }
    assert not any("seed" in name for name in fields if name != "policy_seed")
    assert "noise_seed" not in dataclasses.asdict(episode.public)["capabilities"]


def test_choose_sources_raises_scenario_infeasible_on_a_source_less_network():
    net = tiny_network()
    assert choose_sources(net) == {"K1": "N1"}  # the abstract substation declares the source
    nodes = {
        node_id: (
            Node(n.id, "pole", None, n.component, n.position)
            if n.id == "N1"
            else n
        )
        for node_id, n in net.nodes.items()
    }
    sourceless = Network(nodes=nodes, segments=dict(net.segments), areas={}, reports={})
    with pytest.raises(ScenarioInfeasible) as err:
        choose_sources(sourceless)
    assert err.value.diagnostics["source_less_components"] == ["K1"]


def test_paired_episode_public_views_are_equal_before_any_measurement(monkeypatch):
    """Same fixture, different private episode seeds: the public pre-measurement views match."""
    a = tiny_sim()
    seeds = iter(((111, 222), (333, 444)))
    monkeypatch.setattr(sim_module, "_private_seeds", lambda policy_seed: next(seeds))
    b = tiny_sim()
    a.step(3)
    b.step(3)
    assert a.snapshot == b.snapshot