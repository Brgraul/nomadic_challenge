import json
import math
from dataclasses import asdict, replace

import pytest

from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import Geometry, Interval, LinePoint, union_length_mm
from grid_ops.grid import Network, Node, Segment
from grid_ops.sensing import (
    ConductorPoint,
    ContactCurrent,
    CurrentPayload,
    ExecutedAcquisition,
    LidarPass,
    LidarPayload,
    ScanPose,
    ScanTrajectory,
    SensorCapabilities,
    TdrPayload,
    TdrTest,
    echo_distance_interval,
    tdr_access,
)
from grid_ops.truth.episode import EpisodeSpec, build_episode
from grid_ops.truth.local import LidarScene, SceneOccluder


def analytic_network():
    a = Node("A", "boundary_endpoint", "source", "C", (0.0, 0.0))
    b = Node("B", "terminal", "load", "C", (0.0089932, 0.0))
    segment = Segment("S", "C", "A", "B", 110.0, 1, 1.0, (a.position, b.position))
    return Network({"A": a, "B": b}, {"S": segment}, {}, {})


@pytest.fixture(scope="module")
def episode():
    network = analytic_network()
    registry = CircuitRegistry(network, {})
    geometry = Geometry(network)
    cap = replace(
        SensorCapabilities(),
        systematic_fraction=1.0,
        lidar_fov_deg=120.0,
        lidar_range_m=500.0,
        lidar_resolution_mm=1000,
    )
    return build_episode(
        network, registry, geometry, EpisodeSpec(7, 11, "S", 0.6, "opaque-run"), capabilities=cap
    )


def executed(acquisition, identity="attempt-1", *, landed=True, fraction=1.0):
    duration = (
        acquisition.trajectory.duration_s
        if isinstance(acquisition, LidarPass)
        else (
            acquisition.integration_s
            if isinstance(acquisition, ContactCurrent)
            else acquisition.duration_s
        )
    )
    return ExecutedAcquisition(
        identity,
        acquisition,
        0,
        duration,
        "D",
        acquisition.conductor if landed and not isinstance(acquisition, LidarPass) else None,
        completed_fraction=fraction,
    )


def scan(episode, *, look=0.0, fraction=1.0):
    n = episode.geometry.length_mm("S")
    poses = (ScanPose((0.0, -0.0005), 50.0, look), ScanPose((0.0089932, -0.0005), 50.0, look))
    acquisition = LidarPass(Interval("S", 0, n), ScanTrajectory(poses, 100))
    return executed(acquisition, fraction=fraction)


def test_public_view_excludes_selected_episode_and_noise_seeds(episode):
    raw = json.dumps(asdict(episode.public))
    assert "episode_seed" not in raw and "noise_seed" not in raw and "fault_segment_id" not in raw
    other = build_episode(
        episode.network,
        episode.registry,
        episode.geometry,
        EpisodeSpec(19, 99, "S", 0.5, "opaque-run"),
        capabilities=episode.public.capabilities,
    )
    assert episode.public == other.public
    assert episode.fault.offset_mm != other.fault.offset_mm


def test_causal_chain_has_physical_open_protection_and_loss_of_supply(episode):
    assert episode.fault.occurred_s < episode.protection.opened_s < 0
    assert episode.newly_unsupplied
    assert not episode.net.res_bus.vm_pu.dropna().empty
    assert episode.net.res_bus.vm_pu.dropna().between(0.85, 1.15).all()


def test_current_is_single_landed_conductor_without_other_electrical_quantities(episode):
    acquisition = ContactCurrent(ConductorPoint(LinePoint("S", 800_000), "S/C1"))
    sample = episode.acquire(executed(acquisition))
    assert isinstance(sample.payload, CurrentPayload) and sample.quality.valid
    raw = json.dumps(asdict(sample))
    for name in ("voltage", "p_mw", "q_mvar", "energized", "S/C2"):
        assert name not in raw
    invalid = episode.acquire(executed(acquisition, landed=False))
    assert not invalid.quality.valid and not invalid.quality.contact_ok


def test_current_uses_connected_subspans_not_whole_line_terminal(episode):
    before = ConductorPoint(LinePoint("S", 50_000), "S/C1")
    after = ConductorPoint(LinePoint("S", 800_000), "S/C1")
    assert episode.current_at(before) > 0
    assert episode.current_at(after) == 0


def test_lidar_motion_fov_and_progressive_coverage(episode):
    full = episode.acquire(scan(episode))
    partial = episode.acquire(scan(episode, fraction=0.25))
    assert isinstance(full.payload, LidarPayload) and full.quality.valid
    assert union_length_mm(list(partial.payload.inspected)) < union_length_mm(
        list(full.payload.inspected)
    )
    facing_away = episode.acquire(scan(episode, look=180.0))
    assert not facing_away.quality.valid
    assert facing_away.payload.clearance_m is None
    assert facing_away.payload.missing == (Interval("S", 0, episode.geometry.length_mm("S")),)


def test_occlusion_is_unknown_not_clean_negative(episode):
    old = episode.scene
    episode.scene = LidarScene(
        occluders=(SceneOccluder((-0.01, -0.0004), (0.02, -0.0001), 0, 100),)
    )
    try:
        sample = episode.acquire(scan(episode))
        assert not sample.quality.valid and sample.payload.clearance_m is None
        assert not sample.payload.inspected and sample.payload.missing
    finally:
        episode.scene = old


def test_tdr_known_protector_delay_covers_distance_without_fault_location(episode):
    conductor = ConductorPoint(LinePoint("S", 0), "S/C1")
    access = tdr_access(conductor)
    acquisition = TdrTest(access.id, conductor, access.return_path_ids[0])
    sample = episode.acquire(executed(acquisition))
    assert isinstance(sample.payload, TdrPayload) and sample.quality.valid
    assert sample.payload.echoes
    echo = sample.payload.echoes[0]
    distance = episode.protection.offset_mm / 1000
    lo, hi = echo_distance_interval(echo, sample.payload)
    assert lo <= distance <= hi
    assert math.isclose(echo.delay_s, 2 * distance / sample.payload.propagation_mps, abs_tol=2e-7)
    raw = json.dumps(asdict(sample))
    assert "fault" not in raw and "offset_mm" not in raw and "circuit_id" not in raw


def test_tdr_return_and_isolation_cannot_be_replaced_by_zero_current(episode):
    conductor = ConductorPoint(LinePoint("S", 800_000), "S/C1")
    access = tdr_access(conductor)
    no_return = TdrTest(access.id, conductor, "")
    assert not episode.acquire(executed(no_return)).quality.valid
    isolated = TdrTest(access.id, conductor, access.return_path_ids[0], "isolated-only-v1")
    sample = episode.acquire(executed(isolated))
    assert not sample.quality.valid and "isolation" in sample.quality.reason
    assert not sample.payload.echoes


def test_replay_and_correlated_repeats(episode):
    acquisition = ContactCurrent(ConductorPoint(LinePoint("S", 50_000), "S/C1"))
    one = episode.acquire(executed(acquisition, "one"))
    two = episode.acquire(executed(acquisition, "two"))
    assert one.payload == two.payload
    assert one.quality.correlation_group == two.quality.correlation_group
    assert one == episode.acquire(executed(acquisition, "one"))


def test_symmetric_tdr_branch_locations_remain_observationally_equivalent():
    a = Node("A", "boundary_endpoint", "source", "C", (0.0, -0.009))
    j = Node("J", "terminal", "junction", "C", (0.0, 0.0))
    b = Node("B", "terminal", "left", "C", (-0.009, 0.0))
    c = Node("C", "terminal", "right", "C", (0.009, 0.0))
    nodes = {x.id: x for x in (a, j, b, c)}
    segments = {
        "IN": Segment("IN", "C", "A", "J", 110.0, 1, 1.0, (a.position, j.position)),
        "LEFT": Segment("LEFT", "C", "J", "B", 110.0, 1, 1.0, (j.position, b.position)),
        "RIGHT": Segment("RIGHT", "C", "J", "C", 110.0, 1, 1.0, (j.position, c.position)),
    }
    network = Network(nodes, segments, {}, {})
    geometry = Geometry(network)
    registry = CircuitRegistry(network, {})
    cap = replace(SensorCapabilities(), tdr_delay_sigma_s=0.0)
    left = build_episode(
        network, registry, geometry, EpisodeSpec(7, 11, "LEFT", 0.6), capabilities=cap
    )
    right = build_episode(
        network, registry, geometry, EpisodeSpec(7, 11, "RIGHT", 0.6), capabilities=cap
    )
    conductor = ConductorPoint(LinePoint("IN", 500_000), "IN/C1")
    access = tdr_access(conductor)
    acquisition = TdrTest(access.id, conductor, access.return_path_ids[0])
    left_sample = left.acquire(executed(acquisition))
    right_sample = right.acquire(executed(acquisition))
    assert left_sample.payload.echoes == right_sample.payload.echoes
    assert len(left_sample.payload.echoes) >= 2


def test_tdr_path_budget_is_explicitly_incomplete_not_clean_negative(episode):
    old = episode.public
    episode.public = replace(old, capabilities=replace(old.capabilities, tdr_max_paths=1))
    try:
        conductor = ConductorPoint(LinePoint("S", 50_000), "S/C1")
        access = tdr_access(conductor)
        acquisition = TdrTest(access.id, conductor, access.return_path_ids[0])
        sample = episode.acquire(executed(acquisition))
        assert sample.payload.path_truncated
        assert not sample.quality.valid
        assert "partial" in sample.quality.reason
    finally:
        episode.public = old


@pytest.mark.parametrize(
    "change",
    [
        dict(lidar_max_cells=0),
        dict(tdr_max_paths=-1),
        dict(propagation_mps=0),
        dict(systematic_fraction=1.1),
    ],
)
def test_sensor_profile_rejects_invalid_caps(change):
    with pytest.raises(ValueError):
        replace(SensorCapabilities(), **change)
