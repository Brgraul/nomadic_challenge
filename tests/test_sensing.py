import json
import math
from dataclasses import asdict, replace

import pytest
from pydantic import TypeAdapter, ValidationError

from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import Geometry, Interval, LinePoint, union_length_mm
from grid_ops.grid import Network, Node, Segment
from grid_ops.sensing import (
    CONTACT_GRID_MM,
    TDR_DURATION_S,
    ConductorPoint,
    ConductorScope,
    ContactCurrent,
    CorridorSpan,
    CurrentPayload,
    CurrentRequest,
    ExecutedAcquisition,
    LidarPass,
    LidarPayload,
    LidarRequest,
    MeasurementRequest,
    OutcomeSpec,
    RequestRejection,
    ScanPose,
    ScanTrajectory,
    SensorCapabilities,
    TaskStatusV2,
    TdrPayload,
    TdrRequest,
    TdrTest,
    compile_requests,
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


def long_corridor_network() -> Network:
    a = Node("A", "boundary_endpoint", "source", "C", (0.0, 0.0))
    b = Node("B", "terminal", "load", "C", (0.089932, 0.0))
    segment = Segment("S", "C", "A", "B", 110.0, 1, 10.0, (a.position, b.position))
    return Network({"A": a, "B": b}, {"S": segment}, {}, {})


REQUEST_COMMON = dict(
    hypothesis_ids=("H1",),
    rationale="the corridor's condition is unknown after the storm",
    alternatives_distinguished=("H1", "H2"),
    expected_outcomes=(OutcomeSpec(description="a reading that separates H1 from H2"),),
    evidence_ids=("SYN_REPORT_02",),
    urgency=3,
    source_revision="rev-7",
    replaces_request_ids=(),
)


def lidar_request(
    request_id="R1", *, segment_id="S", start_mm=100_000, end_mm=600_000, **common
):
    return LidarRequest(
        id=request_id,
        target=CorridorSpan(segment_id=segment_id, start_mm=start_mm, end_mm=end_mm),
        **(REQUEST_COMMON | common),
    )


def current_request(
    request_id="R1", *, circuit_id="S/C1", phase="A", start_mm=0, end_mm=6_000_000, **common
):
    return CurrentRequest(
        id=request_id,
        target=ConductorScope(
            circuit_id=circuit_id, phase=phase, start_mm=start_mm, end_mm=end_mm
        ),
        **(REQUEST_COMMON | common),
    )


def tdr_request(
    request_id="R1",
    *,
    circuit_id="S/C1",
    phase="A",
    start_mm=100_000,
    end_mm=600_000,
    coupling_profile_id="synthetic-live-compatible-v1",
    isolation_evidence_ids=(),
    **common,
):
    return TdrRequest(
        id=request_id,
        target=ConductorScope(
            circuit_id=circuit_id, phase=phase, start_mm=start_mm, end_mm=end_mm
        ),
        coupling_profile_id=coupling_profile_id,
        isolation_evidence_ids=isolation_evidence_ids,
        **(REQUEST_COMMON | common),
    )


def test_compile_lidar_emits_forward_and_reverse_tasks_over_the_requested_span():
    network = analytic_network()
    geometry = Geometry(network)
    capabilities = SensorCapabilities()
    result = compile_requests(network, geometry, capabilities, [lidar_request()])
    assert result.rejections == ()
    assert len(result.tasks) == 2
    forward, reverse = result.tasks
    for task in (forward, reverse):
        assert task.id == ""
        assert task.request_id == "R1"
        assert task.alternative_group_id == "R1"
        assert task.status is TaskStatusV2.PENDING
        assert task.urgency == 3
        assert isinstance(task.acquisition, LidarPass)
        assert task.acquisition.target == Interval("S", 100_000, 600_000)
        assert task.acquisition.trajectory.duration_s == 50
    height_m = capabilities.conductor_height_m + 30.0
    poses = (
        ScanPose(geometry.position_of(LinePoint("S", 100_000)), height_m, 0.0, -60.0),
        ScanPose(geometry.position_of(LinePoint("S", 600_000)), height_m, 0.0, -60.0),
    )
    assert forward.acquisition.trajectory.poses == poses
    assert reverse.acquisition.trajectory.poses == poses[::-1]


def test_compile_lidar_duration_never_drops_below_ten_seconds():
    network = analytic_network()
    result = compile_requests(
        network,
        Geometry(network),
        SensorCapabilities(),
        [lidar_request(start_mm=100_000, end_mm=150_000)],
    )
    assert [task.acquisition.trajectory.duration_s for task in result.tasks] == [10, 10]


def test_compile_contact_covers_two_km_grid_plus_midpoint():
    network = long_corridor_network()
    result = compile_requests(network, Geometry(network), SensorCapabilities(), [current_request()])
    offsets = [task.acquisition.conductor.point.offset_mm for task in result.tasks]
    assert offsets == [
        CONTACT_GRID_MM,
        3 * CONTACT_GRID_MM // 2,
        2 * CONTACT_GRID_MM,
    ]
    task = result.tasks[0]
    assert isinstance(task.acquisition, ContactCurrent)
    assert task.acquisition.conductor == ConductorPoint(LinePoint("S", CONTACT_GRID_MM), "S/C1", "A")
    assert task.acquisition.integration_s == 10
    assert [task.acquisition.conductor.circuit_id for task in result.tasks] == ["S/C1"] * 3


def test_compile_contact_deduplicates_midpoint_landing_on_the_grid():
    network = long_corridor_network()
    result = compile_requests(
        network,
        Geometry(network),
        SensorCapabilities(),
        [current_request(start_mm=0, end_mm=4_000_000)],
    )
    offsets = [task.acquisition.conductor.point.offset_mm for task in result.tasks]
    assert offsets == [CONTACT_GRID_MM]


def test_compile_contact_clips_scope_to_the_segment():
    network = long_corridor_network()
    geometry = Geometry(network)
    length_mm = geometry.length_mm("S")
    result = compile_requests(
        network,
        geometry,
        SensorCapabilities(),
        [current_request(start_mm=5_500_000, end_mm=50_000_000)],
    )
    offsets = [task.acquisition.conductor.point.offset_mm for task in result.tasks]
    assert offsets == sorted(
        {3 * CONTACT_GRID_MM, 4 * CONTACT_GRID_MM, (5_500_000 + length_mm) // 2}
    )
    assert all(5_500_000 < offset < length_mm for offset in offsets)


def test_compile_tdr_tests_the_scope_midpoint_with_the_default_profile():
    network = analytic_network()
    result = compile_requests(network, Geometry(network), SensorCapabilities(), [tdr_request()])
    assert len(result.tasks) == 1
    task = result.tasks[0]
    assert task.request_id == "R1"
    assert task.alternative_group_id == "R1"
    assert task.status is TaskStatusV2.PENDING
    acquisition = task.acquisition
    assert isinstance(acquisition, TdrTest)
    conductor = ConductorPoint(LinePoint("S", 350_000), "S/C1", "A")
    access = tdr_access(conductor)
    assert acquisition.access_site_id == access.id
    assert acquisition.conductor == conductor
    assert acquisition.return_path_id == access.return_path_ids[0]
    assert acquisition.coupling_profile_id == "synthetic-live-compatible-v1"
    assert acquisition.duration_s == TDR_DURATION_S
    assert acquisition.isolation_evidence_ids == ()

    evidenced = compile_requests(
        network,
        Geometry(network),
        SensorCapabilities(),
        [
            tdr_request(
                coupling_profile_id="isolated-only-v1", isolation_evidence_ids=("E5",)
            )
        ],
    )
    assert evidenced.tasks[0].acquisition.isolation_evidence_ids == ("E5",)


def test_compile_names_every_rejection_reason():
    network = analytic_network()
    geometry = Geometry(network)
    requests = [
        lidar_request("R1", segment_id="MISSING"),
        lidar_request("R2", start_mm=2_000_000, end_mm=3_000_000),
        current_request("R3", circuit_id="S/CX"),
        current_request("R4", circuit_id="NOWHERE/C1"),
        tdr_request("R5", coupling_profile_id="radio-v1"),
        tdr_request("R6", coupling_profile_id="isolated-only-v1"),
    ]
    result = compile_requests(network, geometry, SensorCapabilities(), requests)
    assert result.tasks == ()
    reasons = {rejection.request_id: rejection.reason for rejection in result.rejections}
    assert "unknown segment MISSING" in reasons["R1"]
    assert "clips to empty" in reasons["R2"]
    assert "circuit S/CX not on segment S" in reasons["R3"]
    assert "unknown segment NOWHERE" in reasons["R4"]
    assert "unknown coupling profile radio-v1" in reasons["R5"]
    assert reasons["R6"] == "validated isolation evidence required; zero current is insufficient"


def test_compile_rejects_only_the_later_duplicate_request_id():
    network = analytic_network()
    requests = [
        lidar_request("R1", start_mm=0, end_mm=100_000),
        lidar_request("R1", start_mm=200_000, end_mm=300_000),
    ]
    result = compile_requests(network, Geometry(network), SensorCapabilities(), requests)
    assert len(result.tasks) == 2
    assert result.rejections == (
        RequestRejection("R1", "duplicate request id R1 in the same call"),
    )


def test_measurement_request_union_roundtrip_and_strict_discriminator():
    adapter = TypeAdapter(MeasurementRequest)
    for request in (lidar_request(), tdr_request(), current_request()):
        parsed = adapter.validate_json(request.model_dump_json())
        assert parsed == request
        assert isinstance(parsed, type(request))
    payload = json.loads(lidar_request().model_dump_json())
    payload["method"] = "voltage_presence"
    with pytest.raises(ValidationError):
        adapter.validate_json(json.dumps(payload))


@pytest.mark.parametrize(
    "fields",
    [dict(start_mm=100_000, end_mm=100_000), dict(start_mm=600_000, end_mm=100_000)],
)
def test_request_spans_must_ascend(fields):
    with pytest.raises(ValidationError):
        CorridorSpan(segment_id="S", **fields)
    with pytest.raises(ValidationError):
        ConductorScope(circuit_id="S/C1", phase="A", **fields)


@pytest.mark.parametrize("urgency", [0, 6])
def test_request_urgency_stays_within_one_to_five(urgency):
    with pytest.raises(ValidationError):
        lidar_request(urgency=urgency)


def test_conductor_scope_parses_only_known_phases():
    with pytest.raises(ValidationError):
        ConductorScope(circuit_id="S/C1", phase="N", start_mm=0, end_mm=100_000)
