"""Behavior tests for the decision transaction on the three-method World."""

from __future__ import annotations

from dataclasses import dataclass, replace

import pytest
from fixtures_three_method import (
    StubPhysics,
    contact_task,
    seed_handshakes,
    site_at,
    tiny_network,
    two_drone_world,
)

from grid_ops import decisions, needs
from grid_ops.geometry import Interval, LinePoint
from grid_ops.hypotheses import (
    HypothesisModel,
    ManagerOutput,
    PredictionModel,
    RevisionModel,
    SpanModel,
)
from grid_ops.proposals import ProposedFinding, ProposedStatus
from grid_ops.routes import SortieRequest, apply_sorties, validate_sorties
from grid_ops.sensing import (
    AcquisitionQuality,
    ConductorScope,
    CurrentRequest,
    LidarPass,
    LidarPayload,
    OutcomeSpec,
    Reading,
    RequestStatus,
    ScanPose,
    ScanTrajectory,
    TaskStatusV2,
)
from grid_ops.world import (
    DEADLINE_S,
    LAND_S,
    TAKEOFF_S,
    Executing,
    GroundSample,
    Land,
    Perched,
    ScheduledStep,
    Takeoff,
    Transit,
    advance_to,
    leg_seconds,
)


@dataclass(frozen=True)
class Cmd:
    finding_id: str
    semantic_key: str
    decision_id: str


def fresh_world():
    world, geo = two_drone_world()
    world.requests_version = 0
    return world, geo, tiny_network()


def request(rid="REQ-1", urgency=5, replaces=()):
    return CurrentRequest(
        id=rid,
        hypothesis_ids=(),
        rationale="distinguish the cause",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=urgency,
        source_revision="0",
        replaces_request_ids=replaces,
        target=ConductorScope(circuit_id="LINE_040/C1", phase="A", start_mm=0, end_mm=1_000_000),
    )


def proposal(world, output: ManagerOutput, fid="F001") -> ProposedFinding:
    pf = ProposedFinding(
        id=fid,
        snapshot_revision=world.revision,
        author="manager",
        status=ProposedStatus.PROPOSED_NOT_ACCEPTED,
        output=output,
        created_s=world.clock_s,
        semantic_key=decisions.semantic_key(world, output),
    )
    world.proposals.append(pf)
    return pf


def commit(world, output, fid="F001", did="D001"):
    pf = proposal(world, output, fid)
    ctx = {"pf": pf}
    cmd = Cmd(fid, pf.semantic_key, did)
    cand = decisions.apply_candidate(world, ctx, cmd, cycle=1, clock_s=world.clock_s)
    decisions.commit_apply(world, cand, ctx, cmd, (), physics=None)
    return cand


def test_accept_request_and_compile_tasks():
    world, geo, net = fresh_world()
    commit(world, ManagerOutput(summary="s", measurement_requests=[request()]))
    accepted = world.requests["REQ-1"]
    assert accepted.status is RequestStatus.ACTIVE
    assert accepted.accepted_s == world.clock_s
    assert accepted.source_revision == "0"
    assert world.requests_version == 1
    _notes, unavailable = needs.refresh(world, net, geo)
    assert not unavailable
    made = [t for t in world.tasks.values() if t.request_id == "REQ-1"]
    assert made and all(t.status is TaskStatusV2.PENDING for t in made)


def test_retire_cancels_pending_tasks():
    world, geo, net = fresh_world()
    commit(world, ManagerOutput(summary="s", measurement_requests=[request()]))
    _notes, _ = needs.refresh(world, net, geo)
    pending = [t.id for t in world.tasks.values() if t.status is TaskStatusV2.PENDING]
    assert pending
    commit(
        world,
        ManagerOutput(summary="stand down", retire_request_ids=["REQ-1"]),
        fid="F002",
        did="D002",
    )
    assert world.requests["REQ-1"].status is RequestStatus.RETIRED
    assert "stand down" in world.requests["REQ-1"].retired_reason
    assert world.requests_version == 2
    _notes, _ = needs.refresh(world, net, geo)
    assert all(world.tasks[tid].status is TaskStatusV2.CANCELLED for tid in pending)
    assert any("retired" in world.tasks[tid].note for tid in pending)


def test_replace_then_old_retired_with_reason():
    world, _geo, _net = fresh_world()
    commit(world, ManagerOutput(summary="s", measurement_requests=[request("REQ-1")]))
    commit(
        world,
        ManagerOutput(summary="s2", measurement_requests=[request("REQ-2", replaces=("REQ-1",))]),
        fid="F002",
        did="D002",
    )
    assert world.requests["REQ-1"].status is RequestStatus.RETIRED
    assert world.requests["REQ-1"].retired_reason == "replaced by REQ-2"
    assert world.requests["REQ-2"].status is RequestStatus.ACTIVE


def test_duplicate_active_request_id_rejected():
    world, _geo, _net = fresh_world()
    commit(world, ManagerOutput(summary="s", measurement_requests=[request("REQ-1")]))
    with pytest.raises(decisions.DecisionRejected):
        commit(world, ManagerOutput(summary="s", measurement_requests=[request("REQ-1")]), fid="F002")


def test_semantic_key_stability():
    world, geo, _net = fresh_world()
    world.hypotheses["H1"] = {"id": "H1", "priority": 3, "lifecycle": "ACTIVE"}

    def revision():
        hyp = HypothesisModel(
            id="H1",
            claim="c",
            scope=[SpanModel(segment_id="LINE_040", start_km=0.0, end_km=1.0)],
            predictions=[
                PredictionModel(id="P1", statement="s", measurement="current", expected="e")
            ],
            assessment="UNRESOLVED",
            priority=3,
        )
        return RevisionModel(
            hypothesis_id="H1",
            change_kinds=["assessment"],
            before=None,
            after=hyp,
            finding_summary="f",
            update_reason="u",
            remaining_uncertainty="r",
            trigger_kind="measurement",
            trigger_event_ids=["E1"],
        )

    out = ManagerOutput(summary="s", revisions=[revision()])
    key1 = decisions.semantic_key(world, out)
    assert key1 == decisions.semantic_key(world, out)
    world.readings["R0001"] = lidar_reading(geo)
    key2 = decisions.semantic_key(world, out)
    assert key2 != key1


def lidar_reading(geo):
    iv = Interval("LINE_040", 0, 700_000)
    poses = (
        ScanPose(geo.position_of(LinePoint("LINE_040", 0)), 50.0, 0.0),
        ScanPose(geo.position_of(LinePoint("LINE_040", 700_000)), 50.0, 0.0),
    )
    acq = LidarPass(iv, ScanTrajectory(poses, 100))
    payload = LidarPayload(
        inspected=(iv,), missing=(), clearance_m=1.2, uncertainty_m=0.05, resolution_mm=10_000
    )
    return Reading(
        id="R0001",
        task_id="T0001",
        request_ids=(),
        drone_id="DRONE_1",
        acquisition=acq,
        acquired_start_s=0,
        acquired_end_s=100,
        payload=payload,
        quality=AcquisitionQuality(True),
    )


def steps_for(world, geo, drone_id: str, site, task_id: str):
    from fixtures_three_method import contact
    pos = geo.position_of(site.point)
    start = world.drones[drone_id].state.perch
    start_pos = geo.position_of(start.point)
    leg = leg_seconds(start_pos, pos, world.drones[drone_id].speed_km_per_min)
    t0, t1, t2 = TAKEOFF_S, TAKEOFF_S + leg, TAKEOFF_S + leg + LAND_S
    return (
        ScheduledStep(0, 0, t0, Takeoff(start, start_pos)),
        ScheduledStep(1, t0, t1, Transit(start_pos, pos)),
        ScheduledStep(2, t1, t2, Land(site, pos)),
        ScheduledStep(
            3, t2, t2 + 10, GroundSample((contact(site),), site, (task_id,), pos)
        ),
    ), site


def run_itinerary(world, geo, drone_id="DRONE_1", until_s=45):
    site = site_at(world, "LINE_025", 500_000)
    contact_task(world, "T0001", site, "REQ-1")
    steps, final = steps_for(world, geo, drone_id, site, "T0001")
    req = SortieRequest(
        drone_id, steps, final, world.revision, world.energy_model.revision,
        world.capabilities.revision,
    )
    result = validate_sorties(world, geo, None, (req,))
    assert result.ok, [str(r) for r in result.rejections]
    physics = StubPhysics()
    apply_sorties(world, physics, result.plans)
    advance_to(world, physics, until_s)
    return site, physics


def test_route_only_continuation_keeps_mid_itinerary_drone():
    world, geo, _net = fresh_world()
    seed_handshakes(world)
    run_itinerary(world, geo)
    assert isinstance(world.drones["DRONE_1"].state, Executing)
    msg, problems = decisions.route_only_continuation(world, geo, None, StubPhysics())
    assert (msg, problems) == ("route-only continuation (unchanged routes)", [])
    assert isinstance(world.drones["DRONE_1"].state, Executing)
    assert world.tasks["T0001"].status is TaskStatusV2.RESERVED


def test_perched_drone_needs_no_continuation_and_stays_put():
    world, geo, _net = fresh_world()
    before = world.drones["DRONE_1"].position
    msg, _problems = decisions.route_only_continuation(world, geo, None, StubPhysics())
    assert msg == "none needed"
    assert world.drones["DRONE_1"].position == before
    assert isinstance(world.drones["DRONE_1"].state, Perched)


def test_landing_only_when_continuation_no_longer_validates():
    world, geo, _net = fresh_world()
    seed_handshakes(world)
    run_itinerary(world, geo)
    world.requests["REQ-1"] = replace(
        world.requests["REQ-1"], status=RequestStatus.RETIRED, retired_reason="withdrawn"
    )
    msg, problems = decisions.route_only_continuation(world, geo, None, StubPhysics())
    assert msg == "route-only continuation (landing-only)", problems
    advance_to(world, StubPhysics(), DEADLINE_S)
    st = world.drones["DRONE_1"].state
    assert isinstance(st, Perched)
    assert not any(
        h.segment_id == st.perch.point.segment_id
        and h.start_mm <= st.perch.point.offset_mm <= h.end_mm
        for h in world.hazards
    )
    assert world.tasks["T0001"].status is TaskStatusV2.PENDING
    assert world.tasks["T0001"].assigned_drone_id is None
