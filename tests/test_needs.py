"""Tasks exist only for accepted measurement requests, through the compiled lifecycle."""

from __future__ import annotations

from dataclasses import replace

from fixtures_three_method import (
    StubPhysics,
    contact_task,
    seed_handshakes,
    site_at,
    tiny_network,
    two_drone_world,
)
from helpers_p2 import tiny_sim

from grid_ops import needs
from grid_ops.coordinator import Coordinator
from grid_ops.geometry import LandingSite
from grid_ops.proposals import ProposedStatus
from grid_ops.routes import SortieRequest, apply_sorties, validate_sorties
from grid_ops.sensing import (
    AcceptedRequest,
    ConductorScope,
    CurrentRequest,
    OutcomeSpec,
    RequestStatus,
    TaskStatusV2,
)
from grid_ops.world import (
    LAND_S,
    TAKEOFF_S,
    GroundSample,
    Land,
    ScheduledStep,
    Takeoff,
    Transit,
    advance_to,
    leg_seconds,
)


def world():
    return two_drone_world()


def refresh(w, geo):
    return needs.refresh(w, tiny_network(), geo)


def request_only(w, rid, urgency=3):
    """Register an active request without pre-creating its tasks, so refresh must compile."""
    request = CurrentRequest(
        id=rid,
        hypothesis_ids=(),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=urgency,
        source_revision="0",
        replaces_request_ids=(),
        target=ConductorScope(circuit_id="LINE_040/C1", phase="A", start_mm=0, end_mm=110_000),
    )
    w.requests[rid] = AcceptedRequest(rid, request, RequestStatus.ACTIVE, w.clock_s, "0")
    return request


def run_one_attempt(w, geo, task_id, drone_id="DRONE_1"):
    """Validate, apply and complete one contact attempt, publishing its reading."""
    conductor = w.tasks[task_id].acquisition.conductor
    landing = LandingSite(conductor.point, conductor.circuit_id)
    pos = geo.position_of(conductor.point)
    start = w.drones[drone_id].state.perch
    start_pos = geo.position_of(start.point)
    leg = leg_seconds(start_pos, pos, w.drones[drone_id].speed_km_per_min)
    t0, t1, t2 = TAKEOFF_S, TAKEOFF_S + leg, TAKEOFF_S + leg + LAND_S
    steps = (
        ScheduledStep(0, 0, t0, Takeoff(start, start_pos)),
        ScheduledStep(1, t0, t1, Transit(start_pos, pos)),
        ScheduledStep(2, t1, t2, Land(landing, pos)),
        ScheduledStep(
            3,
            t2,
            t2 + 10,
            GroundSample((w.tasks[task_id].acquisition,), landing, (task_id,), pos),
        ),
    )
    req = SortieRequest(
        drone_id, steps, landing, w.revision, w.energy_model.revision, w.capabilities.revision
    )
    result = validate_sorties(w, geo, None, (req,))
    assert result.ok, [str(r) for r in result.rejections]
    physics = StubPhysics()
    apply_sorties(w, physics, result.plans)
    advance_to(w, physics, steps[-1].end_s)
    return req


# -- accepted requests only ------------------------------------------------------------------------


def test_no_tasks_without_an_accepted_request():
    sim = tiny_sim()
    sim.step(5)
    assert sim.snapshot.tasks == {} and sim.snapshot.requests == {}
    Coordinator(sim, auto_accept=False).run_turn_now()  # a proposal alone is not accepted
    w = sim.snapshot
    assert w.requests == {} and w.tasks == {}
    assert any(p.status is ProposedStatus.PROPOSED_NOT_ACCEPTED for p in w.proposals)


def test_accepted_requests_compile_pending_tasks():
    w, geo = world()
    request_only(w, "REQ-1", urgency=4)
    refresh(w, geo)
    made = [t for t in w.tasks.values() if t.request_id == "REQ-1"]
    assert made and all(t.status is TaskStatusV2.PENDING for t in made)
    t = made[0]
    assert t.assigned_drone_id is None and t.urgency == 4
    assert t.alternative_group_id == "REQ-1"  # the request owns the alternative group
    assert "compiled from request REQ-1" in t.note


def test_refresh_is_idempotent_without_new_information():
    w, geo = world()
    contact_task(w, "T0001", site_at(w, "LINE_040", 0), "REQ-1")
    refresh(w, geo)
    ids = sorted(w.tasks)
    refresh(w, geo)
    refresh(w, geo)
    assert sorted(w.tasks) == ids


def test_an_answered_group_cancels_its_remaining_unstarted_attempts():
    w, geo = world()
    contact_task(w, "T0001", site_at(w, "LINE_040", 50_000), "REQ-1")
    contact_task(w, "T0002", site_at(w, "LINE_040", 100_000), "REQ-1")
    run_one_attempt(w, geo, "T0001")
    assert w.tasks["T0001"].status is TaskStatusV2.COMPLETED and w.tasks["T0001"].reading_id
    notes, _unavailable = refresh(w, geo)
    assert w.tasks["T0002"].status is TaskStatusV2.CANCELLED
    assert any("already answered" in n for n in notes)
    assert w.tasks["T0001"].status is TaskStatusV2.COMPLETED  # completed work and readings stay


def test_a_locked_attempt_survives_refresh_and_locks_its_group():
    w, geo = world()
    contact_task(w, "T0001", site_at(w, "LINE_040", 0), "REQ-1")
    contact_task(w, "T0002", site_at(w, "LINE_040", 50_000), "REQ-1")
    # a reservation is applied through a validated itinerary (test_routes_v2); fake the lock
    w.set_task(replace(w.tasks["T0001"], status=TaskStatusV2.RESERVED, assigned_drone_id="DRONE_1"))
    notes, _unavailable = refresh(w, geo)
    assert w.tasks["T0001"].status is TaskStatusV2.RESERVED
    assert w.tasks["T0002"].status is TaskStatusV2.CANCELLED  # the group is already being served
    assert any("T0002" in n for n in notes)


def test_completed_attempts_stay_completed_through_refresh():
    w, geo = world()
    seed_handshakes(w)
    contact_task(w, "T0001", site_at(w, "LINE_040", 50_000), "REQ-1")
    run_one_attempt(w, geo, "T0001")
    before = dict(w.readings)
    refresh(w, geo)
    assert w.tasks["T0001"].status is TaskStatusV2.COMPLETED
    assert w.readings == before


def test_retired_requests_lose_unstarted_work_but_locked_attempts_finish():
    w, geo = world()
    contact_task(w, "T0001", site_at(w, "LINE_040", 0), "REQ-1")
    # a second unstarted attempt of the same group, without re-accepting the request
    from fixtures_three_method import contact

    from grid_ops.sensing import MeasurementTaskV2

    w.set_task(
        MeasurementTaskV2(
            "T0002", "REQ-1", contact(site_at(w, "LINE_040", 50_000)), "REQ-1",
            TaskStatusV2.PENDING, 3,
        )
    )
    w.set_task(replace(w.tasks["T0001"], status=TaskStatusV2.RESERVED))
    w.requests["REQ-1"] = replace(
        w.requests["REQ-1"], status=RequestStatus.RETIRED, retired_reason="stand down"
    )
    notes, _unavailable = refresh(w, geo)
    assert w.tasks["T0001"].status is TaskStatusV2.RESERVED  # the drone is already flying it
    assert w.tasks["T0002"].status is TaskStatusV2.CANCELLED
    assert any("retired" in n for n in notes)


def test_rejected_requests_are_reported_as_explicitly_unavailable_work():
    from grid_ops.sensing import CorridorSpan, LidarRequest

    w, geo = world()
    request = LidarRequest(
        id="REQ-BAD",
        hypothesis_ids=(),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=2,
        source_revision="0",
        replaces_request_ids=(),
        target=CorridorSpan(segment_id="LINE_999", start_mm=0, end_mm=110_000),
    )
    w.requests["REQ-BAD"] = AcceptedRequest("REQ-BAD", request, RequestStatus.ACTIVE, 0, "0")
    notes, unavailable = refresh(w, geo)
    assert w.tasks == {}  # nothing was compiled for a request that cannot run
    assert [u.request_id for u in unavailable] == ["REQ-BAD"] and all(u.reason for u in unavailable)
    assert notes == []


def test_the_open_task_cap_is_reported_not_hidden():
    w, geo = world()
    for i in range(needs.MAX_OPEN_TASKS):
        request_only(w, f"REQ-{i:02d}")
    request_only(w, "REQ-OVER")
    _notes, unavailable = refresh(w, geo)
    assert len([t for t in w.tasks.values() if t.status is TaskStatusV2.PENDING]) <= (
        needs.MAX_OPEN_TASKS
    )
    assert any(u.request_id == "REQ-OVER" and "cap" in u.reason for u in unavailable)


def test_refresh_never_invents_a_method():
    w, geo = world()
    refresh(w, geo)  # no requests: nothing appears at all
    assert w.tasks == {} and w.unavailable_work == []
    contact_task(w, "T0001", site_at(w, "LINE_040", 0), "REQ-1")
    refresh(w, geo)
    assert {t.acquisition.method.value for t in w.tasks.values()} == {"contact_current"}