"""Behavior tests for itinerary validation and application."""

from __future__ import annotations

from fixtures_three_method import StubPhysics, contact, site_at, two_drone_world

from grid_ops.energy import BatteryState
from grid_ops.routes import SortieRequest, apply_sorties, validate_sorties
from grid_ops.sensing import (
    AcceptedRequest,
    CurrentRequest,
    MeasurementTaskV2,
    RequestStatus,
    TaskStatusV2,
)
from grid_ops.world import (
    LAND_S,
    TAKEOFF_S,
    Charge,
    Executing,
    GroundSample,
    Land,
    Perched,
    ScheduledStep,
    Takeoff,
    Transit,
    advance_to,
)

REQ = CurrentRequest(
    id="REQ-1",
    hypothesis_ids=("H1",),
    rationale="r",
    alternatives_distinguished=("a",),
    expected_outcomes=(),
    evidence_ids=(),
    urgency=3,
    source_revision="0",
    replaces_request_ids=(),
    target=__import__("grid_ops.sensing", fromlist=["ConductorScope"]).ConductorScope(
        circuit_id="LINE_025/C1", phase="A", start_mm=0, end_mm=1_000_000
    ),
)


def accepted_request() -> AcceptedRequest:
    return AcceptedRequest("REQ-1", REQ, RequestStatus.ACTIVE, 0, "0")


def task_for(world, task_id: str, site, request_id: str = "REQ-1") -> MeasurementTaskV2:
    t = MeasurementTaskV2(
        task_id, request_id, contact(site), request_id, TaskStatusV2.PENDING, 3
    )
    world.set_task(t)
    return t


def steps_for(world, geo, drone_id: str, site, task_id: str):
    from grid_ops.world import leg_seconds

    pos = geo.position_of(site.point)
    start = world.drones[drone_id].state.perch
    start_pos = geo.position_of(start.point)
    leg = leg_seconds(start_pos, pos, world.drones[drone_id].speed_km_per_min)
    t0 = TAKEOFF_S
    t1 = TAKEOFF_S + leg
    t2 = t1 + LAND_S
    return (
        ScheduledStep(0, 0, t0, Takeoff(start, start_pos)),
        ScheduledStep(1, t0, t1, Transit(start_pos, pos)),
        ScheduledStep(2, t1, t2, Land(site, pos)),
        ScheduledStep(3, t2, t2 + 10, GroundSample((contact(site),), site, (task_id,), pos)),
    ), site


def request_for(world, drone_id: str, steps, final):
    return SortieRequest(
        drone_id, steps, final, world.revision, world.energy_model.revision,
        world.capabilities.revision,
    )


def test_valid_contact_itinerary_validates_and_applies():
    world, geo = two_drone_world()
    world.requests["REQ-1"] = accepted_request()
    site = site_at(world, "LINE_025", 500_000)
    task_for(world, "T1", site)
    steps, final = steps_for(world, geo, "DRONE_1", site, "T1")
    result = validate_sorties(world, geo, None, (request_for(world, "DRONE_1", steps, final),))
    assert result.ok, [str(r) for r in result.rejections]
    assert result.plans[0].itinerary.end_s == steps[-1].end_s
    physics = StubPhysics(chargeable=frozenset())
    apply_sorties(world, physics, result.plans)
    assert isinstance(world.drones["DRONE_1"].state, Executing)
    assert world.tasks["T1"].status is TaskStatusV2.RESERVED
    advance_to(world, physics, steps[-1].end_s)
    assert world.tasks["T1"].status is TaskStatusV2.COMPLETED
    assert world.drones["DRONE_1"].state == Perched(final)


def test_method_substitution_rejected():
    world, geo = two_drone_world()
    world.requests["REQ-1"] = accepted_request()
    site = site_at(world, "LINE_025", 500_000)
    task_for(world, "T1", site)
    steps, final = steps_for(world, geo, "DRONE_1", site, "T1")
    # swap the GroundSample's acquisition for a different integration window
    tampered = list(steps)
    tampered[3] = ScheduledStep(
        3, tampered[3].start_s, tampered[3].end_s,
        GroundSample((contact(site, integration_s=99),), site, ("T1",),
                     geo.position_of(site.point)),
    )
    result = validate_sorties(world, geo, None, (request_for(world, "DRONE_1", tuple(tampered), final),))
    assert not result.ok
    assert result.rejections[0].code == "method_substitution"


def test_stale_energy_model_rejected():
    world, geo = two_drone_world()
    world.requests["REQ-1"] = accepted_request()
    site = site_at(world, "LINE_025", 500_000)
    task_for(world, "T1", site)
    steps, final = steps_for(world, geo, "DRONE_1", site, "T1")
    req = request_for(world, "DRONE_1", steps, final)
    req = SortieRequest(req.drone_id, req.steps, req.final_perch, 0, "other-energy", req.sensor_model_revision)
    result = validate_sorties(world, geo, None, (req,))
    assert not result.ok
    assert result.rejections[0].code == "stale_energy_model"


def test_unknown_task_and_retired_request_rejected():
    world, geo = two_drone_world()
    site = site_at(world, "LINE_025", 500_000)
    steps, final = steps_for(world, geo, "DRONE_1", site, "T404")
    result = validate_sorties(world, geo, None, (request_for(world, "DRONE_1", steps, final),))
    assert not result.ok
    assert result.rejections[0].code == "bad_task"
    world.requests["REQ-1"] = AcceptedRequest(
        "REQ-1", REQ, RequestStatus.RETIRED, 0, "0", "superseded"
    )
    steps, final = steps_for(world, geo, "DRONE_1", site, "T1")
    task_for(world, "T1", site)
    result = validate_sorties(world, geo, None, (request_for(world, "DRONE_1", steps, final),))
    assert not result.ok
    assert result.rejections[0].code == "stale_request"


def test_charge_unavailable_rejected():
    world, geo = two_drone_world()
    world.requests["REQ-1"] = accepted_request()
    site = site_at(world, "LINE_025", 500_000)
    task_for(world, "T1", site)
    base, final = steps_for(world, geo, "DRONE_1", site, "T1")
    pos = geo.position_of(site.point)
    charge_step = ScheduledStep(4, base[-1].end_s, base[-1].end_s + 60, Charge(site, 60, pos))
    result = validate_sorties(world, geo, None, (request_for(world, "DRONE_1", (*base, charge_step), final),))
    assert not result.ok
    assert result.rejections[0].code == "charge_unavailable"


def test_overlapping_occupancy_rejected_sequential_allowed():
    world, geo = two_drone_world()
    world.requests["REQ-1"] = accepted_request()
    site = site_at(world, "LINE_025", 500_000)
    task_for(world, "T1", site)
    task_for(world, "T2", site)
    s1, final = steps_for(world, geo, "DRONE_1", site, "T1")
    s2, _ = steps_for(world, geo, "DRONE_2", site, "T2")
    # drone 2 is delayed into drone 1's perched window
    delay = (s1[2].start_s - s2[2].start_s) + 5
    overlapping = tuple(
        ScheduledStep(st.index, st.start_s + delay, st.end_s + delay, st.operation) for st in s2
    )
    result = validate_sorties(
        world, geo, None, (request_for(world, "DRONE_1", s1, final),
                           request_for(world, "DRONE_2", overlapping, final))
    )
    assert not result.ok
    assert any(r.code == "site_occupied" for r in result.rejections)
    # staying on the site as a final perch holds it for good, however late drone 2 arrives
    delay = s1[-1].end_s - s2[0].start_s
    late = tuple(
        ScheduledStep(st.index, st.start_s + delay, st.end_s + delay, st.operation) for st in s2
    )
    result = validate_sorties(
        world, geo, None, (request_for(world, "DRONE_1", s1, final),
                           request_for(world, "DRONE_2", late, final))
    )
    assert any(r.code == "site_occupied" for r in result.rejections)
    # once drone 1 has taken off and perched elsewhere, drone 2 may use the site afterwards
    from grid_ops.world import leg_seconds

    other = site_at(world, "LINE_025", 650_000)
    pos, opos = geo.position_of(site.point), geo.position_of(other.point)
    leg = leg_seconds(pos, opos, world.drones["DRONE_1"].speed_km_per_min)
    t = s1[-1].end_s
    s1_leaves = (
        *s1,
        ScheduledStep(4, t, t + TAKEOFF_S, Takeoff(site, pos)),
        ScheduledStep(5, t + TAKEOFF_S, t + TAKEOFF_S + leg, Transit(pos, opos)),
        ScheduledStep(6, t + TAKEOFF_S + leg, t + TAKEOFF_S + leg + LAND_S, Land(other, opos)),
    )
    delay = t + TAKEOFF_S + 1 - s2[2].start_s
    sequential = tuple(
        ScheduledStep(st.index, st.start_s + delay, st.end_s + delay, st.operation) for st in s2
    )
    result = validate_sorties(
        world, geo, None, (request_for(world, "DRONE_1", s1_leaves, other),
                           request_for(world, "DRONE_2", sequential, final))
    )
    assert result.ok, [str(r) for r in result.rejections]


def test_energy_shortfall_rejected():
    world, geo = two_drone_world()
    world.requests["REQ-1"] = accepted_request()
    site = site_at(world, "LINE_025", 500_000)
    task_for(world, "T1", site)
    steps, final = steps_for(world, geo, "DRONE_1", site, "T1")
    world.drones["DRONE_1"].battery = BatteryState(remaining_units=1000)
    result = validate_sorties(world, geo, None, (request_for(world, "DRONE_1", steps, final),))
    assert not result.ok
    assert result.rejections[0].code == "energy"


def test_apply_sorties_releases_dropped_reservations():
    world, geo = two_drone_world()
    world.requests["REQ-1"] = accepted_request()
    site = site_at(world, "LINE_025", 500_000)
    task_for(world, "T1", site)
    task_for(world, "T2", site)
    world.set_task(
        MeasurementTaskV2("T2", "REQ-1", contact(site), "REQ-1", TaskStatusV2.RESERVED, 3,
                          assigned_drone_id="DRONE_1")
    )
    steps, final = steps_for(world, geo, "DRONE_1", site, "T1")
    result = validate_sorties(world, geo, None, (request_for(world, "DRONE_1", steps, final),))
    assert result.ok
    apply_sorties(world, StubPhysics(), result.plans)
    assert world.tasks["T1"].status is TaskStatusV2.RESERVED
    assert world.tasks["T2"].status is TaskStatusV2.PENDING
    assert world.tasks["T2"].assigned_drone_id is None
