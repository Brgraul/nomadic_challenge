"""Behavior tests for the itinerary planner (three-method value model).

Every emitted `SortieRequest` is checked against the independent validator, the exact mode is
compared against a brute-force enumeration on a tiny instance, and charging, incumbent, fault
and capacity behaviour are exercised directly.
"""

from __future__ import annotations

from itertools import pairwise, permutations, product

from fixtures_three_method import (
    StubPhysics,
    contact_task,
    seed_handshakes,
    site_at,
    two_drone_world,
)

from grid_ops import planning, scoring
from grid_ops.energy import BatteryState
from grid_ops.geometry import Interval, LinePoint
from grid_ops.routes import apply_sorties, validate_sorties
from grid_ops.sensing import (
    AcceptedRequest,
    CorridorSpan,
    LidarPass,
    LidarRequest,
    MeasurementTaskV2,
    OutcomeSpec,
    RequestStatus,
    ScanPose,
    ScanTrajectory,
    TaskStatusV2,
)
from grid_ops.world import (
    DEADLINE_S,
    LAND_S,
    Charge,
    Executing,
    Itinerary,
    Land,
    Perched,
    Scan,
    ScheduledStep,
    Transit,
    advance_to,
    leg_seconds,
)


def _lidar_task(w, geo, task_id, request_id, seg, start_mm, end_mm, urgency=5):
    """A lidar task, already MEASURING by DRONE_1 so the planner may continue it."""
    target = Interval(seg, start_mm, end_mm)
    trajectory = ScanTrajectory(
        (
            ScanPose(geo.position_of(LinePoint(seg, start_mm)), 50.0, 0.0, -60.0),
            ScanPose(geo.position_of(LinePoint(seg, end_mm)), 50.0, 0.0, -60.0),
        ),
        20,
    )
    acquisition = LidarPass(target=target, trajectory=trajectory)
    request = LidarRequest(
        id=request_id,
        hypothesis_ids=(),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=urgency,
        source_revision="rev0",
        replaces_request_ids=(),
        target=CorridorSpan(segment_id=seg, start_mm=start_mm, end_mm=end_mm),
    )
    w.requests[request_id] = AcceptedRequest(
        request_id, request, RequestStatus.ACTIVE, w.clock_s, "rev0"
    )
    w.set_task(
        MeasurementTaskV2(
            task_id,
            request_id,
            acquisition,
            request_id,
            TaskStatusV2.MEASURING,
            urgency,
            assigned_drone_id="DRONE_1",
        )
    )
    return acquisition


def _tiny_two_task_world():
    """Two contact tasks on distinct sites, two perched drones, both circuits chargeable."""
    w, geo = two_drone_world()
    seed_handshakes(w)
    contact_task(w, "T1", site_at(w, "LINE_040", 555_000), "REQ-1")
    contact_task(w, "T2", site_at(w, "LINE_025", 555_000), "REQ-2")
    return w, geo


def _contiguous(steps):
    return all(b.start_s == a.end_s for a, b in pairwise(steps))


def test_two_contact_tasks_served_validated_and_completed():
    w, geo = _tiny_two_task_world()
    result = planning.plan_sorties(w, geo, None)
    best = result.best
    assert best is not None and best.valid, result.stop_reason
    assert {"T1", "T2"} <= set(best.task_ids)
    assert best.score.tiers[scoring.TIERS.index(5)] >= 2 * scoring.REQUEST_POINTS
    check = validate_sorties(w, geo, None, best.sorties)
    assert check.ok, [str(r) for r in check.rejections]
    for req in best.sorties:
        assert _contiguous(req.steps), req.steps
        assert req.steps[-1].end_s <= DEADLINE_S
        assert req.steps[0].start_s == w.clock_s  # perched drones plan from now
    # end-to-end: the plans apply, execute and leave every used drone perched on its final perch
    physics = StubPhysics()
    apply_sorties(w, physics, check.plans)
    end = max(p.itinerary.end_s for p in check.plans)
    advance_to(w, physics, end)
    for req in best.sorties:
        drone = w.drones[req.drone_id]
        assert isinstance(drone.state, Perched) and drone.state.perch == req.final_perch
    assert w.tasks["T1"].status is TaskStatusV2.COMPLETED
    assert w.tasks["T2"].status is TaskStatusV2.COMPLETED


def _brute_force_best(w, geo, ctx):
    """Exhaustive enumeration: every assignment, order and perch choice, charge fixed at zero.
    Candidates are validated and scored exactly like the planner's own candidates."""
    vehicles = planning._vehicles(w, geo)
    chosen, _omitted, blocked = planning._select(w, ctx)
    if blocked is not None:
        return None
    scans, perches = planning._build_nodes(w, geo, vehicles, chosen, exact=True)
    options = []
    for t in chosen:
        opts = [(None, None)]
        opts += [
            (vi, ("t", i)) for i, s in enumerate(scans) if s.task_id == t.id
            for vi in range(len(vehicles))
        ]
        opts += [
            (vi, ("p", j)) for j, p in enumerate(perches) if p.task_id == t.id
            for vi in range(len(vehicles))
        ]
        options.append(opts)
    owner = planning._owners(w)
    best = None
    for combo in product(*options):
        per_drone: dict[int, list] = {}
        for (_t, (vi, node)) in zip(chosen, combo, strict=True):
            if vi is not None:
                per_drone.setdefault(vi, []).append(node)
        for orders in product(
            *(permutations(per_drone.get(vi, [])) for vi in range(len(vehicles)))
        ):
            reqs = []
            for vi, seq in enumerate(orders):
                if not seq:
                    continue
                req = planning._build_sortie(
                    w, geo, vehicles[vi], vi, scans, perches, tuple(seq), lambda v, j: 0, 0
                )
                if req is None:
                    reqs = None
                    break
                reqs.append(req)
            if not reqs:
                continue
            candidate = planning._evaluate(w, geo, None, ctx, tuple(reqs), "brute", [], owner)
            if candidate.valid and (best is None or candidate.score.key > best):
                best = candidate.score.key
    return best


def test_exact_planner_matches_brute_force_enumeration():
    w, geo = _tiny_two_task_world()
    ctx = scoring.build_context(w)
    result = planning.plan_sorties(w, geo, None, exact=True)
    brute = _brute_force_best(w, geo, ctx)
    assert result.blocked is None
    if brute is None:
        assert all(c.task_ids == () for c in result.ranked)  # the planner must find nothing either
        return
    assert result.best is not None and result.best.valid
    planner_key = result.best.score.key
    assert planner_key >= brute  # never worse than the exhaustive optimum
    assert all(
        abs(a - b) <= scoring.QUANTUM for a, b in zip(planner_key, brute, strict=True)
    ), (planner_key, brute)


def test_charge_chosen_only_when_energy_requires_it():
    w, geo = two_drone_world()
    seed_handshakes(w)
    del w.drones["DRONE_2"]
    w.drones["DRONE_1"].battery = BatteryState(remaining_units=100_000)
    contact_task(w, "T1", site_at(w, "LINE_040", 555_000), "REQ-1")
    result = planning.plan_sorties(w, geo, None)
    best = result.best
    assert best is not None and best.valid, result.stop_reason
    assert not any(
        isinstance(step.operation, Charge) for r in best.sorties for step in r.steps
    ), "no gratuitous charge"
    # a second task on the other segment makes the no-charge plan infeasible under the filter
    contact_task(w, "T2", site_at(w, "LINE_025", 555_000), "REQ-2")
    result = planning.plan_sorties(w, geo, None)
    best = result.best
    assert best is not None and best.valid, result.stop_reason
    assert {"T1", "T2"} <= set(best.task_ids)
    charges = [
        step.operation for r in best.sorties for step in r.steps if isinstance(step.operation, Charge)
    ]
    assert charges, "a charge step is required to serve both tasks"
    check = validate_sorties(w, geo, None, best.sorties)
    assert check.ok, [str(r) for r in check.rejections]


def test_incumbent_ranked_and_kept_when_rounds_do_not_better_it():
    w, geo = two_drone_world()
    seed_handshakes(w)
    acquisition = _lidar_task(w, geo, "T-L", "REQ-L", "LINE_040", 100_000, 600_000)
    entry = acquisition.trajectory.poses[0].position
    exit_pos = acquisition.trajectory.poses[-1].position
    perch_site = site_at(w, "LINE_040", 700_000)
    perch_pos = geo.position_of(perch_site.point)
    t1 = 20
    t2 = t1 + leg_seconds(exit_pos, perch_pos, 1.0)
    steps = (
        ScheduledStep(0, 0, t1, Scan(acquisition, "T-L")),
        ScheduledStep(1, t1, t2, Transit(exit_pos, perch_pos)),
        ScheduledStep(2, t2, t2 + LAND_S, Land(perch_site, perch_pos)),
    )
    drone = w.drones["DRONE_1"]
    drone.state = Executing(Itinerary(steps, perch_site), 0)  # the scan just started
    drone.position = entry
    result = planning.plan_sorties(w, geo, None)
    incumbents = [c for c in result.ranked if c.source == "incumbent"]
    assert incumbents and incumbents[0].valid, result.stop_reason
    assert "T-L" in incumbents[0].task_ids
    assert result.best is not None and result.best.source == "incumbent"


def test_fault_injection_stops_cleanly():
    w, geo = _tiny_two_task_world()
    result = planning.plan_sorties(w, geo, None, fault="UNKNOWN")
    assert "no feasible solution" in result.stop_reason
    assert any(s.status == "UNKNOWN" for s in result.stages)
    assert result.ranked and all(c.source == "incumbent" for c in result.ranked)
    assert result.best is not None and result.best.valid


def test_over_capacity_is_blocked():
    w, geo = two_drone_world()
    seed_handshakes(w)
    contact_task(w, "T1", site_at(w, "LINE_040", 100_000), "REQ-1")
    contact_task(w, "T2", site_at(w, "LINE_025", 100_000), "REQ-2")
    contact_task(w, "T3", site_at(w, "LINE_040", 100_001), "REQ-3")
    result = planning.plan_sorties(w, geo, None, cap=2)
    assert result.blocked is not None
    assert result.stop_reason == "PlanningBlocked"
    assert "exceeds the 2-attempt capacity" in result.blocked.reason
