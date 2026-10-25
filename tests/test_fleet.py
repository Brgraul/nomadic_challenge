"""Fleet-wide route passes on the three-method runtime.

A route pass never waits for a language model: it plans over the available fleet, accepts a
continuation only if the complete set validates atomically, keeps the revalidated incumbent when
the planner fails, sends airborne drones to eligible landings when it does not, and holds with a
specific cause when even that is impossible. Route-only passes never change a hypothesis.
"""

from __future__ import annotations

import time
from dataclasses import replace as dataclasses_replace

import pytest
from helpers_p2 import publish, tiny_sim

from grid_ops import planning
from grid_ops.coordinator import Coordinator
from grid_ops.energy import BatteryState
from grid_ops.geometry import LandingSite, LinePoint
from grid_ops.routes import GroundSample, ScheduledStep, SortieRequest
from grid_ops.sensing import ContactCurrent, RequestStatus
from grid_ops.sim import CommitSorties, ExcludeReading, SetLandingPolicy
from grid_ops.world import (
    DEADLINE_S,
    LAND_S,
    TAKEOFF_S,
    CommandRejected,
    Land,
    ScheduledReport,
    Stranded,
    Takeoff,
    Transit,
    leg_seconds,
)


def accepted_sim(**kw):
    """A tiny Simulation whose startup proposal (hypotheses + measurement requests) is accepted."""
    sim = tiny_sim(**kw)
    Coordinator(sim).run_turn_now()
    return sim


def flight_sortie(sim, drone_id, stops):
    """A multi-leg takeoff-transit-land itinerary ending perched at the last stop."""
    w = sim._world
    drone = w.drones[drone_id]
    site = drone.state.perch
    here = sim.geometry.position_of(site.point)
    t = w.clock_s
    steps = []
    for seg, mm in stops:
        target = LandingSite(LinePoint(seg, mm), f"{seg}/C1")
        pos = sim.geometry.position_of(target.point)
        leg = leg_seconds(here, pos, drone.speed_km_per_min)
        steps.append(ScheduledStep(len(steps), t, t + TAKEOFF_S, Takeoff(site, here)))
        t += TAKEOFF_S
        if leg:
            steps.append(ScheduledStep(len(steps), t, t + leg, Transit(here, pos)))
            t += leg
        steps.append(ScheduledStep(len(steps), t, t + LAND_S, Land(target, pos)))
        t += LAND_S
        here, site = pos, target
    return SortieRequest(
        drone_id, tuple(steps), site, w.revision, w.energy_model.revision, w.capabilities.revision
    )


def contact_sortie(sim, drone_id, task_id, final_stop=None):
    """Fly to a compiled ground task, sample it, then optionally continue to another perch."""
    w = sim._world
    task = w.tasks[task_id]
    drone = w.drones[drone_id]
    site = LandingSite(task.acquisition.conductor.point, task.acquisition.conductor.circuit_id)
    stops = [site] + ([final_stop] if final_stop else [])
    last = drone.state.perch
    here = sim.geometry.position_of(last.point)
    t = w.clock_s
    steps = []
    for target in stops:
        pos = sim.geometry.position_of(target.point)
        leg = leg_seconds(here, pos, drone.speed_km_per_min)
        steps.append(ScheduledStep(len(steps), t, t + TAKEOFF_S, Takeoff(last, here)))
        t += TAKEOFF_S
        if leg:
            steps.append(ScheduledStep(len(steps), t, t + leg, Transit(here, pos)))
            t += leg
        steps.append(ScheduledStep(len(steps), t, t + LAND_S, Land(target, pos)))
        t += LAND_S
        here, last = pos, target
        if target == site:
            steps.append(
                ScheduledStep(
                    len(steps), t, t + task.acquisition.integration_s,
                    GroundSample((task.acquisition,), site, (task_id,), pos),
                )
            )
            t += task.acquisition.integration_s
    return SortieRequest(
        drone_id, tuple(steps), last, w.revision, w.energy_model.revision, w.capabilities.revision
    )


def commit_flight(sim, drone_id, stops):
    sim.submit(CommitSorties((flight_sortie(sim, drone_id, stops),), "operator")).result()
    return sim._world.drones[drone_id]


def a_contact_task(sim):
    w = sim._world
    return next(t for t in w.tasks.values() if isinstance(t.acquisition, ContactCurrent))


def pass_now(sim, trigger="test", **kw):
    now = time.perf_counter()
    return sim._route_pass(trigger, (), now, now, **kw)


# -- ordering and triggers ---------------------------------------------------------------------------------------------


def test_evidence_is_published_and_the_coordinator_notified_before_the_route_pass():
    sim = accepted_sim(auto_replan=True)
    sim.submit_route_pass("turnover")
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("solver down"))
    seen = []
    sim.coordinator.listeners.append(
        lambda rev, ids: seen.append((rev, ids, len(sim._world.route_passes), sim.snapshot.revision))
    )
    before = len(sim._world.route_passes)
    events = sim.step(20)
    assert any(e.kind == "reading" for e in events), "no reading was published"
    assert seen, "the coordinator was never notified"
    rev, _ids, passes_at_notice, published_rev = seen[0]
    assert published_rev == rev  # the evidence snapshot was already published ...
    assert passes_at_notice == before  # ... and no route pass had yet run, even though it fails
    assert len(sim._world.route_passes) > before
    assert "solver failure" in sim._world.route_passes[-1].solver


def test_relevant_events_and_commands_trigger_passes_and_irrelevant_ones_do_not():
    sim = accepted_sim(auto_replan=True)
    sim.submit_route_pass("turnover")
    w = sim._world
    commit_flight(sim, "DRONE_1", [("LINE_002", 400_000)])
    n0 = len(w.route_passes)
    sim.step(0.05)  # a few seconds inside the takeoff: nothing relevant happens
    assert len(w.route_passes) == n0
    sim.submit(SetLandingPolicy(True, "operator")).result()
    assert w.route_passes[-1].trigger == "SetLandingPolicy"
    while not sim.snapshot.readings:
        sim.step(0.5)
    assert any("reading" in p.trigger or "landed" in p.trigger for p in w.route_passes)
    n1 = len(w.route_passes)
    rid = sorted(sim.snapshot.readings)[0]
    sim.submit(ExcludeReading(rid, "calibration doubt", "operator")).result()
    assert w.route_passes[-1].trigger == "ExcludeReading" and len(w.route_passes) == n1 + 1
    w.pending_reports.append(
        ScheduledReport(w.clock_s + 60, "LINE_056", "Dispatch: LINE_056 off.")
    )
    sim.step(2)
    assert any("switching_report" in p.trigger for p in w.route_passes)
    sim.step(30)  # evidence expiry is scheduled and observable
    assert any("evidence_expired" in p.trigger for p in w.route_passes)


def test_route_passes_never_change_hypotheses():
    sim = accepted_sim()
    sim.submit_route_pass("turnover")
    w = sim._world
    version, revisions, proposals = (
        w.hypotheses_version,
        len(w.hypothesis_revisions),
        list(w.proposals),
    )
    for _ in range(3):
        pass_now(sim)
    assert (w.hypotheses_version, len(w.hypothesis_revisions)) == (version, revisions)
    assert w.proposals == proposals
    assert len(w.manager_turns) == 1  # only the accepted startup decision's turn


def test_route_pass_timestamps_support_latency_measurement():
    sim = accepted_sim()
    rp = pass_now(sim)
    assert rp.ingested_at <= rp.published_at <= rp.route_published_at
    assert rp.latency_s >= 0 and rp.planner_s >= 0 and rp.id.startswith("RP")


# -- motion, locks and grounded states -----------------------------------------------------------------------------------


def test_executing_drones_are_replanned_from_their_step_boundary_without_energy_changes():
    sim = accepted_sim()  # open tasks, so the planner has something to propose
    drone = commit_flight(sim, "DRONE_1", [("LINE_002", 400_000)])
    sim.step(0.5)  # mid transit
    boundary = drone.state.step.end_s
    battery = drone.battery.remaining_units
    itinerary = drone.state.itinerary
    rp = pass_now(sim)
    drone = sim._world.drones["DRONE_1"]
    if rp.outcome == "accepted-plan" and drone.state.itinerary is not itinerary:
        new = drone.state.itinerary
        assert new.steps[0].start_s == boundary  # the replan starts at the step boundary
        assert isinstance(new.steps[0].operation, Takeoff | Transit)  # never from mid-air nowhere
    else:
        assert rp.outcome in ("kept-incumbent", "no-op")
    assert drone.battery.remaining_units == battery  # a replan spends and refunds nothing


def test_locked_ground_samples_finish_and_only_the_suffix_changes():
    sim = accepted_sim()
    task = a_contact_task(sim)
    sim.submit(CommitSorties((contact_sortie(sim, "DRONE_1", task.id),), "operator")).result()
    w = sim._world
    sim.step(1.05)  # 63 s: the ground sample is in progress
    d = w.drones["DRONE_1"]
    assert d.phase == "executing" and isinstance(d.state.step.operation, GroundSample)
    locked = d.state.step
    assert w.tasks[task.id].status.value == "MEASURING"
    pass_now(sim)
    d = sim._world.drones["DRONE_1"]
    assert d.phase == "executing" and d.state.step.operation == locked.operation
    assert (d.state.step.start_s, d.state.step.end_s) == (locked.start_s, locked.end_s)
    sim.step(1)
    w = sim._world
    assert w.tasks[task.id].status.value == "COMPLETED"
    assert any(r.task_id == task.id for r in w.readings.values())
    assert w.drones["DRONE_1"].phase == "perched"


def test_perched_and_stranded_drones_keep_position_when_nothing_can_be_planned():
    sim = accepted_sim()
    w = sim._world
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("down"))
    w.drones["DRONE_4"].state = Stranded(w.drones["DRONE_4"].state.perch)
    before = {
        i: (w.drones[i].state, w.drones[i].position, w.drones[i].battery.remaining_units)
        for i in ("DRONE_4", "DRONE_5")
    }
    rp = pass_now(sim)
    assert rp.outcome == "no-op"  # nothing airborne, no work the planner could commit
    after = {
        i: (w.drones[i].state, w.drones[i].position, w.drones[i].battery.remaining_units)
        for i in ("DRONE_4", "DRONE_5")
    }
    assert after == before


# -- failures -------------------------------------------------------------------------------------------------------------


def test_solver_failures_invalid_candidates_and_no_improvement_keep_the_validated_incumbent():
    sim = accepted_sim()
    commit_flight(sim, "DRONE_1", [("LINE_002", 400_000)])
    sim.step(0.5)
    state = sim._world.drones["DRONE_1"].state
    for name, planner in (
        ("timeout", lambda *a, **k: (_ for _ in ()).throw(TimeoutError("watchdog"))),
        ("model invalid", lambda *a, **k: (_ for _ in ()).throw(ValueError("MODEL_INVALID"))),
    ):
        sim.coordinator.planner = planner
        rp = pass_now(sim, name)
        assert rp.outcome == "kept-incumbent" and "solver failure" in rp.solver, name
        assert sim._world.drones["DRONE_1"].state == state and sim._world.hold is None
    real = planning.plan_sorties

    def corrupt(*a, **k):
        res = real(*a, **k)
        bad = tuple(
            dataclasses_replace(r, energy_model_revision="tampered-energy-model")
            if r.drone_id == "DRONE_1"
            else r
            for r in (res.ranked[0].sorties if res.ranked else ())
        )
        return (
            dataclasses_replace(
                res, ranked=(dataclasses_replace(res.ranked[0], sorties=bad, source="round-1"),)
            )
            if res.ranked
            else res
        )

    sim.coordinator.planner = corrupt
    assert pass_now(sim, "corrupt").outcome in ("kept-incumbent", "no-op")
    assert sim._world.drones["DRONE_1"].state == state


def test_invalid_incumbent_with_a_failing_solver_falls_back_to_landing_only_recovery():
    sim = accepted_sim()
    w = sim._world
    task = a_contact_task(sim)
    target = LandingSite(LinePoint("LINE_026", 400_000), "LINE_026/C1")
    sim.submit(
        CommitSorties((contact_sortie(sim, "DRONE_1", task.id, final_stop=target),), "operator")
    ).result()
    sim.step(0.5)  # airborne, mid transit towards the task site
    # the request the remaining ground sample serves is withdrawn: the continuation is stale
    w.requests[task.request_id] = dataclasses_replace(
        w.requests[task.request_id], status=RequestStatus.RETIRED, retired_reason="withdrawn"
    )
    publish(w, sim)
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("down"))
    rp = pass_now(sim, "stale incumbent")
    assert rp.outcome == "landing-only" and w.hold is None
    assert w.drones["DRONE_1"].state.itinerary.final_perch != target


def test_no_safe_continuation_holds_with_a_specific_cause_and_never_releases_unsafely():
    sim = accepted_sim()
    w = sim._world
    drone = commit_flight(sim, "DRONE_1", [("LINE_040", 400_000)])
    sim.step(0.5)  # mid transit; the remaining land needs far more energy than is left
    drone.battery = BatteryState(remaining_units=1_000)
    for d in w.drones.values():
        d.battery = BatteryState(remaining_units=1_000)
    publish(w, sim)
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("down"))
    route_before = w.drones["DRONE_1"].state.itinerary
    rp = pass_now(sim, "no continuation")
    assert rp.outcome == "HELD" and "DRONE_1" in rp.detail
    assert w.hold is not None and "no safe continuation" in w.hold.reason
    assert w.drones["DRONE_1"].state.itinerary == route_before  # nothing unsafe was committed
    clock = w.clock_s
    sim.step(5)
    assert w.clock_s == clock  # held: simulated time frozen until a safe continuation exists
    assert pass_now(sim, "retry").outcome == "HELD" and w.hold is not None
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        from grid_ops.sim import Release

        sim.submit(Release(w.next_cycle - 1, "no-one", "operator")).result()


def test_cross_drone_updates_are_all_or_nothing():
    sim = accepted_sim()
    w = sim._world
    commit_flight(sim, "DRONE_1", [("LINE_002", 400_000)])
    commit_flight(sim, "DRONE_2", [("LINE_056", 400_000)])
    sim.step(0.5)
    before = {i: w.drones[i].state for i in ("DRONE_1", "DRONE_2")}
    real = planning.plan_sorties

    def one_bad_sortie(*a, **k):
        res = real(*a, **k)
        cand = res.ranked[0]
        bad = dataclasses_replace(
            cand.sorties[0], energy_model_revision="tampered-energy-model"
        )
        poisoned = dataclasses_replace(cand, sorties=(bad, *cand.sorties[1:]), source="round-1")
        return dataclasses_replace(res, ranked=(poisoned,))

    sim.coordinator.planner = one_bad_sortie
    rp = pass_now(sim, "poisoned")
    assert rp.outcome in ("kept-incumbent", "no-op")
    assert {i: w.drones[i].state for i in before} == before  # neither drone was touched


def test_deadline_edges_and_occupied_endpoints_are_respected():
    sim = accepted_sim()
    w = sim._world
    w.drones["DRONE_5"].state = Stranded(w.drones["DRONE_5"].state.perch)
    occupied = w.drones["DRONE_5"].state.perch
    w.clock_s = DEADLINE_S - 20 * 60
    rp = pass_now(sim, "near the deadline")
    assert rp.outcome in ("accepted-plan", "kept-incumbent", "no-op")
    for d in w.drones.values():
        if d.phase == "executing":
            assert d.state.itinerary.final_perch != occupied
            assert d.state.itinerary.end_s <= DEADLINE_S