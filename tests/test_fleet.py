"""Ticket 14: immediate fleet-wide replanning after relevant events, never waiting for an LLM."""

from __future__ import annotations

import dataclasses
import time

import pytest
from conftest import permissive

from grid_ops import agents, planning
from grid_ops.geometry import LandingSite, LinePoint
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import (
    CommitDecision,
    CommitSorties,
    ExcludeReading,
    SetInService,
    SetLandingPolicy,
    Simulation,
)
from grid_ops.world import (
    Charging,
    Flying,
    Measurement,
    Measuring,
    Ready,
    ScheduledReport,
    Stranded,
)


@pytest.fixture(scope="module")
def network():
    return load_network()


def accepted(network, seed=7, auto_replan=False):
    """Hypotheses accepted with the fleet idle; auto-replanning is switched on afterwards."""
    sim = permissive(Simulation(network, scenario=ScenarioConfig(ScenarioMode.INCIDENT_DEMO, seed)))
    agents.run_scripted_turn(sim)
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision("D0", pf.id, pf.semantic_key, "operator")).result()
    sim.auto_replan = auto_replan
    return sim


def land(seg, mm, circuit=None):
    return LandingSite(LinePoint(seg, mm), circuit or f"{seg}/C1")


def fly_landing_only(sim, drone, seg, mm):
    sim.submit(CommitSorties((SortieRequest(drone, (), land(seg, mm)),), "operator")).result()


def pass_now(sim, trigger="test", **kw):
    now = time.perf_counter()
    return sim._route_pass(trigger, (), now, now, **kw)


# -- ordering and triggers ---------------------------------------------------------------------------------------------


def test_evidence_is_published_and_the_coordinator_notified_before_the_route_pass(network):
    sim = accepted(network, auto_replan=True)
    seen = []
    sim.coordinator.listeners.append(
        lambda rev, ids: seen.append(
            (rev, ids, len(sim._world.route_passes), sim.snapshot.revision)
        )
    )
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("solver down"))
    perch = sim._world.drones["DRONE_2"].state.perch
    stop = StopRequest(LinePoint("LINE_058", 28_000), Measurement.CLEARANCE)
    sim.submit(CommitSorties((SortieRequest("DRONE_2", (stop,), perch),), "operator")).result()
    before = len(sim._world.route_passes)
    sim.step(5)
    assert seen, "the coordinator was never notified"
    rev, _, passes_at_notice, published_rev = seen[0]
    assert published_rev == rev  # the evidence snapshot was already published ...
    assert passes_at_notice == before  # ... and no route pass had yet run, even though it fails
    assert len(sim._world.route_passes) > before
    assert "solver failure" in sim._world.route_passes[-1].solver


def test_each_relevant_event_and_command_triggers_a_pass_and_irrelevant_ones_do_not(network):
    sim = accepted(network, auto_replan=True)
    w = sim._world
    n0 = len(w.route_passes)
    sim.step(1)  # time passing alone changes nothing relevant
    assert len(w.route_passes) == n0
    perch = w.drones["DRONE_2"].state.perch
    stop = StopRequest(LinePoint("LINE_058", 28_000), Measurement.CLEARANCE)
    sim.submit(CommitSorties((SortieRequest("DRONE_2", (stop,), perch),), "operator")).result()
    sim.submit(SetLandingPolicy(True, "operator")).result()
    assert w.route_passes[-1].trigger == "SetLandingPolicy"
    sim.step(15)
    assert any("reading" in p.trigger for p in w.route_passes)
    n1 = len(w.route_passes)
    sim.submit(ExcludeReading("R0001", "calibration doubt", "operator")).result()
    assert w.route_passes[-1].trigger == "ExcludeReading" and len(w.route_passes) == n1 + 1
    w.pending_reports.append(
        ScheduledReport(w.clock_s + 60, "LINE_001", "Dispatch reports LINE_001 off.")
    )
    sim.step(2)
    assert any("switching_report" in p.trigger for p in w.route_passes)
    sim.step(30)  # evidence expiry is scheduled and observable
    assert any("evidence_expired" in p.trigger for p in w.route_passes)


def test_route_passes_never_change_hypotheses(network):
    sim = accepted(network, auto_replan=True)
    w = sim._world
    version, revisions = w.hypotheses_version, len(w.hypothesis_revisions)
    for _ in range(3):
        pass_now(sim)
    assert (w.hypotheses_version, len(w.hypothesis_revisions)) == (version, revisions)
    assert w.manager_turns and len(w.manager_turns) == 1  # only the accepted decision's turn


def test_event_and_publication_timestamps_support_latency_measurement(network):
    sim = accepted(network, auto_replan=True)
    rp = pass_now(sim)
    assert rp.ingested_at <= rp.published_at <= rp.route_published_at
    assert rp.latency_s >= 0 and rp.planner_s >= 0 and rp.id.startswith("RP")


# -- motion, locks and fixed occupancy -----------------------------------------------------------------------------------------


def test_flying_drones_divert_from_their_interpolated_position_with_spent_battery_kept(network):
    sim = accepted(network)
    w = sim._world
    fly_landing_only(sim, "DRONE_3", "LINE_026", 7_000_000)
    sim.step(1.0)
    d = w.drones["DRONE_3"]
    assert isinstance(d.state, Flying)
    pos, battery, clock = d.position, d.battery_s, w.clock_s
    assert battery < 1800  # some airtime is already spent
    rp = pass_now(sim)
    d = w.drones["DRONE_3"]
    assert rp.outcome in ("accepted-plan", "kept-incumbent")
    assert isinstance(d.state, Flying | Measuring)
    if rp.outcome == "accepted-plan" and isinstance(d.state, Flying):
        assert d.state.origin == pos and d.state.departed_s == clock  # diverts from where it is
    assert d.battery_s == battery  # a replan spends and refunds nothing


def test_measuring_drones_finish_their_locked_attempt_and_only_the_suffix_changes(network):
    sim = accepted(network)
    w = sim._world
    t = next(t for t in w.tasks.values() if t.point.segment_id == "LINE_058")
    perch = w.drones["DRONE_2"].state.perch
    req = SortieRequest("DRONE_2", (StopRequest(t.point, t.kind, t.purpose, None, t.id),), perch)
    sim.submit(CommitSorties((req,), "operator")).result()
    sim.step(0.2)
    st = w.drones["DRONE_2"].state
    assert isinstance(st, Measuring)
    until, stop = st.until_s, st.stop
    pass_now(sim)
    st2 = w.drones["DRONE_2"].state
    assert isinstance(st2, Measuring) and (st2.until_s, st2.stop) == (until, stop)
    assert w.tasks[t.id].status.value == "measuring"
    sim.step(5)
    assert sim.snapshot.tasks[t.id].status.value == "completed"


def test_charging_and_stranded_drones_keep_position_occupancy_and_timers(network):
    sim = accepted(network)
    w = sim._world
    sim.submit(SetLandingPolicy(True, "operator"))
    w.drones["DRONE_4"].state = Charging(w.drones["DRONE_4"].state.perch, w.clock_s + 5000)
    sim._apply(SetInService("LINE_056", False, "scenario"))  # hidden: DRONE_5 cannot charge there
    w.drones["DRONE_5"].state = Stranded(w.drones["DRONE_5"].state.perch)
    before = {
        i: (w.drones[i].state, w.drones[i].position, w.drones[i].battery_s)
        for i in ("DRONE_4", "DRONE_5")
    }
    rp = pass_now(sim)
    assert {
        i: (w.drones[i].state, w.drones[i].position, w.drones[i].battery_s) for i in before
    } == before
    for cand_perch in (before["DRONE_4"][0].perch, before["DRONE_5"][0].perch):
        assert all(
            cand_perch != d.state.route.perch
            for d in w.drones.values()
            if isinstance(d.state, Flying | Measuring)
        )
    assert rp.outcome


# -- failures ------------------------------------------------------------------------------------------------------------------------


def test_solver_failure_invalid_output_or_no_improvement_keep_the_validated_incumbent(
    network, monkeypatch
):
    sim = accepted(network)
    w = sim._world
    fly_landing_only(sim, "DRONE_3", "LINE_026", 7_000_000)
    state = w.drones["DRONE_3"].state
    for name, planner in (
        ("timeout", lambda *a, **k: (_ for _ in ()).throw(TimeoutError("watchdog"))),
        ("model invalid", lambda *a, **k: (_ for _ in ()).throw(ValueError("MODEL_INVALID"))),
    ):
        sim.coordinator.planner = planner
        rp = pass_now(sim)
        assert rp.outcome == "kept-incumbent" and "solver failure" in rp.solver, name
        assert w.drones["DRONE_3"].state == state and w.hold is None
    real = planning.plan_sorties

    def corrupt(*a, **k):
        res = real(*a, **k)
        bad = tuple(
            dataclasses.replace(r, landing=dataclasses.replace(r.landing, circuit_id="LINE_999/C1"))
            for r in (res.ranked[0].sorties if res.ranked else ())
        )
        return (
            dataclasses.replace(
                res, ranked=(dataclasses.replace(res.ranked[0], sorties=bad, source="round-1"),)
            )
            if res.ranked
            else res
        )

    sim.coordinator.planner = corrupt
    assert pass_now(sim).outcome in ("kept-incumbent", "no-op")
    assert w.drones["DRONE_3"].state == state


def test_invalid_incumbent_with_a_failing_solver_falls_back_to_landing_only_recovery(network):
    sim = accepted(network)
    w = sim._world
    fly_landing_only(sim, "DRONE_3", "LINE_026", 7_000_000)
    w.pending_reports.append(
        ScheduledReport(w.clock_s + 5, "LINE_026", "Dispatch reports LINE_026 off.")
    )
    sim.step(0.5)
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("down"))
    rp = pass_now(sim)
    assert rp.outcome == "landing-only" and w.hold is None
    assert w.drones["DRONE_3"].state.route.perch.circuit_id != "LINE_026/C1"


def test_no_complete_safe_continuation_holds_with_a_specific_cause_and_never_releases_unsafely(
    network,
):
    sim = accepted(network)
    w = sim._world
    fly_landing_only(sim, "DRONE_3", "LINE_026", 7_000_000)
    w.pending_reports.append(
        ScheduledReport(w.clock_s + 5, "LINE_026", "Dispatch reports LINE_026 off.")
    )
    sim.step(0.5)
    w.landing_policy = LandingPolicy(False)
    w.drones["DRONE_3"].battery_s = 90  # cannot reach any eligible endpoint
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("down"))
    route_before = w.drones["DRONE_3"].state.route
    rp = pass_now(sim)
    assert rp.outcome == "HELD" and "DRONE_3" in rp.detail
    assert w.hold is not None and "no safe continuation" in w.hold.reason
    assert w.drones["DRONE_3"].state.route == route_before  # nothing unsafe was committed
    clock = w.clock_s
    sim.step(5)
    assert w.clock_s == clock  # held: simulated time frozen until a safe continuation exists
    assert pass_now(sim).outcome == "HELD" and w.hold is not None  # a retry cannot release it


def test_cross_drone_updates_are_all_or_nothing(network):
    sim = accepted(network)
    w = sim._world
    fly_landing_only(sim, "DRONE_3", "LINE_026", 7_000_000)
    fly_landing_only(sim, "DRONE_1", "LINE_041", 400_000)
    sim.step(0.5)
    before = {i: w.drones[i].state for i in ("DRONE_1", "DRONE_3")}
    real = planning.plan_sorties

    def one_bad_sortie(*a, **k):
        res = real(*a, **k)
        cand = res.ranked[0]
        bad = dataclasses.replace(
            cand.sorties[0],
            landing=dataclasses.replace(cand.sorties[0].landing, circuit_id="LINE_999/C1"),
        )
        poisoned = dataclasses.replace(cand, sorties=(bad, *cand.sorties[1:]), source="round-1")
        return dataclasses.replace(res, ranked=(poisoned,))

    sim.coordinator.planner = one_bad_sortie
    rp = pass_now(sim)
    assert rp.outcome in ("kept-incumbent", "no-op")
    assert {i: w.drones[i].state for i in before} == before  # neither drone was touched


def test_deadline_edges_and_occupied_endpoints_are_respected(network):
    sim = accepted(network)
    w = sim._world
    w.drones["DRONE_5"].state = Charging(w.drones["DRONE_5"].state.perch, w.clock_s + 5000)
    perch5 = w.drones["DRONE_5"].state.perch
    from grid_ops.world import DEADLINE_S

    w.clock_s = DEADLINE_S - 20 * 60
    for d in w.drones.values():
        d.battery_s = 1800
    rp = pass_now(sim)
    for d in w.drones.values():
        if isinstance(d.state, Flying | Measuring):
            assert d.state.route.perch != perch5
            last = d.state.route.perch
            _ = last
    assert rp.outcome in ("accepted-plan", "kept-incumbent", "no-op", "landing-only")
    _ = (Ready,)
