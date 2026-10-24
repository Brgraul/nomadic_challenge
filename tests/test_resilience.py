"""Ticket 17: response budgets, fault injection, operator view and truth-leak probes."""

from __future__ import annotations

import time

import pytest
from test_fleet import accepted, land, network, pass_now  # noqa: F401

from grid_ops import planning, resilience
from grid_ops.geometry import LinePoint
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.sim import CommitSorties, ExcludeReading, SetLandingPolicy
from grid_ops.world import CommandRejected, Measurement, ScheduledReport

LATENCY_SEEDS = (7, 8, 9)  # the declared representative seed set


def drive_events(sim):
    """The declared representative event set: dispatch, readings, exclusion, policy, switching
    report and evidence expiry, each through the reactive route pass."""
    w = sim._world
    perch = w.drones["DRONE_2"].state.perch
    stops = tuple(
        StopRequest(LinePoint("LINE_058", mm), Measurement.CLEARANCE) for mm in (28_000, 33_000)
    )
    sim.submit(CommitSorties((SortieRequest("DRONE_2", stops, perch),), "operator")).result()
    sim.submit(SetLandingPolicy(True, "operator")).result()
    sim.step(20)
    sim.submit(ExcludeReading("R0001", "calibration doubt", "operator")).result()
    w.pending_reports.append(ScheduledReport(w.clock_s + 60, "LINE_001", "Dispatch: LINE_001 off."))
    sim.step(5)
    sim.step(40)


def test_p95_ingestion_to_validated_plan_is_within_budget_over_declared_seeds(network):  # noqa: F811
    passes = []
    for seed in LATENCY_SEEDS:
        sim = accepted(network, seed=seed, auto_replan=True)
        drive_events(sim)
        passes += sim._world.route_passes
    rep = resilience.latency_report(passes)
    assert rep.n >= 6, rep.summary()
    assert not rep.budget_misses, rep.summary()
    assert rep.p95_s <= resilience.P95_TARGET_S, rep.summary()
    assert rep.stage_means_s["solver_and_scoring"] >= 0


def test_the_shared_budget_reserves_time_for_non_solver_stages(network):  # noqa: F811
    sim = accepted(network, auto_replan=False)
    w = sim.snapshot
    free = planning.plan_sorties(w, sim.geometry, sim.circuits, sim._estimation)
    assert free.replay["wall_cap_s"] <= 1.5
    late = planning.plan_sorties(w, sim.geometry, sim.circuits, sim._estimation, elapsed_s=1.2)
    assert late.replay["wall_cap_s"] == pytest.approx(0.3)
    exhausted = planning.plan_sorties(w, sim.geometry, sim.circuits, sim._estimation, elapsed_s=1.9)
    assert exhausted.replay["wall_cap_s"] == 0.0
    assert exhausted.best is not None and exhausted.best.source == "incumbent"


@pytest.mark.parametrize("fault", ["UNKNOWN", "MODEL_INVALID", "timeout"])
def test_solver_faults_fall_back_to_the_validated_incumbent_and_are_reported(network, fault):  # noqa: F811
    sim = accepted(network, auto_replan=True)
    drive_events(sim)
    before = len(sim._world.route_passes)
    rp = pass_now(sim, "fault", fault=fault)
    assert rp.outcome in ("kept-incumbent", "accepted-plan", "no-op", "landing-only")
    if fault != "timeout":
        assert fault in rp.stage_statuses
        assert resilience.is_fallback(rp)
    assert sim._world.hold is None and len(sim._world.route_passes) == before + 1


def test_a_hung_worker_is_killed_by_the_watchdog_and_restarted(network):  # noqa: F811
    sim = accepted(network, auto_replan=True)
    worker = resilience.PlannerWorker(watchdog_s=1.0)
    try:
        sim.coordinator.planner = worker.plan
        ok = pass_now(sim, "healthy worker")
        assert "solver failure" not in ok.solver and worker.alive
        t0 = time.perf_counter()
        rp = pass_now(sim, "hung worker", worker_fault="hang")
        assert time.perf_counter() - t0 < 6
        assert "watchdog" in rp.solver and worker.restarts == 1 and worker.alive
        assert sim._world.hold is None
        again = pass_now(sim, "after restart")  # the restarted worker serves normally
        assert "solver failure" not in again.solver
    finally:
        worker.close()


def test_worker_termination_mid_solve_is_survived(network):  # noqa: F811
    sim = accepted(network, auto_replan=True)
    worker = resilience.PlannerWorker(watchdog_s=3.0)
    try:
        sim.coordinator.planner = worker.plan
        rp = pass_now(sim, "crashed worker", worker_fault="exit")
        assert "worker-terminated" in rp.solver and worker.restarts == 1
        assert sim._world.hold is None
        assert any("TERMINATED" in f for f in worker.failures)
    finally:
        worker.close()


def test_failure_after_hold_acquisition_keeps_the_hold_until_a_safe_continuation(network):  # noqa: F811
    sim = accepted(network, auto_replan=True)
    w = sim._world
    seen = {}

    def planner(*a, **k):
        seen["hold_during_solve"] = w.hold is not None
        raise RuntimeError("solver died after the hold was taken")

    sim.coordinator.planner = planner
    rp = pass_now(sim, "failure after hold")
    assert seen["hold_during_solve"] is True
    assert w.hold is None and rp.outcome in ("no-op", "kept-incumbent", "landing-only", "HELD")
    assert "solver failure" in rp.solver


def test_stale_commands_are_rejected_without_changing_state(network):  # noqa: F811
    from grid_ops.sim import CommitDecision

    sim = accepted(network, auto_replan=False)
    w = sim._world
    pf = sim.snapshot.proposals[-1]
    version = w.hypotheses_version
    with pytest.raises(CommandRejected, match="not pending"):
        sim.submit(CommitDecision("D-old", pf.id, "0" * 16, "operator")).result()
    assert w.hypotheses_version == version and w.hold is None
    assert w.rejections[-1].reasons  # the refusal is on the public record


def test_operator_view_names_the_action_when_no_continuation_exists(network):  # noqa: F811
    sim = accepted(network, auto_replan=True)
    w = sim._world
    assert resilience.operator_actions(w) == [] or all(
        "HELD" not in a for a in resilience.operator_actions(w)
    )
    from grid_ops import decisions

    w.hold = decisions.Hold(99, None, "no landing site reachable for DRONE_2", w.clock_s)
    acts = resilience.operator_actions(w)
    assert any("HELD" in a and "landing" in a for a in acts)


def test_truth_leak_probe_over_the_fully_connected_workflow(network):  # noqa: F811
    from test_isolation import assert_no_hidden

    from grid_ops import agents, views
    from grid_ops.manager_tools import Toolbox

    sim = accepted(network, auto_replan=True)
    drive_events(sim)
    pass_now(sim, "fault", fault="UNKNOWN")
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("down"))
    pass_now(sim, "crash")
    agents.run_scripted_turn(sim, trigger_ids=("EVT0001",))
    w = sim.snapshot
    for payload in (
        w,
        w.route_passes,
        w.task_log,
        w.rejections,
        [e.text for e in w.log],
        views.resilience_lines(w),
        views.route_pass_rows(w),
        views.coordinator_turn_rows(w),
        resilience.operator_actions(w),
        resilience.latency_report(w.route_passes).summary(),
        sim.preview_plan(),
    ):
        assert_no_hidden(payload)
    tb = Toolbox(sim.snapshot, sim.geometry, sim.circuits, sim._estimation)
    assert_no_hidden([n for n in dir(tb) if not n.startswith("_")])
