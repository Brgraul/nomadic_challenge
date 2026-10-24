"""Ticket 15: the manager and planner run asynchronously while accepted routes keep flying."""

from __future__ import annotations

import time

import pytest
from conftest import permissive

from grid_ops import agents, coordinator
from grid_ops.coordinator import ASSESSED_EVENT_KINDS, Coordinator
from grid_ops.geometry import LandingSite, LinePoint
from grid_ops.grid import load_network
from grid_ops.proposals import ProposedStatus
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import CommitDecision, ExcludeReading, Release, SetLandingPolicy, Simulation
from grid_ops.world import ScheduledReport


@pytest.fixture(scope="module")
def network():
    return load_network()


def make_sim(network, *, speed=1.0, seed=7, auto=True, start=True):
    sim = permissive(
        Simulation(
            network,
            minutes_per_second=speed,
            scenario=ScenarioConfig(ScenarioMode.INCIDENT_DEMO, seed),
            auto_replan=False,
        )
    )
    agents.run_scripted_turn(sim)
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision("D0", pf.id, pf.semantic_key, "operator")).result()
    sim.auto_replan = auto
    if start:
        sim.start()
    return sim


def wait(cond, seconds=30.0, every=0.1):
    end = time.time() + seconds
    while time.time() < end:
        if cond():
            return True
        time.sleep(every)
    return False


def reading_event_ids(world):
    return {i for i, e in world.events if e.kind == "reading"}


@pytest.fixture
def sims():
    made = []
    yield made
    for s in made:
        s.stop()


# -- evidence is never dropped ------------------------------------------------------------------------------------------


def test_every_reading_batch_reaches_the_manager_even_when_routes_are_blocked(network, sims):
    sim = make_sim(network, speed=15)
    sims.append(sim)
    sim.submit(SetLandingPolicy(True, "operator")).result()  # the fleet is dispatched ...
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("solver down"))
    seen: list[str] = []
    co = Coordinator(
        sim,
        manager_fn=lambda s, trigger_ids=(): (
            seen.extend(trigger_ids),
            agents.run_scripted_turn(s, trigger_ids),
        )[1],
        auto_accept=False,
    )
    co.start()
    try:
        assert wait(lambda: len(reading_event_ids(sim.snapshot)) >= 1, 40)
        sim.paused = True
        time.sleep(1.5)
        readings = reading_event_ids(sim.snapshot)
        assert wait(lambda: readings <= set(seen), 20), (readings, seen)
        assert all(
            sim.snapshot.route_passes[i].outcome
            in ("kept-incumbent", "no-op", "HELD", "landing-only")
            for i in range(len(sim.snapshot.route_passes))
            if "solver failure" in sim.snapshot.route_passes[i].solver
        )
    finally:
        co.stop()


def test_bursts_are_coalesced_into_a_turn_without_losing_trigger_ids(network):
    sim = make_sim(network, start=False, auto=False)
    calls: list[tuple[str, ...]] = []

    def manager(s, trigger_ids=()):
        calls.append(tuple(trigger_ids))
        return agents.run_scripted_turn(s, trigger_ids)

    co = Coordinator(sim, manager_fn=manager, auto_accept=False, debounce_s=0)
    co._on_batch(
        1, ("EVT0001", "EVT0002")
    )  # nonexistent ids are filtered like any irrelevant event
    assert co.pending_trigger_ids() == ()
    perch = sim._world.drones["DRONE_2"].state.perch
    from grid_ops.routes import SortieRequest, StopRequest
    from grid_ops.sim import CommitSorties
    from grid_ops.world import Measurement

    stops = tuple(
        StopRequest(LinePoint("LINE_064", mm), Measurement.CLEARANCE)
        for mm in (9_000_000, 9_800_000)
    )
    sim.submit(CommitSorties((SortieRequest("DRONE_2", stops, perch),), "operator")).result()
    sim.coordinator.listeners.append(co._on_batch)
    sim.step(25)
    ids = co.pending_trigger_ids()
    readings = reading_event_ids(sim.snapshot)
    assert readings and set(ids) == readings  # both readings queued for one assessment
    turn = co.run_turn_now()
    assert set(turn.trigger_ids) == readings and len(calls) == 1
    assert turn.status == "proposed"


# -- roles ---------------------------------------------------------------------------------------------------------------------------


def test_planner_runs_only_when_semantic_planning_inputs_change_and_authorises_nothing(network):
    sim = make_sim(network, start=False, auto=False)
    co = Coordinator(sim)
    drones_before = {i: d.state for i, d in sim._world.drones.items()}
    first = co.planner_step()
    assert first is not None and first.authorises_dispatch is False and first.summary
    assert co.planner_step() is None  # nothing changed: no new preview
    assert {i: d.state for i, d in sim._world.drones.items()} == drones_before  # no dispatch
    sim.submit(SetLandingPolicy(False, "operator")).result()
    second = co.planner_step()
    assert second is not None and second.semantic_key != first.semantic_key
    assert len(sim.snapshot.planner_previews) == 2


def test_planner_previews_explain_infeasibility(network):
    sim = make_sim(network, start=False, auto=False)
    for d in sim._world.drones.values():
        d.battery_s = 60
    sim.snapshot = __import__("copy").deepcopy(sim._world)
    prev = Coordinator(sim).planner_step()
    assert prev is not None and prev.unassigned
    assert all(reason for _, reason in prev.unassigned)


# -- delay, death, supersession ------------------------------------------------------------------------------------------


def test_a_manager_delayed_20_wall_seconds_cannot_hold_a_valid_fleet(network, sims):
    sim = make_sim(network, speed=30)
    sims.append(sim)
    started = {"t": None, "clock": None}

    def slow_manager(s, trigger_ids=()):
        started["t"], started["clock"] = time.time(), s.snapshot.clock_s
        time.sleep(20)
        return agents.run_scripted_turn(s, trigger_ids)

    co = Coordinator(sim, manager_fn=slow_manager, auto_accept=False)
    co.start()
    try:
        sim.submit(SetLandingPolicy(True, "operator")).result()
        assert wait(lambda: started["t"] is not None, 60)
        passes_at_start = len(sim.snapshot.route_passes)
        time.sleep(8)  # the manager is still thinking
        mid = sim.snapshot
        assert mid.clock_s > started["clock"] + 60  # simulated time kept flowing
        assert mid.hold is None  # nothing is held waiting for the model
        assert len(mid.route_passes) >= passes_at_start
        pending = [t for t in mid.coordinator_turns if t.status == "running"]
        assert pending and pending[0].trigger_ids is not None  # shown as pending reasoning
        assert mid.hypotheses_version == 1  # accepted state does not depend on the unfinished call
    finally:
        co.stop()


def test_incompatible_proposals_are_archived_as_superseded_and_regenerated(network):
    sim = make_sim(network, start=False, auto=False)
    from grid_ops.routes import SortieRequest, StopRequest
    from grid_ops.sim import CommitSorties
    from grid_ops.world import Measurement

    perch = sim._world.drones["DRONE_2"].state.perch
    stops = (StopRequest(LinePoint("LINE_064", 9_000_000), Measurement.CLEARANCE),)
    sim.submit(CommitSorties((SortieRequest("DRONE_2", stops, perch),), "operator")).result()
    sim.step(15)
    calls = []

    def racing_manager(s, trigger_ids=()):
        turn = agents.run_scripted_turn(s, trigger_ids)
        if not calls:  # evidence changes between the proposal and its acceptance
            s.submit(ExcludeReading("R0001", "calibration doubt", "operator")).result()
        calls.append(turn.finding_id)
        return turn

    co = Coordinator(sim, manager_fn=racing_manager)
    turn = co.run_turn_now(tuple(i for i, e in sim.snapshot.events if e.kind == "reading"))
    assert len(calls) >= 2  # regenerated on the new evidence
    statuses = [p.status for p in sim.snapshot.proposals]
    assert ProposedStatus.SUPERSEDED in statuses  # the stale one is archived, not lost
    assert turn.trigger_ids  # every trigger id survived the regeneration


def test_old_cycles_cannot_release_newer_holds(network):
    sim = make_sim(network, start=False, auto=True)
    w = sim._world
    first = sim._route_pass("first", (), 0.0, 0.0)
    cycle_a = w.next_cycle - 1
    w.hold = __import__("grid_ops.decisions", fromlist=["Hold"]).Hold(
        cycle_a + 1, "D1", "newer", w.clock_s
    )
    w.next_cycle += 1
    from grid_ops.world import CommandRejected

    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(cycle_a, "D1", "operator")).result()
    assert first.outcome


def test_coordinator_death_leaves_a_validated_continuation_and_accepted_state_intact(network, sims):
    sim = make_sim(network, speed=10)
    sims.append(sim)

    def dies(s, trigger_ids=()):
        raise SystemExit("manager process killed")

    co = Coordinator(sim, manager_fn=dies, auto_accept=False)
    co.start()
    hyp = dict(sim.snapshot.hypotheses)
    sim.submit(SetLandingPolicy(True, "operator")).result()
    assert wait(lambda: co.dead, 60)
    assert "SystemExit" in co.failure
    assert wait(
        lambda: any(p.trigger == "coordinator death" for p in sim.snapshot.route_passes), 30
    )
    w = sim.snapshot
    assert w.hypotheses == hyp  # nothing half-accepted
    assert any(p.trigger == "coordinator death" for p in w.route_passes)
    last = w.route_passes[-1]
    assert (
        w.hold is None or "no safe continuation" in w.hold.reason
    )  # continuation or explicit exception
    assert any(t.status == "failed" and "coordinator died" in t.note for t in w.coordinator_turns)
    clock = w.clock_s
    time.sleep(1.5)
    assert sim.snapshot.clock_s >= clock  # the simulation itself is unaffected
    _ = (last, ASSESSED_EVENT_KINDS, coordinator, LandingSite, ScheduledReport)
