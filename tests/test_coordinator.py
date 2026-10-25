"""Coordinator tests on the three-method runtime: the manager and planner roles around a fleet.

The simulation keeps flying and replanning whatever the manager is doing: the coordinator only
coalesces arriving evidence into the next manager turn without losing a trigger id, records the
proposal, and submits it back as an ordinary explained decision. A slow, failed or dead manager
never holds the fleet and never leaves half-accepted state behind.
"""

from __future__ import annotations

import time

import pytest
from helpers_p2 import inject_lidar, publish, tiny_sim

from grid_ops import agents, decisions
from grid_ops.coordinator import Coordinator
from grid_ops.proposals import ProposedStatus
from grid_ops.sim import ExcludeReading, Release, SetLandingPolicy
from grid_ops.world import CommandRejected


@pytest.fixture()
def sims():
    made = []
    yield made
    for s in made:
        s.stop()


def accepted_sim(**kw):
    """A tiny Simulation whose startup proposal (hypotheses + measurement requests) is accepted."""
    sim = tiny_sim(**kw)
    Coordinator(sim).run_turn_now()
    return sim


def flow_sim(**kw):
    """Startup accepted, the first plan flown out, at least one reading published."""
    sim = accepted_sim(**kw)
    sim.submit_route_pass("turnover")
    while not sim.snapshot.readings:
        sim.step(20)
    return sim


def live_sim(**kw):
    """Startup accepted and the first plan committed, so readings still arrive after start()."""
    sim = accepted_sim(**kw)
    sim.submit_route_pass("turnover")
    assert any(d.phase == "executing" for d in sim._world.drones.values())
    return sim


def reading_event_ids(world):
    return {i for i, e in world.events if e.kind == "reading"}


def wait(cond, seconds=30.0, every=0.05):
    end = time.time() + seconds
    while time.time() < end:
        if cond():
            return True
        time.sleep(every)
    return False


# -- batching: evidence is never dropped --------------------------------------------------------------------------------


def test_irrelevant_events_are_filtered_and_bursts_coalesce_into_one_turn():
    sim = flow_sim()
    co = Coordinator(sim, auto_accept=False, debounce_s=0)
    ids = reading_event_ids(sim.snapshot)
    assert ids
    co._on_batch(sim.snapshot.revision, ("EVT9999", sorted(ids)[0]))  # unknown ids are filtered
    assert co.pending_trigger_ids() == (sorted(ids)[0],)
    # a second burst before the turn runs is coalesced into the same pending batch
    inject_lidar(sim, "RPLUS1", "LINE_056", 0, sim.geometry.length_mm("LINE_056"), 2.4)
    publish(sim._world, sim)
    new_id = next(i for i, e in reversed(sim._world.events) if e.kind == "reading")
    co._on_batch(sim.snapshot.revision, ("EVT9998", "EVT9999", new_id))
    batch = co.pending_trigger_ids()
    assert len(batch) == 2 and "EVT9999" not in batch and "EVT9998" not in batch
    turn = co.run_turn_now()
    assert set(turn.trigger_ids) >= set(batch)  # every queued trigger reached the manager
    assert turn.status == "proposed" and co.turns_run == 1


def test_every_reading_batch_reaches_the_manager_even_when_routes_are_blocked(sims):
    sim = live_sim(minutes_per_second=180)
    sims.append(sim)
    sim.auto_replan = True
    sim.coordinator.planner = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("solver down"))
    seen: set[str] = set()

    def manager(tb, trigger_ids=()):
        seen.update(trigger_ids)
        return agents.run_scripted_turn(tb, trigger_ids)

    co = Coordinator(sim, manager_fn=manager, auto_accept=False)
    co.start()
    sim.start()
    try:
        assert wait(lambda: reading_event_ids(sim.snapshot), 60)
        sim.paused = True
        time.sleep(1.0)
        readings = reading_event_ids(sim.snapshot)
        assert wait(lambda: readings <= seen, 30), (readings, seen)
        for p in sim.snapshot.route_passes:
            if "solver failure" in p.solver:
                assert p.outcome in ("kept-incumbent", "no-op", "HELD", "landing-only")
    finally:
        co.stop()


# -- the planner role: previews that authorise nothing ------------------------------------------------------------------


def test_planner_previews_only_rerun_when_semantic_inputs_change():
    sim = flow_sim()
    co = Coordinator(sim)
    drones_before = {i: d.state for i, d in sim.snapshot.drones.items()}
    first = co.planner_step()
    assert first is not None and first.authorises_dispatch is False and first.summary
    assert co.planner_step() is None  # nothing changed: no new preview
    assert {i: d.state for i, d in sim.snapshot.drones.items()} == drones_before
    assert len(sim.snapshot.planner_previews) == 1
    sim.submit(SetLandingPolicy(True, "operator")).result()
    second = co.planner_step()
    assert second is not None and second.semantic_key != first.semantic_key
    assert len(sim.snapshot.planner_previews) == 2


def test_planner_previews_explain_infeasibility():
    from grid_ops.energy import BatteryState

    sim = accepted_sim()  # accepted requests with every task still open
    w = sim._world
    for d in w.drones.values():
        d.battery = BatteryState(remaining_units=1_200)
    publish(w, sim)
    prev = Coordinator(sim).planner_step()
    assert prev is not None and prev.unassigned
    assert all(reason for _, reason in prev.unassigned)


# -- delay, death, supersession ------------------------------------------------------------------------------------------


def test_a_delayed_manager_cannot_hold_the_fleet(sims):
    sim = live_sim(minutes_per_second=180)
    sims.append(sim)
    started = {}

    def slow_manager(tb, trigger_ids=()):
        started["clock"] = tb.world.clock_s
        time.sleep(1.5)
        return agents.run_scripted_turn(tb, trigger_ids)

    co = Coordinator(sim, manager_fn=slow_manager, auto_accept=False)
    co.start()
    sim.start()
    try:
        assert wait(lambda: "clock" in started, 60)  # a live reading reached the manager
        time.sleep(0.7)  # the manager is still thinking
        mid = sim.snapshot
        assert mid.clock_s > started["clock"] + 60  # simulated time kept flowing
        assert mid.hold is None  # nothing is held waiting for the model
        assert any(t.status == "running" and t.trigger_ids for t in mid.coordinator_turns)
        version = mid.hypotheses_version
        time.sleep(1.5)
        assert sim.snapshot.hypotheses_version == version  # nothing accepted mid-call
    finally:
        co.stop()


def test_superseded_proposals_are_archived_and_regenerated_with_triggers_kept():
    sim = flow_sim()
    co = Coordinator(sim)  # auto_accept: the recorded proposal is committed immediately
    triggers = reading_event_ids(sim.snapshot)
    original = co._proposal
    calls = []

    def racing(finding_id):
        pf = original(finding_id)
        if not calls:  # evidence changes between recording the proposal and accepting it
            sim.submit(
                ExcludeReading(
                    sorted(sim.snapshot.readings)[0], "calibration doubt", "operator"
                )
            ).result()
        calls.append(finding_id)
        return pf

    co._proposal = racing
    turn = co.run_turn_now(tuple(sorted(triggers)))
    assert len(calls) >= 2  # regenerated on the new evidence
    statuses = [p.status for p in sim.snapshot.proposals]
    assert ProposedStatus.SUPERSEDED in statuses  # the stale one is archived, not lost
    assert turn.status == "accepted"
    assert set(triggers) <= set(turn.trigger_ids)  # every trigger id survived the regeneration


def test_coordinator_death_leaves_a_validated_continuation_and_accepted_state_intact():
    sim = live_sim()
    hyp = dict(sim.snapshot.hypotheses)

    def dies(tb, trigger_ids=()):
        raise SystemExit("manager process killed")

    co = Coordinator(sim, manager_fn=dies, auto_accept=False)
    co.start()
    for _ in range(60):  # advance until a live reading wakes the loop and kills it
        if co.dead:
            break
        if sim.snapshot.readings:
            time.sleep(0.5)  # the loop wakes within 0.5 s and runs the fatal turn
        else:
            sim.step(20)
    assert co.dead, "the manager never ran"
    assert "SystemExit" in co.failure
    assert wait(
        lambda: any(p.trigger == "coordinator death" for p in sim.snapshot.route_passes), 30
    )
    w = sim.snapshot
    assert w.hypotheses == hyp  # nothing half-accepted
    assert any(
        t.status == "failed" and "coordinator died" in t.note for t in w.coordinator_turns
    )
    assert w.hold is None or "no safe continuation" in w.hold.reason
    clock = w.clock_s
    sim.step(5)
    assert sim.snapshot.clock_s >= clock  # the simulation itself is unaffected


def test_old_cycles_cannot_release_newer_holds():
    sim = flow_sim()
    sim.submit_route_pass("first")
    w = sim._world
    cycle_a = w.next_cycle - 1
    w.hold = decisions.Hold(cycle_a + 1, "D1", "newer", w.clock_s)
    w.next_cycle += 1
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(cycle_a, "D1", "operator")).result()