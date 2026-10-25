"""Decision transactions on the three-method runtime: atomic, idempotent, or not at all.

A decision accepts a proposal (hypotheses, request lifecycle, optional sorties) in one block or
leaves every observable state untouched. Retries replay the stored outcome; stale proposals are
archived as SUPERSEDED before any hold; a rejection after a hold falls back to the validated
route-only continuation, and holds release only on a safe continuation.
"""

from __future__ import annotations

import copy
from dataclasses import replace as dataclasses_replace

import pytest
from helpers_p2 import inject_lidar, publish, tiny_sim

from grid_ops import agents, decisions
from grid_ops.coordinator import Coordinator
from grid_ops.energy import BatteryState
from grid_ops.geometry import LandingSite, LinePoint
from grid_ops.manager_tools import build_toolbox
from grid_ops.proposals import ProposedStatus
from grid_ops.routes import ScheduledStep, SortieRequest
from grid_ops.sensing import RequestStatus
from grid_ops.sim import (
    CommitDecision,
    CommitSorties,
    ReassignEpoch,
    RecordProposal,
    RejectFinding,
    Release,
)
from grid_ops.world import (
    LAND_S,
    TAKEOFF_S,
    CommandRejected,
    Land,
    Takeoff,
    Transit,
    leg_seconds,
)


def accepted_sim(**kw):
    """A tiny Simulation whose startup proposal (hypotheses + measurement requests) is accepted."""
    sim = tiny_sim(**kw)
    Coordinator(sim).run_turn_now()
    return sim


def to_reading(sim, deadline_s=4 * 60 * 60):
    """Advance until a reading is published."""
    while not sim.snapshot.readings and sim.snapshot.clock_s < deadline_s:
        sim.step(20)
    assert sim.snapshot.readings


def flow_sim(**kw):
    """Startup accepted, the first plan flown out, at least one reading published."""
    sim = accepted_sim(**kw)
    sim.submit_route_pass("turnover")
    to_reading(sim)
    return sim


def recorded_proposal(sim, trigger_kinds=("reading", "switching_report", "evidence_expired")):
    """One scripted manager turn on the current evidence, recorded as a pending proposal."""
    w = sim.snapshot
    tb = build_toolbox(w, sim.network, sim.circuits)
    triggers = tuple(i for i, e in w.events if e.kind in trigger_kinds)
    result = agents.run_scripted_turn(tb, triggers)
    assert result.errors == [], result.errors
    sim.submit(
        RecordProposal(
            "manager-scripted", w.revision, result.output, ProposedStatus.PROPOSED_NOT_ACCEPTED,
            "test proposal", decisions.semantic_key(w, result.output),
        )
    )
    return sim.snapshot.proposals[-1]


def commit(sim, decision_id, pf, **kw):
    return sim.submit(CommitDecision(decision_id, pf.id, pf.semantic_key, "operator", **kw)).result()


def frozen(world):
    """Accepted operational state that a rejected decision must not touch."""
    return copy.deepcopy(
        (
            world.hypotheses,
            world.hypotheses_version,
            world.requests,
            world.tasks,
            world.drones,
            world.manager_turns,
            world.hold,
        )
    )


def flight_sortie(sim, drone_id, stops):
    """A multi-leg takeoff-transit-land itinerary ending perched at the last stop, so the drone
    stays executing with remaining steps while the tests act on it."""
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


def broken_sortie(sim, drone_id, seg, mm):
    """A well-shaped itinerary that fails validation: it assumes a stale energy model."""
    req = flight_sortie(sim, drone_id, [(seg, mm)])
    return SortieRequest(
        req.drone_id, req.steps, req.final_perch, req.based_on_revision, "tampered-energy-model",
        req.sensor_model_revision,
    )


def flying_drone(sim, drone_id="DRONE_1", stops=(("LINE_002", 400_000), ("LINE_026", 400_000))):
    """Put a drone in the air, mid transit, with remaining steps after the current one."""
    sim.submit(CommitSorties((flight_sortie(sim, drone_id, list(stops)),), "operator")).result()
    sim.step(0.5)  # exactly past the takeoff: mid air, itinerary not yet half done
    assert sim._world.drones[drone_id].phase == "executing"
    return sim._world.drones[drone_id]


def sample_flight_sortie(sim, drone_id, task_id, final_stop):
    """Fly to a compiled ground task, sample it, then continue to a separate final perch."""
    from grid_ops.world import GroundSample

    w = sim._world
    task = w.tasks[task_id]
    drone = w.drones[drone_id]
    site = LandingSite(task.acquisition.conductor.point, task.acquisition.conductor.circuit_id)
    last = drone.state.perch
    here = sim.geometry.position_of(last.point)
    t = w.clock_s
    stops = [(site, None), (final_stop, None)]
    steps = []
    for target, _ in stops:
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
        if target == site:  # the ground sample happens at the task site
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


def request_proposal(sim, request):
    """A pending proposal that only carries one measurement request (validated first)."""
    from grid_ops import proposals as proposals_mod
    from grid_ops.hypotheses import ManagerOutput

    w = sim.snapshot
    out = ManagerOutput(summary=f"request {request.id}", measurement_requests=[request])
    tb = build_toolbox(w, sim.network, sim.circuits)
    assert proposals_mod.validate(out, tb) == []
    sim.submit(
        RecordProposal(
            "manager-scripted", w.revision, out, ProposedStatus.PROPOSED_NOT_ACCEPTED,
            "test request", decisions.semantic_key(w, out),
        )
    )
    return sim.snapshot.proposals[-1]


def new_contact_request(sim, request_id, circuit_id="LINE_002/C1"):
    from grid_ops.sensing import ConductorScope, CurrentRequest, OutcomeSpec

    return CurrentRequest(
        id=request_id,
        hypothesis_ids=(),
        rationale="test the request lifecycle",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=5,
        source_revision=str(sim.snapshot.revision),
        replaces_request_ids=(),
        target=ConductorScope(circuit_id=circuit_id, phase="A", start_mm=0, end_mm=1_000_000),
    )


# -- the happy path and retries --------------------------------------------------------------------------------------


def test_accepting_a_decision_applies_explained_revisions_atomically():
    sim = flow_sim()
    pf = recorded_proposal(sim)
    assert pf.status is ProposedStatus.PROPOSED_NOT_ACCEPTED
    assert "accepted" in commit(sim, "D1", pf)
    w = sim.snapshot
    assert w.hypotheses and w.hypotheses_version >= 1
    rev = w.hypothesis_revisions[-1]
    assert rev.decision_id == "D1" and rev.finding_id == pf.id and rev.before is not None
    assert rev.finding_summary and rev.update_reason and rev.remaining_uncertainty
    assert w.requests and all(a.status is RequestStatus.ACTIVE for a in w.requests.values())
    assert w.proposals[-1].status is ProposedStatus.ACCEPTED
    assert w.manager_turns[-1].decision_id == "D1" and w.manager_turns[-1].finding_id == pf.id
    assert w.hold is None and w.decision_outcomes["D1"].outcome is decisions.Outcome.ACCEPTED


def test_duplicate_commands_return_the_stored_outcome_without_repeating_effects():
    sim = flow_sim()
    pf = recorded_proposal(sim)
    commit(sim, "D1", pf)
    before, cycles = frozen(sim.snapshot), sim.snapshot.next_cycle
    assert "already ACCEPTED" in commit(sim, "D1", pf)
    assert frozen(sim.snapshot) == before and sim.snapshot.next_cycle == cycles
    # a retried rejection also replays its stored outcome
    with pytest.raises(CommandRejected, match="not pending"):
        commit(sim, "D2", pf)  # a fresh id, but the proposal is no longer pending
    with pytest.raises(CommandRejected, match="already REJECTED"):
        commit(sim, "D2", pf)


def test_pure_motion_does_not_invalidate_a_proposal():
    sim = flow_sim()
    pf = recorded_proposal(sim)
    sim.step(2)  # the clock and drones move; no evidence, epoch, model or request changes
    assert "accepted" in commit(sim, "D1", pf)


# -- stale proposals are superseded before any hold -------------------------------------------------------------------


def test_changed_evidence_rejects_the_proposal_and_leaves_the_fleet_alone():
    sim = flow_sim()
    pf = recorded_proposal(sim)
    before, cycles = frozen(sim.snapshot), sim.snapshot.next_cycle
    # a fresh reading over an untied corridor changes what the explanation rests on
    seg = "LINE_056"
    inject_lidar(sim, "RNEW1", seg, 0, sim.geometry.length_mm(seg), 2.4, world=sim._world)
    publish(sim._world, sim)
    with pytest.raises(CommandRejected, match="changed since the explanation was written"):
        commit(sim, "D1", pf)
    assert frozen(sim.snapshot) == before
    assert sim.snapshot.next_cycle == cycles and sim.snapshot.hold is None  # rejected before a hold
    assert sim.snapshot.proposals[-1].status is ProposedStatus.SUPERSEDED
    assert "changed" in sim.snapshot.proposals[-1].history[-1][1]


def test_epoch_reassignment_rejects_incompatible_proposals():
    sim = flow_sim()
    pf = recorded_proposal(sim)
    rid = sorted(sim.snapshot.readings)[0]
    sim.submit(ReassignEpoch(rid, "post-repair-epoch", "recheck after repairs", "operator")).result()
    with pytest.raises(CommandRejected, match="changed"):
        commit(sim, "D1", pf)
    assert sim.snapshot.proposals[-1].status is ProposedStatus.SUPERSEDED


def inject_current(sim, rid, seg, mm, rms_a=12.0, uncertainty_a=1.0, *, world=None):
    """Publish one contact-current reading into the live world (as a landed drone would)."""
    from grid_ops.sensing import (
        AcquisitionQuality,
        ConductorPoint,
        ContactCurrent,
        CurrentPayload,
        Reading,
    )
    from grid_ops.world import Event

    w = world if world is not None else sim._world
    conductor = ConductorPoint(LinePoint(seg, mm), f"{seg}/C1")
    acq = ContactCurrent(conductor, 10)
    payload = CurrentPayload(conductor, rms_a, uncertainty_a, 10)
    reading = Reading(
        id=rid,
        task_id="",
        request_ids=(),
        drone_id="DRONE_5",
        acquisition=acq,
        acquired_start_s=w.clock_s,
        acquired_end_s=w.clock_s + 10,
        payload=payload,
        quality=AcquisitionQuality(True),
    )
    w.readings[rid] = reading
    w.events.append((f"EVT{len(w.events) + 1:04d}", Event(w.clock_s, "reading", "DRONE_5", rid)))
    w.revision += 1
    publish(w, sim)
    return reading


def test_changed_before_values_supersede_the_older_proposal():
    sim = flow_sim()
    first = recorded_proposal(sim)
    commit(sim, "D1", first)  # revises the accepted hypotheses
    # fresh evidence for both hypotheses so the next scripted turn has something to revise
    seg = "LINE_002"
    inject_lidar(sim, "RNEW2", seg, 0, sim.geometry.length_mm(seg), 0.2, world=sim._world)
    inject_current(sim, "RNEW3", "LINE_040", 200_000)
    second = recorded_proposal(sim)
    newer = recorded_proposal(sim)  # same before-values as `second`
    commit(sim, "D2", newer)  # the accepted state moves again
    # `second` now rests on stale before-values: it is superseded, never half-applied
    with pytest.raises(CommandRejected, match="changed since the explanation was written"):
        commit(sim, "D3", second)
    live = next(p for p in sim.snapshot.proposals if p.id == second.id)
    assert live.status is ProposedStatus.SUPERSEDED


# -- all or nothing ----------------------------------------------------------------------------------------------------


def test_later_sortie_rejection_leaves_every_earlier_drone_and_hypothesis_unchanged():
    sim = flow_sim()
    flying_drone(sim, "DRONE_1")
    pf = recorded_proposal(sim)
    good = flight_sortie(sim, "DRONE_2", [("LINE_056", 400_000)])
    bad = broken_sortie(sim, "DRONE_3", "LINE_002", 700_000)
    before = frozen(sim.snapshot)
    with pytest.raises(CommandRejected, match="DRONE_3"):
        commit(sim, "D1", pf, sorties=(good, bad))
    after = sim.snapshot
    assert frozen(after) == before
    assert after.proposals[-1].status is ProposedStatus.PROPOSED_NOT_ACCEPTED
    assert "D1 not accepted" in after.proposals[-1].history[-1][1]
    assert after.decision_outcomes["D1"].outcome is decisions.Outcome.REJECTED
    assert after.hold is None  # the executing drone has a safe continuation
    # the same proposal with only the valid sortie, under a new id, accepts everything together
    commit(sim, "D3", pf, sorties=(good,))
    assert sim.snapshot.drones["DRONE_2"].phase == "executing"
    assert sim.snapshot.decision_outcomes["D3"].outcome is decisions.Outcome.ACCEPTED


def test_failure_after_hold_acquisition_discards_every_candidate_mutation(monkeypatch):
    sim = flow_sim()
    flying_drone(sim, "DRONE_1")
    pf = recorded_proposal(sim)
    before = frozen(sim.snapshot)

    def boom(*a, **k):
        raise decisions.DecisionRejected(["injected failure after the hold was taken"])

    monkeypatch.setattr(decisions, "apply_candidate", boom)
    with pytest.raises(CommandRejected, match="injected failure"):
        commit(sim, "D1", pf)
    assert frozen(sim.snapshot) == before and sim.snapshot.hold is None
    monkeypatch.undo()
    commit(sim, "D2", sim.snapshot.proposals[-1])  # nothing was left half-applied
    assert sim.snapshot.decision_outcomes["D2"].outcome is decisions.Outcome.ACCEPTED


# -- holds, release, continuation ----------------------------------------------------------------------------------------


def test_rejection_after_a_hold_replaces_a_no_longer_valid_continuation():
    sim = flow_sim()
    w = sim._world
    # a fresh active request and its compiled task, served by DRONE_1 on a two-stop itinerary
    pf = request_proposal(sim, new_contact_request(sim, "REQ-TEST"))
    commit(sim, "DR", pf)
    task = next(t for t in w.tasks.values() if t.request_id == "REQ-TEST")
    sim.submit(
        CommitSorties(
            (
                sample_flight_sortie(sim, "DRONE_1", task.id, LandingSite(
                    LinePoint("LINE_026", 400_000), "LINE_026/C1")),
            ),
            "operator",
        )
    ).result()
    sim.step(0.5)  # airborne, mid transit
    target = w.drones["DRONE_1"].state.itinerary.final_perch
    # the request the remaining ground sample serves is withdrawn: the continuation is stale
    w.requests["REQ-TEST"] = dataclasses_replace(
        w.requests["REQ-TEST"], status=RequestStatus.RETIRED, retired_reason="withdrawn by test"
    )
    publish(w, sim)
    prop = recorded_proposal(sim)
    bad = broken_sortie(sim, "DRONE_3", "LINE_056", 400_000)
    with pytest.raises(CommandRejected):
        commit(sim, "D1", prop, sorties=(bad,))
    out = sim.snapshot.decision_outcomes["D1"]
    assert out.cycle_id is not None and out.fallback.startswith("route-only continuation")
    d = sim.snapshot.drones["DRONE_1"]
    assert d.state.itinerary.final_perch != target  # the stale continuation was replaced
    assert sim.snapshot.hold is None  # a safe continuation exists, so the cycle is released


def test_no_safe_continuation_keeps_a_specific_hold_and_freezes_sim_time():
    sim = flow_sim()
    w = sim._world
    perch = w.drones["DRONE_1"].state.perch
    drone = flying_drone(sim, "DRONE_1", stops=[(perch.point.segment_id, perch.point.offset_mm + 1_000)])
    # the remaining itinerary needs more energy than the battery now holds, and every reachable
    # landing needs more still: no complete safe continuation exists
    drone.battery = BatteryState(remaining_units=1_000)
    publish(w, sim)
    prop = recorded_proposal(sim)
    with pytest.raises(CommandRejected):
        commit(sim, "D1", prop)
    world = sim.snapshot
    assert world.decision_outcomes["D1"].fallback == "HELD"
    assert world.hold is not None and "DRONE_1" in world.hold.reason
    clock = world.clock_s
    sim.step(5)
    assert sim.snapshot.clock_s == clock  # held: simulated time does not run
    assert sim.paused is False  # operator pause is independent
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(world.hold.cycle_id, "D1", "operator")).result()
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(99, "D1", "operator")).result()


def test_late_releases_are_rejected():
    sim = flow_sim()
    pf = recorded_proposal(sim)
    commit(sim, "D1", pf)
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(1, "D1", "operator")).result()  # released by its own acceptance cycle


def test_operator_can_reject_a_pending_finding_and_history_is_kept():
    sim = flow_sim()
    pf = recorded_proposal(sim)
    with pytest.raises(CommandRejected, match="reason"):
        sim.submit(RejectFinding(pf.id, " ", "operator")).result()
    sim.submit(RejectFinding(pf.id, "wrong corridor", "operator")).result()
    p = next(x for x in sim.snapshot.proposals if x.id == pf.id)
    assert p.status is ProposedStatus.REJECTED
    assert [h[1] for h in p.history][-1] == "rejected by operator: wrong corridor"
    with pytest.raises(CommandRejected, match="not pending"):
        commit(sim, "D1", p)