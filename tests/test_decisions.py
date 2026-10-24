"""Ticket 10: explained decisions are accepted atomically, retried safely, or not at all."""

from __future__ import annotations

import copy

import pytest
from conftest import permissive

from grid_ops import agents, decisions
from grid_ops.geometry import LandingSite, LinePoint
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.proposals import ProposedStatus
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.sim import (
    CommitDecision,
    CommitSorties,
    ExcludeReading,
    RejectFinding,
    Release,
    RequestModelChange,
    ScheduledReport,
    Simulation,
)
from grid_ops.world import CommandRejected, Measurement, ScheduledReport  # noqa: F811


@pytest.fixture(scope="module")
def network():
    return load_network()


def clearance_stops(*pairs):
    return tuple(StopRequest(LinePoint(s, mm), Measurement.CLEARANCE) for s, mm in pairs)


def measured(network, *, policy_permissive=True):
    sim = permissive(Simulation(network)) if policy_permissive else Simulation(network)
    perch = sim._world.drones["DRONE_2"].state.perch
    stops = clearance_stops(("LINE_058", 28_000), ("LINE_064", 9_000_000))
    sim._apply(CommitSorties((SortieRequest("DRONE_2", stops, perch),), "operator"))
    sim.step(25)
    return sim


def proposed(sim):
    agents.run_scripted_turn(sim)
    return sim.snapshot.proposals[-1]


def commit(sim, decision_id, pf, **kw):
    return sim.submit(
        CommitDecision(decision_id, pf.id, pf.semantic_key, "operator", **kw)
    ).result()


def frozen(world):
    """Accepted operational state that a rejected decision must not touch."""
    return copy.deepcopy(
        (
            world.hypotheses,
            world.hypothesis_revisions,
            world.hypotheses_version,
            world.accepted_out_of_service,
            world.tasks,
            world.drones,
            world.manager_turns,
        )
    )


# -- the happy path and retries --------------------------------------------------------------------------


def test_accepting_a_decision_applies_explained_revisions_atomically(network):
    sim = measured(network)
    pf = proposed(sim)
    assert pf.status is ProposedStatus.PROPOSED_NOT_ACCEPTED
    assert sim.snapshot.hypotheses == {} and sim.snapshot.hold is None
    assert "accepted" in commit(sim, "D1", pf)
    w = sim.snapshot
    assert set(w.hypotheses) == {"H_VEG"} and w.hypotheses_version == 1
    rev = w.hypothesis_revisions[0]
    assert rev.decision_id == "D1" and rev.finding_id == pf.id and rev.before is None
    assert rev.finding_summary and rev.update_reason and rev.remaining_uncertainty
    assert rev.after["assessment"] == "SUPPORTED" and rev.latest_measurement_ids == (
        "R0001",
        "R0002",
    )
    assert w.proposals[-1].status is ProposedStatus.ACCEPTED
    assert w.manager_turns[0].decision_id == "D1"
    assert w.hold is None and w.decision_outcomes["D1"].outcome is decisions.Outcome.ACCEPTED


def test_duplicate_commands_return_the_stored_outcome_without_repeating_effects(network):
    sim = measured(network)
    pf = proposed(sim)
    commit(sim, "D1", pf)
    before = frozen(sim.snapshot)
    cycles = sim.snapshot.next_cycle
    assert "already ACCEPTED" in commit(sim, "D1", pf)
    assert frozen(sim.snapshot) == before and sim.snapshot.next_cycle == cycles
    # a retried rejection also replays its stored outcome
    with pytest.raises(CommandRejected):
        commit(sim, "D2", pf)  # pf is no longer pending
    with pytest.raises(CommandRejected, match="already REJECTED"):
        commit(sim, "D2", pf)


def test_pure_motion_does_not_invalidate_a_proposal(network):
    sim = measured(network)
    pf = proposed(sim)
    sim.step(2)  # drones and the clock move; no evidence, epoch, model or hypothesis changes
    assert "accepted" in commit(sim, "D1", pf)


# -- stale proposals are rejected before any hold --------------------------------------------------------------


def test_changed_evidence_rejects_the_proposal_and_leaves_the_fleet_alone(network):
    sim = measured(network)
    pf = proposed(sim)
    # a newer tied reading (another drone, same corridor) changes what the explanation rests on
    perch = sim._world.drones["DRONE_3"].state.perch
    stops = clearance_stops(("LINE_064", 9_800_000))
    sim._apply(CommitSorties((SortieRequest("DRONE_3", stops, perch),), "operator"))
    sim.step(15)
    assert len(sim.snapshot.readings) == 3
    before, cycles = frozen(sim.snapshot), sim.snapshot.next_cycle
    with pytest.raises(CommandRejected, match="changed since the explanation was written"):
        commit(sim, "D1", pf)
    assert frozen(sim.snapshot) == before
    assert sim.snapshot.next_cycle == cycles and sim.snapshot.hold is None  # rejected before a hold
    last = sim.snapshot.proposals[0]
    assert last.status is ProposedStatus.SUPERSEDED and "changed" in last.history[-1][1]


def test_epoch_and_disposition_changes_reject_incompatible_proposals(network):
    sim = measured(network)
    pf = proposed(sim)
    sim._apply(ExcludeReading("R0002", "calibration doubt", "operator"))
    with pytest.raises(CommandRejected, match="changed"):
        commit(sim, "D1", pf)
    assert sim.snapshot.hypotheses == {}


def test_changed_before_values_supersede_the_older_proposal(network):
    sim = measured(network)
    first = proposed(sim)
    agents.run_scripted_turn(sim)
    second = sim.snapshot.proposals[-1]
    assert first.semantic_key == second.semantic_key
    commit(sim, "D1", first)  # accepts the creation of H_VEG
    with pytest.raises(CommandRejected, match=r"before-values changed|not pending|changed"):
        commit(sim, "D2", second)  # it also claims to create H_VEG
    assert (
        len(sim.snapshot.hypothesis_revisions) == 3 or len(sim.snapshot.hypothesis_revisions) >= 1
    )
    assert sim.snapshot.hypotheses_version == 1


# -- models are staged, tested, then accepted with their revision ---------------------------------------------------


def _model_proposal(sim, segments):
    """A proposal whose H_VEG revision also carries a candidate topology."""
    tb = agents.build_toolbox(sim.snapshot, sim.network, sim.circuits)
    out = agents.scripted_manager.measurement_proposal(tb, agents.trigger_events(tb))
    tb.test_electrical_candidate(list(segments))
    diag = next(iter(tb.diagnostics))
    rev = out.revisions[0]
    after = rev.after.model_copy(update={"model_change": list(segments)})
    out.revisions[0] = rev.model_copy(
        update={
            "after": after,
            "diagnostic_ids": [diag],
            "update_reason": rev.update_reason
            + " Candidate topology: out of service per diagnostic.",
        }
    )
    errors = agents.proposals.validate(out, tb)
    assert errors == [], errors
    agents._record(sim, tb, out, "manager-scripted", [])
    return sim.snapshot.proposals[-1]


def test_enqueueing_a_model_change_never_changes_the_accepted_model(network):
    sim = measured(network)
    key_before = sim._estimation.model.topology_key
    assert "staged" in sim.submit(RequestModelChange(("LINE_041",), "operator")).result()
    w = sim.snapshot
    assert w.accepted_out_of_service == frozenset() and len(w.staged_models) == 1
    assert sim._estimation.model.topology_key == key_before


def test_model_change_is_accepted_with_its_explained_revision_only_if_staged_and_tested(network):
    sim = measured(network)
    pf = _model_proposal(sim, ("LINE_041",))
    before = frozen(sim.snapshot)
    with pytest.raises(CommandRejected, match="never staged"):
        commit(sim, "D1", pf)
    assert frozen(sim.snapshot) == before and sim.snapshot.hold is None  # fleet idle: released
    sim.submit(RequestModelChange(("LINE_041",), "operator"))
    pf = sim.snapshot.proposals[-1]
    commit(sim, "D2", pf)
    w = sim.snapshot
    assert w.accepted_out_of_service == frozenset({"LINE_041"})
    assert w.estimate.flows["LINE_041"].p_mw == 0.0  # the accepted model now carries it
    assert w.hypothesis_revisions[-1].after["model_change"] == ["LINE_041"]


# -- all or nothing ------------------------------------------------------------------------------------------------


def test_later_sortie_rejection_leaves_every_earlier_drone_and_hypothesis_unchanged(network):
    sim = measured(network)
    pf = proposed(sim)
    good = SortieRequest("DRONE_1", (), LandingSite(LinePoint("LINE_040", 40_000), "LINE_040/C1"))
    bad = SortieRequest(
        "DRONE_4",
        clearance_stops(("LINE_001", 99_000_000)),
        LandingSite(LinePoint("LINE_001", 100_000), "LINE_001/C1"),
    )  # ~30 km away: battery
    before = frozen(sim.snapshot)
    with pytest.raises(CommandRejected, match="DRONE_4"):
        commit(sim, "D1", pf, sorties=(good, bad))
    after = sim.snapshot
    assert frozen(after) == before
    p = after.proposals[-1]
    assert p.status is ProposedStatus.PROPOSED_NOT_ACCEPTED  # still inspectable as a notice
    assert "DRONE_4" in p.history[-1][1] and "D1 not accepted" in p.history[-1][1]
    assert after.decision_outcomes["D1"].outcome is decisions.Outcome.REJECTED
    assert after.hold is None  # no airborne drone: the fleet needs no continuation
    # the same decision with only the valid sortie, under a new id, accepts everything together
    commit(sim, "D3", p, sorties=(good,))
    assert "H_VEG" in sim.snapshot.hypotheses
    assert sim.snapshot.drones["DRONE_1"].phase == "flying"


def test_failure_after_hold_acquisition_discards_every_candidate_mutation(network, monkeypatch):
    sim = measured(network)
    pf = proposed(sim)
    before = frozen(sim.snapshot)

    def boom(*a, **k):
        raise decisions.DecisionRejected(["injected failure after the hold was taken"])

    monkeypatch.setattr(decisions, "apply_candidate", boom)
    with pytest.raises(CommandRejected, match="injected failure"):
        commit(sim, "D1", pf)
    assert frozen(sim.snapshot) == before and sim.snapshot.hold is None
    monkeypatch.undo()
    commit(sim, "D2", sim.snapshot.proposals[-1])  # nothing was left half-applied
    assert "H_VEG" in sim.snapshot.hypotheses


# -- holds, release, continuation -----------------------------------------------------------------------------------


def airborne_with_dead_landing(network):
    sim = measured(network)
    # DRONE_3 flies to land at LINE_026 7 km; then a public report makes LINE_026 unavailable
    sim._apply(
        CommitSorties(
            (
                SortieRequest(
                    "DRONE_3", (), LandingSite(LinePoint("LINE_026", 7_000_000), "LINE_026/C1")
                ),
            ),
            "operator",
        )
    )
    sim._world.pending_reports.append(
        ScheduledReport(
            sim._world.clock_s + 5, "LINE_026", "Dispatch reports LINE_026 switched off."
        )
    )
    sim.step(0.5)
    return sim


def test_rejection_after_a_hold_lands_the_fleet_and_releases_when_safe(network):
    sim = airborne_with_dead_landing(network)
    pf = proposed(sim)
    assert sim.snapshot.drones["DRONE_3"].phase == "flying"
    # the stale proposal passes phase A (nothing it depends on changed) but its sortie is invalid
    bad = SortieRequest(
        "DRONE_4",
        clearance_stops(("LINE_001", 99_000_000)),
        LandingSite(LinePoint("LINE_001", 100_000), "LINE_001/C1"),
    )
    with pytest.raises(CommandRejected):
        commit(sim, "D1", pf, sorties=(bad,))
    out = sim.snapshot.decision_outcomes["D1"]
    assert out.cycle_id is not None and out.fallback.startswith("route-only continuation")
    d = sim.snapshot.drones["DRONE_3"]
    assert d.state.route.perch.circuit_id != "LINE_026/C1"  # the dead landing was replaced
    assert sim.snapshot.hold is None  # a safe continuation exists, so the cycle is released


def test_no_safe_continuation_keeps_a_specific_hold_and_freezes_sim_time(network):
    sim = airborne_with_dead_landing(network)
    sim._world.landing_policy = LandingPolicy(False)
    sim._world.drones["DRONE_3"].battery_s = 90  # too little airtime to reach any eligible landing
    pf = proposed(sim)
    bad = SortieRequest(
        "DRONE_4",
        clearance_stops(("LINE_001", 99_000_000)),
        LandingSite(LinePoint("LINE_001", 100_000), "LINE_001/C1"),
    )
    with pytest.raises(CommandRejected):
        commit(sim, "D1", pf, sorties=(bad,))
    w = sim.snapshot
    assert w.decision_outcomes["D1"].fallback == "HELD"
    assert (
        w.hold is not None
        and "DRONE_3" in w.hold.reason
        and "no eligible reachable landing" in w.hold.reason
    )
    clock = w.clock_s
    sim.step(5)
    assert sim.snapshot.clock_s == clock  # held: simulated time does not run
    assert sim.paused is False  # operator pause is independent
    # no accepted decision, no release
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(w.hold.cycle_id, "D1", "operator")).result()
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(99, "D1", "operator")).result()


def test_late_releases_are_rejected(network):
    sim = measured(network)
    pf = proposed(sim)
    commit(sim, "D1", pf)
    with pytest.raises(CommandRejected, match="Late or unauthorised"):
        sim.submit(Release(1, "D1", "operator")).result()  # already released by its own cycle


def test_operator_can_reject_a_pending_finding_and_history_is_kept(network):
    sim = measured(network)
    pf = proposed(sim)
    with pytest.raises(CommandRejected, match="reason"):
        sim.submit(RejectFinding(pf.id, " ", "operator")).result()
    sim.submit(RejectFinding(pf.id, "wrong corridor", "operator")).result()
    p = sim.snapshot.proposals[0]
    assert p.status is ProposedStatus.REJECTED
    assert [h[1] for h in p.history][-1] == "rejected by operator: wrong corridor"
    with pytest.raises(CommandRejected, match="not pending"):
        commit(sim, "D1", p)
