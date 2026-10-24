"""Ticket 16: concluding and reopening an outage investigation safely."""

from __future__ import annotations

import copy

import pytest
from conftest import permissive

from grid_ops import agents, scripted_manager
from grid_ops.conclusion import InvestigationStatus, benchmark_checks, investigation_report
from grid_ops.geometry import LinePoint
from grid_ops.grid import load_network
from grid_ops.hypotheses import AlternativeDisposition, ConclusionModel, ReopenModel
from grid_ops.manager_tools import build_toolbox
from grid_ops.proposals import validate
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import (
    AcceptLimits,
    CommitDecision,
    CommitSorties,
    ReopenInvestigation,
    Simulation,
)
from grid_ops.world import CommandRejected, Measurement, TaskStatus


@pytest.fixture(scope="module")
def network():
    return load_network()


def sim_with_hazard(network, *, airborne=False):
    """Startup hypotheses accepted, DRONE_2 has read the LINE_058 vegetation hazard."""
    sim = permissive(Simulation(network))
    agents.run_scripted_turn(sim)
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision("D0", pf.id, pf.semantic_key, "operator")).result()
    w = sim._world
    perch = w.drones["DRONE_2"].state.perch
    stops = [StopRequest(LinePoint("LINE_058", 28_000), Measurement.CLEARANCE)]
    if airborne:
        stops.append(StopRequest(LinePoint("LINE_064", 9_000_000), Measurement.CLEARANCE))
        stops.append(StopRequest(LinePoint("LINE_064", 8_000_000), Measurement.CLEARANCE))
    sim.submit(CommitSorties((SortieRequest("DRONE_2", tuple(stops), perch),), "operator")).result()
    sim.step(4 if airborne else 15)
    return sim


def report_evidence(sim):
    """The public switching status check on LINE_024: electrical evidence in the same component."""
    return next(
        e.id
        for e in sim._world.knowledge.evidence.values()
        if e.source == "report" and e.circuit_id.startswith("LINE_024")
    )


def proposal(sim, **conclusion_kw):
    tb = build_toolbox(sim.snapshot, sim.network, sim.circuits)
    out = scripted_manager.measurement_proposal(tb, agents.trigger_events(tb))
    base = dict(
        kind="cause_identified",
        cause_hypothesis_id="H_VEG",
        causal_statement="Vegetation contact on LINE_058 is the most likely cause of the "
        "protection event in the sector.",
        causal_link_evidence_ids=["HZ001", report_evidence(sim)],
        alternatives=[],
        recommendation_id="REC-H_VEG",
        remaining_uncertainty="The mechanism of the protection event is inferred, not observed.",
    )
    base.update(conclusion_kw)
    out.conclusion = ConclusionModel(**base)
    return tb, out


def errors(sim, **kw):
    tb, out = proposal(sim, **kw)
    return validate(out, tb)


# -- what completion requires -----------------------------------------------------------------------------------------------


def test_a_valid_conclusion_has_a_causal_link_alternatives_a_recommendation_and_uncertainty(
    network,
):
    sim = sim_with_hazard(network)
    assert errors(sim) == []


def test_a_detected_local_hazard_alone_cannot_complete_the_diagnosis(network):
    sim = sim_with_hazard(network)
    errs = errors(sim, causal_link_evidence_ids=["HZ001"])
    assert any("hazard alone cannot complete outage diagnosis" in e for e in errs)
    # the urgent hazard action itself stays available: its recommendation is still valid
    _, out = proposal(sim, causal_link_evidence_ids=["HZ001"])
    assert out.recommendations and out.recommendations[0].urgency == 5


@pytest.mark.parametrize(
    ("changes", "needle"),
    [
        ({"remaining_uncertainty": " "}, "remaining uncertainty is required"),
        ({"recommendation_id": "REC-NOPE"}, "scoped recommendation is required"),
        ({"cause_hypothesis_id": "H_MISSING"}, "not accepted or revised"),
        ({"causal_link_evidence_ids": ["HZ001", "R9999"]}, "R9999 does not exist"),
    ],
)
def test_conclusion_rejections(network, changes, needle):
    sim = sim_with_hazard(network)
    assert any(needle in e for e in errors(sim, **changes))


def test_material_alternatives_must_be_addressed(network):
    sim = sim_with_hazard(network)
    tb, out = proposal(sim)
    alt = out.revisions[0].after.model_copy(
        update={"id": "H_ALT", "priority": 4, "assessment": "PLAUSIBLE"}
    )
    out.revisions.append(
        out.revisions[0].model_copy(update={"hypothesis_id": "H_ALT", "after": alt})
    )
    assert any("material alternative H_ALT is not addressed" in e for e in validate(out, tb))
    out.conclusion.alternatives = [
        AlternativeDisposition(
            hypothesis_id="H_ALT", disposition="indistinguishable", note="no data"
        )
    ]
    assert not any("H_ALT is not addressed" in e for e in validate(out, tb))


def test_ambiguous_causes_stay_unresolved_with_a_discriminating_recommendation(network):
    sim = sim_with_hazard(network)
    tb, out = proposal(
        sim,
        kind="unresolved_ambiguous",
        discriminating_recommendation_id=None,
        cause_hypothesis_id=None,
    )
    assert any("discriminating-reading or field-inspection" in e for e in validate(out, tb))
    out.conclusion.discriminating_recommendation_id = "REC-H_VEG"
    assert validate(out, tb) == []
    agents._record(sim, tb, out, "manager-scripted", [])
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision("D9", pf.id, pf.semantic_key, "operator")).result()
    inv = sim.snapshot.investigation
    assert inv.status is InvestigationStatus.OPEN and "Discriminating step" in inv.unresolved
    assert any(
        t.status is TaskStatus.PENDING for t in sim.snapshot.tasks.values()
    )  # still collecting
    with pytest.raises(CommandRejected, match="no stated unresolved"):
        raise CommandRejected("There is no stated unresolved limit to accept.")
    sim.submit(
        AcceptLimits("Cause stays ambiguous between vegetation and a hot joint.", "operator")
    ).result()
    assert sim.snapshot.investigation.status is InvestigationStatus.CONCLUDED_WITH_LIMITS
    assert all(n.status.value == "RETIRED" for n in sim.snapshot.needs.values())


def test_operator_cannot_accept_limits_when_none_are_stated(network):
    sim = sim_with_hazard(network)
    with pytest.raises(CommandRejected, match="no stated unresolved limit"):
        sim.submit(AcceptLimits("close it", "operator")).result()


# -- accepted completion -----------------------------------------------------------------------------------------------------------------


def accept_conclusion(sim, did="DC"):
    tb, out = proposal(sim)
    assert validate(out, tb) == []
    agents._record(sim, tb, out, "manager-scripted", [])
    pf = sim.snapshot.proposals[-1]
    return sim.submit(CommitDecision(did, pf.id, pf.semantic_key, "operator")).result()


def test_accepted_completion_retires_needs_cancels_unstarted_work_and_recovers_the_fleet_atomically(
    network,
):
    sim = sim_with_hazard(network, airborne=True)
    w = sim._world
    d = w.drones["DRONE_2"]
    assert d.phase in ("flying", "measuring") and d.state.route.stops
    locked_before = [t.id for t in w.tasks.values() if t.status is TaskStatus.MEASURING]
    pending_before = [t.id for t in w.tasks.values() if t.status is TaskStatus.PENDING]
    assert pending_before
    accept_conclusion(sim)
    assert w.investigation.status is InvestigationStatus.CONCLUDED
    assert w.investigation.conclusion["cause_hypothesis_id"] == "H_VEG"
    assert all(n.status.value == "RETIRED" for n in w.needs.values())
    assert all(w.tasks[t].status is TaskStatus.CANCELLED for t in pending_before)  # unstarted: gone
    assert all(w.tasks[t].status is TaskStatus.MEASURING for t in locked_before)  # locked: finish
    d = w.drones["DRONE_2"]
    assert d.state.route.stops == () and d.state.route.perch  # recovery: landing only
    assert w.hold is None
    sim.step(30)
    assert sim.snapshot.drones["DRONE_2"].phase in ("charging", "ready")  # it landed safely
    assert not [t for t in sim.snapshot.tasks.values() if t.status is TaskStatus.PENDING]


def test_failed_recovery_leaves_the_decision_unaccepted(network, monkeypatch):
    from grid_ops import decisions

    sim = sim_with_hazard(network, airborne=True)
    monkeypatch.setattr(decisions, "recovery_requests", lambda *a: ((), ["DRONE_2: no landing"]))
    before = copy.deepcopy((sim._world.hypotheses, sim._world.investigation))
    with pytest.raises(CommandRejected, match="no landing"):
        accept_conclusion(sim)
    assert (sim._world.hypotheses, sim._world.investigation) == before
    assert sim._world.investigation.status is InvestigationStatus.OPEN


def test_incomplete_flow_observability_neither_blocks_completion_nor_reopens_it(network):
    sim = sim_with_hazard(network)
    est = sim._world.estimate
    assert len(est.observable_segments) < 5  # nearly all flows are unobserved
    accept_conclusion(sim)
    w = sim._world
    tasks_after = sorted(w.tasks)
    sim._world.sensor_model = type(w.sensor_model)(shared_rho=0.5)  # covariance changes alone ...
    sim._refresh_estimate()
    sim._refresh_tasks()
    assert sorted(w.tasks) == tasks_after  # ... do not regenerate retired work
    assert (
        w.investigation.reopen_flags == ()
        and w.investigation.status is InvestigationStatus.CONCLUDED
    )


# -- reopening ----------------------------------------------------------------------------------------------------------------------------------


def test_relevant_new_evidence_flags_and_an_explained_revision_or_operator_reopens(network):
    sim = sim_with_hazard(network)
    accept_conclusion(sim)
    w = sim._world
    perch = w.drones["DRONE_3"].state.perch
    stop = StopRequest(LinePoint("LINE_064", 9_300_000), Measurement.CLEARANCE)
    sim.submit(CommitSorties((SortieRequest("DRONE_3", (stop,), perch),), "operator")).result()
    sim.step(15)
    flags = sim.snapshot.investigation.reopen_flags
    assert flags and "inside the cause scope" in flags[0]
    # an explained reopen proposal must cite the new evidence
    tb = build_toolbox(sim.snapshot, sim.network, sim.circuits)
    out = scripted_manager.measurement_proposal(tb, agents.trigger_events(tb))
    out.reopen = ReopenModel(reason="A new qualified negative changes the scope.", evidence_ids=[])
    assert any("needs a reason and the relevant new evidence" in e for e in validate(out, tb))
    out.reopen = ReopenModel(
        reason="A new qualified negative changes the scope.",
        evidence_ids=[sorted(sim.snapshot.readings)[-1]],
    )
    assert not [e for e in validate(out, tb) if e.startswith("reopen:")]
    # the operator can reopen directly; needs regenerate from the still-active hypotheses
    assert (
        "reopened" in sim.submit(ReopenInvestigation("new evidence in scope", "operator")).result()
    )
    assert sim.snapshot.investigation.status is InvestigationStatus.OPEN
    assert any(n.status.value == "ACTIVE" for n in sim.snapshot.needs.values())
    with pytest.raises(CommandRejected, match="not concluded"):
        sim.submit(ReopenInvestigation("again", "operator")).result()


# -- separate objectives ----------------------------------------------------------------------------------------------------------------------------


def test_benchmark_completion_enforces_its_own_requirements_and_is_scoped_separately(network):
    sim = permissive(
        Simulation(network, scenario=ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 7))
    )
    checks = {c.name: c for c in benchmark_checks(sim._world)}
    assert set(checks) == {"observability", "uncertainty", "consistency", "detectability"}
    assert not any(c.met for c in checks.values())  # nothing measured yet
    assert "unsupported ones cannot satisfy completion" in checks["uncertainty"].detail
    report = investigation_report(sim._world)
    assert set(report) >= {
        "incident_status",
        "operating_assessment",
        "local_investigation",
        "benchmark",
    }
    inc = permissive(Simulation(network))
    assert investigation_report(inc._world)["benchmark"] == "not selected"


def test_at_the_deadline_the_console_reports_remaining_work_not_success(network):
    sim = sim_with_hazard(network)
    w = sim._world
    w.clock_s = 4 * 3600
    rep = investigation_report(w)
    assert rep["deadline"]["outcome"] == "NOT complete"
    assert (
        rep["deadline"]["remaining_work"]
        and "no concluded diagnosis" in rep["deadline"]["unmet_checks"]
    )
