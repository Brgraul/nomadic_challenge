"""Ticket 16: concluding and reopening an outage investigation safely.

Evidence is built by injecting LiDAR readings into a live Simulation (helpers_p2): a detected
hazard alone never completes the diagnosis, the causal link needs switching or electrical
evidence in the same component, material alternatives need dispositions, and the separate
benchmark objective is only smoke-checked here (the three-method runtime keeps world.estimate
at None and reports the benchmark as not selected for incident mode).
"""

from __future__ import annotations

import pytest
from conftest import permissive
from helpers_p2 import inject_lidar

from grid_ops import agents, coordinator, scripted_manager
from grid_ops.conclusion import InvestigationStatus, investigation_report, reopen_relevance
from grid_ops.grid import load_network
from grid_ops.hypotheses import AlternativeDisposition, ReopenModel
from grid_ops.manager_tools import build_toolbox
from grid_ops.proposals import ProposedStatus, validate
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sensing import TaskStatusV2
from grid_ops.sim import (
    AcceptLimits,
    CommitDecision,
    RecordProposal,
    ReopenInvestigation,
    Simulation,
)
from grid_ops.world import CommandRejected


def toolbox(sim):
    return build_toolbox(sim.snapshot, sim.network, sim.circuits)


@pytest.fixture(scope="module")
def network():
    return load_network()


def record_and_commit(sim, out, decision_id):
    """Record the proposal against the pinned snapshot, then accept it as one decision."""
    tb = toolbox(sim)
    errs = validate(out, tb)
    key = coordinator.decisions_semantic_key(sim.snapshot, out)
    status = ProposedStatus.REJECTED if errs else ProposedStatus.PROPOSED_NOT_ACCEPTED
    sim.submit(
        RecordProposal("manager-scripted", sim.snapshot.revision, out, status,
                       "; ".join(errs) or "test proposal", key)
    ).result()
    if errs:
        return errs
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision(decision_id, pf.id, pf.semantic_key, "operator")).result()
    return []


def startup_commit(sim, decision_id="D0"):
    tb = toolbox(sim)
    turn = agents.run_scripted_turn(tb)
    assert turn.errors == [], turn.errors
    errs = record_and_commit(sim, turn.output, decision_id)
    assert errs == []


def hazard_sim(network):
    """Startup hypotheses accepted; LINE_058 carries an injected low-clearance hazard."""
    sim = permissive(Simulation(network))
    startup_commit(sim)
    inject_lidar(sim, "R0001", "LINE_058", 0, sim.geometry.length_mm("LINE_058"), 0.3)
    return sim


def conclusion_output(sim, *, address_accepted=True, **conclusion_kw):
    """The scripted measurement proposal with the scripted conclusion attached."""
    tb = toolbox(sim)
    out = scripted_manager.add_conclusion(
        tb, scripted_manager.measurement_proposal(tb, agents.assessment_trigger_ids(tb))
    )
    assert out.conclusion is not None
    if address_accepted:
        material = sorted(h for h in tb.world.hypotheses if h != "H_VEG")
        out.conclusion.alternatives = [
            AlternativeDisposition(hypothesis_id=h, disposition="indistinguishable", note="no data")
            for h in material
        ]
    for name, value in conclusion_kw.items():
        setattr(out.conclusion, name, value)
    return tb, out


def errors(sim, **conclusion_kw):
    tb, out = conclusion_output(sim, **conclusion_kw)
    return validate(out, tb)


# -- what completion requires ------------------------------------------------------------------------------------------


def test_a_valid_conclusion_has_a_causal_link_alternatives_a_recommendation_and_uncertainty(
    network,
):
    sim = hazard_sim(network)
    assert errors(sim) == []


def test_a_detected_local_hazard_alone_cannot_complete_the_diagnosis(network):
    sim = hazard_sim(network)
    errs = errors(sim, causal_link_evidence_ids=["HZ001"])
    assert any("hazard alone cannot complete outage diagnosis" in e for e in errs)
    # the urgent hazard action itself stays available: its recommendation is still valid
    _, out = conclusion_output(sim, causal_link_evidence_ids=["HZ001"])
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
    sim = hazard_sim(network)
    assert any(needle in e for e in errors(sim, **changes))


def test_material_alternatives_must_be_addressed(network):
    sim = hazard_sim(network)
    tb, out = conclusion_output(sim, address_accepted=False)
    assert any("material alternative H_SUPPLY is not addressed" in e for e in validate(out, tb))
    out.conclusion.alternatives = [
        AlternativeDisposition(
            hypothesis_id="H_SUPPLY", disposition="indistinguishable", note="no data"
        )
    ]
    assert not any("not addressed" in e for e in validate(out, tb))
    # a material alternative revised this turn is material too, until it is disposed of
    alt = out.revisions[0].after.model_copy(update={"id": "H_ALT", "priority": 4})
    out.revisions.append(out.revisions[0].model_copy(update={"hypothesis_id": "H_ALT", "after": alt}))
    assert any("material alternative H_ALT is not addressed" in e for e in validate(out, tb))
    out.conclusion.alternatives.append(
        AlternativeDisposition(hypothesis_id="H_ALT", disposition="indistinguishable", note="no data")
    )
    assert not any("not addressed" in e for e in validate(out, tb))


def test_ambiguous_causes_stay_unresolved_with_a_discriminating_recommendation(network):
    sim = hazard_sim(network)
    tb, out = conclusion_output(
        sim,
        kind="unresolved_ambiguous",
        cause_hypothesis_id=None,
        causal_link_evidence_ids=[],
        alternatives=[],
        discriminating_recommendation_id=None,
    )
    assert any("discriminating-reading or field-inspection" in e for e in validate(out, tb))
    out.conclusion.discriminating_recommendation_id = "REC-H-VEG"
    assert validate(out, tb) == []
    errs = record_and_commit(sim, out, "D9")
    assert errs == []
    inv = sim.snapshot.investigation
    assert inv.status is InvestigationStatus.OPEN and "Discriminating step" in inv.unresolved
    assert any(t.status is TaskStatusV2.PENDING for t in sim.snapshot.tasks.values())
    sim.submit(AcceptLimits("Cause stays ambiguous between vegetation and a hot joint.",
                            "operator")).result()
    assert sim.snapshot.investigation.status is InvestigationStatus.CONCLUDED_WITH_LIMITS


def test_operator_cannot_accept_limits_when_none_are_stated(network):
    sim = permissive(Simulation(network))
    with pytest.raises(CommandRejected, match="no stated unresolved limit"):
        sim.submit(AcceptLimits("close it", "operator")).result()


# -- reopening ----------------------------------------------------------------------------------------------------------


def test_relevant_new_evidence_flags_and_an_explained_revision_or_operator_reopens(network):
    sim = hazard_sim(network)
    tb, out = conclusion_output(sim)
    assert record_and_commit(sim, out, "DC") == []
    assert sim.snapshot.investigation.status is InvestigationStatus.CONCLUDED
    inject_lidar(sim, "R0003", "LINE_058", 0, sim.geometry.length_mm("LINE_058"), 0.31)
    flags = reopen_relevance(sim._world, sim._world.investigation.concluded_s)
    assert flags and "inside the cause scope" in flags[0]
    assert any("R0003" in f for f in flags)
    # an explained reopen proposal must cite the relevant new evidence
    tb = toolbox(sim)
    out = scripted_manager.measurement_proposal(tb, agents.assessment_trigger_ids(tb))
    out.reopen = ReopenModel(reason="A new hazard reading changes the scope.", evidence_ids=[])
    assert any("needs a reason and the relevant new evidence" in e for e in validate(out, tb))
    out.reopen = ReopenModel(reason="A new hazard reading inside the cause scope.",
                             evidence_ids=["R0003"])
    assert not [e for e in validate(out, tb) if e.startswith("reopen:")]
    # the operator can reopen directly
    assert ("reopened" in sim.submit(ReopenInvestigation("new evidence in scope",
                                                         "operator")).result())
    assert sim.snapshot.investigation.status is InvestigationStatus.OPEN
    with pytest.raises(CommandRejected, match="not concluded"):
        sim.submit(ReopenInvestigation("again", "operator")).result()


# -- separate objectives ------------------------------------------------------------------------------------------------


def test_investigation_report_scopes_the_results_and_smoke_checks_the_benchmark(network):
    sim = hazard_sim(network)
    rep = investigation_report(sim._world)
    assert set(rep) >= {"incident_status", "operating_assessment", "local_investigation"}
    assert rep["benchmark"] == "not selected"  # incident mode: the benchmark is not the objective
    sim._world.clock_s = 4 * 3600
    rep = investigation_report(sim._world)
    assert rep["deadline"]["outcome"] == "NOT complete"
    assert "no concluded diagnosis" in rep["deadline"]["unmet_checks"]
    healthy = Simulation(network, scenario=ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 7))
    assert investigation_report(healthy._world)["benchmark"]  # selected: checks are reported