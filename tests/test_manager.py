"""Ticket 09: read-only pinned manager tools and the evidence-linked proposal contract.

Measured evidence is built by injecting LiDAR readings into a live Simulation (helpers_p2), so
the tests exercise the proposal contract without routing drones: citation completeness, tied and
contrary readings, unsupported numbers, independent recommendation checks and the rejected-
proposal trail.
"""

from __future__ import annotations

import ast
import copy
from pathlib import Path

import pytest
from conftest import permissive
from helpers_p2 import inject_lidar
from pydantic import ValidationError

from grid_ops import agents, coordinator, proposals, scripted_manager
from grid_ops.grid import load_network
from grid_ops.hypotheses import (
    Assessment,
    HypothesisModel,
    ManagerOutput,
    RecommendationModel,
    SpanModel,
    TurnAssessment,
)
from grid_ops.manager_tools import Toolbox, build_toolbox
from grid_ops.proposals import ProposedStatus, validate
from grid_ops.sim import RecordProposal, SetLandingPolicy, Simulation

SRC = Path(__file__).resolve().parents[1] / "src" / "grid_ops"


@pytest.fixture(scope="module")
def network():
    return load_network()


def startup_sim(network):
    return permissive(Simulation(network))


def measured_sim(network):
    """DRONE_2's two lidar readings, injected: LINE_058 vegetation hazard, LINE_064 clean."""
    sim = permissive(Simulation(network))
    inject_lidar(sim, "R0001", "LINE_058", 0, sim.geometry.length_mm("LINE_058"), 0.3)
    inject_lidar(sim, "R0002", "LINE_064", 8_950_000, 9_050_000, 2.0)
    return sim


def toolbox(sim) -> Toolbox:
    return build_toolbox(sim.snapshot, sim.network, sim.circuits)


def record(sim, out, *, status=ProposedStatus.PROPOSED_NOT_ACCEPTED, note="test proposal"):
    key = coordinator.decisions_semantic_key(sim.snapshot, out)
    return sim.submit(
        RecordProposal("manager-scripted", sim.snapshot.revision, out, status, note, key)
    ).result()


def good_output(sim):
    tb = toolbox(sim)
    return scripted_manager.measurement_proposal(tb, agents.assessment_trigger_ids(tb))


def with_revision(output: ManagerOutput, **changes) -> ManagerOutput:
    out = copy.deepcopy(output)
    out.revisions[0] = out.revisions[0].model_copy(update=changes)
    return out


# -- read-only and pinned -----------------------------------------------------------------------------------------------


def test_tools_are_read_only_and_pinned_to_an_immutable_snapshot(network):
    sim = measured_sim(network)
    tb = toolbox(sim)
    before = tb.list_readings()
    assert before.startswith(f"[snapshot r{tb.snapshot.revision} ")
    # the live world moves on; the pinned snapshot does not
    sim.step(30)
    assert tb.list_readings() == before
    assert sim.snapshot.clock_s > tb.snapshot.world.clock_s
    # a toolbox holds data only: no simulation, no truth, no command queue
    forbidden = {"submit", "_truth", "evaluator_truth", "_commands", "sim"}
    assert not forbidden & set(vars(tb)) | {n for n in dir(tb) if n in forbidden}
    names = {n for n in dir(tb) if not n.startswith("_")}
    assert not any(
        w in n for n in names for w in ("dispatch", "commit", "set_", "exclude", "release")
    )


def test_manager_modules_never_touch_truth_or_commands():
    banned = {
        "evaluator_truth",
        "_truth",
        "TruthState",
        "SetInService",
        "Dispatch",
        "CommitSorties",
        "ExcludeReading",
        "ReinstateReading",
        "ReassignEpoch",
        "SetLandingPolicy",
        "evaluation",
    }
    for name in ("manager_tools.py", "proposals.py", "scripted_manager.py", "hypotheses.py"):
        tree = ast.parse((SRC / name).read_text())
        used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        used |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        used |= {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) for a in n.names}
        assert not banned & used, (name, banned & used)
    tree = ast.parse((SRC / "agents.py").read_text())
    used = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    used |= {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    assert not (banned - {"evaluation"}) & used  # only RecordProposal reaches the sim


def test_pinned_snapshot_is_a_copy_not_a_handle(network):
    sim = startup_sim(network)
    tb = toolbox(sim)
    assert tb.snapshot.world is not sim.snapshot and tb.snapshot.world is not sim._world
    tb.snapshot.world.known_switching["LINE_999"] = 1  # mutating the copy cannot reach the sim
    assert "LINE_999" not in sim._world.known_switching


# -- startup -------------------------------------------------------------------------------------------------------------


def test_startup_proposal_cites_reports_and_states_no_measurements(network):
    sim = startup_sim(network)
    tb = toolbox(sim)
    turn = agents.run_scripted_turn(tb)
    assert turn.errors == []
    record(sim, turn.output)
    pf = sim.snapshot.proposals[0]
    assert pf.status is ProposedStatus.PROPOSED_NOT_ACCEPTED and pf.author == "manager-scripted"
    assert pf.snapshot_revision == sim.snapshot.revision
    for rev in pf.output.revisions:
        assert rev.trigger_kind == "startup" and rev.before is None
        assert any(e.startswith("SYN_REPORT") for e in rev.contributing_evidence_ids)
        assert "no drone measurements" in (rev.finding_summary + rev.update_reason).lower()
        assert rev.finding_summary and rev.update_reason and rev.remaining_uncertainty
        h = rev.after
        assert h.predictions and h.scope and h.predictions[0].expected
        assert (
            h.assessment is Assessment.UNRESOLVED and 1 <= h.priority <= 5
        )  # urgency != probability
    assert sim.snapshot.hypotheses == {}  # proposals never change accepted state


def test_urgency_and_assessment_are_separate_fields():
    with pytest.raises(ValidationError):
        HypothesisModel(
            id="H", claim="c", scope=[], predictions=[], assessment="SUPPORTED", priority=6
        )
    with pytest.raises(ValidationError):
        HypothesisModel(id="H", claim="c", scope=[], predictions=[], assessment="0.9", priority=3)
    with pytest.raises(ValidationError):
        SpanModel(segment_id="L", start_km=2, end_km=1)


# -- measurement-triggered -----------------------------------------------------------------------------------------------


def test_measurement_revision_cites_triggers_every_latest_tied_reading_and_the_hazard(network):
    sim = measured_sim(network)
    tb = toolbox(sim)
    turn = agents.run_scripted_turn(tb, agents.assessment_trigger_ids(tb))
    assert turn.errors == [], turn.errors
    veg = next(r for r in turn.output.revisions if r.hypothesis_id == "H_VEG")
    assert veg.trigger_kind == "measurement" and veg.trigger_event_ids
    assert {"R0001", "R0002"} == set(veg.latest_measurement_ids)
    assert veg.after.assessment is Assessment.SUPPORTED and veg.after.priority == 5
    cited = {v.reading_id: v for v in veg.cited_values}
    assert cited["R0001"].quantity == "clearance_m" and cited["R0001"].unit == "m"
    assert cited["R0001"].location == "lidar inspected intervals"
    assert "visibility" in cited["R0001"].conditions
    assert veg.contrary_evidence_ids == []  # an above-threshold reading is inconclusive, not contrary
    assert "does not show the condition caused the outage" in veg.finding_summary
    rec = turn.output.recommendations[0]
    assert rec.scope is not None and rec.urgency == 5 and rec.evidence_ids[0].startswith("R")
    assert "HZ001" in sim.snapshot.hazards[0].id
    record(sim, turn.output)
    assert sim.snapshot.proposals[-1].status is ProposedStatus.PROPOSED_NOT_ACCEPTED


# -- structured validation rejects bad output ----------------------------------------------------------------------------


@pytest.fixture(scope="module")
def measured(network):
    sim = measured_sim(network)
    return sim, good_output(sim)


def test_valid_output_passes(measured):
    sim, out = measured
    assert validate(out, toolbox(sim)) == []


@pytest.mark.parametrize(
    ("changes", "needle"),
    [
        ({"update_reason": ""}, "missing update_reason"),
        ({"finding_summary": " "}, "missing finding_summary"),
        ({"remaining_uncertainty": ""}, "missing remaining_uncertainty"),
        ({"contributing_evidence_ids": ["SYN_REPORT_02", "R9999"]}, "R9999 does not exist"),
        ({"trigger_event_ids": ["EVT9999"]}, "EVT9999 does not exist"),
        ({"trigger_event_ids": []}, "must cite trigger events"),
        ({"trigger_kind": "none"}, "needs a trigger"),
        (
            {"update_reason": "Clearance was 0.9 m at the inspected stop."},
            "unsupported numerical claim '0.9 m'",
        ),
        (
            {"operator_event_id": "OP9999", "trigger_kind": "operator", "trigger_event_ids": []},
            "real operator event",
        ),
    ],
)
def test_validation_rejects(measured, changes, needle):
    sim, out = measured
    errors = validate(with_revision(out, **changes), toolbox(sim))
    assert any(needle in e for e in errors), (needle, errors)


def test_omitted_latest_measurements_and_unsupported_cited_values_are_rejected(measured):
    sim, out = measured
    veg = out.revisions[0]
    short = with_revision(out, latest_measurement_ids=["R0001"])
    assert any(
        "omits the latest tied measurements ['R0002']" in e for e in validate(short, toolbox(sim))
    )
    nov = with_revision(out, cited_values=[v for v in veg.cited_values if v.reading_id != "R0002"])
    assert any(
        "no cited value/unit/location/conditions for R0002" in e
        for e in validate(nov, toolbox(sim))
    )
    wrong = veg.cited_values[0].model_copy(update={"value": 9.99})
    bad = with_revision(out, cited_values=[wrong, *veg.cited_values[1:]])
    assert any("does not match" in e for e in validate(bad, toolbox(sim)))


def test_supported_numbers_are_accepted(measured):
    sim, out = measured
    value = out.revisions[0].cited_values[0].value
    ok = with_revision(out, update_reason=f"The clearance was {value:.3f} m at the inspected stop.")
    assert validate(ok, toolbox(sim)) == []


def test_startup_rules_are_enforced(network):
    sim = startup_sim(network)
    out = scripted_manager.startup_proposal(toolbox(sim))
    assert validate(out, toolbox(sim)) == []
    silent = with_revision(
        out, update_reason="Created from the reports.", finding_summary="Report only."
    )
    assert any("no drone measurements" in e for e in validate(silent, toolbox(sim)))
    uncited = with_revision(
        out,
        contributing_evidence_ids=[],
        after=out.revisions[0].after.model_copy(update={"supporting_evidence_ids": []}),
    )
    assert any("must cite the reports" in e for e in validate(uncited, toolbox(sim)))
    later = measured_sim(network)
    assert any("already exist" in e for e in validate(out, toolbox(later)))


def test_operator_triggered_revisions_must_identify_a_real_event(network):
    sim = startup_sim(network)
    sim.submit(SetLandingPolicy(True, "operator"))
    op_id = sim.snapshot.operator_events[-1][0]
    out = scripted_manager.startup_proposal(toolbox(sim))
    revised = with_revision(
        out,
        trigger_kind="operator",
        operator_event_id=op_id,
        contributing_evidence_ids=["SYN_REPORT_02"],
    )
    assert validate(revised, toolbox(sim)) == []


def test_recommendations_are_independently_checked(measured):
    sim, out = measured
    tb = toolbox(sim)
    rec = out.recommendations[0]
    for changes, needle in (
        ({"scope": None}, "must be scoped"),
        ({"rationale": ""}, "rationale"),
        ({"evidence_ids": ["R0002"], "action_kind": "repair"}, "repair needs a detected hazard"),
        ({"evidence_ids": ["HZ999"]}, "does not exist"),
    ):
        bad = copy.deepcopy(out)
        bad.recommendations[0] = rec.model_copy(update=changes)
        assert any(needle in e for e in validate(bad, tb)), (changes, validate(bad, tb))
    # the detected hazard itself does justify a scoped repair over its own interval
    repair = rec.model_copy(
        update={
            "action_kind": "repair",
            "evidence_ids": ["HZ001"],
            "scope": SpanModel(segment_id="LINE_058", start_km=0.0, end_km=0.058),
        }
    )
    ok = copy.deepcopy(out)
    ok.recommendations[0] = repair
    assert validate(ok, tb) == []
    clean = RecommendationModel(
        id="X",
        action_kind="repair",
        text="repair",
        urgency=2,
        evidence_ids=["R0002"],
        scope=SpanModel(segment_id="LINE_064", start_km=8.8, end_km=9.5),
        rationale="r",
        uncertainty="u",
    )
    bad = copy.deepcopy(out)
    bad.recommendations = [clean]
    assert any("clean or inconclusive result cannot justify repair" in e for e in validate(bad, tb))


def test_unchanged_hypotheses_and_before_values(measured):
    sim, out = measured
    tb = toolbox(sim)
    both = copy.deepcopy(out)
    both.unchanged = [TurnAssessment(hypothesis_id="H_VEG", assessment_text="nothing new")]
    errs = validate(both, tb)
    assert any("appears as revised and unchanged" in e for e in errs)
    assert any("unknown hypothesis" in e for e in errs)
    stale = with_revision(out, before=out.revisions[0].after, change_kinds=["assessment"])
    assert any("before-values do not match" in e for e in validate(stale, tb))


# -- the rejected-proposal trail -----------------------------------------------------------------------------------------


def test_rejected_proposals_are_retained_with_their_errors(network):
    sim = startup_sim(network)
    tb = toolbox(sim)
    bad = with_revision(scripted_manager.startup_proposal(tb), update_reason="")
    errs = validate(bad, tb)
    assert any("missing update_reason" in e for e in errs)
    record(sim, bad, status=ProposedStatus.REJECTED, note="; ".join(errs))
    pf = sim.snapshot.proposals[-1]
    assert pf.status is ProposedStatus.REJECTED
    assert "missing update_reason" in pf.history[0][1]
    assert sim.snapshot.hypotheses == {}


def test_operator_view_data_for_the_console_exists(network):
    sim = startup_sim(network)
    tb = toolbox(sim)
    turn = agents.run_scripted_turn(tb)
    record(sim, turn.output)
    assert proposals.ProposedStatus.PROPOSED_NOT_ACCEPTED in {
        p.status for p in sim.snapshot.proposals
    }