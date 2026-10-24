"""A deterministic manager that follows the proposal contract.

It exists so the contract (validation, citations, scoped recommendations) is testable without a
language model. It reasons only from the pinned toolbox, like the LLM manager, and writes plain
findings and update reasons from the evidence it cites.
"""

from __future__ import annotations

from grid_ops.assessment import Outcome
from grid_ops.hypotheses import (
    Assessment,
    CitedValue,
    HypothesisModel,
    ManagerOutput,
    PredictionModel,
    RecommendationModel,
    RevisionModel,
    SpanModel,
    TurnAssessment,
)
from grid_ops.manager_tools import Toolbox

# What each broad report area could plausibly hide, expressed as a falsifiable local claim.
REPORT_CLAIMS = {
    "SYN_REPORT_02": (
        "H_VEG",
        "Vegetation is within 0.5 m of a conductor somewhere in the SYN_REPORT_02 corridor.",
        "clearance",
        "minimum clearance below 0.5 m on the inspected interval",
        [],
    ),
    "SYN_REPORT_05": (
        "H_HOT",
        "A loaded hot joint exists on a conductor in the SYN_REPORT_05 area.",
        "conductor_temp",
        "excess temperature above 10 C over the load-dependent baseline",
        ["circuit current above 20 A"],
    ),
    "SYN_REPORT_06": (
        "H_MOTION",
        "A span in the SYN_REPORT_06 area moves abnormally in gusts.",
        "motion",
        "motion amplitude above 0.20 m",
        ["a qualifying gust window"],
    ),
}


def _scope_for(tb: Toolbox, area_id: str) -> list[SpanModel]:
    from grid_ops.geometry import Geometry

    geo = Geometry(tb.network)
    area = tb.network.areas[area_id]
    spans = []
    for seg in sorted(tb.network.segments):
        for iv in geo.clip_intervals(seg, area):
            if iv.end_mm > iv.start_mm:
                spans.append(
                    SpanModel(segment_id=seg, start_km=iv.start_mm / 1e6, end_km=iv.end_mm / 1e6)
                )
    return spans


def _hypothesis(tb: Toolbox, report_id: str, assessment: Assessment, priority: int, **kw):
    hid, claim, kind, expected, conditions = REPORT_CLAIMS[report_id]
    return HypothesisModel(
        id=hid,
        claim=claim,
        scope=_scope_for(tb, report_id)[:12],
        predictions=[
            PredictionModel(
                id=f"{hid}-P1",
                statement=f"If the claim holds, a {kind} reading in the scope shows: {expected}.",
                measurement=kind,
                expected=expected,
                required_conditions=conditions,
            )
        ],
        assessment=assessment,
        priority=priority,
        supporting_evidence_ids=kw.get("support", []),
        contradicting_evidence_ids=kw.get("contra", []),
    )


def startup_proposal(tb: Toolbox) -> ManagerOutput:
    revisions = []
    for report_id, priority in (("SYN_REPORT_02", 3), ("SYN_REPORT_05", 2), ("SYN_REPORT_06", 2)):
        after = _hypothesis(tb, report_id, Assessment.UNRESOLVED, priority, support=[report_id])
        revisions.append(
            RevisionModel(
                hypothesis_id=after.id,
                change_kinds=["creation"],
                before=None,
                after=after,
                finding_summary=(
                    f"{report_id} is a low-confidence area report; it motivates a local check, "
                    "not a confirmed fault location."
                ),
                update_reason=(
                    "At startup only the reports exist and no drone measurements exist yet, so "
                    "the claim is created as UNRESOLVED with an urgency set by report confidence."
                ),
                remaining_uncertainty="Everything: location, existence and relation to the outage.",
                trigger_kind="startup",
                contributing_evidence_ids=[report_id],
            )
        )
    return ManagerOutput(
        summary="Initial hypotheses from reports; no measurements yet.",
        revisions=revisions,
        recommendations=[
            RecommendationModel(
                id="REC-START-1",
                action_kind="measure",
                text="Send a clearance measurement to the SYN_REPORT_02 corridor first.",
                scope=revisions[0].after.scope[0],
                urgency=3,
                evidence_ids=["SYN_REPORT_02"],
                rationale="The report names vegetation; clearance can falsify it locally.",
                uncertainty="The report is low to medium confidence and area-wide.",
            )
        ],
        untested_alternatives=[
            "Electrical causes of SYN_REPORT_03's protection event are untested."
        ],
    )


def measurement_proposal(tb: Toolbox, trigger_event_ids: list[str]) -> ManagerOutput:
    """Revise the report hypotheses against the latest tied local readings."""
    w = tb.world
    revisions, recs, unchanged = [], [], []
    from grid_ops.proposals import _tied_readings

    for report_id in REPORT_CLAIMS:
        hid = REPORT_CLAIMS[report_id][0]
        scope = _scope_for(tb, report_id)
        probe = RevisionModel(
            hypothesis_id=hid,
            change_kinds=["assessment"],
            before=None,
            after=_hypothesis(tb, report_id, Assessment.UNRESOLVED, 2),
            finding_summary="x",
            update_reason="x",
            remaining_uncertainty="x",
            trigger_kind="measurement",
        )
        probe.after.scope = scope[:12]
        tied = _tied_readings(tb, probe)
        if not tied:
            continue
        outcomes = {rid: w.interpretations[rid].outcome for rid in tied}
        positives = [r for r, o in outcomes.items() if o is Outcome.POSITIVE]
        negatives = [r for r, o in outcomes.items() if o is Outcome.QUALIFIED_NEGATIVE]
        if positives:
            assessment, priority = Assessment.SUPPORTED, 5
        elif negatives and not any(o is Outcome.INCONCLUSIVE for o in outcomes.values()):
            assessment, priority = Assessment.UNLIKELY, 1
        else:
            assessment, priority = Assessment.UNRESOLVED, 3
        old = w.hypotheses.get(hid)
        if old and old["assessment"] == str(assessment) and old["priority"] == priority:
            unchanged += turn_assessments(tb, [hid])  # nothing to revise: a turn assessment only
            continue
        cited = []
        for rid in tied:
            r = w.readings[rid]
            name, value = r.payload[0] if r.payload else ("none", 0.0)
            cited.append(
                CitedValue(
                    reading_id=rid,
                    quantity=name,
                    value=value,
                    unit={"clearance_m": "m", "temp_c": "C", "amplitude_m": "m"}.get(name, ""),
                    location=f"{r.point.segment_id} @ {r.point.offset_km:.3f} km",
                    conditions=", ".join(f"{k}={v:g}" for k, v in r.conditions),
                )
            )
        after = _hypothesis(
            tb, report_id, assessment, priority, support=positives, contra=negatives
        )
        after.scope = scope[:12]
        kinds = ["assessment", "priority"] if hid in w.hypotheses else ["creation", "assessment"]
        summary = {
            Assessment.SUPPORTED: "A threshold event was detected on the inspected interval.",
            Assessment.UNLIKELY: "Qualified negative readings cover the tested interval.",
            Assessment.UNRESOLVED: "The readings do not settle the claim.",
        }[assessment]
        revisions.append(
            RevisionModel(
                hypothesis_id=hid,
                change_kinds=kinds,
                before=None if hid not in w.hypotheses else HypothesisModel(**w.hypotheses[hid]),
                after=after,
                finding_summary=summary + " This does not show the condition caused the outage.",
                update_reason=(
                    f"Latest tied readings {', '.join(tied)} were taken after the previous turn; "
                    f"outcomes: {', '.join(f'{r}={o}' for r, o in outcomes.items())}."
                ),
                remaining_uncertainty="Untested intervals and the outage mechanism remain open.",
                trigger_kind="measurement",
                trigger_event_ids=trigger_event_ids,
                contributing_evidence_ids=[report_id, *tied],
                latest_measurement_ids=tied,
                contrary_evidence_ids=negatives
                if assessment is Assessment.SUPPORTED
                else positives,
                cited_values=cited,
            )
        )
        if positives:
            hz = [h for h in w.hazards if set(h.evidence_ids) & set(positives)]
            if hz:
                h0 = hz[0]
                recs.append(
                    RecommendationModel(
                        id=f"REC-{hid}",
                        action_kind="inspect",
                        text=f"Field inspection of {h0.segment_id} {h0.start_mm / 1e6:.3f}-"
                        f"{h0.end_mm / 1e6:.3f} km.",
                        scope=SpanModel(
                            segment_id=h0.segment_id,
                            start_km=h0.start_mm / 1e6,
                            end_km=h0.end_mm / 1e6,
                        ),
                        urgency=5,
                        evidence_ids=[h0.id, *positives],
                        rationale="A detected local hazard: this is an urgent hazard action.",
                        uncertainty="Whether it relates to the outage is unresolved.",
                    )
                )
    return ManagerOutput(
        summary="Report hypotheses revised against the latest local readings.",
        revisions=revisions,
        unchanged=unchanged,
        recommendations=recs,
        untested_alternatives=["Electrical alternatives are untested without current readings."],
    )


def turn_assessments(tb: Toolbox, hypothesis_ids: list[str]) -> list[TurnAssessment]:
    return [
        TurnAssessment(hypothesis_id=h, assessment_text="No new evidence touches this scope.")
        for h in hypothesis_ids
    ]


def add_conclusion(tb: Toolbox, output: ManagerOutput) -> ManagerOutput:
    """If a detected hazard is connected to the outage by switching or electrical evidence in the
    same component, and every material alternative has a disposition, propose concluding."""
    from grid_ops.hypotheses import AlternativeDisposition, ConclusionModel

    w = tb.world
    rec = next((r for r in output.recommendations if r.scope is not None), None)
    if not w.hazards or rec is None:
        return output
    hz = w.hazards[0]
    comp = tb.network.segments[hz.segment_id].component
    link = [
        e.id
        for e in w.knowledge.evidence.values()
        if e.source in ("report", "reading")
        and e.circuit_id.split("/")[0] in tb.network.segments
        and tb.network.segments[e.circuit_id.split("/")[0]].component == comp
        and e.claim.value in ("dead", "live")
    ]
    cause = next((r for r in output.revisions if str(r.after.assessment) == "SUPPORTED"), None)
    if not link or cause is None:
        return output
    others = [
        r.hypothesis_id
        for r in output.revisions
        if r.hypothesis_id != cause.hypothesis_id and r.after.priority >= 3
    ]
    output.conclusion = ConclusionModel(
        kind="cause_identified",
        cause_hypothesis_id=cause.hypothesis_id,
        causal_statement=(
            f"{cause.hypothesis_id}: a detected hazard on {hz.segment_id} lies in the same "
            "network component as the switched-out circuit; it is the best-supported candidate "
            "for the protection event, not a proven mechanism."
        ),
        causal_link_evidence_ids=[hz.id, *hz.evidence_ids, link[0]],
        alternatives=[
            AlternativeDisposition(hypothesis_id=h, disposition="indistinguishable", note="no data")
            for h in others
        ],
        recommendation_id=rec.id,
        remaining_uncertainty="The protection event's mechanism is inferred; other report "
        "areas stay untested.",
    )
    return output
