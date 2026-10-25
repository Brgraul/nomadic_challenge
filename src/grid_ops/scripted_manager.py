"""A deterministic manager that follows the proposal contract.

It exists so the contract (validation, citations, scoped recommendations, explicit requests) is
testable without a language model. It reasons only from the pinned toolbox, like the LLM manager,
and writes plain findings and update reasons from the evidence it cites. Zero current is never
treated as proof of de-energization.
"""

from __future__ import annotations

import json

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
from grid_ops.sensing import ContactCurrent, LidarPayload

CLEARANCE_THRESHOLD_M = 0.5


def _parse_out(text: str):
    """Tool outputs carry the snapshot label before the JSON payload; try each bracket start."""
    starts = [i for i, ch in enumerate(text) if ch in "{["]
    for i in starts:
        try:
            return json.loads(text[i:])
        except json.JSONDecodeError:
            continue
    return []


def _spans(tb: Toolbox, area_id: str) -> list[SpanModel]:
    spans = _parse_out(tb.segments_in_report(area_id))
    return [
        SpanModel(segment_id=s["segment_id"], start_km=s["start_km"], end_km=s["end_km"])
        for s in spans
        if isinstance(s, dict) and "segment_id" in s
    ]


def _supply_evidence_id(tb: Toolbox, segment_id: str) -> str:
    """A citable id for the switching report: the report itself, else its derived evidence."""
    for report_id in tb.network.reports:
        if segment_id in tb.network.reports[report_id].description:
            return report_id
    for e in tb.world.knowledge.evidence.values():
        if e.source == "report" and e.circuit_id.split("/")[0] == segment_id:
            return e.id
    return segment_id


def _segment_span(tb: Toolbox, segment_id: str) -> SpanModel:
    seg = tb.network.segments[segment_id]
    return SpanModel(segment_id=segment_id, start_km=0.0, end_km=round(seg.length_km, 3))


def _request(tb, rid, hid, rationale, alternatives, outcomes, evidence, urgency, target):
    from grid_ops.sensing import CorridorSpan, CurrentRequest, LidarRequest, OutcomeSpec

    common = dict(
        id=rid,
        hypothesis_ids=(hid,),
        rationale=rationale,
        alternatives_distinguished=alternatives,
        expected_outcomes=tuple(OutcomeSpec(description=o) for o in outcomes),
        evidence_ids=evidence,
        urgency=urgency,
        source_revision=str(tb.snapshot.revision),
        replaces_request_ids=(),
    )
    if isinstance(target, CorridorSpan):
        return LidarRequest(**common, target=target)
    return CurrentRequest(**common, target=target)


def _supply_segment(tb: Toolbox) -> str:
    """The segment the supply claim tests: a reported switched segment, else a segment a report
    names, else the first mapped segment."""
    w = tb.world
    for seg in sorted(w.known_switching):
        return seg
    for report in sorted(tb.network.reports.values(), key=lambda r: r.id):
        for seg in sorted(tb.network.segments):
            if seg in report.description:
                return seg
    return sorted(tb.network.segments)[0]


def startup_proposal(tb: Toolbox) -> ManagerOutput:
    veg_spans = _spans(tb, "SYN_REPORT_02")
    veg_scope = veg_spans[:6] if veg_spans else []
    supply_segment = _supply_segment(tb)
    supply_evidence = _supply_evidence_id(tb, supply_segment)
    revisions = [
        RevisionModel(
            hypothesis_id="H_VEG",
            change_kinds=["creation"],
            before=None,
            after=HypothesisModel(
                id="H_VEG",
                claim="Vegetation is within 0.5 m of a conductor somewhere in the "
                "SYN_REPORT_02 corridor.",
                scope=veg_scope,
                predictions=[
                    PredictionModel(
                        id="H_VEG-P1",
                        statement=(
                            "If the claim holds, a lidar corridor scan of the scope shows a "
                            "minimum clearance below the declared 0.5 m threshold."
                        ),
                        measurement="clearance",
                        expected="minimum clearance below 0.5 m on the inspected interval",
                        required_conditions=["the corridor is geometrically visible to the lidar"],
                    )
                ],
                assessment=Assessment.UNRESOLVED,
                priority=3,
                supporting_evidence_ids=["SYN_REPORT_02"],
            ),
            finding_summary=(
                "SYN_REPORT_02 is a low-confidence area report; it motivates a lidar corridor "
                "scan, not a confirmed fault location. No drone measurements exist yet."
            ),
            update_reason=(
                "At startup only the reports exist and no drone measurements exist yet, so the "
                "claim is created as UNRESOLVED with an urgency set by report confidence."
            ),
            remaining_uncertainty="Everything: location, existence and relation to the outage.",
            trigger_kind="startup",
            contributing_evidence_ids=["SYN_REPORT_02"],
        ),
        RevisionModel(
            hypothesis_id="H_SUPPLY",
            change_kinds=["creation"],
            before=None,
            after=HypothesisModel(
                id="H_SUPPLY",
                claim=(
                    f"{supply_segment} is de-energized as its switching report states, so a "
                    "landed contact sample reads no current."
                ),
                scope=[_segment_span(tb, supply_segment)],
                predictions=[
                    PredictionModel(
                        id="H_SUPPLY-P1",
                        statement=(
                            "If the claim holds, a landed contact-current sample on the circuit "
                            "reads zero (or near-zero) RMS current."
                        ),
                        measurement="current",
                        expected="RMS current indistinguishable from zero on the sampled conductor",
                        required_conditions=["the drone can land on the conductor"],
                    )
                ],
                assessment=Assessment.UNRESOLVED,
                priority=4,
                supporting_evidence_ids=[supply_evidence],
            ),
            finding_summary=(
                f"The switching report states {supply_segment} is de-energized; no drone "
                "measurements exist yet, so the claim stays untested."
            ),
            update_reason=(
                "At startup only the reports exist and no drone measurements exist yet, so the "
                "claim is created as UNRESOLVED with an urgency set by the supply impact."
            ),
            remaining_uncertainty="Reports are approximate and can be wrong; only a landed "
            "contact sample tests the claim, and even zero current leaves live-unloaded and "
            "disconnected alternatives open.",
            trigger_kind="startup",
            contributing_evidence_ids=[supply_evidence],
        ),
    ]
    requests = []
    if veg_scope:
        first = veg_scope[0]
        requests.append(
            _request(
                tb,
                "REQ-L-VTG",
                "H_VEG",
                "A moving lidar scan is the only way to test the vegetation claim over the "
                "reported corridor.",
                ("vegetation within 0.5 m versus clear corridor",),
                ("minimum clearance below 0.5 m on some inspected interval",),
                ("SYN_REPORT_02",),
                3,
                __import__("grid_ops.sensing", fromlist=["CorridorSpan"]).CorridorSpan(
                    segment_id=first.segment_id,
                    start_mm=round(first.start_km * 1e6),
                    end_mm=round(first.end_km * 1e6),
                ),
            )
        )
    circuit_id = f"{supply_segment}/C1"
    requests.append(
        _request(
            tb,
            "REQ-C-SUP",
            "H_SUPPLY",
            "A landed contact sample tests the switching report's de-energized claim.",
            (
                "report correct (no current) versus report wrong (current flows: live-unloaded "
                "or disconnected interpretations stay open)",
            ),
            ("RMS current at or near zero", "RMS current clearly above zero"),
            [supply_evidence],
            4,
            __import__("grid_ops.sensing", fromlist=["ConductorScope"]).ConductorScope(
                circuit_id=circuit_id,
                phase="A",
                start_mm=0,
                end_mm=round(tb.network.segments[supply_segment].length_km * 1e6),
            ),
        )
    )
    recs = []
    if veg_scope:
        recs.append(
            RecommendationModel(
                id="REC-START-1",
                action_kind="measure",
                text="Scan the SYN_REPORT_02 corridor with the lidar first.",
                scope=veg_scope[0],
                urgency=3,
                evidence_ids=["SYN_REPORT_02"],
                rationale="The report names vegetation; a lidar scan can falsify it locally.",
                uncertainty="The report is low to medium confidence and area-wide.",
            )
        )
    return ManagerOutput(
        summary="Initial hypotheses from the reports; no measurements yet.",
        revisions=revisions,
        recommendations=recs,
        measurement_requests=requests,
        untested_alternatives=[
            "Electrical causes of SYN_REPORT_03's protection event are untested."
        ],
    )


def _tied_lidar(tb: Toolbox, spans: list[SpanModel]) -> list[str]:
    latest: dict[str, tuple[int, str]] = {}
    for rid, reading in tb.world.readings.items():
        payload = reading.payload
        if not isinstance(payload, LidarPayload) or not payload.inspected:
            continue
        seg = payload.inspected[0].segment_id
        for sp in spans:
            if sp.segment_id != seg:
                continue
            lo, hi = sp.start_km * 1e6, sp.end_km * 1e6
            overlap = any(
                i.start_mm <= hi and i.end_mm >= lo for i in payload.inspected
            )
            if overlap and (seg not in latest or reading.acquired_end_s >= latest[seg][0]):
                latest[seg] = (reading.acquired_end_s, rid)
    return sorted(rid for _, rid in latest.values())


def _current_readings(tb: Toolbox, circuit_id: str) -> list[str]:
    out = []
    for rid, reading in tb.world.readings.items():
        a = reading.acquisition
        if isinstance(a, ContactCurrent) and a.conductor.circuit_id == circuit_id:
            out.append((reading.acquired_end_s, rid))
    return sorted(rid for _, rid in out)


def measurement_proposal(tb: Toolbox, trigger_event_ids: list[str]) -> ManagerOutput:
    """Revise the report hypotheses against the latest lidar and contact readings."""
    w = tb.world
    revisions, recs, unchanged = [], [], []
    veg_spans = _spans(tb, "SYN_REPORT_02")
    supply_segment = _supply_segment(tb)

    tied = _tied_lidar(tb, veg_spans) if veg_spans else []
    if tied:
        values = []
        positive = False
        for rid in tied:
            payload = tb.world.readings[rid].payload
            if payload.clearance_m is not None:
                values.append((rid, payload.clearance_m))
                positive = positive or payload.clearance_m < CLEARANCE_THRESHOLD_M
        cited = [
            CitedValue(
                reading_id=rid,
                quantity="clearance_m",
                value=value,
                unit="m",
                location="lidar inspected intervals",
                conditions="geometric visibility recorded on the reading",
            )
            for rid, value in values
        ]
        old = w.hypotheses.get("H_VEG")
        if positive:
            assessment, priority, summary = Assessment.SUPPORTED, 5, (
                "A lidar scan found clearance below the declared threshold on the inspected "
                "interval."
            )
            recs.append(
                RecommendationModel(
                    id="REC-H-VEG",
                    action_kind="inspect",
                    text="Field inspection of the low-clearance interval first.",
                    scope=veg_spans[0],
                    urgency=5,
                    evidence_ids=[*tied],
                    rationale="A detected low-clearance hazard: an urgent hazard action.",
                    uncertainty="Whether it caused the outage is unresolved.",
                )
            )
        else:
            assessment, priority, summary = Assessment.UNRESOLVED, 3, (
                "The lidar scans so far do not show clearance below the threshold."
            )
        after = HypothesisModel(
            id="H_VEG",
            claim="Vegetation is within 0.5 m of a conductor somewhere in the SYN_REPORT_02 "
            "corridor.",
            scope=veg_spans[:6],
            predictions=[
                PredictionModel(
                    id="H_VEG-P1",
                    statement="A lidar corridor scan shows the minimum clearance state.",
                    measurement="clearance",
                    expected="minimum clearance below 0.5 m on the inspected interval",
                    required_conditions=["the corridor is geometrically visible to the lidar"],
                )
            ],
            assessment=assessment,
            priority=priority,
            supporting_evidence_ids=tied if positive else [],
            contradicting_evidence_ids=[] if positive else tied,
        )
        revisions.append(
            RevisionModel(
                hypothesis_id="H_VEG",
                change_kinds=["assessment", "priority"] if old else ["creation", "assessment"],
                before=HypothesisModel(**old) if old else None,
                after=after,
                finding_summary=summary + " This does not show the condition caused the outage.",
                update_reason=f"Latest lidar readings {', '.join(tied)} arrived after the "
                "previous turn.",
                remaining_uncertainty="Uninspected intervals and the outage mechanism remain open.",
                trigger_kind="measurement",
                trigger_event_ids=trigger_event_ids,
                contributing_evidence_ids=["SYN_REPORT_02", *tied],
                latest_measurement_ids=tied,
                cited_values=cited,
            )
        )
    elif "H_VEG" in w.hypotheses:
        unchanged += turn_assessments(tb, ["H_VEG"])

    supply_readings = _current_readings(tb, f"{supply_segment}/C1")
    supply_evidence = _supply_evidence_id(tb, supply_segment)
    if supply_readings:
        rid = supply_readings[-1]
        payload = tb.world.readings[rid].payload
        old = w.hypotheses.get("H_SUPPLY")
        cited = [
            CitedValue(
                reading_id=rid,
                quantity="rms_a",
                value=payload.rms_a,
                unit="A",
                location=f"{payload.conductor.point.segment_id} @ "
                f"{payload.conductor.point.offset_mm / 1e6:.3f} km",
                conditions=f"integration {payload.integration_s} s, quality "
                f"{'ok' if tb.world.readings[rid].quality.valid else 'invalid'}",
            )
        ]
        if payload.rms_a > 2 * payload.uncertainty_a:
            assessment, priority = Assessment.REFUTED, 1
            summary = (
                "A landed contact sample reads clear current on the reported de-energized "
                "circuit; the switching report is contradicted."
            )
            contra = [rid]
            alternatives = []
        else:
            assessment, priority = Assessment.UNRESOLVED, 4
            summary = (
                "The contact sample reads zero current, which is compatible with both a "
                "live-unloaded and a disconnected circuit; the report is neither confirmed nor "
                "refuted."
            )
            contra = []
            alternatives = [
                "the circuit is live but unloaded",
                "the circuit is disconnected downstream of the sample point",
            ]
        after = HypothesisModel(
            id="H_SUPPLY",
            claim=f"{supply_segment} is de-energized as its switching report states.",
            scope=[_segment_span(tb, supply_segment)],
            predictions=[
                PredictionModel(
                    id="H_SUPPLY-P1",
                    statement="A landed contact sample reads the circuit's current state.",
                    measurement="current",
                    expected="RMS current indistinguishable from zero on the sampled conductor",
                    required_conditions=["the drone can land on the conductor"],
                )
            ],
            assessment=assessment,
            priority=priority,
            supporting_evidence_ids=[] if assessment is Assessment.REFUTED else [supply_evidence],
            contradicting_evidence_ids=contra,
        )
        revisions.append(
            RevisionModel(
                hypothesis_id="H_SUPPLY",
                change_kinds=["assessment", "priority"] if old else ["creation", "assessment"],
                before=HypothesisModel(**old) if old else None,
                after=after,
                finding_summary=summary,
                update_reason=f"Latest contact reading {rid} arrived after the previous turn.",
                remaining_uncertainty="Zero current is never proof of de-energization; both "
                "alternatives stay open until a distinguishing observation exists.",
                trigger_kind="measurement",
                trigger_event_ids=trigger_event_ids,
                contributing_evidence_ids=[supply_evidence, rid],
                latest_measurement_ids=[rid],
                contrary_evidence_ids=contra,
                cited_values=cited,
            )
        )
        if alternatives:
            revisions[-1] = replace_untested(revisions[-1], alternatives)
    elif "H_SUPPLY" in w.hypotheses:
        unchanged += turn_assessments(tb, ["H_SUPPLY"])

    return ManagerOutput(
        summary="Report hypotheses revised against the latest three-method readings.",
        revisions=revisions,
        unchanged=unchanged,
        recommendations=recs,
        untested_alternatives=[],
    )


def replace_untested(rev: RevisionModel, alternatives: list[str]) -> RevisionModel:
    rev.remaining_uncertainty = (
        rev.remaining_uncertainty + " Open alternatives: " + "; ".join(alternatives) + "."
    )
    return rev


def turn_assessments(tb: Toolbox, hypothesis_ids: list[str]) -> list[TurnAssessment]:
    return [
        TurnAssessment(hypothesis_id=h, assessment_text="No new evidence touches this scope.")
        for h in hypothesis_ids
    ]


def add_conclusion(tb: Toolbox, output: ManagerOutput) -> ManagerOutput:
    """If a detected hazard is connected to the outage by switching or electrical evidence in
    the same component, and every material alternative has a disposition, propose concluding."""
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
        if e.source in ("report", "handshake", "telemetry")
        and e.circuit_id.split("/")[0] in tb.network.segments
        and tb.network.segments[e.circuit_id.split("/")[0]].component == comp
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
