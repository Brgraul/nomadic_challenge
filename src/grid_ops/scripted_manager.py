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
from grid_ops.outage import MIN_USEFUL_BITS, StopReason
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
            overlap = any(i.start_mm <= hi and i.end_mm >= lo for i in payload.inspected)
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
            assessment, priority, summary = (
                Assessment.SUPPORTED,
                5,
                (
                    "A lidar scan found clearance below the declared threshold on the inspected "
                    "interval."
                ),
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
            assessment, priority, summary = (
                Assessment.UNRESOLVED,
                3,
                ("The lidar scans so far do not show clearance below the threshold."),
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

    # a revision that changes nothing is an unchanged-turn assessment, never a revision
    kept = []
    for rev in revisions:
        if rev.before is not None and rev.before == rev.after:
            unchanged += turn_assessments(tb, [rev.hypothesis_id])
        else:
            kept.append(rev)
    revisions = kept
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


# -- adaptive requests from the public reconstruction ------------------------------------

FAULT_ID = "H_FAULT"
ADAPTIVE_PREFIX = "REQ-A"


def _whole_path_span(tb: Toolbox, segment_id: str) -> SpanModel:
    """The whole mapped path, in the same millimetre offsets readings use (rounded outward)."""
    length_mm = tb.world.segment_lengths[segment_id]
    return SpanModel(segment_id=segment_id, start_km=0.0, end_km=-(-length_mm // 1000) / 1000)


def _fault_segment(tb: Toolbox) -> str | None:
    b = tb.world.outage_belief
    if b is None or not b.regions:
        return None
    best = max(b.regions, key=lambda r: (r.fault_mass, r.segment_id))
    return best.segment_id if best.fault_mass > 0 else None


def _fault_hypothesis(tb: Toolbox, segment_id: str, assessment: Assessment) -> HypothesisModel:
    return HypothesisModel(
        id=FAULT_ID,
        claim=(
            f"A primary open-conductor fault lies on {segment_id}, followed by a supply-side "
            "protective trip, so the conductor beyond the protective device is disconnected."
        ),
        scope=[_whole_path_span(tb, segment_id)],
        predictions=[
            PredictionModel(
                id=f"{FAULT_ID}-P1",
                statement=(
                    "If the claim holds, a coupled reflectometry test from the disconnected side "
                    "shows an open-end echo at the fault and at the protective device."
                ),
                measurement="echo",
                expected="open-end echoes at distances matching one fault cell",
                required_conditions=["a valid return path", "landed access to the conductor"],
            ),
            PredictionModel(
                id=f"{FAULT_ID}-P2",
                statement=(
                    "If the claim holds, a landed contact sample on the disconnected side reads "
                    "near-zero current, which a live-unloaded branch would also show."
                ),
                measurement="current",
                expected="RMS current near zero on the sampled conductor",
                required_conditions=["the drone can land on the conductor"],
            ),
        ],
        assessment=assessment,
        priority=4,
        supporting_evidence_ids=[],
    )


def _proposal_request(tb: Toolbox, n: int, proposal, evidence: list[str], alternatives):
    from grid_ops.sensing import ConductorScope, CorridorSpan, OutcomeSpec, TdrRequest

    common = dict(
        id=f"{ADAPTIVE_PREFIX}{tb.snapshot.revision}-{n}",
        hypothesis_ids=(FAULT_ID,),
        rationale=(
            f"This {proposal.method} measurement on {proposal.segment_id} is the one the public "
            "reconstruction ranks highest for separating the leading alternatives."
        ),
        alternatives_distinguished=tuple(alternatives),
        expected_outcomes=(
            OutcomeSpec(description="an outcome consistent with one alternative group"),
            OutcomeSpec(description="an outcome consistent with the other alternative group"),
        ),
        evidence_ids=tuple(evidence),
        urgency=4,
        source_revision=str(tb.snapshot.revision),
        replaces_request_ids=(),
    )
    n_mm = tb.world.segment_lengths[proposal.segment_id]
    if proposal.method == "lidar":
        return _request(
            tb,
            common["id"],
            FAULT_ID,
            common["rationale"],
            common["alternatives_distinguished"],
            [o.description for o in common["expected_outcomes"]],
            evidence,
            4,
            CorridorSpan(
                segment_id=proposal.segment_id, start_mm=proposal.start_mm, end_mm=proposal.end_mm
            ),
        )
    x = proposal.start_mm
    scope = ConductorScope(
        circuit_id=proposal.circuit_id,
        phase="A",
        start_mm=max(0, x - 500) if proposal.method == "tdr" else x,
        end_mm=min(n_mm, x + 500) if proposal.method == "tdr" else min(n_mm, x + 1),
    )
    if proposal.method == "tdr":
        return TdrRequest(**common, target=scope)
    return _request(
        tb,
        common["id"],
        FAULT_ID,
        common["rationale"],
        common["alternatives_distinguished"],
        [o.description for o in common["expected_outcomes"]],
        evidence,
        4,
        scope,
    )


UNRESOLVED_STOPS = frozenset(
    {
        StopReason.AMBIGUOUS,
        StopReason.MODEL_MISMATCHED,
        StopReason.NO_SAFE_EXPERIMENT,
        StopReason.BUDGET_EXHAUSTED,
        StopReason.MODEL_LIMITED,
    }
)
STALE_REQUEST_S = 1200  # an unserved adaptive request this old is retired and not re-proposed


def _request_key(request) -> tuple:
    target = request.target
    if hasattr(target, "segment_id"):
        return (str(request.method), target.segment_id, target.start_mm)
    return (str(request.method), target.circuit_id, (target.start_mm + target.end_mm) // 2)


def _unserved_adaptive(tb: Toolbox, *, stale: bool) -> list:
    """Adaptive requests with no completed task: still active (stale=False selects the young
    ones, stale=True the old ones) or already retired unserved."""
    from grid_ops.sensing import RequestStatus, TaskStatusV2

    w = tb.world
    out = []
    for a in w.requests.values():
        if not a.id.startswith(ADAPTIVE_PREFIX):
            continue
        if any(
            t.request_id == a.id and t.status is TaskStatusV2.COMPLETED for t in w.tasks.values()
        ):
            continue
        old = a.accepted_s <= w.clock_s - STALE_REQUEST_S
        if (a.status is RequestStatus.ACTIVE and old == stale) or (
            stale and a.status is RequestStatus.RETIRED
        ):
            out.append(a)
    return out


def _outstanding_adaptive(tb: Toolbox) -> bool:
    return any(a.status == "ACTIVE" for a in _unserved_adaptive(tb, stale=False))


def add_adaptive(
    tb: Toolbox, output: ManagerOutput, trigger_event_ids: list[str], conclude: bool = False
) -> ManagerOutput:
    """Track the leading fault hypothesis and request the highest-value separating measurement."""
    from grid_ops.assessment import Outcome
    from grid_ops.proposals import tied_readings

    w = tb.world
    segment = _fault_segment(tb)
    if segment is None or w.outage_stop is None:
        return output
    stop = w.outage_stop
    started = bool(w.readings)
    supported = stop.reason is StopReason.SUPPORTED
    leading = next((f for f in w.outage_belief.supported_facts if f.kind == "fault_segment"), None)
    if supported:
        assessment = Assessment.SUPPORTED
    elif leading is not None and leading.probability >= 0.5:
        assessment = Assessment.PLAUSIBLE
    else:
        assessment = Assessment.UNRESOLVED
    new = _fault_hypothesis(tb, segment, assessment)
    old = w.hypotheses.get(FAULT_ID)
    reports = sorted(tb.network.reports)[:1]
    tied = tied_readings(w, new.scope) if started else []
    supports = assessment in (Assessment.SUPPORTED, Assessment.PLAUSIBLE)
    contrary = [
        rid
        for rid in tied
        if (i := w.interpretations.get(rid)) is not None
        and i.outcome is (Outcome.QUALIFIED_NEGATIVE if supports else Outcome.POSITIVE)
    ]
    cited = [
        CitedValue(
            reading_id=rid,
            quantity="clearance_m",
            value=w.readings[rid].payload.clearance_m,
            unit="m",
            location="lidar inspected intervals",
            conditions="geometric visibility recorded on the reading",
        )
        for rid in tied
        if w.readings[rid].payload.clearance_m is not None
    ]
    evidence = [*reports, *sorted(w.readings)[-3:]]
    if old is None or old != new.model_dump(mode="json"):
        revision = RevisionModel(
            hypothesis_id=FAULT_ID,
            change_kinds=["creation"] if old is None else ["assessment", "scope"],
            before=None if old is None else HypothesisModel(**old),
            after=new,
            finding_summary=(
                "The public reconstruction ranks this segment highest for the primary fault; "
                "the claim is conditional on the declared model family and is not a measured fact."
                + ("" if started else " No drone measurements exist yet.")
            ),
            update_reason=(
                "The reconstruction was refreshed from the executed readings."
                if started
                else "At startup only the reports exist and no drone measurements exist yet."
            ),
            remaining_uncertainty=(
                "Alternatives the evidence cannot yet separate: " + "; ".join(stop.unresolved[:3])
                if stop.unresolved
                else "Fault position within the segment and the faulted circuit remain open."
            ),
            trigger_kind="measurement" if started else "startup",
            trigger_event_ids=trigger_event_ids if started else [],
            contributing_evidence_ids=evidence if started else reports,
            latest_measurement_ids=tied,
            contrary_evidence_ids=contrary,
            cited_values=cited,
        )
        output.revisions.append(revision)
    else:
        output.unchanged.append(
            TurnAssessment(hypothesis_id=FAULT_ID, assessment_text="No new evidence touches this.")
        )
    unserved = _unserved_adaptive(tb, stale=True)
    output.retire_request_ids += [a.id for a in unserved if str(a.status) == "ACTIVE"]
    unserved_keys = {_request_key(a.request) for a in unserved}
    if not supported and not _outstanding_adaptive(tb) and w.experiment_proposals:
        alternatives = [a for a in stop.unresolved[:2]] or ["fault position within the segment"]

        def key_of(p) -> tuple:
            if p.method == "lidar":
                return (p.method, p.segment_id, p.start_mm)
            return (p.method, p.circuit_id, p.start_mm)

        ranked = sorted(
            (
                p
                for p in w.experiment_proposals
                if p.eig_bits >= MIN_USEFUL_BITS and key_of(p) not in unserved_keys
            ),
            key=lambda p: (p.batch_order == 0, p.batch_order, -p.eig_bits),
        )
        picked = ranked[:3]
        for n, proposal in enumerate(picked):
            output.measurement_requests.append(
                _proposal_request(
                    tb,
                    n,
                    proposal,
                    [e for e in evidence if e in tb.network.reports or e in w.readings],
                    alternatives,
                )
            )
    closed = w.investigation.status.value != "OPEN"
    if conclude and closed:
        pass  # a concluded case is reopened only by new relevant evidence, never re-concluded
    elif conclude and supported:
        output = _adaptive_conclusion(tb, output, segment)
    elif conclude and stop.reason in UNRESOLVED_STOPS:
        output = _adaptive_unresolved(tb, output, segment)
    output.summary = "Fault hypothesis tracked against the reconstruction; " + output.summary
    return output


def _adaptive_conclusion(tb: Toolbox, output: ManagerOutput, segment: str) -> ManagerOutput:
    from grid_ops.hypotheses import AlternativeDisposition, ConclusionModel
    from grid_ops.sensing import ContactCurrent as CC
    from grid_ops.sensing import TdrTest

    w = tb.world
    fact = next(
        (f for f in w.outage_belief.supported_facts if f.kind == "fault_interval" and f.supported),
        None,
    )
    if fact is None or fact.interval is None:
        return output
    link = sorted(
        rid
        for rid, r in w.readings.items()
        if isinstance(r.acquisition, CC | TdrTest) and r.quality.valid
    )
    if not link:
        return output
    span = SpanModel(
        segment_id=segment,
        start_km=round(fact.interval.start_mm / 1e6, 6),
        end_km=round(fact.interval.end_mm / 1e6, 6),
    )
    output.recommendations.append(
        RecommendationModel(
            id="REC-FAULT-1",
            action_kind="inspect",
            text="Field inspection of the supported fault interval before any switching.",
            scope=span,
            urgency=5,
            evidence_ids=link,
            rationale="The reconstruction supports a single fault interval from executed echoes.",
            uncertainty="Conditional on the declared model family; unmodelled causes are possible.",
        )
    )
    others = [
        h
        for h, v in w.hypotheses.items()
        if h != FAULT_ID and v["priority"] >= 3 and v["lifecycle"] == "ACTIVE"
    ]
    output.conclusion = ConclusionModel(
        kind="cause_identified",
        cause_hypothesis_id=FAULT_ID,
        causal_statement=(
            f"{FAULT_ID}: executed echo and current evidence support one open-conductor fault "
            f"interval on {segment}, conditional on the declared model family."
        ),
        causal_link_evidence_ids=link,
        alternatives=[
            AlternativeDisposition(
                hypothesis_id=h, disposition="indistinguishable", note="not separated"
            )
            for h in others
        ],
        recommendation_id="REC-FAULT-1",
        remaining_uncertainty="Conditional on the declared model; other causes remain possible.",
    )
    return output


def _adaptive_unresolved(tb: Toolbox, output: ManagerOutput, segment: str) -> ManagerOutput:
    """Stop honestly: the reconstruction cannot be sharpened by anything the fleet can fly."""
    from grid_ops.hypotheses import ConclusionModel

    w = tb.world
    stop = w.outage_stop
    if output.conclusion is not None or any(r.id == "REC-NEXT" for r in output.recommendations):
        return output
    output.recommendations.append(
        RecommendationModel(
            id="REC-NEXT",
            action_kind="inspect",
            text=f"Field-inspect {segment}, the leading candidate, to separate what is unresolved.",
            scope=_whole_path_span(tb, segment),
            urgency=4,
            evidence_ids=sorted(w.readings)[-3:] or sorted(tb.network.reports)[:1],
            rationale=f"The reconstruction stopped as {stop.reason}: {stop.detail}",
            uncertainty="The fault position and mechanism are not established by the evidence.",
        )
    )
    output.conclusion = ConclusionModel(
        kind="unresolved_ambiguous",
        causal_statement=(
            f"The cause is not established: the reconstruction stopped as {stop.reason}."
        ),
        discriminating_recommendation_id="REC-NEXT",
        remaining_uncertainty="; ".join(stop.unresolved[:3])
        or "The evidence does not separate the remaining alternatives.",
    )
    return output
