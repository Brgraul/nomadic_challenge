"""Validation of manager output into proposed (never accepted) findings.

Structured validation is deterministic and independent of whoever wrote the output: it rejects
missing explanations, evidence links that do not exist in the pinned snapshot, unsupported
numerical claims, incomplete measurement citations, unsupported recommendations and electrical
changes that were not tested first. A valid proposal is only ever PROPOSED_NOT_ACCEPTED; acceptance
is a separate atomic decision (ticket 10).
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, fields
from enum import StrEnum

from grid_ops.assessment import Outcome
from grid_ops.conclusion import check_conclusion, is_closed
from grid_ops.hypotheses import ManagerOutput, RecommendationModel, RevisionModel
from grid_ops.manager_tools import Toolbox
from grid_ops.sensing import RequestStatus

# Only LiDAR carries a local inspected extent in the three-method runtime, so it is the only
# local kind that can break a tie between readings.
LOCAL_KINDS = ("lidar",)
# Declared thresholds and levels the text may legitimately quote.
DECLARED_NUMBERS = (0.5, 0.4, 2.0, 10.0, 15.0, 20.0, 0.2, 0.3, 5.0, 95.0, 30.0, 4.0, 1.0)
NUMBER = re.compile(r"(-?\d+(?:\.\d+)?)\s*(km|mm|kV|MW|Mvar|°C|A|m)\b")


class ProposedStatus(StrEnum):
    PROPOSED_NOT_ACCEPTED = "PROPOSED_NOT_ACCEPTED"
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    SUPERSEDED = "SUPERSEDED"


@dataclass(frozen=True)
class ProposedFinding:
    id: str
    snapshot_revision: int
    author: str
    status: ProposedStatus
    output: ManagerOutput
    created_s: int
    history: tuple[tuple[int, str], ...] = ()  # (clock_s, note): acceptance/supersession trail
    semantic_key: str = ""  # what the explanation's meaning depended on (see decisions)


def _close(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=0.01, abs_tol=0.011)


def known_ids(tb: Toolbox) -> set[str]:
    w = tb.world
    ids = set(w.readings) | set(tb.network.reports) | {i for i, _ in w.events}
    ids |= set(w.knowledge.evidence) | {h.id for h in w.hazards} | set(tb.diagnostics)
    ids |= {i for i, _ in w.operator_events} | set(w.hypotheses)
    return ids


def _tied_readings(tb: Toolbox, rev: RevisionModel) -> list[str]:
    return tied_readings(tb.world, rev.after.scope)


def tied_readings(world, spans) -> list[str]:
    """Latest lidar reading per segment whose inspected extent falls inside the scope."""
    latest: dict[tuple[str, str], tuple[int, str]] = {}
    for r in world.readings.values():
        inspected = getattr(r.payload, "inspected", ())
        if not inspected:
            continue
        segment = inspected[0].segment_id
        for sp in spans:
            if segment != sp.segment_id:
                continue
            lo, hi = sp.start_km * 1e6, sp.end_km * 1e6
            if any(iv.start_mm <= hi and iv.end_mm >= lo for iv in inspected):
                key = (segment, "lidar")
                if key not in latest or r.acquired_end_s >= latest[key][0]:
                    latest[key] = (r.acquired_end_s, r.id)
    return sorted(i for _, i in latest.values())


def payload_values(reading) -> dict[str, float]:
    """Scalar numbers a citation can quote from one reading's payload and acquisition."""
    p, a = reading.payload, reading.acquisition
    out = {}
    for f in fields(p):
        v = getattr(p, f.name)
        if isinstance(v, int | float) and not isinstance(v, bool):
            out[f.name] = float(v)
    for name in ("integration_s", "duration_s"):
        v = getattr(a, name, None)
        if isinstance(v, int | float):
            out[f"acquisition_{name}"] = float(v)
    for iv in (*getattr(p, "inspected", ()), *getattr(p, "missing", ())):
        out[f"{iv.segment_id}:{iv.start_mm}mm"] = float(iv.start_mm)
        out[f"{iv.segment_id}:{iv.end_mm}mm"] = float(iv.end_mm)
    out["acquired_start_s"] = float(reading.acquired_start_s)
    out["acquired_end_s"] = float(reading.acquired_end_s)
    return out


def _allowed_numbers(tb: Toolbox, rev: RevisionModel) -> list[float]:
    allowed = list(DECLARED_NUMBERS)
    cited = set(rev.contributing_evidence_ids) | set(rev.latest_measurement_ids)
    cited |= {v.reading_id for v in rev.cited_values} | set(rev.contrary_evidence_ids)
    for rid in cited:
        r = tb.world.readings.get(rid)
        if r is None:
            continue
        allowed += list(payload_values(r).values())
    allowed += [v.value for v in rev.cited_values]
    for sp in rev.after.scope + (rev.before.scope if rev.before else []):
        allowed += [sp.start_km, sp.end_km]
    for did in rev.diagnostic_ids:
        d = tb.diagnostics.get(did)
        if d:
            allowed += [
                x for r in d.summary.values() for x in r.values() if isinstance(x, int | float)
            ]
    return allowed


def validate(output: ManagerOutput, tb: Toolbox) -> list[str]:
    errors: list[str] = []
    ids = known_ids(tb)
    w = tb.world
    seen_h: set[str] = set()
    if not (
        output.revisions
        or output.unchanged
        or output.recommendations
        or output.conclusion
        or output.reopen
        or output.measurement_requests
        or output.retire_request_ids
    ):
        errors.append("empty proposal: revise, assess as unchanged, or recommend something")
    for rev in output.revisions:
        tag = f"revision {rev.hypothesis_id}"
        seen_h.add(rev.hypothesis_id)
        for field_name in ("finding_summary", "update_reason", "remaining_uncertainty"):
            if not getattr(rev, field_name).strip():
                errors.append(f"{tag}: missing {field_name}")
        if rev.after.id != rev.hypothesis_id or (rev.before and rev.before.id != rev.hypothesis_id):
            errors.append(f"{tag}: before/after ids do not match the hypothesis id")
        accepted = w.hypotheses.get(rev.hypothesis_id)
        if rev.before is None and accepted is not None:
            errors.append(f"{tag}: claims creation but the hypothesis already exists")
        if rev.before is not None and (
            accepted is None or accepted != rev.before.model_dump(mode="json")
        ):
            errors.append(f"{tag}: before-values do not match the accepted hypothesis")
        if not rev.change_kinds:
            errors.append(f"{tag}: no change kind declared")
        if rev.before is not None and rev.before == rev.after:
            errors.append(f"{tag}: unchanged hypotheses get a turn assessment, not a revision")
        if rev.before is None and "creation" not in rev.change_kinds:
            errors.append(f"{tag}: a new hypothesis must declare change kind 'creation'")
        for ref in (
            rev.contributing_evidence_ids
            + rev.latest_measurement_ids
            + rev.contrary_evidence_ids
            + rev.diagnostic_ids
            + rev.trigger_event_ids
            + rev.after.supporting_evidence_ids
            + rev.after.contradicting_evidence_ids
        ):
            if ref not in ids:
                errors.append(
                    f"{tag}: evidence link {ref} does not exist in snapshot r{tb.snapshot.revision}"
                )
        for p in rev.after.predictions:
            if not p.statement.strip() or not p.expected.strip():
                errors.append(
                    f"{tag}: prediction {p.id} is not falsifiable (statement/expected empty)"
                )
        if not rev.after.predictions:
            errors.append(f"{tag}: a hypothesis must declare falsifiable predictions")
        if not rev.after.scope:
            errors.append(f"{tag}: a hypothesis must declare a spatial scope")
        text = f"{rev.finding_summary} {rev.update_reason} {rev.remaining_uncertainty}".lower()
        match rev.trigger_kind:
            case "startup":
                if w.readings:
                    errors.append(f"{tag}: startup trigger but drone measurements already exist")
                cited = rev.contributing_evidence_ids + rev.after.supporting_evidence_ids
                cites_report = any(i in tb.network.reports for i in cited)
                if not cites_report:
                    errors.append(f"{tag}: a startup revision must cite the reports it rests on")
                if "no drone measurements" not in text:
                    errors.append(
                        f"{tag}: a startup revision must state that no drone measurements exist"
                    )
            case "measurement":
                if not rev.trigger_event_ids:
                    errors.append(f"{tag}: measurement-triggered revision must cite trigger events")
                for e in rev.trigger_event_ids:
                    kind = next((ev.kind for i, ev in w.events if i == e), None)
                    if kind not in (
                        "reading",
                        "evidence_expired",
                        "switching_report",
                        "charged",
                        None,
                    ):
                        errors.append(f"{tag}: trigger {e} is not a measurement-related event")
                tied = _tied_readings(tb, rev)
                missing = set(tied) - set(rev.latest_measurement_ids)
                if missing:
                    errors.append(f"{tag}: omits the latest tied measurements {sorted(missing)}")
                cited_v = {v.reading_id for v in rev.cited_values}
                for rid in rev.latest_measurement_ids:
                    if rid not in cited_v:
                        errors.append(f"{tag}: no cited value/unit/location/conditions for {rid}")
                errors += _contrary_check(tb, rev, tied)
            case "operator":
                if rev.operator_event_id is None or rev.operator_event_id not in {
                    i for i, _ in w.operator_events
                }:
                    errors.append(
                        f"{tag}: operator-triggered change must identify a real operator event"
                    )
            case "none":
                errors.append(f"{tag}: every semantic revision needs a trigger")
        # numerical claims
        allowed = _allowed_numbers(tb, rev)
        for num, unit in NUMBER.findall(
            f"{rev.finding_summary} {rev.update_reason} {rev.remaining_uncertainty}"
        ):
            if not any(_close(float(num), a) for a in allowed):
                errors.append(f"{tag}: unsupported numerical claim '{num} {unit}'")
        for v in rev.cited_values:
            r = w.readings.get(v.reading_id)
            if r is None:
                errors.append(f"{tag}: cited value for unknown reading {v.reading_id}")
                continue
            actual = payload_values(r).get(v.quantity)
            if actual is None or not _close(v.value, actual):
                errors.append(f"{tag}: cited {v.quantity}={v.value} does not match {v.reading_id}")
    both = seen_h & {u.hypothesis_id for u in output.unchanged}
    for h in sorted(both):
        errors.append(f"{h}: appears as revised and unchanged")
    for u in output.unchanged:
        if u.hypothesis_id not in w.hypotheses:
            errors.append(f"{u.hypothesis_id}: unchanged assessment for an unknown hypothesis")
        if not u.assessment_text.strip():
            errors.append(f"{u.hypothesis_id}: unchanged hypothesis needs a turn assessment")
    for rec in output.recommendations:
        errors += check_recommendation(rec, tb, ids)
    errors += check_conclusion(output, tb)
    if (
        output.conclusion is not None
        and output.conclusion.kind == "cause_identified"
        and is_closed(w)
    ):
        errors.append("conclusion: the investigation is already concluded; reopen it first")
    if output.reopen is not None:
        if not is_closed(w):
            errors.append("reopen: the investigation is not concluded")
        for e in output.reopen.evidence_ids:
            if e not in ids:
                errors.append(
                    f"reopen: evidence {e} does not exist in snapshot r{tb.snapshot.revision}"
                )
        if not output.reopen.reason.strip() or not output.reopen.evidence_ids:
            errors.append("reopen: needs a reason and the relevant new evidence")
    errors += _request_errors(output, tb)
    return errors


def _request_errors(output: ManagerOutput, tb: Toolbox) -> list[str]:
    """Structural checks on the output's request lifecycle; feasibility is compile's job."""
    errs: list[str] = []
    w = tb.world
    seen: set[str] = set()
    for req in output.measurement_requests:
        if req.id in seen:
            errs.append(f"measurement request {req.id}: duplicate id in one output")
        seen.add(req.id)
    if len(set(output.retire_request_ids)) != len(output.retire_request_ids):
        errs.append("retire_request_ids: duplicate ids in one output")
    replaced = {r for req in output.measurement_requests for r in req.replaces_request_ids}
    for rid in sorted(replaced | set(output.retire_request_ids)):
        accepted = w.requests.get(rid)
        if accepted is None:
            errs.append(f"request {rid} does not exist in snapshot r{tb.snapshot.revision}")
        elif accepted.status is not RequestStatus.ACTIVE:
            errs.append(f"request {rid} is {accepted.status}, not ACTIVE")
    for rid in sorted(replaced & set(output.retire_request_ids)):
        errs.append(f"request {rid} is both replaced and retired")
    for req in output.measurement_requests:
        created = {r.hypothesis_id for r in output.revisions}
        for hid in req.hypothesis_ids:
            if hid not in w.hypotheses and hid not in created:
                errs.append(f"measurement request {req.id}: unknown hypothesis {hid}")
        evidence = (*req.evidence_ids, *getattr(req, "isolation_evidence_ids", ()))
        for eid in evidence:
            if eid not in known_ids(tb):
                errs.append(f"measurement request {req.id}: evidence link {eid} does not exist")
        if req.source_revision != str(tb.snapshot.revision):
            errs.append(
                f"measurement request {req.id}: source_revision {req.source_revision} is not "
                f"the pinned revision {tb.snapshot.revision}"
            )
    return errs


def _contrary_check(tb: Toolbox, rev: RevisionModel, tied: list[str]) -> list[str]:
    errs = []
    supports = rev.after.assessment in ("SUPPORTED", "PLAUSIBLE")
    contrary = []
    for rid in tied:
        interp = tb.world.interpretations.get(rid)
        if interp is None:
            continue
        if supports and interp.outcome is Outcome.QUALIFIED_NEGATIVE:
            contrary.append(rid)
        if not supports and interp.outcome is Outcome.POSITIVE:
            contrary.append(rid)
    missing = set(contrary) - set(rev.contrary_evidence_ids)
    if missing:
        errs.append(
            f"revision {rev.hypothesis_id}: contrary evidence {sorted(missing)} is not cited"
        )
    return errs


def check_recommendation(rec: RecommendationModel, tb: Toolbox, ids: set[str]) -> list[str]:
    """Independent check: a recommendation must be scoped and rest on evidence over that scope."""
    tag = f"recommendation {rec.id}"
    errs = []
    if not rec.rationale.strip() or not rec.uncertainty.strip():
        errs.append(f"{tag}: needs a rationale and the remaining uncertainty")
    for e in rec.evidence_ids:
        if e not in ids:
            errs.append(
                f"{tag}: evidence link {e} does not exist in snapshot r{tb.snapshot.revision}"
            )
    if rec.action_kind in ("inspect", "repair", "measure") and rec.scope is None:
        errs.append(f"{tag}: a {rec.action_kind} recommendation must be scoped to a span")
    if rec.action_kind == "repair":
        hz = [h for h in tb.world.hazards if h.id in rec.evidence_ids]
        ok = any(
            rec.scope
            and h.segment_id == rec.scope.segment_id
            and h.start_mm <= rec.scope.end_km * 1e6
            and h.end_mm >= rec.scope.start_km * 1e6
            for h in hz
        )
        if not ok:
            errs.append(
                f"{tag}: repair needs a detected hazard over its scope; a clean or "
                "inconclusive result cannot justify repair"
            )
    if rec.action_kind in ("inspect", "measure") and not rec.evidence_ids:
        errs.append(f"{tag}: needs at least one evidence link")
    return errs
