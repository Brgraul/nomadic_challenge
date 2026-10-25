"""Public interpretation of LiDAR readings: coverage and scoped vegetation-clearance results.

LiDAR is the only local method in the three-method runtime. A below-threshold clearance over an
inspected interval is an observed geometric fact and a hazard for landing checks; a clearance at
or above the threshold, or no coverage, is inconclusive and never clears the scope. Nothing here
claims that a local hazard caused the outage, or that clean readings prove mechanical integrity.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

from grid_ops.geometry import MM_PER_KM, Interval, union_length_mm
from grid_ops.knowledge import FaultAssessment
from grid_ops.scenario import CLEARANCE_THRESHOLD_M
from grid_ops.sensing import LidarPass, LidarPayload

if TYPE_CHECKING:
    from grid_ops.world import World

LIDAR = "lidar"
CONDITIONS = {"vegetation_clearance": LIDAR}
HAZARD_TEXT = "low clearance (threshold {t} m)"


class Verdict(StrEnum):
    PASS = "PASS"
    FAIL = "FAIL"
    INDETERMINATE = "INDETERMINATE"
    NOT_SUPPORTED = "NOT_SUPPORTED"


class Outcome(StrEnum):
    POSITIVE = "positive"  # threshold event detected over inspected intervals
    QUALIFIED_NEGATIVE = "qualified_negative"  # kept for proposal validation; lidar never emits it
    INCONCLUSIVE = "inconclusive"


@dataclass(frozen=True)
class SpanRef:
    segment_id: str
    start_mm: int
    end_mm: int


@dataclass(frozen=True)
class LidarCoverage:
    """What one lidar reading inspected and missed over its requested target."""

    reading_id: str
    segment_id: str
    target: Interval
    inspected: tuple[Interval, ...]
    missing: tuple[Interval, ...]
    epoch: str


@dataclass(frozen=True)
class ReadingInterpretation:
    reading_id: str
    outcome: Outcome
    reasons: tuple[str, ...]
    inspected: tuple[Interval, ...]
    value: float | None = None


@dataclass(frozen=True)
class ScopeAssessment:
    scope: SpanRef
    condition: str
    verdict: Verdict
    failed: tuple[Interval, ...]
    tested_clear: tuple[Interval, ...]
    inconclusive: tuple[Interval, ...]
    untested: tuple[Interval, ...]
    evidence_ids: tuple[str, ...]
    assumptions: tuple[str, ...]
    explanation: str


ASSUMPTIONS = {
    LIDAR: (
        "Gaussian clearance error 0.05 m",
        "adverse value over the inspected interval",
        "an above-threshold clearance is inconclusive, never a clear",
    ),
}


# -- interpretation of one reading ---------------------------------------------------------------


def interpret_lidar(reading) -> ReadingInterpretation:
    """A reading is POSITIVE only where its own payload shows a below-threshold clearance."""
    payload = reading.payload
    inspected = tuple(payload.inspected)
    if payload.clearance_m is None:
        return ReadingInterpretation(
            reading.id,
            Outcome.INCONCLUSIVE,
            ("clearance missing; no outage conclusion",),
            inspected,
            None,
        )
    if payload.clearance_m < CLEARANCE_THRESHOLD_M:
        return ReadingInterpretation(
            reading.id,
            Outcome.POSITIVE,
            (
                f"clearance {payload.clearance_m:.2f} m below the "
                f"{CLEARANCE_THRESHOLD_M} m threshold",
            ),
            inspected,
            payload.clearance_m,
        )
    return ReadingInterpretation(
        reading.id,
        Outcome.INCONCLUSIVE,
        ("clearance above threshold or missing; no outage conclusion",),
        inspected,
        payload.clearance_m,
    )


# -- ingestion -----------------------------------------------------------------------------------


def ingest_lidar(world: World, reading) -> None:
    """Record coverage and interpretation for one lidar reading; a below-threshold clearance
    becomes a scoped hazard for landing checks. Idempotent per reading id."""
    if not isinstance(reading.payload, LidarPayload) or reading.id in world.interpretations:
        return
    interp = interpret_lidar(reading)
    world.interpretations[reading.id] = interp
    target = reading.acquisition.target
    world.coverage.append(
        LidarCoverage(
            reading.id,
            target.segment_id,
            target,
            reading.payload.inspected,
            reading.payload.missing,
            reading.evidence_epoch,
        )
    )
    if interp.outcome is Outcome.POSITIVE:
        for iv in interp.inspected:
            world.hazards.append(
                FaultAssessment(
                    f"HZ{len(world.hazards) + 1:03d}",
                    iv.segment_id,
                    iv.start_mm,
                    iv.end_mm,
                    "FAIL",
                    (reading.id,),
                    (HAZARD_TEXT.format(t=CLEARANCE_THRESHOLD_M),),
                )
            )


# -- interval arithmetic -------------------------------------------------------------------------


def merge(intervals: list[Interval]) -> list[Interval]:
    out: list[Interval] = []
    for iv in sorted(intervals, key=lambda i: (i.segment_id, i.start_mm)):
        if out and out[-1].segment_id == iv.segment_id and iv.start_mm <= out[-1].end_mm:
            if iv.end_mm > out[-1].end_mm:
                out[-1] = Interval(iv.segment_id, out[-1].start_mm, iv.end_mm)
        else:
            out.append(iv)
    return out


def _minus(base: Interval, remove: tuple[Interval, ...] | list[Interval]) -> tuple[Interval, ...]:
    parts = [(base.start_mm, base.end_mm)]
    for r in merge(list(remove)):
        nxt = []
        for a, b in parts:
            if r.end_mm <= a or r.start_mm >= b:
                nxt.append((a, b))
                continue
            if a < r.start_mm:
                nxt.append((a, r.start_mm))
            if r.end_mm < b:
                nxt.append((r.end_mm, b))
        parts = nxt
    return tuple(Interval(base.segment_id, a, b) for a, b in parts if b > a)


def _clip(ivs: list[Interval], scope: SpanRef) -> list[Interval]:
    out = []
    for iv in ivs:
        a, b = max(iv.start_mm, scope.start_mm), min(iv.end_mm, scope.end_mm)
        if b > a:
            out.append(Interval(scope.segment_id, a, b))
    return out


# -- coverage and scoped assessments -------------------------------------------------------------


@dataclass(frozen=True)
class CoverageReport:
    segment_id: str
    kind: str
    inspected: tuple[Interval, ...]
    missing: tuple[Interval, ...]
    uninspected: tuple[Interval, ...]
    inspected_mm: int
    scope_mm: int
    stop_count: int


def coverage_report(world: World, scope: SpanRef, kind: str = LIDAR) -> CoverageReport:
    """Union of inspected and missing extent across lidar readings whose target overlaps the
    scope. Missing extent counts only where no reading inspected."""
    base = Interval(scope.segment_id, scope.start_mm, scope.end_mm)
    inspected_raw: list[Interval] = []
    missing_raw: list[Interval] = []
    count = 0
    for reading in world.readings.values():
        acquisition = reading.acquisition
        if not isinstance(acquisition, LidarPass):
            continue
        if acquisition.target.segment_id != scope.segment_id:
            continue
        if (
            acquisition.target.end_mm <= scope.start_mm
            or acquisition.target.start_mm >= scope.end_mm
        ):
            continue
        count += 1
        inspected_raw += _clip(list(reading.payload.inspected), scope)
        missing_raw += _clip(list(reading.payload.missing), scope)
    inspected = merge(inspected_raw)
    missing = [x for part in merge(missing_raw) for x in _minus(part, inspected)]
    uninspected = _minus(base, inspected)
    return CoverageReport(
        scope.segment_id,
        kind,
        tuple(inspected),
        tuple(missing),
        uninspected,
        union_length_mm(inspected),
        scope.end_mm - scope.start_mm,
        count,
    )


def assess_scope(world: World, scope: SpanRef, condition: str) -> ScopeAssessment:
    """Verdict for one condition over one scope, from the lidar readings that inspected it."""
    kind = CONDITIONS.get(condition)
    base = Interval(scope.segment_id, scope.start_mm, scope.end_mm)
    if kind is None:
        return ScopeAssessment(
            scope,
            condition,
            Verdict.NOT_SUPPORTED,
            (),
            (),
            (),
            (base,),
            (),
            (),
            f"No sensor here can test '{condition}'; it stays unassessed, not cleared.",
        )
    failed: list[Interval] = []
    inconclusive: list[Interval] = []
    evidence: list[str] = []
    for rec in world.coverage:
        if rec.segment_id != scope.segment_id or rec.reading_id not in world.interpretations:
            continue
        interp = world.interpretations[rec.reading_id]
        part = _clip(list(interp.inspected), scope)
        if not part:
            continue
        evidence.append(rec.reading_id)
        if interp.outcome is Outcome.POSITIVE:
            failed += part
        else:
            inconclusive += part
    failed_m, inconclusive_m = merge(failed), merge(inconclusive)
    untested = _minus(base, failed_m + inconclusive_m)
    if failed_m:
        verdict = Verdict.FAIL
        text = f"Threshold event detected on {_mm(failed_m)}."
    else:
        verdict = Verdict.INDETERMINATE
        text = (
            f"Inconclusive {_mm(inconclusive_m) or 'none'}; "
            f"untested {_mm(list(untested)) or 'none'}. Untested alternatives remain open; "
            "an above-threshold clearance is never a clear."
        )
    disclaimer = (
        " Local findings do not show this condition caused the outage; clean readings do not "
        "prove mechanical integrity."
    )
    return ScopeAssessment(
        scope,
        condition,
        verdict,
        tuple(failed_m),
        (),
        tuple(inconclusive_m),
        tuple(untested),
        tuple(sorted(set(evidence))),
        ASSUMPTIONS[kind],
        text + disclaimer,
    )


def _mm(ivs: list[Interval]) -> str:
    return ", ".join(
        f"{iv.segment_id} {iv.start_mm / MM_PER_KM:.3f}-{iv.end_mm / MM_PER_KM:.3f} km"
        for iv in ivs
    )


def segment_assessments(world: World) -> list[ScopeAssessment]:
    """Assessments of the tested condition over each scanned segment's full length."""
    seen = {c.segment_id for c in world.coverage}
    return [
        assess_scope(world, SpanRef(seg, 0, world.segment_lengths[seg]), "vegetation_clearance")
        for seg in sorted(seen)
    ]
