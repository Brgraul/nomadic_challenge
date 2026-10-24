"""Public interpretation of local sensor readings: coverage and scoped condition results.

A clean reading contradicts a local claim only where the sensor model gives adequate detection
probability under the recorded conditions (>= 95% against the declared defect level) and only
for the interval actually inspected. Thermal negatives without excitation and motion negatives
outside a qualifying gust are inconclusive. Nothing here claims that a local defect caused the
outage, or that clean readings prove mechanical integrity.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

from grid_ops.geometry import MM_PER_KM, Interval, union_length_mm
from grid_ops.knowledge import FaultAssessment
from grid_ops.scenario import (
    CLEARANCE_SIGMA_M,
    CLEARANCE_THRESHOLD_M,
    DEFECT_CLEARANCE_M,
    DEFECT_EXCESS_C,
    DEFECT_MOTION_M,
    DETECTION_REQUIREMENT,
    ENV_TEMP_C,
    HEATING_C_PER_A2,
    MOTION_SIGMA_M,
    MOTION_THRESHOLD_M,
    TEMP_SIGMA_C,
    THERMAL_EXCESS_THRESHOLD_C,
    THERMAL_EXCITATION_A,
)

if TYPE_CHECKING:
    from grid_ops.world import Reading, World

LOCAL_KINDS = ("clearance", "conductor_temp", "motion")


class Verdict(StrEnum):
    PASS = "PASS"
    FAIL = "FAIL"
    INDETERMINATE = "INDETERMINATE"
    NOT_SUPPORTED = "NOT_SUPPORTED"


class Outcome(StrEnum):
    POSITIVE = "positive"  # threshold event detected
    QUALIFIED_NEGATIVE = "qualified_negative"  # adequate detection conditions, nothing found
    INCONCLUSIVE = "inconclusive"


# condition name -> measurement kind that can test it
CONDITIONS = {
    "vegetation_clearance": "clearance",
    "hot_joint": "conductor_temp",
    "conductor_motion": "motion",
}
HAZARD_TEXT = {
    "clearance": "low clearance (threshold {t} m)",
    "conductor_temp": "thermal excess (threshold {t} C)",
    "motion": "conductor motion (threshold {t} m)",
}


def _phi(x: float) -> float:
    return 0.5 * (1 + math.erf(x / math.sqrt(2)))


@dataclass(frozen=True)
class SpanRef:
    segment_id: str
    start_mm: int
    end_mm: int


@dataclass(frozen=True)
class InspectionCoverage:
    """What one accepted local reading actually inspected, and under which conditions."""

    reading_id: str
    segment_id: str
    kind: str
    nominal: Interval
    inspected: tuple[Interval, ...]
    occluded: tuple[Interval, ...]
    visible_fraction: float
    conditions: tuple[tuple[str, float], ...]
    epoch: str


@dataclass(frozen=True)
class ReadingInterpretation:
    reading_id: str
    outcome: Outcome
    detection_probability: float
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


# -- interpretation of one reading ---------------------------------------------------------------


def detection_probability(kind: str, conditions: dict[str, float]) -> float:
    """P(reading flags a threshold event | the worst credible defect), 0 if conditions are
    inadequate to see it at all."""
    if conditions.get("visible_fraction", 0.0) <= 0:
        return 0.0
    match kind:
        case "clearance":
            return _phi((CLEARANCE_THRESHOLD_M - DEFECT_CLEARANCE_M) / CLEARANCE_SIGMA_M)
        case "conductor_temp":
            if conditions.get("excitation_current_a", 0.0) <= THERMAL_EXCITATION_A:
                return 0.0
            return _phi((DEFECT_EXCESS_C - THERMAL_EXCESS_THRESHOLD_C) / TEMP_SIGMA_C)
        case "motion":
            if not conditions.get("qualifying_gust", 0.0):
                return 0.0
            return _phi((DEFECT_MOTION_M - MOTION_THRESHOLD_M) / MOTION_SIGMA_M)
    return 0.0


def inspected_intervals(reading: Reading) -> tuple[Interval, ...]:
    return tuple(Interval(reading.point.segment_id, a, b) for a, b in reading.inspected)


def interpret(reading: Reading, nominal: Interval) -> ReadingInterpretation:
    kind = str(reading.kind)
    cond = dict(reading.conditions)
    values = dict(reading.payload)
    inspected = inspected_intervals(reading)
    p = detection_probability(kind, cond)
    if not inspected or not values:
        return ReadingInterpretation(
            reading.id, Outcome.INCONCLUSIVE, 0.0, ("conductor not visible: nothing inspected",), ()
        )
    match kind:
        case "clearance":
            value = values["clearance_m"]
            positive = value < CLEARANCE_THRESHOLD_M
        case "conductor_temp":
            current = cond.get("excitation_current_a", 0.0)
            baseline = ENV_TEMP_C + HEATING_C_PER_A2 * current**2
            value = values["temp_c"] - baseline
            positive = value > THERMAL_EXCESS_THRESHOLD_C and p > 0
        case "motion":
            value = values["amplitude_m"]
            positive = value > MOTION_THRESHOLD_M and p > 0
        case _:
            return ReadingInterpretation(reading.id, Outcome.INCONCLUSIVE, 0.0, ("not local",), ())
    if positive:
        return ReadingInterpretation(
            reading.id, Outcome.POSITIVE, p, ("threshold event detected",), inspected, value
        )
    if p >= DETECTION_REQUIREMENT:
        return ReadingInterpretation(
            reading.id,
            Outcome.QUALIFIED_NEGATIVE,
            p,
            (f"adequate conditions, detection probability {p:.3f}",),
            inspected,
            value,
        )
    why = {
        "conductor_temp": "thermal excitation too low (current below the excitation threshold)",
        "motion": "no qualifying gust window during the reading",
        "clearance": "insufficient detection probability",
    }[kind]
    return ReadingInterpretation(reading.id, Outcome.INCONCLUSIVE, p, (why,), inspected, value)


# -- ingestion -----------------------------------------------------------------------------------


def ingest_local_reading(world: World, reading: Reading, nominal: Interval) -> None:
    """Record coverage; a detected threshold event becomes a scoped hazard for landing checks."""
    inspected = inspected_intervals(reading)
    occluded = _minus(nominal, inspected)
    span = max(nominal.end_mm - nominal.start_mm, 1)
    world.coverage.append(
        InspectionCoverage(
            reading.id,
            reading.point.segment_id,
            str(reading.kind),
            nominal,
            inspected,
            occluded,
            sum(i.end_mm - i.start_mm for i in inspected) / span,
            reading.conditions,
            reading.operating_epoch,
        )
    )
    interp = interpret(reading, nominal)
    world.interpretations[reading.id] = interp
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
                    (HAZARD_TEXT[str(reading.kind)].format(t=_threshold(str(reading.kind))),),
                )
            )


def _threshold(kind: str) -> float:
    return {
        "clearance": CLEARANCE_THRESHOLD_M,
        "conductor_temp": THERMAL_EXCESS_THRESHOLD_C,
        "motion": MOTION_THRESHOLD_M,
    }[kind]


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
    occluded: tuple[Interval, ...]
    uninspected: tuple[Interval, ...]
    inspected_mm: int
    scope_mm: int
    stop_count: int


def coverage_report(world: World, scope: SpanRef, kind: str) -> CoverageReport:
    recs = [c for c in world.coverage if c.segment_id == scope.segment_id and c.kind == kind]
    inspected = merge(_clip([i for c in recs for i in c.inspected], scope))
    occluded_raw = merge(_clip([o for c in recs for o in c.occluded], scope))
    occluded = [
        o for part in occluded_raw for o in _minus(part, inspected)
    ]  # occluded only where no other reading saw it
    base = Interval(scope.segment_id, scope.start_mm, scope.end_mm)
    uninspected = _minus(base, inspected)
    return CoverageReport(
        scope.segment_id,
        kind,
        tuple(inspected),
        tuple(merge(occluded)),
        uninspected,
        union_length_mm(inspected),
        scope.end_mm - scope.start_mm,
        len(recs),
    )


ASSUMPTIONS = {
    "clearance": (
        "Gaussian clearance error 0.05 m",
        "adverse value over the inspected interval",
        "95% detection against a 0.4 m defect",
    ),
    "conductor_temp": (
        "thermal error 1 C; baseline = environment + k*I^2",
        "excitation current above 20 A required",
        "95% detection against a 15 C excess",
    ),
    "motion": (
        "motion error 0.02 m",
        "a qualifying gust window is required",
        "95% detection against 0.30 m",
    ),
}


def assess_scope(world: World, scope: SpanRef, condition: str) -> ScopeAssessment:
    """Verdict for one condition over one scope, from the readings that inspected it."""
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
    clear: list[Interval] = []
    inconclusive: list[Interval] = []
    evidence: list[str] = []
    for rec in world.coverage:
        if rec.segment_id != scope.segment_id or rec.kind != kind:
            continue
        interp = world.interpretations[rec.reading_id]
        part = _clip(list(interp.inspected), scope)
        if not part:
            continue
        evidence.append(rec.reading_id)
        match interp.outcome:
            case Outcome.POSITIVE:
                failed += part
            case Outcome.QUALIFIED_NEGATIVE:
                clear += part
            case Outcome.INCONCLUSIVE:
                inconclusive += part
    failed_m, clear_m = merge(failed), merge(clear)
    inconclusive_m = [x for iv in merge(inconclusive) for x in _minus(iv, failed_m + clear_m)]
    clear_only = [x for iv in clear_m for x in _minus(iv, failed_m)]
    untested = _minus(base, merge(failed_m + clear_only + inconclusive_m))
    assumptions = ASSUMPTIONS[kind]
    disclaimer = (
        " Local findings do not show this condition caused the outage; clean readings do not "
        "prove mechanical integrity."
    )
    covered_clear = union_length_mm(clear_only)
    if failed_m:
        verdict = Verdict.FAIL
        text = (
            f"Threshold event detected on {_mm(failed_m)}; "
            f"qualified-clear on {_mm(clear_only) or 'none'}."
        )
    elif covered_clear >= base.end_mm - base.start_mm and base.end_mm > base.start_mm:
        verdict = Verdict.PASS
        text = "Qualified negative evidence covers the whole scope under adequate conditions."
    else:
        verdict = Verdict.INDETERMINATE
        text = (
            f"Qualified-clear {_mm(clear_only) or 'none'}; "
            f"inconclusive {_mm(inconclusive_m) or 'none'}; "
            f"untested {_mm(list(untested)) or 'none'}. Untested alternatives remain open."
        )
    return ScopeAssessment(
        scope,
        condition,
        verdict,
        tuple(failed_m),
        tuple(clear_only),
        tuple(inconclusive_m),
        tuple(untested),
        tuple(sorted(set(evidence))),
        assumptions,
        text + disclaimer,
    )


def _mm(ivs: list[Interval]) -> str:
    return ", ".join(
        f"{iv.segment_id} {iv.start_mm / MM_PER_KM:.3f}-{iv.end_mm / MM_PER_KM:.3f} km"
        for iv in ivs
    )


def segment_assessments(world: World) -> list[ScopeAssessment]:
    """Assessments of every tested condition over each segment's full length."""
    seen = {(c.segment_id, c.kind) for c in world.coverage}
    out = []
    for seg, kind in sorted(seen):
        length = world.segment_lengths[seg]
        for condition, k in CONDITIONS.items():
            if k == kind:
                out.append(assess_scope(world, SpanRef(seg, 0, length), condition))
    return out
