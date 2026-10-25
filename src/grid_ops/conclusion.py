"""Concluding and reopening an outage investigation.

Completion is about the OUTAGE CAUSE, not about flow reconstruction. A concluded incident needs an
evidence-supported causal connection (a detected local hazard alone is never enough), the material
alternatives addressed, a scoped recommendation and explicit remaining uncertainty. Incomplete
global flow observability neither blocks completion nor reopens it. The separate healthy
benchmark has its own, stricter checks (observability, uncertainty, consistency, detectability).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

from grid_ops.scenario import ScenarioMode
from grid_ops.sensing import ContactCurrent, LidarPass, TdrTest

if TYPE_CHECKING:
    from grid_ops.hypotheses import ManagerOutput
    from grid_ops.manager_tools import Toolbox
    from grid_ops.world import World

MATERIAL_PRIORITY = 3  # hypotheses at or above this urgency are material alternatives
RADIUS_TARGET_MW = 25.0  # benchmark: published 95% radius must be at most this
DETECTABLE_FRACTION = 0.5  # benchmark: share of real rows that are testable


class InvestigationStatus(StrEnum):
    OPEN = "OPEN"
    CONCLUDED = "CONCLUDED"
    CONCLUDED_WITH_LIMITS = "CONCLUDED_WITH_LIMITS"  # operator accepted stated unresolved limits


@dataclass(frozen=True)
class Investigation:
    status: InvestigationStatus = InvestigationStatus.OPEN
    conclusion: dict | None = None
    decision_id: str | None = None
    concluded_s: int | None = None
    history: tuple[tuple[int, str], ...] = ()
    reopen_flags: tuple[str, ...] = ()  # relevant new evidence awaiting an explained review
    unresolved: str = ""  # a stated ambiguity and its discriminating recommendation


def is_closed(world: World) -> bool:
    return world.investigation.status is not InvestigationStatus.OPEN


# -- checking a proposed conclusion ------------------------------


def _component_of(tb: Toolbox, segment: str) -> str:
    return tb.network.segments[segment].component


def _reading_segment(reading) -> str | None:
    acquisition = reading.acquisition
    if isinstance(acquisition, LidarPass):
        return acquisition.target.segment_id
    if isinstance(acquisition, ContactCurrent | TdrTest):
        return acquisition.conductor.point.segment_id
    return None


def _evidence_segment(tb: Toolbox, evidence_id: str) -> str | None:
    w = tb.world
    if evidence_id in w.knowledge.evidence:
        c = w.knowledge.evidence[evidence_id].circuit_id
        return c.split("/")[0]
    for h in w.hazards:
        if h.id == evidence_id:
            return h.segment_id
    r = w.readings.get(evidence_id)
    return _reading_segment(r) if r else None


def check_conclusion(output: ManagerOutput, tb: Toolbox) -> list[str]:
    c = output.conclusion
    if c is None:
        return []
    w = tb.world
    errs: list[str] = []
    recs = {r.id: r for r in output.recommendations}
    if not c.remaining_uncertainty.strip():
        errs.append("conclusion: explicit remaining uncertainty is required")
    if c.kind == "unresolved_ambiguous":
        rec = recs.get(c.discriminating_recommendation_id or "")
        if rec is None or rec.action_kind not in ("measure", "inspect") or rec.scope is None:
            errs.append(
                "conclusion: an ambiguous cause needs a scoped discriminating-reading or "
                "field-inspection recommendation"
            )
        return errs
    # cause identified
    revised = {r.hypothesis_id: r.after for r in output.revisions}
    cause = revised.get(c.cause_hypothesis_id or "")
    if cause is None and c.cause_hypothesis_id in w.hypotheses:
        from grid_ops.hypotheses import HypothesisModel

        cause = HypothesisModel(**w.hypotheses[c.cause_hypothesis_id])
    if cause is None:
        return [
            *errs,
            "conclusion: the cause hypothesis is not accepted or revised in this proposal",
        ]
    if str(cause.assessment) != "SUPPORTED":
        errs.append("conclusion: the cause hypothesis must be SUPPORTED by evidence")
    hazard_segs = {_evidence_segment(tb, e) for e in c.causal_link_evidence_ids}
    has_local = any(
        e.startswith("HZ")
        or (e in w.readings and isinstance(w.readings[e].acquisition, LidarPass))
        for e in c.causal_link_evidence_ids
    )
    link = [
        e
        for e in c.causal_link_evidence_ids
        if e in w.knowledge.evidence
        and w.knowledge.evidence[e].source in ("reading", "report", "handshake")
        and not (e in w.readings and isinstance(w.readings[e].acquisition, LidarPass))
    ]
    for e in c.causal_link_evidence_ids:
        if (
            e not in w.readings
            and e not in w.knowledge.evidence
            and not any(h.id == e for h in w.hazards)
        ):
            errs.append(f"conclusion: causal link evidence {e} does not exist in the snapshot")
    hazard_components = {_component_of(tb, s) for s in hazard_segs if s in tb.network.segments}
    linked = [
        e
        for e in link
        if (
            _evidence_segment(tb, e) in tb.network.segments
            and _component_of(tb, _evidence_segment(tb, e)) in hazard_components
        )
    ]
    if not linked:
        errs.append(
            "conclusion: a detected local hazard alone cannot complete outage diagnosis; cite "
            "electrical or switching evidence that connects it to the outage"
        )
    if not has_local:
        errs.append("conclusion: the causal link must include the detected condition itself")
    material = {
        hid
        for hid, h in w.hypotheses.items()
        if h["priority"] >= MATERIAL_PRIORITY
        and hid != c.cause_hypothesis_id
        and h["lifecycle"] == "ACTIVE"
    } | {
        r.hypothesis_id
        for r in output.revisions
        if r.after.priority >= MATERIAL_PRIORITY and r.hypothesis_id != c.cause_hypothesis_id
    }
    addressed = {a.hypothesis_id for a in c.alternatives}
    for hid in sorted(material - addressed):
        errs.append(f"conclusion: material alternative {hid} is not addressed")
    rec = recs.get(c.recommendation_id or "")
    if rec is None or rec.scope is None or rec.action_kind == "none":
        errs.append("conclusion: a scoped recommendation is required")
    return errs


# -- reopening ------------------------------


def reopen_relevance(world: World, since_s: int) -> list[str]:
    """New evidence that bears on a concluded case: lidar readings inside the cause scope that
    detected a hazard, new hazards, or known switching in the cause's component. Coverage growth
    alone is deliberately not a trigger."""
    inv = world.investigation
    if inv.conclusion is None:
        return []
    cause_id = inv.conclusion.get("cause_hypothesis_id")
    cause = world.hypotheses.get(cause_id or "")
    flags = []
    if cause:
        spans = cause["scope"]
        for r in world.readings.values():
            if r.acquired_end_s <= since_s or not isinstance(r.acquisition, LidarPass):
                continue
            target = r.acquisition.target
            inside = any(
                s["segment_id"] == target.segment_id
                and s["start_km"] * 1e6 <= target.end_mm
                and s["end_km"] * 1e6 >= target.start_mm
                for s in spans
            )
            interp = world.interpretations.get(r.id)
            if inside and interp is not None and str(interp.outcome) == "positive":
                flags.append(f"{r.id}: {interp.outcome} reading inside the cause scope")
    for seg, t in world.known_switching.items():
        if t > since_s:
            flags.append(f"switching report on {seg}")
    return flags


# -- benchmark completion (a different objective) ------------------------------


@dataclass(frozen=True)
class Check:
    name: str
    met: bool
    detail: str


def benchmark_checks(world: World) -> list[Check]:
    from grid_ops.consistency import GlobalStatus, Testability

    est = world.estimate
    if world.scenario_mode is not ScenarioMode.HEALTHY_BENCHMARK:
        return [Check("benchmark", False, "not the selected objective in this scenario")]
    if est is None or not hasattr(est, "flows"):
        return [Check("estimate", False, "estimate unavailable")]
    n_active = sum(1 for f in est.flows.values() if f.p_mw != 0.0 or f.observable)
    observable = len(est.observable_segments)
    radii = [f.uncertainty_radius_95_mw for f in est.flows.values()]
    supported = [r for r in radii if r is not None]
    cons = world.consistency
    testable = sum(1 for c in cons.checks if c.testability is Testability.TESTABLE) if cons else 0
    n_checks = len(cons.checks) if cons else 0
    return [
        Check(
            "observability", observable >= n_active > 0, f"{observable} of {n_active} flows pinned"
        ),
        Check(
            "uncertainty",
            len(supported) == len(radii) and max(supported, default=1e9) <= RADIUS_TARGET_MW,
            f"{len(supported)} of {len(radii)} flows have a published 95% radius "
            "(unsupported ones cannot satisfy completion)",
        ),
        Check(
            "consistency",
            cons is not None
            and cons.global_status not in (GlobalStatus.INCONSISTENT, GlobalStatus.NOT_TESTABLE),
            f"global test {cons.global_status if cons else 'none'}",
        ),
        Check(
            "detectability",
            n_checks > 0 and testable / n_checks >= DETECTABLE_FRACTION,
            f"{testable} of {n_checks} real rows are testable",
        ),
    ]


def investigation_report(world: World) -> dict:
    """Three separately scoped results, plus what remains if time is up."""
    inv = world.investigation
    local = [h for h in world.hazards]
    report = {
        "incident_status": str(inv.status),
        "operating_assessment": f"{len(world.readings)} three-method readings",
        "local_investigation": f"{len(local)} detected hazard(s); "
        f"{len(world.coverage)} inspections recorded",
        "benchmark": [(c.name, c.met, c.detail) for c in benchmark_checks(world)]
        if world.scenario_mode is ScenarioMode.HEALTHY_BENCHMARK
        else "not selected",
    }
    if world.finished:
        open_tasks = [t.id for t in world.tasks.values() if t.status in ("pending", "reserved")]
        unmet = [] if inv.status is not InvestigationStatus.OPEN else ["no concluded diagnosis"]
        if world.scenario_mode is ScenarioMode.HEALTHY_BENCHMARK:
            unmet += [c.name for c in benchmark_checks(world) if not c.met]
        report["deadline"] = {
            "outcome": "success" if not unmet else "NOT complete",
            "remaining_work": open_tasks,
            "unmet_checks": unmet,
        }
    return report
