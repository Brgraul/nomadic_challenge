"""Measurement needs and the tasks that serve them.

A need exists only because an *accepted* hypothesis has an open causal question. It states the
uncertainty, the alternatives a reading would distinguish and how a result could change the
diagnosis or action. Tasks are attempts at specific sites that serve one or more needs; refresh
is deterministic and runs whenever evidence, covariance, coverage, reservations, expiry or
availability change, even when hypotheses did not. General flow-observability gaps create work
only in the explicitly selected healthy_benchmark.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum

from grid_ops import assessment, estimation
from grid_ops.assessment import SpanRef
from grid_ops.geometry import Geometry, LinePoint
from grid_ops.scenario import ScenarioMode
from grid_ops.world import Measurement, MeasurementTask, TaskStatus, World

MAX_SITES_PER_NEED = 12
MAX_RECONSTRUCTION_NEEDS = 12  # staged: equal-urgency work must fit under the planner's attempt cap
LOCAL = {"clearance", "conductor_temp", "motion"}
ELECTRICAL = {"current", "voltage_presence"}


class NeedStatus(StrEnum):
    ACTIVE = "ACTIVE"
    RETIRED = "RETIRED"


@dataclass(frozen=True)
class MeasurementNeed:
    id: str
    hypothesis_ids: tuple[str, ...]
    kind: str
    scope: SpanRef
    causal_uncertainty: str
    alternatives_distinguished: str
    result_effect: str
    urgency: int  # copied from the accepted hypothesis: the manager owns it
    status: NeedStatus = NeedStatus.ACTIVE
    purpose: str = "diagnostic"  # diagnostic | confirmation | repeat | reconstruction
    repeat_of_reading_id: str | None = None
    retired_reason: str = ""


@dataclass(frozen=True)
class Unavailable:
    need_id: str
    reason: str


def group_id(kind: str, point: LinePoint) -> str:
    """Electrical observations of one segment aggregate share one canonical group; local sites
    keep their exact spatial identity."""
    if kind in ELECTRICAL:
        return f"electrical:{point.segment_id}:{kind}"
    return f"local:{point.segment_id}@{point.offset_mm}mm:{kind}"


def _hypothesis_needs(
    world: World, geometry: Geometry
) -> tuple[list[MeasurementNeed], list[Unavailable]]:
    needs: list[MeasurementNeed] = []
    unavailable: list[Unavailable] = []
    for hid, h in sorted(world.hypotheses.items()):
        if h["lifecycle"] != "ACTIVE" or h["assessment"] == "REFUTED":
            continue
        for pred in h["predictions"]:
            kind = pred["measurement"]
            for sp in h["scope"]:
                scope = SpanRef(
                    sp["segment_id"], round(sp["start_km"] * 1e6), round(sp["end_km"] * 1e6)
                )
                nid = f"N-{hid}-{pred['id']}-{scope.segment_id}-{scope.start_mm}"
                purpose, repeat = "diagnostic", None
                if kind in LOCAL:
                    a = _condition_assessment(world, scope, kind)
                    if a is None:
                        continue
                    if a.verdict.value == "PASS":
                        unavailable.append(
                            Unavailable(nid, "scope already cleared by qualified negatives")
                        )
                        continue
                    if a.verdict.value == "FAIL" and h["assessment"] == "SUPPORTED":
                        purpose = "confirmation"
                        repeat = next(iter(a.evidence_ids), None)
                    elif a.inconclusive:
                        purpose = "repeat"
                        repeat = next(iter(a.evidence_ids), None)
                needs.append(
                    MeasurementNeed(
                        nid,
                        (hid,),
                        kind,
                        scope,
                        causal_uncertainty=f"{h['claim']} (assessment {h['assessment']})",
                        alternatives_distinguished=(
                            "present in this span versus absent; other report areas remain untested"
                        ),
                        result_effect=(
                            f"{pred['expected']}: would support the claim and raise field-action "
                            "priority; a qualified negative would retire this span"
                        ),
                        urgency=h["priority"],
                        purpose=purpose,
                        repeat_of_reading_id=repeat,
                    )
                )
    return needs, unavailable


def _condition_assessment(world: World, scope: SpanRef, kind: str):
    cond = {
        "clearance": "vegetation_clearance",
        "conductor_temp": "hot_joint",
        "motion": "conductor_motion",
    }.get(kind)
    return None if cond is None else assessment.assess_scope(world, scope, cond)


def _reconstruction_needs(world: World, est_state) -> list[MeasurementNeed]:
    """General observability gaps: only for the explicitly selected benchmark."""
    if world.scenario_mode is not ScenarioMode.HEALTHY_BENCHMARK or est_state.model is None:
        return []
    model = est_state.model
    obs = estimation.observability(model, estimation.effective_readings(world))
    observed = set(obs.observable_segments)
    needs = []
    for seg in obs.selected_segments:
        if seg in observed:
            continue
        needs.append(
            MeasurementNeed(
                f"N-H0-{seg}",
                ("H0",),
                "current",
                SpanRef(seg, 0, world.segment_lengths[seg]),
                causal_uncertainty="benchmark: flow on this segment is not pinned by real readings",
                alternatives_distinguished="none (reconstruction, not diagnosis)",
                result_effect="adds structural rank toward the benchmark's observability target",
                urgency=1,
                purpose="reconstruction",
            )
        )
    return needs[:MAX_RECONSTRUCTION_NEEDS]


def _sites_for(world: World, geometry: Geometry, need: MeasurementNeed) -> list[LinePoint]:
    seg = need.scope.segment_id
    if need.kind in ELECTRICAL:
        n = world.segment_lengths[seg]
        return [LinePoint(seg, n // 2)]
    covered_ok = {
        i
        for a in assessment.segment_assessments(world)
        if a.scope.segment_id == seg
        for i in a.tested_clear
    }
    pts = geometry.sample_points(seg, 1.0)
    out = []
    for p in pts:
        if not (need.scope.start_mm <= p.offset_mm <= need.scope.end_mm):
            continue
        mid_clear = any(c.start_mm <= p.offset_mm <= c.end_mm for c in covered_ok)
        if mid_clear and need.purpose == "diagnostic":
            continue
        out.append(p)
    return out[:MAX_SITES_PER_NEED] if need.purpose == "diagnostic" else out[:2]


def refresh(world: World, geometry: Geometry, est_state) -> tuple[list[str], list[Unavailable]]:
    """Recompute needs and open tasks from accepted hypotheses and current evidence.

    Only PENDING tasks are replaced. Reserved, measuring, completed and failed attempts keep
    their history, and a task shared by several needs survives while any of them is active.
    Returns (change notes, unavailable-work explanations)."""
    notes: list[str] = []
    if world.investigation.status.value != "OPEN":  # a concluded case retires its diagnostic needs
        fresh, unavailable = [], []
    else:
        fresh, unavailable = _hypothesis_needs(world, geometry)
    fresh += _reconstruction_needs(world, est_state)
    fresh_by_id = {n.id: n for n in fresh}
    # retire needs that no longer have an active justification
    for nid, need in list(world.needs.items()):
        if need.status is NeedStatus.ACTIVE and nid not in fresh_by_id:
            world.needs[nid] = replace(
                need, status=NeedStatus.RETIRED, retired_reason=_why_retired(world, need)
            )
            notes.append(f"{nid} retired: {world.needs[nid].retired_reason}")
    for nid, need in fresh_by_id.items():
        old = world.needs.get(nid)
        if old is None or old.status is NeedStatus.RETIRED or old != need:
            world.needs[nid] = need
    # desired attempts: (site, kind, repeat link) -> serving needs
    wanted: dict[tuple[str, str, str | None], tuple[LinePoint, list[str], MeasurementNeed]] = {}
    for need in fresh:
        if (
            need.kind in ELECTRICAL
            and est_state.estimate is not None
            and not hasattr(est_state.estimate, "flows")
        ):
            unavailable.append(
                Unavailable(
                    need.id, "estimate unavailable: diagnostic option only, no information gain"
                )
            )
        for p in _sites_for(world, geometry, need):
            key = (
                p.segment_id + f"@{p.offset_mm}" if need.kind in LOCAL else group_id(need.kind, p),
                need.kind,
                need.repeat_of_reading_id,
            )
            wanted.setdefault(key, (p, [], need))[1].append(need.id)
    existing = {
        (_task_key(t)): t
        for t in world.tasks.values()
        if t.status in (TaskStatus.PENDING, TaskStatus.RESERVED, TaskStatus.MEASURING)
    }
    for key, (point, need_ids, need) in wanted.items():
        have = existing.get(key)
        if have is not None:
            if set(have.need_ids) != set(need_ids):
                world.set_task(replace(have, need_ids=tuple(sorted(need_ids))))
            continue
        site = geometry.site(point)
        tid = f"T{world.next_task_number:04d}"
        world.next_task_number += 1
        world.set_task(
            MeasurementTask(
                id=tid,
                site_id=site.id,
                point=point,
                kind=Measurement(need.kind),
                purpose=f"{need.purpose}: {need.causal_uncertainty}",
                status=TaskStatus.PENDING,
                repeat_of_reading_id=need.repeat_of_reading_id,
                urgency=need.urgency,
                need_ids=tuple(sorted(need_ids)),
                group_id=group_id(need.kind, point),
            )
        )
        notes.append(f"{tid} created for {', '.join(need_ids)}")
    closed = world.investigation.status.value != "OPEN"
    # unstarted attempts that no active need justifies any more are cancelled
    active = {n for n, v in world.needs.items() if v.status is NeedStatus.ACTIVE}
    for t in list(world.tasks.values()):
        if t.status not in (TaskStatus.PENDING, TaskStatus.RESERVED):
            continue
        if closed:
            reason = "the investigation was concluded; collection stopped"
        elif not t.need_ids:
            continue
        elif not (set(t.need_ids) & active):
            reason = "its need was retired and no other need justifies it"
        elif _task_key(t) not in wanted:
            reason = "its site is already answered by qualified evidence"
        else:
            continue
        world.set_task(replace(t, status=TaskStatus.CANCELLED, assigned_drone_id=None, note=reason))
        notes.append(f"{t.id} cancelled: {reason}")
    world.unavailable_work = unavailable
    return notes, unavailable


def _task_key(t: MeasurementTask) -> tuple[str, str, str | None]:
    base = t.group_id if t.kind in ELECTRICAL else f"{t.point.segment_id}@{t.point.offset_mm}"
    return (base, str(t.kind), t.repeat_of_reading_id)


def _why_retired(world: World, need: MeasurementNeed) -> str:
    for hid in need.hypothesis_ids:
        h = world.hypotheses.get(hid)
        if h is None:
            return "its hypothesis is not accepted"
        if h["lifecycle"] != "ACTIVE":
            return f"{hid} was retired"
        if h["assessment"] == "REFUTED":
            return f"{hid} was refuted"
    if world.investigation.status.value != "OPEN":
        return "the investigation was concluded"
    return "the question is answered by qualified evidence or the scope is covered"


def trim_routes(world: World) -> list[str]:
    """Remove cancelled stops from drone routes (locked measurements always finish)."""
    from grid_ops.world import Flying, Measuring, Route, leg_seconds

    notes = []
    for d in world.drones.values():
        st = d.state
        if not isinstance(st, Flying | Measuring):
            continue
        keep = tuple(
            s for s in st.route.stops if world.tasks[s.task_id].status is not TaskStatus.CANCELLED
        )
        if len(keep) == len(st.route.stops):
            continue
        route = Route(keep, st.route.perch, st.route.perch_position)
        if isinstance(st, Measuring):
            d.state = Measuring(st.stop, st.started_s, st.until_s, route)
        else:
            d.state = Flying(
                d.position,
                world.clock_s,
                world.clock_s + leg_seconds(d.position, route.target, d.speed_km_per_min),
                route,
            )
        notes.append(
            f"{d.id}: removed {len(st.route.stops) - len(keep)} retired stop(s) from its route"
        )
    return notes


def need_by_task(world: World, task: MeasurementTask) -> list[MeasurementNeed]:
    return [world.needs[n] for n in task.need_ids if n in world.needs]
