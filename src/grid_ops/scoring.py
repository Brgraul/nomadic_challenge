"""One fixed joint scorer for itineraries: unique diagnostic benefit under manager-owned urgency.

Rewards go only to accepted active requests (or their compiled tasks) and are earned once per
alternative group. LiDAR benefit is the length of not-yet-inspected 10 m cells inside the
requested span, so overlapping scans and interrupted repeats cannot inflate value; contact and
TDR requests earn one flat group award however many candidate sites serve them. Urgency comes
from the request, which the manager owns. Scores are integer synthetic units compared
lexicographically: urgency tiers 5 to 1 first, then risk, switching, airtime and charge waiting.
They are not a calibrated causal entropy; calibrated reconstruction is P3.
"""

from __future__ import annotations

from dataclasses import dataclass

from grid_ops.geometry import Interval
from grid_ops.sensing import LidarPass, RequestStatus
from grid_ops.world import TaskStatusV2, World

CELL_MM = 10_000  # fixed 10 m local coverage grid
POINTS_PER_M = 1.0  # length-conserving: a metre of undecided scope is worth one point
REQUEST_POINTS = 500  # one award per contact/TDR alternative group
QUANTUM = 1  # integer tolerance: scores within one unit are equal
TIERS = (5, 4, 3, 2, 1)
OPEN = (TaskStatusV2.PENDING, TaskStatusV2.RESERVED, TaskStatusV2.MEASURING)

SCALE_NOTE = (
    "synthetic units: lidar coverage 1 point per metre of undecided scope, contact/tdr 500 per "
    "request group; not a calibrated causal entropy"
)


@dataclass(frozen=True)
class PlanScore:
    tiers: tuple[int, ...]  # synthetic points per urgency tier, 5 down to 1
    risk: int
    switching: int
    airtime_s: int
    charge_s: int
    quantum: int = QUANTUM
    notes: tuple[str, ...] = ()

    @property
    def key(self) -> tuple[int, ...]:
        """Lexicographic key: benefit tiers, then lower risk, switching, airtime, charge."""
        return (*self.tiers, -self.risk, -self.switching, -self.airtime_s, -self.charge_s)

    def better_than(self, other: PlanScore) -> bool:
        return self.key > other.key

    def equal_within_quantum(self, other: PlanScore) -> bool:
        return all(abs(a - b) <= self.quantum for a, b in zip(self.key, other.key, strict=True))


def _cells(start_mm: int, end_mm: int):
    step = CELL_MM
    first, last = start_mm // step, max(end_mm - 1, start_mm) // step
    for i in range(first, last + 1):
        lo, hi = max(i * step, start_mm), min((i + 1) * step, end_mm)
        if hi > lo:
            yield i, hi - lo


def _inspected_by_segment(world: World) -> dict[str, list[Interval]]:
    out: dict[str, list[Interval]] = {}
    for reading in world.readings.values():
        payload = reading.payload
        if hasattr(payload, "inspected"):
            out.setdefault(payload.inspected[0].segment_id if payload.inspected else "", [])
            for iv in payload.inspected:
                out[iv.segment_id].append(iv)
    return out


def _uncovered_length_mm(world: World, span: Interval, covered: list[Interval]) -> int:
    """Length of `span` not covered by any inspected interval (merged per segment)."""
    if not covered:
        return span.end_mm - span.start_mm
    ordered = sorted(covered, key=lambda i: i.start_mm)
    uncovered = 0
    cursor = span.start_mm
    for iv in ordered:
        if iv.end_mm <= cursor:
            continue
        if iv.start_mm > cursor:
            uncovered += min(iv.start_mm, span.end_mm) - cursor
        cursor = max(cursor, iv.end_mm)
        if cursor >= span.end_mm:
            return uncovered
    if cursor < span.end_mm:
        uncovered += span.end_mm - cursor
    return uncovered


@dataclass
class ScoringContext:
    """Everything the scorer needs, frozen at the start of a decision so candidates and
    incumbents are compared under identical assumptions."""

    request_urgency: dict[str, int]  # request id -> urgency
    request_by_task: dict[str, str]  # task id -> request id
    groups: dict[str, tuple[str, int, tuple[str, ...]]]  # group id -> (kind, tier, task ids)
    cell_values: dict[tuple, float]  # ("cell", segment, cell) -> points
    cell_owners: dict[tuple, int]
    covered: dict[str, list[Interval]]
    open_tasks: tuple[str, ...]


def build_context(world: World) -> ScoringContext:
    covered = _inspected_by_segment(world)
    request_urgency = {
        a.id: a.request.urgency for a in world.requests.values() if a.status is RequestStatus.ACTIVE
    }
    request_by_task = {t.id: t.request_id for t in world.tasks.values()}
    cell_values: dict[tuple, float] = {}
    cell_owners: dict[tuple, int] = {}
    groups: dict[str, tuple[str, int, tuple[str, ...]]] = {}
    by_group: dict[str, list[str]] = {}
    for task in world.tasks.values():
        if task.status not in OPEN:
            continue
        by_group.setdefault(task.alternative_group_id, []).append(task.id)
        if not isinstance(task.acquisition, LidarPass):
            continue
        urgency = request_urgency.get(task.request_id, task.urgency)
        target = task.acquisition.target
        span = Interval(target.segment_id, target.start_mm, target.end_mm)
        for cell, length in _cells(span.start_mm, span.end_mm):
            key = ("cell", target.segment_id, cell)
            cell_owners[key] = max(cell_owners.get(key, 0), urgency)
            cell_values[key] = max(cell_values.get(key, 0.0), length / 1000 * POINTS_PER_M)
    for group_id, task_ids in by_group.items():
        sample = next(t for t in world.tasks.values() if t.id == task_ids[0])
        groups[group_id] = (
            "lidar" if isinstance(sample.acquisition, LidarPass) else "ground",
            request_urgency.get(sample.request_id, sample.urgency),
            tuple(task_ids),
        )
    open_tasks = tuple(t.id for t in world.tasks.values() if t.status in OPEN)
    return ScoringContext(
        request_urgency, request_by_task, groups, cell_values, cell_owners, covered, open_tasks
    )


def score_plan(
    world: World,
    ctx: ScoringContext,
    task_ids: list[str],
    *,
    risk: int = 0,
    switching: int = 0,
    airtime_s: int = 0,
    charge_s: int = 0,
) -> PlanScore:
    """Score a set of attempts. Order, duplicates and overlap never change the value."""
    tasks = [world.tasks[t] for t in dict.fromkeys(task_ids) if t in world.tasks]
    notes: list[str] = []
    tiers = {u: 0.0 for u in TIERS}
    earned_groups = {ctx.request_by_task.get(t.id) for t in tasks}
    earned_groups.discard(None)
    for group_id in earned_groups:
        kind, tier, _ = ctx.groups.get(group_id, ("ground", 0, ()))
        if kind == "lidar":
            continue
        tiers[tier] += REQUEST_POINTS
    for task in tasks:
        if not isinstance(task.acquisition, LidarPass):
            continue
        target = task.acquisition.target
        span = Interval(target.segment_id, target.start_mm, target.end_mm)
        for cell, length in _cells(span.start_mm, span.end_mm):
            key = ("cell", target.segment_id, cell)
            if key not in ctx.cell_values:
                continue
            cell_span = Interval(
                target.segment_id, cell * CELL_MM, min((cell + 1) * CELL_MM, span.end_mm)
            )
            span_len = _uncovered_length_mm(
                world, cell_span, ctx.covered.get(target.segment_id, [])
            )
            if span_len <= 0:
                continue
            tiers[ctx.cell_owners[key]] += min(length, span_len) / 1000 * POINTS_PER_M
    return PlanScore(
        tuple(round(t) for t in (tiers[u] for u in TIERS)),
        risk,
        switching,
        airtime_s,
        charge_s,
        QUANTUM,
        tuple(notes),
    )


def unresolved_requests(world: World) -> list[tuple[str, str]]:
    """Accepted active requests with no completed valid attempt: explicitly unresolved."""
    answered = _answered_groups_impl(world)
    out = []
    for a in world.requests.values():
        if a.status is RequestStatus.ACTIVE and a.id not in answered:
            out.append((a.id, "no completed valid attempt yet"))
    return out


def _answered_groups_impl(world: World) -> set[str]:
    from grid_ops.needs import _answered_groups

    return _answered_groups(world)


def explain(world: World, ctx: ScoringContext, score: PlanScore) -> list[str]:
    out = [
        f"tiers 5..1 = {score.tiers}; risk {score.risk}, switching {score.switching}, "
        f"airtime {score.airtime_s} s, charge {score.charge_s} s",
        f"quantisation tolerance: {score.quantum} unit",
        SCALE_NOTE,
    ]
    out += list(score.notes)
    return out
