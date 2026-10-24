"""One fixed joint scorer for plans: unique diagnostic benefit under manager-owned urgency.

Rewards go only to active diagnostic needs (or explicit benchmark targets) and are checked before
any scoring. Shared evidence forms one benefit group owned by the highest urgency among the
needs it serves, so overlapping tasks, differently worded hypotheses or finer cell subdivision can
never inflate a plan's value. Local coverage is length-conserving on a fixed 10 m grid;
confirmation is awarded once per justified group; electrical benefit is the joint reduction in
conditional variance of the targeted flows (full correlations, conditioned in urgency order),
normalised by a constant fixed for the decision. Scores are integer synthetic units compared
lexicographically: urgency tiers 5 to 1 first, then risk, switching and airtime. They are not a
calibrated causal entropy.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from grid_ops import assessment, estimation
from grid_ops.estimation import ElectricalModel, EstimateUnavailable
from grid_ops.needs import ELECTRICAL, LOCAL, NeedStatus
from grid_ops.world import MeasurementTask, Reading, TaskStatus, World

CELL_M = 10  # fixed local coverage grid
POINTS_PER_M = 1.0  # length-conserving: a metre of undecided scope is worth one point
CONFIRMATION_POINTS = 500
ELECTRICAL_NORMALISED_POINTS = 1000  # full variance reduction of all targets = this many points
QUANTUM = 1  # integer tolerance: scores within one unit are equal
TIERS = (5, 4, 3, 2, 1)
OPEN = (TaskStatus.PENDING, TaskStatus.RESERVED, TaskStatus.MEASURING)

SCALE_NOTE = (
    "synthetic units: coverage 1 point per metre of undecided scope, confirmation 500 per group, "
    "electrical 1000 for removing all targeted variance; not a calibrated causal entropy"
)


@dataclass(frozen=True)
class PlanScore:
    tiers: tuple[int, ...]  # synthetic points per urgency tier, 5 down to 1
    risk: int
    switching: int
    airtime_s: int
    electrical_available: bool
    quantum: int = QUANTUM
    notes: tuple[str, ...] = ()

    @property
    def key(self) -> tuple[int, ...]:
        """Lexicographic comparison key: all benefit tiers, then lower risk, switching, airtime."""
        return (*self.tiers, -self.risk, -self.switching, -self.airtime_s)

    def better_than(self, other: PlanScore) -> bool:
        return self.key > other.key

    def equal_within_quantum(self, other: PlanScore) -> bool:
        return all(abs(a - b) <= self.quantum for a, b in zip(self.key, other.key, strict=True))


@dataclass
class ScoringContext:
    """Everything the scorer needs, frozen at the start of a decision so candidates and
    incumbents are compared under identical assumptions across surrogate rounds."""

    owners: dict[tuple, int]  # benefit group -> owning urgency
    values: dict[tuple, float]  # benefit group -> points (non-electrical)
    electrical_groups: dict[str, list[tuple[str, int]]]  # group id -> [(target segment, urgency)]
    base_readings: tuple[Reading, ...]
    model: ElectricalModel | None
    sensor_model: object
    shared_rho: float
    prior_variance: dict[str, float] = field(default_factory=dict)
    norm: float = 0.0
    electrical_available: bool = True
    cell_m: int = CELL_M


def eligible(world: World, task: MeasurementTask) -> bool:
    """Eligibility is checked before scoring: an attempt earns nothing unless an active need
    justifies it (or it is locked, in-progress work)."""
    if task.status not in OPEN:
        return False
    return any(
        n in world.needs and world.needs[n].status is NeedStatus.ACTIVE for n in task.need_ids
    )


def _cells(start_mm: int, end_mm: int, cell_m: int):
    step = cell_m * 1000
    first, last = start_mm // step, max(end_mm - 1, start_mm) // step
    for i in range(first, last + 1):
        lo, hi = max(i * step, start_mm), min((i + 1) * step, end_mm)
        if hi > lo:
            yield i, hi - lo


def build_context(
    world: World,
    est_state,
    cell_m: int = CELL_M,
) -> ScoringContext:
    """Ownership: every benefit group belongs to the highest urgency among accepted needs that
    can earn it. Urgency comes from the manager's accepted hypotheses, never from the planner."""
    owners: dict[tuple, int] = {}
    values: dict[tuple, float] = {}
    electrical: dict[str, list[tuple[str, int]]] = {}
    for need in world.needs.values():
        if need.status is not NeedStatus.ACTIVE:
            continue
        urgency = _accepted_urgency(world, need)
        if need.kind in LOCAL:
            decided = _decided_cells(world, need, cell_m)
            for cell, length in _cells(need.scope.start_mm, need.scope.end_mm, cell_m):
                key = ("coverage", need.kind, need.scope.segment_id, cell)
                if key in decided:
                    continue
                owners[key] = max(owners.get(key, 0), urgency)
                values[key] = max(values.get(key, 0.0), length / 1000 * POINTS_PER_M)
            if need.purpose in ("confirmation", "repeat") and need.repeat_of_reading_id:
                key = ("confirm", need.repeat_of_reading_id, need.kind)
                owners[key] = max(owners.get(key, 0), urgency)
                values[key] = CONFIRMATION_POINTS
        elif need.kind in ELECTRICAL:
            gid = f"electrical:{need.scope.segment_id}:{need.kind}"
            electrical.setdefault(gid, []).append((need.scope.segment_id, urgency))
            owners[("electrical", gid)] = max(owners.get(("electrical", gid), 0), urgency)
    model = est_state.model
    readings = estimation.effective_readings(world)
    cov = None
    available = model is not None
    if model is not None:
        cov = estimation.posterior_flow_covariance(
            model, readings, shared_rho=world.sensor_model.shared_rho
        )
        available = not isinstance(cov, EstimateUnavailable)
    prior, norm = {}, 0.0
    if available and cov is not None:
        idx = {s: i for i, s in enumerate(model.segments)}
        for targets in electrical.values():
            for seg, _ in targets:
                prior[seg] = float(cov[idx[seg], idx[seg]])
        norm = sum(v for v in prior.values()) or 1.0
    return ScoringContext(
        owners,
        values,
        electrical,
        tuple(readings),
        model,
        world.sensor_model,
        world.sensor_model.shared_rho,
        prior,
        norm,
        available,
        cell_m,
    )


def _accepted_urgency(world: World, need) -> int:
    accepted = [
        world.hypotheses[h]["priority"] for h in need.hypothesis_ids if h in world.hypotheses
    ]
    return max(accepted) if accepted else need.urgency


def _decided_cells(world: World, need, cell_m: int) -> set[tuple]:
    """Cells already settled by qualified evidence: they carry no further benefit."""
    cond = {
        "clearance": "vegetation_clearance",
        "conductor_temp": "hot_joint",
        "motion": "conductor_motion",
    }[need.kind]
    a = assessment.assess_scope(world, need.scope, cond)
    out: set[tuple] = set()
    for iv in (*a.tested_clear, *a.failed):
        for cell, length in _cells(iv.start_mm, iv.end_mm, cell_m):
            if length == cell_m * 1000:  # only whole settled cells
                out.add(("coverage", need.kind, need.scope.segment_id, cell))
    return out


def _task_groups(task: MeasurementTask, ctx: ScoringContext, geometry) -> set[tuple]:
    kind = str(task.kind)
    if kind in ELECTRICAL:
        return (
            {("electrical", task.group_id)}
            if ("electrical", task.group_id) in ctx.owners
            else set()
        )
    fp = geometry.footprint(task.point)
    groups = {
        ("coverage", kind, task.point.segment_id, cell)
        for cell, _ in _cells(fp.start_mm, fp.end_mm, ctx.cell_m)
    }
    groups = {g for g in groups if g in ctx.owners}
    if task.repeat_of_reading_id and ("confirm", task.repeat_of_reading_id, kind) in ctx.owners:
        groups.add(("confirm", task.repeat_of_reading_id, kind))
    return groups


def score_plan(
    world: World,
    ctx: ScoringContext,
    geometry,
    task_ids: list[str],
    *,
    risk: int = 0,
    switching: int = 0,
    airtime_s: int = 0,
) -> PlanScore:
    """Score a set of attempts. Order, duplicates and overlap never change the value."""
    tasks = [world.tasks[t] for t in dict.fromkeys(task_ids) if t in world.tasks]
    notes: list[str] = []
    tiers = {u: 0.0 for u in TIERS}
    earned: set[tuple] = set()
    ineligible = [t.id for t in tasks if not eligible(world, t)]
    if ineligible:
        notes.append(f"ineligible, earn nothing: {', '.join(ineligible)}")
    live = [t for t in tasks if eligible(world, t)]
    for t in live:
        earned |= _task_groups(t, ctx, geometry)
    for g in earned:
        if g[0] == "electrical":
            continue
        tiers[ctx.owners[g]] += ctx.values[g]
    # electrical: joint conditional-variance reduction, conditioned in urgency order
    electrical_ok = ctx.electrical_available
    elec_groups = {g[1] for g in earned if g[0] == "electrical"}
    if elec_groups and ctx.model is not None and electrical_ok:
        new = []
        for i, gid in enumerate(
            sorted(elec_groups, key=lambda x: (-ctx.owners[("electrical", x)], x))
        ):
            seg = gid.split(":")[1]
            new.append(_hypothetical(world, ctx, seg, f"HYP{i:03d}"))
        idx = {s: i for i, s in enumerate(ctx.model.segments)}
        before = ctx.prior_variance
        applied: list = []
        last = {s: before[s] for s in before}
        for gid, hyp in zip(
            sorted(elec_groups, key=lambda x: (-ctx.owners[("electrical", x)], x)), new, strict=True
        ):
            applied.append(hyp)
            cov = estimation.posterior_flow_covariance(
                ctx.model, [*ctx.base_readings, *applied], shared_rho=ctx.shared_rho
            )
            if isinstance(cov, EstimateUnavailable):
                electrical_ok = False
                break
            now = {s: float(cov[idx[s], idx[s]]) for s in before}
            u = ctx.owners[("electrical", gid)]
            gain = sum(last[s] - now[s] for s, _ in ctx.electrical_groups[gid] if s in now)
            tiers[u] += max(gain, 0.0) / ctx.norm * ELECTRICAL_NORMALISED_POINTS
            last = now
    if elec_groups and not electrical_ok:
        notes.append("electrical utility unavailable (no usable covariance); no gain invented")
    return PlanScore(
        tuple(round(tiers[u]) for u in TIERS),
        risk,
        switching,
        airtime_s,
        electrical_ok or not elec_groups,
        QUANTUM,
        tuple(notes),
    )


def _hypothetical(world: World, ctx: ScoringContext, segment: str, rid: str) -> Reading:
    """A reading that does not exist yet, carrying only the public calibration covariance."""
    from grid_ops.geometry import LinePoint
    from grid_ops.world import Measurement

    circuits = world.circuits_by_segment[segment]
    cov = ctx.sensor_model.covariance("current", circuits)
    payload = tuple((n, 0.0) for n in cov.names)
    return Reading(
        rid,
        "-",
        "-",
        "-",
        LinePoint(segment, 0),
        Measurement.CURRENT,
        world.clock_s,
        "E0",
        payload,
        ctx.sensor_model.revision,
        "hypothetical",
        cov,
    )


def explain(world: World, ctx: ScoringContext, score: PlanScore) -> list[str]:
    out = [
        f"tiers 5..1 = {score.tiers}; risk {score.risk}, switching {score.switching}, "
        f"airtime {score.airtime_s} s",
        f"quantisation tolerance: {score.quantum} unit",
        SCALE_NOTE,
    ]
    out += list(score.notes)
    if not ctx.electrical_available:
        out.append("electrical utility unavailable for this decision")
    return out
