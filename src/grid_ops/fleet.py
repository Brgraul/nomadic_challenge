"""Immediate fleet-wide route pass after relevant events.

The pass never waits for a language model. It plans over the whole available fleet from live
positions, then accepts a continuation only if the complete set validates atomically. When the
solver fails, times out, returns an invalid plan or finds no improvement, the revalidated incumbent
stands; if that is no longer valid every airborne drone is sent to an eligible landing; if even
that is impossible the cycle stays held with a specific, operator-visible cause. A route-only
decision never changes a hypothesis.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from grid_ops import decisions, planning
from grid_ops.routes import apply_sorties, validate_sorties
from grid_ops.world import World

RELEVANT_EVENTS = frozenset(
    {"reading", "switching_report", "evidence_expired", "charged", "charge_failed", "landed"}
)


@dataclass(frozen=True)
class RoutePass:
    """One immediate route pass, with the timestamps needed to measure end-to-end latency."""

    id: str
    trigger: str  # event kinds / command that caused it
    event_ids: tuple[str, ...]
    sim_clock_s: int
    ingested_at: float  # wall clock (perf_counter) when the observation batch was ingested
    published_at: float  # wall clock when the evidence snapshot was published
    route_published_at: float  # wall clock when the accepted continuation was published
    outcome: str  # accepted-plan | kept-incumbent | landing-only | HELD | no-op
    detail: str
    solver: str = ""  # stop reason / failure kind
    stage_statuses: tuple[str, ...] = ()
    planner_s: float = 0.0

    @property
    def latency_s(self) -> float:
        return self.route_published_at - self.ingested_at


@dataclass
class Coordinator:
    """What the sim notifies. Listeners see the published evidence before any route pass."""

    listeners: list = field(default_factory=list)
    planner: object = planning.plan_sorties  # replaceable: fault injection and replay
    passes: int = 0

    def notify(self, revision: int, event_ids: tuple[str, ...]) -> None:
        for cb in self.listeners:
            cb(revision, event_ids)


def route_pass(
    world: World,
    geometry,
    circuits,
    physics,
    coordinator: Coordinator,
    *,
    trigger: str,
    event_ids: tuple[str, ...],
    ingested_at: float,
    published_at: float,
    **plan_kw,
) -> RoutePass:
    """Plan, validate and atomically accept (or fall back). Mutates `world` only on success."""
    coordinator.passes += 1
    pid = f"RP{coordinator.passes:04d}"
    t0 = time.perf_counter()
    result = None
    failure = ""
    try:
        result = coordinator.planner(
            world, geometry, circuits, revision=world.revision, **plan_kw
        )
    except Exception as e:  # solver crash, timeout surfaced as an exception, invalid model
        failure = f"solver failure: {type(e).__name__}: {e}"
    planner_s = time.perf_counter() - t0
    statuses = tuple(s.status for s in result.stages) if result else ()
    solver_note = failure or (result.stop_reason if result else "no result")

    def done(outcome: str, detail: str) -> RoutePass:
        return RoutePass(
            pid,
            trigger,
            event_ids,
            world.clock_s,
            ingested_at,
            published_at,
            time.perf_counter(),
            outcome,
            detail,
            solver_note,
            statuses,
            planner_s,
        )

    incumbent_ok = True
    airborne = decisions.current_requests(world)
    if airborne:
        incumbent_ok = validate_sorties(world, geometry, circuits, airborne).ok
    if result is not None and result.ranked:
        for cand in result.ranked:  # best first; each one is validated independently again
            if cand.source == "incumbent":
                if incumbent_ok:
                    return done("kept-incumbent", "no candidate beat the validated current routes")
                continue
            if not cand.valid:
                continue
            check = validate_sorties(world, geometry, circuits, cand.sorties)
            if not check.ok:
                continue
            apply_sorties(world, physics, check.plans)
            return done("accepted-plan", f"{cand.source}: {len(cand.sorties)} sortie(s)")
    if incumbent_ok:
        return done("kept-incumbent" if airborne else "no-op", failure or "nothing to change")
    outcome, problems = decisions.route_only_continuation(world, geometry, circuits, physics)
    if outcome == "HELD":
        return done("HELD", "; ".join(problems))
    return done("landing-only", outcome)
