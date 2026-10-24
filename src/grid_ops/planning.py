"""Bounded multi-drone sortie optimisation with OR-Tools CP-SAT.

`plan_sorties` is pure: it reads the public world and returns ranked candidate plans; the sim
alone commits, and only after independent validation (`routes.validate_sorties`). The model's
constraints are the same authoritative rules: conservative rounded seconds, remaining battery,
dwell, landing eligibility through predicted arrival, exact-site occupancy and T+04:00.

Objective: staged solves in urgency order (tiers 5 to 1) over unique benefit groups, then risk,
switching and airtime, each stage fixing the value reached by the previous one. Each stage's
status is reported honestly (OPTIMAL, FEASIBLE with its bound, UNKNOWN, ...). Up to three
reranking rounds produce distinct candidates that the fixed joint scorer ranks against the
current routes (the incumbent), all inside one shared deterministic search budget.
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field

import ortools
from ortools.sat.python import cp_model

from grid_ops import decisions, knowledge, scoring
from grid_ops.geometry import Geometry, LandingSite, haversine_km
from grid_ops.routes import SortieRequest, StopRequest, validate_sorties
from grid_ops.world import (
    DEADLINE_S,
    DWELL_S,
    Flying,
    Measuring,
    Ready,
    TaskStatus,
    World,
    leg_seconds,
)

ATTEMPT_CAP = 20
MAX_ROUNDS = 3
RESPONSE_BUDGET_S = 2.0  # observation ingestion to validated plan publication
NON_SOLVER_RESERVE_S = 0.5  # held back for diagnostics, refresh, construction, scoring, acceptance
DETERMINISTIC_BUDGET = 0.1  # total deterministic-time units per decision
WALL_CAP_S = 1.5
SEED = 0
LANDINGS_PER_ANCHOR = 2
MAX_LANDINGS = 16  # pool cap for the bounded model; None means exact (tiny instances)
NEXT_TASKS = 6  # arcs kept per task toward its nearest successors
LANDINGS_PER_TASK = 4
INF_S = 10**9
SOLVER_VERSION = f"ortools {ortools.__version__}"
STATUS = {
    cp_model.OPTIMAL: "OPTIMAL",
    cp_model.FEASIBLE: "FEASIBLE",
    cp_model.INFEASIBLE: "INFEASIBLE",
    cp_model.MODEL_INVALID: "MODEL_INVALID",
    cp_model.UNKNOWN: "UNKNOWN",
}


@dataclass(frozen=True)
class PlanVehicle:
    drone_id: str
    kind: str  # ready | flying | measuring
    position: tuple[float, float]
    start_s: int
    battery_s: int  # airtime left at start_s (after any locked dwell)
    speed: float


@dataclass(frozen=True)
class PlanTask:
    id: str
    position: tuple[float, float]
    dwell_s: int


@dataclass(frozen=True)
class PlanLanding:
    site: LandingSite
    position: tuple[float, float]
    until_s: int  # the endpoint's evidence is valid strictly before this second
    risk: int  # 1 if accepted only by the explicit uncertain-endpoint policy


@dataclass(frozen=True)
class PlanningBlocked:
    reason: str


@dataclass(frozen=True)
class Omission:
    task_id: str
    reason: str


@dataclass(frozen=True)
class StageReport:
    round: int
    name: str
    status: str
    objective: int | None
    bound: int | None
    wall_s: float


@dataclass(frozen=True)
class Candidate:
    sorties: tuple[SortieRequest, ...]
    task_ids: tuple[str, ...]  # every attempt the plan serves, locked work included
    score: scoring.PlanScore
    source: str  # incumbent | round-N
    valid: bool = True
    problems: tuple[str, ...] = ()


@dataclass(frozen=True)
class PlanningResult:
    revision: int
    clock_s: int
    ranked: tuple[Candidate, ...]  # best first; only independently validated plans
    stages: tuple[StageReport, ...]
    unassigned: tuple[Omission, ...]
    omitted: tuple[Omission, ...]
    blocked: PlanningBlocked | None
    stop_reason: str
    replay: dict = field(default_factory=dict)

    @property
    def best(self) -> Candidate | None:
        return self.ranked[0] if self.ranked else None


# -- selection under the attempt cap ------------------------------------------


def _tier(score: scoring.PlanScore) -> int:
    return next((u for u, v in zip(scoring.TIERS, score.tiers, strict=True) if v), 0)


def select_tasks(world: World, ctx, geometry: Geometry, cap: int = ATTEMPT_CAP):
    """At most `cap` attempts. Locked work is never a decision variable; every highest-urgency
    need keeps a representative; the rest rank by standalone benefit. Every omission is reported,
    and required work that cannot fit is a PlanningBlocked."""
    open_tasks = [
        t
        for t in world.tasks.values()
        if t.status in (TaskStatus.PENDING, TaskStatus.RESERVED) and scoring.eligible(world, t)
    ]
    solo = {t.id: scoring.score_plan(world, ctx, geometry, [t.id]) for t in open_tasks}
    top = max((_tier(s) for s in solo.values()), default=0)
    reps: dict[str, str] = {}
    for t in sorted(open_tasks, key=lambda t: t.id):
        if top and _tier(solo[t.id]) == top:
            for n in t.need_ids:
                cur = reps.get(n)
                if cur is None or sum(solo[t.id].tiers) > sum(solo[cur].tiers):
                    reps[n] = t.id
    required = sorted(set(reps.values()))
    locked = [t for t in world.tasks.values() if t.status is TaskStatus.MEASURING]
    if len(required) + len(locked) > cap:
        return (
            [],
            [],
            PlanningBlocked(
                f"required work alone ({len(required)} highest-urgency needs and {len(locked)} "
                f"locked attempts) exceeds the {cap}-attempt capacity"
            ),
            solo,
        )
    rest = sorted(
        (t for t in open_tasks if t.id not in required),
        key=lambda t: (-_tier(solo[t.id]), -sum(solo[t.id].tiers), t.id),
    )
    room = cap - len(required)
    chosen = [world.tasks[i] for i in required] + rest[:room]
    omitted = [
        Omission(t.id, f"over the {cap}-attempt cap (tier {_tier(solo[t.id])})")
        for t in rest[room:]
    ]
    return chosen, omitted, None, solo


# -- model inputs ------------------------------------------


def _vehicles(world: World) -> list[PlanVehicle]:
    out = []
    for d in sorted(world.drones.values(), key=lambda d: d.id):
        st = d.state
        if isinstance(st, Ready):
            out.append(
                PlanVehicle(
                    d.id, "ready", d.position, world.clock_s, d.battery_s, d.speed_km_per_min
                )
            )
        elif isinstance(st, Flying):
            out.append(
                PlanVehicle(
                    d.id, "flying", d.position, world.clock_s, d.battery_s, d.speed_km_per_min
                )
            )
        elif isinstance(st, Measuring):
            locked = st.until_s - world.clock_s
            out.append(
                PlanVehicle(
                    d.id,
                    "measuring",
                    st.stop.position,
                    st.until_s,
                    d.battery_s - locked,
                    d.speed_km_per_min,
                )
            )
    return out


def _landing_pool(
    world: World, geometry: Geometry, vehicles, tasks, limit: int | None = MAX_LANDINGS
) -> list[PlanLanding]:
    """Eligible, unoccupied endpoints near any anchor (vehicle start or candidate stop), with exact
    projections so sparse sampling can never hide a feasible landing."""
    if not vehicles:
        return []
    anchors = [v.position for v in vehicles] + [t.position for t in tasks]
    cands = knowledge.landing_candidates(
        world,
        geometry,
        vehicles[0].drone_id,
        lambda site: world.clock_s,
        final_stop_positions=tuple(anchors),
    )
    ok = [c for c in cands if c.eligibility.eligible and not c.occupant]
    taken = {d.state.perch for d in world.drones.values() if isinstance(d.state, Ready)}
    found: dict[LandingSite, PlanLanding] = {}
    for a in anchors:
        near = sorted(
            (c for c in ok if c.site not in taken),
            key=lambda c: haversine_km(a, geometry.position_of(c.site.point)),
        )[:LANDINGS_PER_ANCHOR]
        for c in near:
            ch = world.knowledge.chargeability[c.site.circuit_id]
            uncertain = int(ch.state.value != "confirmed")
            until = INF_S if (uncertain or ch.expires_s is None) else ch.expires_s
            found.setdefault(
                c.site, PlanLanding(c.site, geometry.position_of(c.site.point), until, uncertain)
            )
    pool = sorted(found.values(), key=lambda p: (p.site.circuit_id, p.site.point.offset_mm))
    if limit is not None and len(pool) > limit:
        pool.sort(key=lambda p: min(haversine_km(a, p.position) for a in anchors))
        pool = sorted(pool[:limit], key=lambda p: (p.site.circuit_id, p.site.point.offset_mm))
    return pool


def incumbent_candidate(world, geometry, circuits, ctx, locked_ids) -> Candidate:
    """Current routes, trimmed to what is still justified, revalidated and rescored."""
    reqs = decisions.current_requests(world)
    ok = validate_sorties(world, geometry, circuits, reqs) if reqs else None
    valid = ok is None or ok.ok
    ids = [
        s.task_id
        for v in world.drones.values()
        if isinstance(v.state, Flying | Measuring)
        for s in v.state.route.stops
    ]
    ids += locked_ids
    air = 0
    if ok is not None and ok.ok:
        air = sum(p.landing_arrive_s - p.start_s for p in ok.plans)
    score = scoring.score_plan(world, ctx, geometry, ids, airtime_s=air)
    problems = () if valid else tuple(str(r) for r in ok.rejections)
    return Candidate(tuple(reqs), tuple(dict.fromkeys(ids)), score, "incumbent", valid, problems)


# -- the CP-SAT model ------------------------------------------


class _Model:
    def __init__(
        self, vehicles, tasks, landings, groups, incumbent_owner, exact=False, start_paths=None
    ):
        self.vehicles, self.tasks, self.landings = vehicles, tasks, landings
        self.m = cp_model.CpModel()
        m = self.m
        nv, nt, nl = len(vehicles), len(tasks), len(landings)
        self.nv, self.nt, self.nl = nv, nt, nl
        self.visit = {(v, k): m.NewBoolVar(f"visit_{v}_{k}") for v in range(nv) for k in range(nt)}
        self.arc: dict[tuple, cp_model.IntVar] = {}
        self.out: dict[tuple, list] = {}
        self.into: dict[tuple, list] = {}
        self.time = {}
        self.used, self.wait, self.air, self.lu = [], [], [], {}
        self.pos = []
        force: set[tuple] = set()  # arcs of the starting solution always exist, pruned or not
        for vi, (stops, landing) in (start_paths or {}).items():
            chain = ["s", *[("t", k) for k in stops], ("l", landing)]
            force |= {(vi, a, b) for a, b in itertools.pairwise(chain)}
        for vi, veh in enumerate(vehicles):
            pos = {"s": veh.position}
            pos.update({("t", k): tasks[k].position for k in range(nt)})
            pos.update({("l", k): landings[k].position for k in range(nl)})
            self.pos.append(pos)
            for n in pos:
                self.time[vi, n] = m.NewIntVar(0, DEADLINE_S, f"t_{vi}_{n}")
            m.Add(self.time[vi, "s"] == veh.start_s)
            sources = ["s", *[("t", k) for k in range(nt)]]
            targets = [*[("t", k) for k in range(nt)], *[("l", k) for k in range(nl)]]
            for a in sources:
                dwell_a = tasks[a[1]].dwell_s if a != "s" else 0
                keep = _kept_targets(a, targets, pos, veh.speed, exact)
                for b in targets:
                    if a == b or (b not in keep and (vi, a, b) not in force):
                        continue
                    dur = leg_seconds(pos[a], pos[b], veh.speed) + dwell_a
                    if veh.start_s + dur > DEADLINE_S or dur > veh.battery_s:
                        continue
                    x = m.NewBoolVar(f"arc_{vi}_{a}_{b}")
                    self.arc[vi, a, b] = x
                    self.out.setdefault((vi, a), []).append(x)
                    self.into.setdefault((vi, b), []).append(x)
                    m.Add(self.time[vi, b] >= self.time[vi, a] + dur).OnlyEnforceIf(x)
            used = m.NewBoolVar(f"used_{vi}")
            self.used.append(used)
            m.Add(sum(self.out.get((vi, "s"), [])) == used)
            if veh.kind == "ready":
                w = m.NewBoolVar(f"wait_{vi}")
                m.Add(used + w == 1)  # a Ready drone either flies or waits, at no cost
                self.wait.append(w)
                # no fictitious flights: a Ready drone only takes off to measure something
                m.Add(sum(self.visit[vi, k] for k in range(nt)) >= 1).OnlyEnforceIf(used)
            else:
                m.Add(used == 1)  # an airborne drone must have an executable continuation
                self.wait.append(None)
            for k in range(nt):
                ins, outs = self.into.get((vi, ("t", k)), []), self.out.get((vi, ("t", k)), [])
                m.Add(sum(ins) == self.visit[vi, k])
                m.Add(sum(outs) == self.visit[vi, k])
                m.Add(self.time[vi, ("t", k)] + tasks[k].dwell_s <= DEADLINE_S).OnlyEnforceIf(
                    self.visit[vi, k]
                )
            air = m.NewIntVar(0, DEADLINE_S, f"air_{vi}")
            self.air.append(air)
            lus = []
            for k in range(nl):
                ins = self.into.get((vi, ("l", k)), [])
                lu = m.NewBoolVar(f"lu_{vi}_{k}")
                self.lu[vi, k] = lu
                m.Add(sum(ins) == lu)
                lus.append(lu)
                m.Add(self.time[vi, ("l", k)] < landings[k].until_s).OnlyEnforceIf(lu)
                m.Add(self.time[vi, ("l", k)] - veh.start_s <= veh.battery_s).OnlyEnforceIf(lu)
                m.Add(air == self.time[vi, ("l", k)] - veh.start_s).OnlyEnforceIf(lu)
            m.Add(sum(lus) == used)
            m.Add(air == 0).OnlyEnforceIf(used.Not())
        for k in range(nt):
            m.Add(sum(self.visit[vi, k] for vi in range(nv)) <= 1)  # unique ownership
        for k in range(nl):
            lands_k = [self.lu[vi, k] for vi in range(nv)]
            m.AddAtMostOne(lands_k)  # at most one drone per exact landing site
        self.any_visit = []
        for k in range(nt):
            a = m.NewBoolVar(f"any_{k}")
            m.Add(sum(self.visit[vi, k] for vi in range(nv)) == a)
            self.any_visit.append(a)
        self.group_vars = {}
        for i, (key, (tier, value, task_idx)) in enumerate(groups.items()):
            g = m.NewBoolVar(f"g_{i}")
            m.Add(g <= sum(self.any_visit[k] for k in task_idx))
            self.group_vars[key] = (g, tier, value, task_idx)
        self.risk = sum(landings[k].risk * self.lu[vi, k] for vi in range(nv) for k in range(nl))
        self.switching = []
        for k, t in enumerate(tasks):
            owner = incumbent_owner.get(t.id)
            if owner is None:
                self.switching.append(self.any_visit[k])
            else:
                vi = next((i for i, v in enumerate(vehicles) if v.drone_id == owner), None)
                self.switching.append(1 - (self.visit[vi, k] if vi is not None else 0))

    def hint_paths(self, paths):
        """Give the solver a complete feasible starting solution (paths: vi -> (stops, landing))."""
        m = self.m
        visited = {(vi, k) for vi, (stops, _) in paths.items() for k in stops}
        for key, var in self.visit.items():
            m.AddHint(var, int(key in visited))
        on_arc: set = set()
        for vi, (stops, landing) in paths.items():
            veh, pos = self.vehicles[vi], self.pos[vi]
            node, t = "s", veh.start_s
            self.hint_time = getattr(self, "hint_time", {})
            for k in [*(("t", s) for s in stops), ("l", landing)]:
                dwell = self.tasks[node[1]].dwell_s if node != "s" else 0
                t = t + leg_seconds(pos[node], pos[k], veh.speed) + dwell
                on_arc.add((vi, node, k))
                self.hint_time[vi, k] = t
                node = k
        for key, var in self.arc.items():
            m.AddHint(var, int(key in on_arc))
        for vi in range(self.nv):
            used = vi in paths
            m.AddHint(self.used[vi], int(used))
            if self.wait[vi] is not None:
                m.AddHint(self.wait[vi], int(not used))
            for k in range(self.nl):
                m.AddHint(self.lu[vi, k], int(used and paths[vi][1] == k))
            m.AddHint(self.time[vi, "s"], self.vehicles[vi].start_s)
            for n in self.pos[vi]:
                if n != "s":
                    m.AddHint(
                        self.time[vi, n],
                        getattr(self, "hint_time", {}).get((vi, n), self.vehicles[vi].start_s),
                    )
        for g, _, _, idx in self.group_vars.values():
            m.AddHint(g, int(any((vi, k) in visited for vi in range(self.nv) for k in idx)))
        for vi in range(self.nv):
            if vi in paths:
                stops, landing = paths[vi]
                m.AddHint(
                    self.air[vi], self.hint_time[vi, ("l", landing)] - self.vehicles[vi].start_s
                )
            else:
                m.AddHint(self.air[vi], 0)

    def exclude(self, visited: set[tuple[int, int]]):
        """Require at least one assignment to differ from an earlier round's plan."""
        diffs = [self.visit[key].Not() for key in visited]
        diffs += [var for key, var in self.visit.items() if key not in visited]
        self.m.AddBoolOr(diffs)

    def stages(self):
        out = []
        for u in scoring.TIERS:
            terms = [value * g for g, tier, value, _ in self.group_vars.values() if tier == u]
            out.append((f"tier-{u}", terms, "max"))
        out.append(("risk", [] if isinstance(self.risk, int) else [self.risk], "min"))
        out.append(("switching", self.switching, "min"))
        out.append(("airtime", self.air, "min"))
        return out


def _kept_targets(a, targets, pos, speed, exact):
    """Bounded model: from a task keep only its nearest successors and landings. From the start
    node every target stays, so a nearby task is never hidden. `exact` keeps everything."""
    if exact or a == "s":
        return set(targets)
    tt = sorted(
        (b for b in targets if b[0] == "t" and b != a),
        key=lambda b: leg_seconds(pos[a], pos[b], speed),
    )
    ll = sorted(
        (b for b in targets if b[0] == "l"), key=lambda b: leg_seconds(pos[a], pos[b], speed)
    )
    return set(tt[:NEXT_TASKS]) | set(ll[:LANDINGS_PER_TASK])


def _build_groups(world, ctx, geometry, tasks):
    """benefit group -> (tier, points, tasks that can earn it). Electrical groups use their
    standalone joint score as an additive surrogate; the fixed joint scorer corrects it later."""
    groups: dict = {}
    for k, t in enumerate(tasks):
        task = world.tasks[t.id]
        for g in scoring._task_groups(task, ctx, geometry):
            if g[0] == "electrical":
                s = scoring.score_plan(world, ctx, geometry, [t.id])
                tier = _tier(s)
                value = sum(s.tiers)
            else:
                tier, value = ctx.owners[g], ctx.values[g]
            value = max(round(value), 0)
            if g in groups:
                groups[g][2].append(k)
            else:
                groups[g] = (tier, value, [k])
    # cells served by exactly the same attempts are interchangeable: merge them into one group
    merged: dict = {}
    for key, (tier, value, idx) in groups.items():
        sig = (tier, tuple(sorted(idx)), key[0] if key[0] != "coverage" else "coverage")
        if key[0] == "coverage":
            if sig in merged:
                merged[sig] = (tier, merged[sig][1] + value, idx)
            else:
                merged[sig] = (tier, value, idx)
        else:
            merged[key] = (tier, value, idx)
    return merged


def _extract(model: _Model, solver) -> dict:
    nv = len(model.vehicles)
    visited = {key for key, var in model.visit.items() if solver.Value(var)}
    paths = []
    for vi in range(nv):
        if not solver.Value(model.used[vi]):
            continue
        node, stops, landing = "s", [], None
        for _ in range(len(model.tasks) + 2):
            nxt = next(
                (
                    b
                    for (v, a, b), x in model.arc.items()
                    if v == vi and a == node and solver.Value(x)
                ),
                None,
            )
            if nxt is None:
                break
            if nxt[0] == "t":
                stops.append(nxt[1])
                node = nxt
            else:
                landing = nxt[1]
                break
        paths.append((vi, stops, landing))
    return {"visited": visited, "paths": paths}


def _to_requests(world, model: _Model, paths, tasks_by_idx):
    reqs = []
    for vi, stops, landing in paths:
        drone = model.vehicles[vi].drone_id
        srs = []
        for k in stops:
            t = world.tasks[tasks_by_idx[k].id]
            srs.append(StopRequest(t.point, t.kind, t.purpose, t.repeat_of_reading_id, t.id))
        reqs.append(SortieRequest(drone, tuple(srs), model.landings[landing].site))
    return tuple(reqs)


# -- the planner ------------------------------------------


def plan_sorties(
    world: World,
    geometry: Geometry,
    circuits,
    est_state,
    *,
    revision: int = 0,
    deterministic_budget: float = DETERMINISTIC_BUDGET,
    wall_cap_s: float = WALL_CAP_S,
    rounds: int = MAX_ROUNDS,
    cap: int = ATTEMPT_CAP,
    exact: bool = False,
    landing_limit: int | None = MAX_LANDINGS,
    fault: str | None = None,
    elapsed_s: float = 0.0,
) -> PlanningResult:
    """`elapsed_s` is the time the caller already spent since ingestion; the solver wall cap is
    the smaller of `wall_cap_s` and what is left of the response budget after the reserve.
    `fault` injects UNKNOWN / MODEL_INVALID / timeout into every solve for failure testing."""
    t_start = time.perf_counter()
    wall_cap_s = max(0.0, min(wall_cap_s, RESPONSE_BUDGET_S - NON_SOLVER_RESERVE_S - elapsed_s))
    if fault == "timeout":
        wall_cap_s = 0.02
    replay = {
        "solver": SOLVER_VERSION,
        "seed": SEED,
        "workers": 1,
        "deterministic_budget": deterministic_budget,
        "wall_cap_s": wall_cap_s,
        "rounds": rounds,
        "fault": fault,
    }
    ctx = scoring.build_context(world, est_state)
    locked_ids = [t.id for t in world.tasks.values() if t.status is TaskStatus.MEASURING]
    incumbent = incumbent_candidate(world, geometry, circuits, ctx, locked_ids)
    chosen, omitted, blocked, _ = select_tasks(world, ctx, geometry, cap)
    ranked: list[Candidate] = [incumbent] if incumbent.valid else []
    if blocked is not None:
        return PlanningResult(
            revision, world.clock_s, tuple(ranked), (), (), (), blocked, "PlanningBlocked", replay
        )
    vehicles = _vehicles(world)
    tasks = [PlanTask(t.id, geometry.position_of(t.point), DWELL_S[t.kind]) for t in chosen]
    landings = _landing_pool(
        world, geometry, vehicles, tasks, None if exact and landing_limit is None else landing_limit
    )
    owner = {
        s.task_id: d.id
        for d in world.drones.values()
        if isinstance(d.state, Flying | Measuring)
        for s in d.state.route.stops
    }
    groups = _build_groups(world, ctx, geometry, tasks)
    stages: list[StageReport] = []
    seen: list[set] = []
    stop_reason = "completed"
    per_round = deterministic_budget / max(rounds, 1)
    for rnd in range(1, rounds + 1):
        remaining_wall = wall_cap_s - (time.perf_counter() - t_start)
        if remaining_wall <= 0.01:
            stop_reason = "wall cap reached before all rounds ran"
            break
        start = {}
        for variant in range(rnd - 1 + 8):  # a hint that does not repeat an excluded plan
            start = _greedy_paths(vehicles, tasks, landings, groups, variant if rnd > 1 else 0)
            visited = {(vi, k) for vi, (stops, _) in start.items() for k in stops}
            if rnd == 1 or visited not in seen:
                break
        model = _Model(vehicles, tasks, landings, groups, owner, exact, start)
        model.hint_paths(start)
        for prev in seen:
            model.exclude(prev)
        if fault in ("UNKNOWN", "MODEL_INVALID"):
            solution, reports = None, [StageReport(rnd, "injected", fault, None, None, 0.0)]
        else:
            solution, reports = _solve_staged(model, per_round, remaining_wall, rnd)
        stages += reports
        if solution is None:
            stop_reason = f"round {rnd}: no feasible solution ({reports[-1].status})"
            break
        seen.append(solution["visited"])
        reqs = _to_requests(world, model, solution["paths"], tasks)
        cand = _evaluate(world, geometry, circuits, ctx, reqs, f"round-{rnd}", locked_ids, owner)
        if cand.valid and not any(c.sorties == cand.sorties for c in ranked):
            ranked.append(cand)
    ranked.sort(key=lambda c: (c.score.key, c.source == "incumbent"), reverse=True)
    best = ranked[0] if ranked else None
    served = set(best.task_ids) if best else set()
    unassigned = tuple(
        Omission(t.id, _why_unassigned(world, geometry, circuits, t.id, vehicles))
        for t in chosen
        if t.id not in served
    )
    return PlanningResult(
        revision,
        world.clock_s,
        tuple(ranked),
        tuple(stages),
        unassigned,
        tuple(omitted),
        None,
        stop_reason,
        replay,
    )


def _solve_staged(model: _Model, budget: float, wall_s: float, rnd: int):
    solver = cp_model.CpSolver()
    p = solver.parameters
    p.num_workers = 1
    p.random_seed = SEED
    p.log_search_progress = False
    p.cp_model_presolve = False  # the models are small; presolve alone would eat the budget
    p.cp_model_probing_level = 0
    stages = [s for s in model.stages() if s[1] is not None and len(s[1]) > 0]
    per = budget / max(len(stages), 1)
    reports, last = [], None
    for name, terms, sense in stages:
        if not terms:
            reports.append(StageReport(rnd, name, "SKIPPED", None, None, 0.0))
            continue
        obj = sum(terms)
        model.m.Maximize(obj) if sense == "max" else model.m.Minimize(obj)
        p.max_deterministic_time = per
        p.max_time_in_seconds = max(wall_s / len(stages), 0.01)
        t0 = time.perf_counter()
        status = solver.Solve(model.m)
        wall = time.perf_counter() - t0
        sname = STATUS.get(status, str(status))
        if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
            val = round(solver.ObjectiveValue())
            reports.append(
                StageReport(rnd, name, sname, val, round(solver.BestObjectiveBound()), wall)
            )
            last = _extract(model, solver)
            model.m.Add(obj >= val) if sense == "max" else model.m.Add(obj <= val)
        else:
            reports.append(StageReport(rnd, name, sname, None, None, wall))
            break
    return last, reports


def _evaluate(world, geometry, circuits, ctx, reqs, source, locked_ids, owner) -> Candidate:
    """Independently validate a solver plan and score it with the fixed joint scorer."""
    result = validate_sorties(world, geometry, circuits, reqs) if reqs else None
    if result is not None and not result.ok:
        return Candidate(
            reqs,
            (),
            scoring.PlanScore((0,) * 5, 0, 0, 0, True),
            source,
            False,
            tuple(str(r) for r in result.rejections),
        )
    ids = [s.task_id for r in reqs for s in r.stops]
    ids += locked_ids
    air = sum(p.landing_arrive_s - p.start_s for p in result.plans) if result else 0
    switched = sum(
        1
        for t in dict.fromkeys(ids)
        if owner.get(t) not in {r.drone_id for r in reqs if any(s.task_id == t for s in r.stops)}
    )
    risk = 0
    if result:
        for p in result.plans:
            v = knowledge.assess_landing(world, p.landing, p.landing_arrive_s)
            risk += int(v.reasons[0].startswith("uncertain"))
    score = scoring.score_plan(
        world, ctx, geometry, ids, risk=risk, switching=switched, airtime_s=air
    )
    return Candidate(reqs, tuple(dict.fromkeys(ids)), score, source, True)


def _why_unassigned(world, geometry, circuits, task_id, vehicles) -> str:
    t = world.tasks[task_id]
    for v in vehicles:
        d = world.drones[v.drone_id]
        perch = d.state.perch if hasattr(d.state, "perch") else d.state.route.perch
        req = SortieRequest(
            v.drone_id,
            (StopRequest(t.point, t.kind, t.purpose, t.repeat_of_reading_id, t.id),),
            perch,
        )
        if validate_sorties(world, geometry, circuits, (req,)).ok:
            return "feasible alone, but left out for higher-urgency or higher-benefit work"
    return "infeasible for every drone within battery, endpoint eligibility or the deadline"


def _greedy_paths(vehicles, tasks, landings, groups, variant=0):
    """A complete feasible start for the solver (a hint, never an answer): airborne drones head
    for their nearest reachable landing; Ready drones take the highest-urgency single stop (and a
    second one if it fits), else wait. The solver and the independent validator decide."""
    value = {k: (0, 0) for k in range(len(tasks))}
    for tier, v, idx in groups.values():
        for k in idx:
            t0, v0 = value[k]
            value[k] = (max(t0, tier), v0 + v / max(len(idx), 1))
    order = sorted(range(len(tasks)), key=lambda k: (-value[k][0], -value[k][1], tasks[k].id))
    if order and variant:
        order = order[variant % len(order) :] + order[: variant % len(order)]
    taken_tasks: set[int] = set()
    taken_lands: set[int] = set()
    paths = {}

    def nearest_landing(pos, v, t_arrive_base):
        best = None
        for j, l_ in enumerate(landings):
            if j in taken_lands:
                continue
            dur = leg_seconds(pos, l_.position, v.speed)
            arrive = t_arrive_base + dur
            if arrive < l_.until_s and arrive <= DEADLINE_S and (best is None or dur < best[0]):
                best = (dur, j)
        return best

    for vi, v in enumerate(vehicles):
        if v.kind == "ready":
            continue
        best = nearest_landing(v.position, v, v.start_s)
        if best is not None and v.start_s + best[0] - v.start_s <= v.battery_s:
            paths[vi] = ([], best[1])
            taken_lands.add(best[1])
    for vi, v in enumerate(vehicles):
        if v.kind != "ready":
            continue
        for k in order:
            if k in taken_tasks or value[k][0] == 0:
                continue
            go = leg_seconds(v.position, tasks[k].position, v.speed)
            t_done = v.start_s + go + tasks[k].dwell_s
            land = nearest_landing(tasks[k].position, v, t_done)
            if land is None or t_done + land[0] - v.start_s > v.battery_s:
                continue
            paths[vi] = ([k], land[1])
            taken_tasks.add(k)
            taken_lands.add(land[1])
            break
    return paths
