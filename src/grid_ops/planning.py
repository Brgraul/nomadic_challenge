"""Bounded multi-drone itinerary optimisation with OR-Tools CP-SAT.

`plan_sorties` is pure: it reads the public world and returns ranked candidate plans; the sim
alone commits, and only after independent validation (`routes.validate_sorties`). Vehicles are
Perched drones (planning from the current clock) and Executing drones (planning from the end of
their current step; the locked prefix is never a decision variable). Nodes are airborne scan
tasks and perch visits, each perch visit serving exactly one ground task at one candidate site;
every used vehicle path ends at a perch visit, whose site becomes the itinerary's `final_perch`.

The model is deliberately conservative in two ways while `routes.validate_sorties` stays the
exact authority and every candidate is revalidated before ranking: a perch node is offered to at
most one drone per plan (the validator instead checks exact timed per-site occupancy, so two
nodes on one site can still be rejected there), and the energy filter is a prefix-ordered shadow
ledger - static per-operation costs against the remaining battery, with charge gain counted only
at CONFIRMED sites and only toward costs incurred after that charge - instead of the exact
capacity-capped ledger replay.

Objective: staged solves following `scoring.PlanScore.key` - per urgency tier the maximal group
value, then risk (charge at uncertain endpoints under the explicit policy), then switching
(owner changes versus the incumbent), then airtime, then total charge time. Up to three
reranking rounds produce distinct candidates that are ranked against the revalidated incumbent,
all inside one shared deterministic search budget.
"""

from __future__ import annotations

import itertools
import time
from dataclasses import dataclass, field

import ortools
from ortools.sat.python import cp_model

from grid_ops import knowledge, scoring
from grid_ops.energy import EnergyModel
from grid_ops.geometry import Geometry, LandingSite, haversine_km
from grid_ops.grid import LonLat
from grid_ops.knowledge import ChargeState
from grid_ops.routes import SortieRequest, current_requests, validate_sorties
from grid_ops.sensing import ContactCurrent, LidarPass, TaskStatusV2, TdrTest
from grid_ops.world import (
    DEADLINE_S,
    LAND_S,
    TAKEOFF_S,
    Charge,
    Executing,
    GroundSample,
    Land,
    Perched,
    Scan,
    ScheduledStep,
    Stranded,
    Takeoff,
    Transit,
    Wait,
    World,
    ground_duration_s,
    leg_seconds,
    position_at,
)

ATTEMPT_CAP = 20
MAX_ROUNDS = 3
RESPONSE_BUDGET_S = 2.0  # observation ingestion to validated plan publication
NON_SOLVER_RESERVE_S = 0.5  # held back for diagnostics, refresh, construction, scoring, acceptance
DETERMINISTIC_BUDGET = 0.1  # total deterministic-time units per decision
WALL_CAP_S = 1.5
SEED = 0
CHARGE_OPTIONS_S = (0, 900, 1800, 3600, 5400, 7200, 9000, 10800)
LANDINGS_PER_TASK = 2  # candidate perch sites kept per ground task
MAX_LANDINGS = 16  # pool cap for the bounded model; exact mode keeps every candidate
NEXT_TASKS = 6  # arcs kept per node toward its nearest successors
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
    kind: str  # perched | executing
    position: LonLat  # where the vehicle's first leg starts
    start_s: int  # the first planned step begins here
    airborne: bool  # takeoff needed before the first transit when False
    site: LandingSite | None  # where a grounded vehicle takes off from
    battery_units: int
    speed: float


@dataclass(frozen=True)
class PlanScan:
    task_id: str
    acquisition: LidarPass
    entry: LonLat
    exit: LonLat
    duration_s: int


@dataclass(frozen=True)
class PlanPerch:
    task_id: str
    acquisition: ContactCurrent | TdrTest
    site: LandingSite
    position: LonLat
    member_s: int
    charge_options: tuple[int, ...]  # (0,) unless the site's chargeability allows charging
    uncertain: bool  # charge options exist only by the explicit uncertain-endpoint policy


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


def _select(world: World, ctx, cap: int = ATTEMPT_CAP):
    """At most `cap` attempts. Locked (in-progress) work is never a decision variable; every
    highest-urgency alternative group keeps one representative task; the rest rank by standalone
    benefit. Every omission is reported, and required work that cannot fit is a PlanningBlocked."""
    open_tasks = [
        t
        for t in world.tasks.values()
        if t.status in (TaskStatusV2.PENDING, TaskStatusV2.RESERVED)
    ]
    solo = {t.id: scoring.score_plan(world, ctx, [t.id]) for t in open_tasks}
    top = max((_tier(s) for s in solo.values()), default=0)
    locked = [t for t in world.tasks.values() if t.status is TaskStatusV2.MEASURING]
    reps: dict[str, str] = {}
    for t in sorted(open_tasks, key=lambda t: t.id):
        if top and _tier(solo[t.id]) == top:
            cur = reps.get(t.alternative_group_id)
            if cur is None or sum(solo[t.id].tiers) > sum(solo[cur].tiers):
                reps[t.alternative_group_id] = t.id
    required = sorted(set(reps.values()))
    if len(required) + len(locked) > cap:
        return (
            [],
            [],
            PlanningBlocked(
                f"required work alone ({len(required)} highest-urgency groups and "
                f"{len(locked)} locked attempts) exceeds the {cap}-attempt capacity"
            ),
        )
    rest = sorted(
        (t for t in open_tasks if t.id not in required),
        key=lambda t: (-_tier(solo[t.id]), -sum(solo[t.id].tiers), t.id),
    )
    room = max(cap - len(required) - len(locked), 0)
    chosen = [world.tasks[i] for i in required] + [t for t in rest[:room]]
    omitted = [
        Omission(t.id, f"over the {cap}-attempt cap (tier {_tier(solo[t.id])})")
        for t in rest[room:]
    ]
    return chosen, omitted, None


# -- model inputs ------------------------------------------


def _step_task_ids(step: ScheduledStep) -> tuple[str, ...]:
    op = step.operation
    if isinstance(op, Scan):
        return (op.task_id,)
    if isinstance(op, GroundSample):
        return op.task_ids
    return ()


def _owners(world: World) -> dict[str, str]:
    """Incumbent owner per task: every task on a running itinerary or reserved to a drone."""
    owner = {}
    for d in world.drones.values():
        st = d.state
        if isinstance(st, Executing):
            for step in st.itinerary.steps:
                for task_id in _step_task_ids(step):
                    owner[task_id] = d.id
    for t in world.tasks.values():
        if t.assigned_drone_id is not None and t.status in (
            TaskStatusV2.RESERVED,
            TaskStatusV2.MEASURING,
        ):
            owner[t.id] = t.assigned_drone_id
    return owner


def _vehicles(world: World, geometry: Geometry) -> tuple[PlanVehicle, ...]:
    """Perched drones start at the current clock; Executing drones start at their current step's
    end from that step's anchor position. The locked prefix is not modelled."""
    out = []
    for d in sorted(world.drones.values(), key=lambda d: d.id):
        st = d.state
        if isinstance(st, (Perched, Stranded)):
            out.append(
                PlanVehicle(
                    d.id,
                    "perched",
                    geometry.position_of(st.perch.point),
                    world.clock_s,
                    False,
                    st.perch,
                    d.battery.remaining_units,
                    d.speed_km_per_min,
                )
            )
        elif isinstance(st, Executing):
            op = st.step.operation
            if isinstance(op, Wait) and not op.airborne:
                continue  # a grounded wait has no site to take off from; the itinerary stands
            grounded = isinstance(op, (Land, GroundSample, Charge))
            out.append(
                PlanVehicle(
                    d.id,
                    "executing",
                    position_at(st.step, 1.0),
                    st.step.end_s,
                    not grounded,
                    op.site if grounded else None,
                    d.battery.remaining_units,
                    d.speed_km_per_min,
                )
            )
    return tuple(out)


def _site_pool(
    world: World, geometry: Geometry, vehicles, grounds, exact: bool
) -> list[LandingSite]:
    """Eligible, unoccupied perch sites near the vehicles and the ground tasks, with the exact
    projections `landing_candidates` derives from those anchors."""
    if not vehicles:
        return []
    anchors = tuple(v.position for v in vehicles) + tuple(
        geometry.position_of(t.acquisition.conductor.point) for t in grounds
    )
    seen: dict[LandingSite, LandingSite] = {}
    for veh in vehicles:
        for c in knowledge.landing_candidates(
            world, geometry, veh.drone_id, lambda site: world.clock_s, anchors
        ):
            if c.eligibility.eligible and not c.occupant:
                seen.setdefault(c.site, c.site)
    pool = sorted(seen.values(), key=lambda s: (s.circuit_id, s.point.offset_mm))
    if not exact and len(pool) > MAX_LANDINGS:
        pool.sort(
            key=lambda s: min(haversine_km(a, geometry.position_of(s.point)) for a in anchors)
        )
        pool = sorted(pool[:MAX_LANDINGS], key=lambda s: (s.circuit_id, s.point.offset_mm))
    return pool


def _charge_options(world: World, site: LandingSite, member_s: int) -> tuple[int, ...]:
    """Charge options for one perch visit: only CONFIRMED sites (or UNKNOWN sites under the
    explicit uncertain-endpoint policy) may charge, capped so the fixed visit fits the deadline."""
    ch = world.knowledge.chargeability.get(site.circuit_id)
    confirmed = ch is not None and ch.state is ChargeState.CONFIRMED
    uncertain = (
        ch is not None
        and ch.state is ChargeState.UNKNOWN
        and world.landing_policy.allow_uncertain
    )
    if not (confirmed or uncertain):
        return (0,)
    cap = max(0, DEADLINE_S - (LAND_S + member_s + TAKEOFF_S))
    return tuple(o for o in CHARGE_OPTIONS_S if o <= cap)


def _build_nodes(
    world: World, geometry: Geometry, vehicles, tasks, exact: bool = False
) -> tuple[tuple[PlanScan, ...], tuple[PlanPerch, ...]]:
    """Decision nodes: one scan node per lidar task, up to `LANDINGS_PER_TASK` perch nodes per
    ground task over the eligible site pool."""
    scans = tuple(
        PlanScan(
            t.id,
            t.acquisition,
            t.acquisition.trajectory.poses[0].position,
            t.acquisition.trajectory.poses[-1].position,
            t.acquisition.trajectory.duration_s,
        )
        for t in tasks
        if isinstance(t.acquisition, LidarPass)
    )
    grounds = [t for t in tasks if isinstance(t.acquisition, (ContactCurrent, TdrTest))]
    perches: list[PlanPerch] = []
    if grounds:
        pool = _site_pool(world, geometry, vehicles, grounds, exact)
        for t in grounds:
            pos = geometry.position_of(t.acquisition.conductor.point)
            near = sorted(
                pool,
                key=lambda s: (
                    haversine_km(pos, geometry.position_of(s.point)),
                    s.circuit_id,
                    s.point.offset_mm,
                ),
            )[:LANDINGS_PER_TASK]
            for site in near:
                member_s = ground_duration_s((t.acquisition,))
                perches.append(
                    PlanPerch(
                        t.id,
                        t.acquisition,
                        site,
                        geometry.position_of(site.point),
                        member_s,
                        _charge_options(world, site, member_s),
                        _is_uncertain(world, site),
                    )
                )
    return scans, tuple(perches)


def _is_uncertain(world: World, site: LandingSite) -> bool:
    ch = world.knowledge.chargeability.get(site.circuit_id)
    return (
        ch is not None
        and ch.state is ChargeState.UNKNOWN
        and world.landing_policy.allow_uncertain
    )


def _build_groups(world: World, ctx, scans, perches) -> dict:
    """Benefit group -> (tier, points, nodes that can earn it). Ground requests earn one flat
    group award however many candidate sites serve them; lidar earns per uncovered cell."""
    groups: dict = {}
    for j, p in enumerate(perches):
        task = world.tasks[p.task_id]
        key = ("req", task.request_id)
        tier = ctx.request_urgency.get(task.request_id, task.urgency)
        if key in groups:
            groups[key][2].append(("p", j))
        else:
            groups[key] = (tier, scoring.REQUEST_POINTS, [("p", j)])
    for i, s in enumerate(scans):
        target = s.acquisition.target
        for cell, _length in scoring._cells(target.start_mm, target.end_mm):
            key = ("cell", target.segment_id, cell)
            if key not in ctx.cell_values:
                continue
            if key in groups:
                groups[key][2].append(("t", i))
            else:
                groups[key] = (
                    ctx.cell_owners[key], max(round(ctx.cell_values[key]), 0), [("t", i)]
                )
    # cells served by exactly the same nodes are interchangeable: merge them into one group
    merged: dict = {}
    for key, (tier, value, nodes_) in groups.items():
        if key[0] != "cell":
            merged[key] = (tier, value, nodes_)
            continue
        sig = (tier, tuple(sorted(nodes_)))
        if sig in merged:
            merged[sig] = (tier, merged[sig][1] + value, nodes_)
        else:
            merged[sig] = (tier, value, nodes_)
    return merged


# -- extraction: solver paths to explicit itineraries ------------------------------------------


def _build_sortie(
    world: World,
    geometry: Geometry,
    veh: PlanVehicle,
    vi: int,
    scans,
    perches,
    seq: tuple,
    charge_of,
    revision: int,
) -> SortieRequest | None:
    """Exact ScheduledSteps for one vehicle's node sequence; charge_of(vi, perch_index) gives the
    chosen charge seconds. Steps are contiguous and start at the vehicle's start boundary."""
    t = veh.start_s
    steps: list[ScheduledStep] = []
    pos = veh.position
    if not veh.airborne:
        site = veh.site
        if site is None:
            return None
        at = geometry.position_of(site.point)
        steps.append(ScheduledStep(len(steps), t, t + TAKEOFF_S, Takeoff(site, at)))
        t += TAKEOFF_S
        pos = at
    last = len(seq) - 1
    for i, node in enumerate(seq):
        if node[0] == "t":
            sc = scans[node[1]]
            leg = leg_seconds(pos, sc.entry, veh.speed)
            if leg > 0:
                steps.append(ScheduledStep(len(steps), t, t + leg, Transit(pos, sc.entry)))
                t, pos = t + leg, sc.entry
            steps.append(
                ScheduledStep(len(steps), t, t + sc.duration_s, Scan(sc.acquisition, sc.task_id))
            )
            t, pos = t + sc.duration_s, sc.exit
        else:
            p = perches[node[1]]
            leg = leg_seconds(pos, p.position, veh.speed)
            if leg > 0:
                steps.append(ScheduledStep(len(steps), t, t + leg, Transit(pos, p.position)))
                t, pos = t + leg, p.position
            steps.append(ScheduledStep(len(steps), t, t + LAND_S, Land(p.site, p.position)))
            t += LAND_S
            steps.append(
                ScheduledStep(
                    len(steps),
                    t,
                    t + p.member_s,
                    GroundSample((p.acquisition,), p.site, (p.task_id,), p.position),
                )
            )
            t += p.member_s
            charge_s = charge_of(vi, node[1])
            if charge_s > 0:
                steps.append(
                    ScheduledStep(len(steps), t, t + charge_s, Charge(p.site, charge_s, p.position))
                )
                t += charge_s
            if i != last:
                steps.append(
                    ScheduledStep(len(steps), t, t + TAKEOFF_S, Takeoff(p.site, p.position))
                )
                t += TAKEOFF_S
    if not steps or t > DEADLINE_S:
        return None
    if seq[-1][0] == "t":
        # a scan-ending path must still land: return to the vehicle's perch, else the nearest
        # candidate perch site in the pool
        terminal = None
        if not veh.airborne and veh.site is not None:
            terminal = veh.site
        elif perches:
            terminal = min(perches, key=lambda p: leg_seconds(pos, p.position, veh.speed)).site
        if terminal is None:
            return None
        t_pos = geometry.position_of(terminal.point)
        leg = leg_seconds(pos, t_pos, veh.speed)
        if leg > 0:
            steps.append(ScheduledStep(len(steps), t, t + leg, Transit(pos, t_pos)))
            t, pos = t + leg, t_pos
        if t + LAND_S > DEADLINE_S:
            return None
        steps.append(ScheduledStep(len(steps), t, t + LAND_S, Land(terminal, t_pos)))
        t += LAND_S
        final_perch = terminal
    else:
        final_perch = perches[seq[-1][1]].site
    return SortieRequest(
        veh.drone_id,
        tuple(steps),
        final_perch,
        revision,
        world.energy_model.revision,
        world.capabilities.revision,
    )


def _req_task_ids(req: SortieRequest) -> list[str]:
    return [task_id for step in req.steps for task_id in _step_task_ids(step)]


def _plan_airtime_s(plans) -> int:
    """Airborne seconds across validated itineraries: transit, scan, land, takeoff."""
    total = 0
    for p in plans:
        for step in p.itinerary.steps:
            op = step.operation
            if isinstance(op, (Transit, Scan, Land, Takeoff)) or (
                isinstance(op, Wait) and op.airborne
            ):
                total += step.duration_s
    return total


def _plan_charge_s(plans) -> int:
    return sum(
        step.operation.charge_s
        for p in plans
        for step in p.itinerary.steps
        if isinstance(step.operation, Charge)
    )


def _evaluate(world, geometry, circuits, ctx, reqs, source, locked_ids, owner) -> Candidate:
    """Independently validate a candidate and score it with the fixed joint scorer. Invalid
    candidates keep their rejection reasons as problems and are never ranked."""
    empty = scoring.PlanScore((0,) * len(scoring.TIERS), 0, 0, 0, 0)
    plans = ()
    if reqs:
        try:
            result = validate_sorties(world, geometry, circuits, reqs)
        except (ValueError, AttributeError) as e:  # malformed continuation data
            return Candidate(reqs, (), empty, source, False, (f"validation error: {e}",))
        if not result.ok:
            return Candidate(
                reqs, (), empty, source, False, tuple(str(r) for r in result.rejections)
            )
        plans = result.plans
    ids = [task_id for r in reqs for task_id in _req_task_ids(r)] + list(locked_ids)
    serving = {task_id: r.drone_id for r in reqs for task_id in _req_task_ids(r)}
    switched = sum(1 for tid in dict.fromkeys(ids) if owner.get(tid) != serving.get(tid))
    risk = 0
    for p in plans:
        for step in p.itinerary.steps:
            op = step.operation
            if isinstance(op, Charge):
                ch = world.knowledge.chargeability.get(op.site.circuit_id)
                if ch is not None and ch.state is ChargeState.UNKNOWN:
                    risk += 1
    score = scoring.score_plan(
        world,
        ctx,
        ids,
        risk=risk,
        switching=switched,
        airtime_s=_plan_airtime_s(plans),
        charge_s=_plan_charge_s(plans),
    )
    return Candidate(reqs, tuple(dict.fromkeys(ids)), score, source, True)


def _incumbent_candidate(world, geometry, circuits, ctx, locked_ids, owner) -> Candidate:
    """Current routes, revalidated and rescored like every round candidate."""
    reqs = tuple(current_requests(world))
    return _evaluate(world, geometry, circuits, ctx, reqs, "incumbent", locked_ids, owner)


# -- the CP-SAT model ------------------------------------------


def _kept_targets(a, nodes, exits, entries, speed, exact):
    """Bounded model: from a node keep only its nearest successors (scans and perch visits) and,
    separately, its nearest perch visits so a path can always end. From the start node every
    target stays, so nearby work is never hidden. `exact` keeps everything."""
    if exact or a == "s":
        return set(nodes)
    near = sorted(
        (b for b in nodes if b != a), key=lambda b: leg_seconds(exits[a], entries[b], speed)
    )
    perches = sorted(
        (b for b in nodes if b[0] == "p"), key=lambda b: leg_seconds(exits[a], entries[b], speed)
    )
    return set(near[:NEXT_TASKS]) | set(perches[:LANDINGS_PER_TASK])


class _Model:
    def __init__(
        self,
        vehicles,
        scans,
        perches,
        groups,
        owner,
        energy: EnergyModel,
        assigned,
        exact=False,
        start_paths=None,
    ):
        self.vehicles, self.scans, self.perches = vehicles, scans, perches
        self.m = cp_model.CpModel()
        m = self.m
        self.nv, self.ns, self.np = len(vehicles), len(scans), len(perches)
        self.nodes = [("t", i) for i in range(self.ns)] + [("p", j) for j in range(self.np)]
        self.entry = {**{("t", i): scans[i].entry for i in range(self.ns)},
                      **{("p", j): perches[j].position for j in range(self.np)}}
        self.exit = {**{("t", i): scans[i].exit for i in range(self.ns)},
                     **{("p", j): perches[j].position for j in range(self.np)}}
        self.dwell = {("t", i): scans[i].duration_s for i in range(self.ns)} | {
            ("p", j): LAND_S + perches[j].member_s + TAKEOFF_S for j in range(self.np)
        }
        self.task_of = {
            n: (scans[n[1]].task_id if n[0] == "t" else perches[n[1]].task_id) for n in self.nodes
        }
        owner_vi = {v.drone_id: vi for vi, v in enumerate(vehicles)}
        self.visit: dict[tuple, cp_model.IntVar] = {}
        for n in self.nodes:
            allowed = assigned.get(self.task_of[n])
            for vi in range(len(vehicles)):
                if allowed is None or owner_vi.get(allowed) == vi:
                    self.visit[vi, n] = m.NewBoolVar(f"visit_{vi}_{n}")
        self._make_charge_vars(m)
        self.arc: dict[tuple, cp_model.IntVar] = {}
        self.arc_air: dict[tuple, int] = {}
        self.arc_cost: dict[tuple, int] = {}
        self.out: dict[tuple, list] = {}
        self.into: dict[tuple, list] = {}
        self.time = {}
        self.gross: dict[tuple, cp_model.IntVar] = {}
        self.gain: dict[tuple, cp_model.IntVar] = {}
        self.used, self.wait, self.air, self.end = [], [], [], {}
        force: set[tuple] = set()  # arcs of the starting solution always exist, pruned or not
        for vi, seq in (start_paths or {}).items():
            force |= {(vi, a, b) for a, b in itertools.pairwise(["s", *seq])}
        for vi, veh in enumerate(vehicles):
            self.time[vi, "s"] = m.NewIntVar(0, DEADLINE_S, f"t_{vi}_s")
            for n in self.nodes:
                self.time[vi, n] = m.NewIntVar(0, DEADLINE_S, f"t_{vi}_{n}")
            m.Add(self.time[vi, "s"] == veh.start_s)
            # prefix-ordered energy ledger shadow: gross cost and charge gain per node
            self.gross[vi, "s"] = m.NewIntVar(0, 10**8, f"gross_{vi}_s")
            self.gain[vi, "s"] = m.NewIntVar(0, energy.capacity_units, f"gain_{vi}_s")
            m.Add(self.gross[vi, "s"] == 0)
            m.Add(self.gain[vi, "s"] == 0)
            for n in self.nodes:
                self.gross[vi, n] = m.NewIntVar(0, 10**8, f"gross_{vi}_{n}")
                self.gain[vi, n] = m.NewIntVar(0, energy.capacity_units, f"gain_{vi}_{n}")
            for a in ["s", *self.nodes]:
                if a != "s" and (vi, a) not in self.visit:
                    continue
                keep = _kept_targets(a, self.nodes, self.exit, self.entry, veh.speed, exact)
                for b in self.nodes:
                    if b == a or (b not in keep and (vi, a, b) not in force):
                        continue
                    if (vi, b) not in self.visit:
                        continue
                    origin = veh.position if a == "s" else self.exit[a]
                    travel = leg_seconds(origin, self.entry[b], veh.speed)
                    takeoff_s = 0 if (a != "s" or veh.airborne) else TAKEOFF_S
                    dwell_a = 0 if a == "s" else self.dwell[a]
                    if veh.start_s + travel + takeoff_s + dwell_a > DEADLINE_S:
                        continue
                    cost = energy.units_per_airborne_s * (travel + takeoff_s)
                    if a == "s" and not veh.airborne:
                        cost += energy.takeoff_units
                    if cost > veh.battery_units:
                        continue
                    x = m.NewBoolVar(f"arc_{vi}_{a}_{b}")
                    self.arc[vi, a, b] = x
                    self.arc_air[vi, a, b] = travel + takeoff_s
                    self.arc_cost[vi, a, b] = cost
                    self.out.setdefault((vi, a), []).append(x)
                    self.into.setdefault((vi, b), []).append(x)
                    arrival = dwell_a + travel + takeoff_s
                    if a != "s" and a[0] == "p":
                        arrival += self.charge[vi, a[1]]
                    m.Add(self.time[vi, b] >= self.time[vi, a] + arrival).OnlyEnforceIf(x)
                    # prefix-ordered energy filter: cost before b's charge vs gain so far, then
                    # the rest of b's visit against the gain including b's own charge
                    pre, post, rate = self._visit_energy(vi, b, energy)
                    m.Add(
                        self.gross[vi, a] + cost + pre - self.gain[vi, a] <= veh.battery_units
                    ).OnlyEnforceIf(x)
                    m.Add(
                        self.gross[vi, b] >= self.gross[vi, a] + cost + pre + post
                    ).OnlyEnforceIf(x)
                    m.Add(
                        self.gross[vi, b] <= self.gross[vi, a] + cost + pre + post
                    ).OnlyEnforceIf(x)
                    m.Add(self.gain[vi, b] >= self.gain[vi, a] + rate).OnlyEnforceIf(x)
                    m.Add(self.gain[vi, b] <= self.gain[vi, a] + rate).OnlyEnforceIf(x)
                    m.Add(
                        self.gross[vi, b] - self.gain[vi, b] <= veh.battery_units
                    ).OnlyEnforceIf(x)
            used = m.NewBoolVar(f"used_{vi}")
            self.used.append(used)
            m.Add(sum(self.out.get((vi, "s"), [])) == used)
            w = m.NewBoolVar(f"wait_{vi}")
            m.Add(used + w == 1)  # a vehicle either flies a perched-ending path or waits
            self.wait.append(w)
            air_terms = [x * self.arc_air[key] for key, x in self.arc.items() if key[0] == vi]
            for i in range(self.ns):
                if (vi, ("t", i)) not in self.visit:
                    continue
                air_terms.append(self.visit[vi, ("t", i)] * scans[i].duration_s)
            for j in range(self.np):
                if (vi, ("p", j)) not in self.visit:
                    continue
                air_terms.append(self.visit[vi, ("p", j)] * (LAND_S + TAKEOFF_S))
            air = m.NewIntVar(0, DEADLINE_S, f"air_{vi}")
            self.air.append(air)
            m.Add(air == sum(air_terms))
        for vi in range(self.nv):
            ends = []
            for j in range(self.np):
                if (vi, ("p", j)) not in self.visit:
                    continue
                v = self.visit[vi, ("p", j)]
                e = m.NewBoolVar(f"end_{vi}_{j}")
                self.end[vi, j] = e
                ends.append(e)
                m.Add(sum(self.into.get((vi, ("p", j)), [])) == v)
                m.Add(sum(self.out.get((vi, ("p", j)), [])) == v - e)
                m.Add(
                    self.time[vi, ("p", j)] + self.dwell[("p", j)] + self.charge[vi, j]
                    <= DEADLINE_S
                ).OnlyEnforceIf(v)
            for i in range(self.ns):
                if (vi, ("t", i)) not in self.visit:
                    continue
                v = self.visit[vi, ("t", i)]
                m.Add(sum(self.into.get((vi, ("t", i)), [])) == v)
                m.Add(sum(self.out.get((vi, ("t", i)), [])) == v)
                m.Add(
                    self.time[vi, ("t", i)] + self.dwell[("t", i)] <= DEADLINE_S
                ).OnlyEnforceIf(v)
            m.Add(sum(ends) == self.used[vi])
        self.any = {}
        for n in self.nodes:
            a = m.NewBoolVar(f"any_{n}")
            m.Add(a == sum(self.visit.get((vi, n), 0) for vi in range(self.nv)))
            self.any[n] = a
        by_task: dict[str, list] = {}
        for n in self.nodes:
            by_task.setdefault(self.task_of[n], []).append(n)
        for _tid, ns in by_task.items():  # unique ownership across every node serving the task
            m.Add(sum(self.any[n] for n in ns) <= 1)
        self.group_vars = {}
        for i, (key, (tier, value, nodes_)) in enumerate(groups.items()):
            g = m.NewBoolVar(f"g_{i}")
            m.Add(g <= sum(self.any[n] for n in nodes_))
            self.group_vars[key] = (g, tier, value, nodes_)
        self.risk = sum(self.risk_terms) if self.risk_terms else 0
        self.switching = []
        for tid in sorted(by_task):
            own = owner.get(tid)
            vi = owner_vi.get(own) if own is not None else None
            if vi is None:
                self.switching.append(sum(self.any[n] for n in by_task[tid]))
            else:
                self.switching.append(
                    1 - sum(self.visit.get((vi, n), 0) for n in by_task[tid])
                )

    def _charge_domain(self, j: int):
        return cp_model.Domain.FromValues(self.perches[j].charge_options)

    def _visit_energy(self, vi, node, energy: EnergyModel):
        """(pre, post, gain_rate) energy parts of visiting `node`: the costs incurred before the
        visit's own charge step, the costs after it, and the charge gain rate (linear in the
        charge variable, nonzero only at CONFIRMED sites)."""
        if node[0] == "t":
            return energy.units_per_airborne_s * self.scans[node[1]].duration_s, 0, 0
        p = self.perches[node[1]]
        pre = (
            energy.landing_units
            + energy.units_per_airborne_s * LAND_S
            + energy.ground_units_per_s * p.member_s
        )
        post = energy.takeoff_units + energy.units_per_airborne_s * TAKEOFF_S
        if p.uncertain:
            return pre, post + energy.ground_units_per_s * self.charge[vi, node[1]], 0
        return pre, post, energy.net_charge_units_per_s * self.charge[vi, node[1]]

    def _make_charge_vars(self, m):
        """Charge seconds per (vehicle, perch visit), created before the arc loop needs them."""
        self.charge: dict[tuple, cp_model.IntVar] = {}
        self.charge_vars: list = []
        self.risk_terms: list = []
        for vi in range(self.nv):
            for j in range(self.np):
                if (vi, ("p", j)) not in self.visit:
                    continue
                c = m.NewIntVarFromDomain(self._charge_domain(j), f"charge_{vi}_{j}")
                self.charge[vi, j] = c
                self.charge_vars.append(c)
                m.Add(c == 0).OnlyEnforceIf(self.visit[vi, ("p", j)].Not())
                if self.perches[j].uncertain:
                    rb = m.NewBoolVar(f"risk_{vi}_{j}")
                    m.Add(c == 0).OnlyEnforceIf(rb.Not())
                    first = min(o for o in self.perches[j].charge_options if o > 0)
                    m.Add(c >= first).OnlyEnforceIf(rb)
                    self.risk_terms.append(rb)

    def hint_paths(self, paths):
        """Give the solver a complete feasible starting solution (paths: vi -> node sequence)."""
        m = self.m
        visited = {(vi, n) for vi, seq in paths.items() for n in seq}
        for key, var in self.visit.items():
            m.AddHint(var, int(key in visited))
        on_arc: set = set()
        self.hint_time = {}
        hint_air: dict[int, int] = {}
        for vi, seq in paths.items():
            veh = self.vehicles[vi]
            node, t, air = "s", veh.start_s, 0
            if not veh.airborne:
                t += TAKEOFF_S
                air += TAKEOFF_S
            for n in seq:
                origin = veh.position if node == "s" else self.exit[node]
                travel = leg_seconds(origin, self.entry[n], veh.speed)
                t += travel + (self.dwell[node] if node != "s" else 0)
                air += travel
                if n[0] == "t":
                    air += self.scans[n[1]].duration_s
                else:
                    air += LAND_S + TAKEOFF_S
                on_arc.add((vi, node, n))
                self.hint_time[vi, n] = t
                node = n
            hint_air[vi] = air
        for key, var in self.arc.items():
            m.AddHint(var, int(key in on_arc))
        for vi in range(self.nv):
            used = vi in paths
            m.AddHint(self.used[vi], int(used))
            m.AddHint(self.wait[vi], int(not used))
            m.AddHint(self.time[vi, "s"], self.vehicles[vi].start_s)
            for n in self.nodes:
                m.AddHint(self.time[vi, n], self.hint_time.get((vi, n), self.vehicles[vi].start_s))
            last = paths[vi][-1] if used else None
            for j in range(self.np):
                if (vi, ("p", j)) in self.visit:
                    m.AddHint(self.charge[vi, j], 0)
                    m.AddHint(self.end[vi, j], int(last == ("p", j)))
            m.AddHint(self.air[vi], hint_air.get(vi, 0))
        for g, _tier, _value, nodes_ in self.group_vars.values():
            m.AddHint(g, int(any((vi, n) in visited for vi in range(self.nv) for n in nodes_)))

    def exclude(self, visited: set[tuple]):
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
        out.append(("charge", self.charge_vars, "min"))
        return out


def _extract(model: _Model, solver) -> dict:
    visited = {key for key, var in model.visit.items() if solver.Value(var)}
    paths = {}
    for vi in range(model.nv):
        if not solver.Value(model.used[vi]):
            continue
        node, seq = "s", []
        for _ in range(len(model.nodes) + 1):
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
            seq.append(nxt)
            node = nxt
        paths[vi] = tuple(seq)
    return {"visited": visited, "paths": paths}


# -- the planner ------------------------------------------


def plan_sorties(
    world: World,
    geometry: Geometry,
    circuits,
    *,
    revision: int = 0,
    deterministic_budget: float = DETERMINISTIC_BUDGET,
    wall_cap_s: float = WALL_CAP_S,
    rounds: int = MAX_ROUNDS,
    cap: int = ATTEMPT_CAP,
    exact: bool = False,
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
    ctx = scoring.build_context(world)
    owner = _owners(world)
    assigned = {
        t.id: t.assigned_drone_id for t in world.tasks.values() if t.assigned_drone_id is not None
    }
    locked_ids = [t.id for t in world.tasks.values() if t.status is TaskStatusV2.MEASURING]
    incumbent = _incumbent_candidate(world, geometry, circuits, ctx, locked_ids, owner)
    ranked: list[Candidate] = [incumbent] if incumbent.valid else []
    chosen, omitted, blocked = _select(world, ctx, cap)
    if blocked is not None:
        return PlanningResult(
            revision, world.clock_s, tuple(ranked), (), (), (), blocked, "PlanningBlocked", replay
        )
    vehicles = _vehicles(world, geometry)
    scans, perches = _build_nodes(world, geometry, vehicles, chosen, exact)
    groups = _build_groups(world, ctx, scans, perches)
    stages: list[StageReport] = []
    seen: list[set] = []
    stop_reason = "completed"
    if not vehicles:
        stop_reason = "no available vehicle"
    elif not scans and not perches:
        stop_reason = "no visit nodes for the chosen tasks"
    else:
        per_round = deterministic_budget / max(rounds, 1)
        for rnd in range(1, rounds + 1):
            remaining_wall = wall_cap_s - (time.perf_counter() - t_start)
            if remaining_wall <= 0.01:
                stop_reason = "wall cap reached before all rounds ran"
                break
            start = {}
            for variant in range(rnd - 1 + 8):  # a hint that does not repeat an excluded plan
                start = _greedy_paths(
                    vehicles, scans, perches, groups, assigned, variant if rnd > 1 else 0
                )
                visited = {(vi, n) for vi, seq in start.items() for n in seq}
                if rnd == 1 or visited not in seen:
                    break
            model = _Model(
                vehicles, scans, perches, groups, owner, world.energy_model, assigned, exact, start
            )
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
            reqs = _round_requests(world, geometry, vehicles, model, solution, revision)
            source = f"round-{rnd}"
            cand = _evaluate(world, geometry, circuits, ctx, reqs, source, locked_ids, owner)
            if cand.valid:
                if not any(c.sorties == cand.sorties for c in ranked):
                    ranked.append(cand)
            else:
                replay.setdefault("rejected", []).append(
                    {"source": source, "problems": list(cand.problems)}
                )
    ranked.sort(key=lambda c: (c.score.key, c.source == "incumbent"), reverse=True)
    best = ranked[0] if ranked else None
    served = set(best.task_ids) if best else set()
    unassigned = tuple(
        Omission(
            t.id,
            _why_unassigned(world, geometry, circuits, t.id, vehicles, scans, perches, assigned),
        )
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


def _round_requests(world, geometry, vehicles, model, solution, revision) -> tuple:
    solver = solution["solver"]

    def charge_of(vi, j):
        var = model.charge.get((vi, j))
        return solver.Value(var) if var is not None else 0

    reqs = []
    for vi, seq in solution["paths"].items():
        req = _build_sortie(
            world, geometry, vehicles[vi], vi, model.scans, model.perches, seq, charge_of, revision
        )
        if req is not None:
            reqs.append(req)
    return tuple(reqs)


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
            last = {**_extract(model, solver), "solver": solver}
            model.m.Add(obj >= val) if sense == "max" else model.m.Add(obj <= val)
        else:
            reports.append(StageReport(rnd, name, sname, None, None, wall))
            break
    return last, reports


def _why_unassigned(world, geometry, circuits, task_id, vehicles, scans, perches, assigned) -> str:
    nodes = [("t", i) for i, s in enumerate(scans) if s.task_id == task_id]
    nodes += [("p", j) for j, p in enumerate(perches) if p.task_id == task_id]
    if not nodes:
        return "no eligible visit node (no eligible perch site for this task)"
    for vi, veh in enumerate(vehicles):
        own = assigned.get(task_id)
        if own is not None and veh.drone_id != own:
            continue
        for node in nodes:
            req = _build_sortie(
                world, geometry, veh, vi, scans, perches, (node,), lambda v, j: 0, world.revision
            )
            if req is not None and validate_sorties(world, geometry, circuits, (req,)).ok:
                return "feasible alone, but left out for higher-urgency or higher-benefit work"
    return "infeasible for every drone within battery, eligibility or the deadline"


def _greedy_paths(vehicles, scans, perches, groups, assigned, variant=0):
    """A complete feasible start for the solver (a hint, never an answer): each vehicle takes the
    best remaining task - a perch visit directly, a scan followed by the nearest available perch
    visit - else waits. The solver and the independent validator decide."""
    value = {}
    for tier, group_value, nodes_ in groups.values():
        for n in nodes_:
            t0, v0 = value.get(n, (0, 0.0))
            value[n] = (max(t0, tier), v0 + group_value / max(len(nodes_), 1))
    order = sorted(
        value,
        key=lambda n: (-value[n][0], -value[n][1], _task_label(n, scans, perches)),
    )
    if order and variant:
        cut = variant % len(order)
        order = order[cut:] + order[:cut]
    taken_tasks: set[str] = set()
    taken_nodes: set = set()
    paths = {}

    def feasible_perch(veh, t0, pos, j):
        p = perches[j]
        t_end = (
            t0
            + leg_seconds(pos, p.position, veh.speed)
            + LAND_S
            + p.member_s
            + TAKEOFF_S
        )
        return t_end <= DEADLINE_S

    for vi, veh in enumerate(vehicles):
        t0 = veh.start_s + (0 if veh.airborne else TAKEOFF_S)
        for n in order:
            tid = _task_label(n, scans, perches)
            own = assigned.get(tid)
            if own is not None and own != veh.drone_id:
                continue
            if tid in taken_tasks or n in taken_nodes:
                continue
            if n[0] == "p":
                if not feasible_perch(veh, t0, veh.position, n[1]):
                    continue
                paths[vi] = (n,)
            else:
                sc = scans[n[1]]
                t_scan = t0 + leg_seconds(veh.position, sc.entry, veh.speed) + sc.duration_s
                if t_scan > DEADLINE_S:
                    continue
                perch = min(
                    (
                        j
                        for j in range(len(perches))
                        if ("p", j) not in taken_nodes
                        and perches[j].task_id not in taken_tasks
                        and assigned.get(perches[j].task_id) in (None, veh.drone_id)
                        and feasible_perch(veh, t_scan, sc.exit, j)
                    ),
                    key=lambda j: leg_seconds(sc.exit, perches[j].position, veh.speed),
                    default=None,
                )
                if perch is None:
                    continue
                paths[vi] = (n, ("p", perch))
                taken_nodes.add(("p", perch))
            taken_tasks.add(tid)
            taken_nodes.add(n)
            break
    return paths


def _task_label(node, scans, perches) -> str:
    return scans[node[1]].task_id if node[0] == "t" else perches[node[1]].task_id
