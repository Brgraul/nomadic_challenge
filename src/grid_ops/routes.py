"""Itinerary validation and atomic application.

`validate_sorties` is the independent acceptance check: it replays every itinerary from
authoritative geometry, the live drone state and the shared energy law (never from a solver's or
caller's claimed times), enforces the operation state machine, method fidelity, timed site
occupancy, charge evidence and the T+04:00 deadline, and returns either full plans or every
rejection reason. `apply_sorties` mutates the World only with plans that validated.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise

from grid_ops.circuits import CircuitRegistry
from grid_ops.energy import BatteryState, OperationKind, integrate
from grid_ops.geometry import Geometry, LandingSite
from grid_ops.grid import LonLat
from grid_ops.knowledge import ChargeState
from grid_ops.sensing import (
    RequestStatus,
    TaskStatusV2,
)
from grid_ops.world import (
    DEADLINE_S,
    OPERATION_KIND,
    Charge,
    Drone,
    Executing,
    GroundSample,
    Itinerary,
    Land,
    Perched,
    Scan,
    ScheduledStep,
    Stranded,
    Takeoff,
    Transit,
    Wait,
    World,
    format_clock,
    leg_seconds,
    position_at,
    start_itinerary,
)


@dataclass(frozen=True)
class SortieRequest:
    drone_id: str
    steps: tuple[ScheduledStep, ...]
    final_perch: LandingSite
    based_on_revision: int
    energy_model_revision: str
    sensor_model_revision: str
    evidence_dependency_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class Rejection:
    drone_id: str | None
    code: str
    detail: str

    def __str__(self) -> str:
        who = f"{self.drone_id}: " if self.drone_id else ""
        return f"{who}{self.code}: {self.detail}"


@dataclass(frozen=True)
class SortiePlan:
    drone_id: str
    itinerary: Itinerary


@dataclass(frozen=True)
class Validation:
    plans: tuple[SortiePlan, ...]
    rejections: tuple[Rejection, ...]

    @property
    def ok(self) -> bool:
        return not self.rejections


def _start_boundary(drone: Drone, world: World) -> tuple[int, bool, LandingSite | None, str] | str:
    """(start_s, airborne, current_site, mode) where mode is perched | continue."""
    st = drone.state
    match st:
        case Perched(perch):
            return world.clock_s, False, perch, "perched"
        case Stranded(perch):
            return world.clock_s, False, perch, "perched"
        case Executing(itinerary=it, step_index=i):
            op = it.steps[i].operation
            grounded = isinstance(op, (Land, GroundSample, Charge)) or (
                isinstance(op, Wait) and not op.airborne
            )
            if grounded:
                return it.steps[i].end_s, False, _grounded_site(it, i), "continue"
            return it.steps[i].end_s, True, None, "continue"
    return f"{drone.phase} drones cannot take an itinerary"


def _grounded_site(it: Itinerary, i: int) -> LandingSite | None:
    """The site a grounded drone stands on: the op's site, or the last perching site before a
    bare Wait."""
    op = it.steps[i].operation
    if isinstance(op, (Land, GroundSample, Charge)):
        return op.site
    for step in reversed(it.steps[: i + 1]):
        if isinstance(step.operation, (Land, GroundSample, Charge)):
            return step.operation.site
    return None


def _cut_preamble(drone: Drone, world: World) -> tuple[BatteryState, int] | None:
    """Energy and time an in-progress scan contributes before a cut continuation starts."""
    st = drone.state
    if not isinstance(st, Executing) or not isinstance(st.step.operation, Scan):
        return None
    elapsed = world.clock_s - st.step.start_s
    if elapsed <= 0:
        return None
    battery, _ = integrate(drone.battery, OperationKind.SCAN, elapsed, model=world.energy_model)
    return battery, elapsed


def _cut_allowed(drone: Drone, world: World, steps: tuple[ScheduledStep, ...]) -> bool:
    """An in-progress Scan may be cut at the current clock; a GroundSample never may."""
    st = drone.state
    if not isinstance(st, Executing) or not isinstance(st.step.operation, Scan):
        return False
    return bool(steps) and steps[0].start_s == world.clock_s and world.clock_s < st.step.end_s


def validate_sorties(
    world: World,
    geometry: Geometry,
    circuits: CircuitRegistry,
    requests: tuple[SortieRequest, ...],
) -> Validation:
    rejections: list[Rejection] = []
    plans: list[SortiePlan] = []
    seen_drones: set[str] = set()
    claimed_tasks: dict[str, str] = {}
    # perched intervals (start, end, drone) claimed per exact site; existing perches run open-ended
    occupancy: dict[LandingSite, list[tuple[int, int, str]]] = {}
    for d in world.drones.values():
        st = d.state
        if isinstance(st, (Perched, Stranded)):
            occupancy.setdefault(st.perch, []).append((world.clock_s, 10**12, d.id))
        elif isinstance(st, Executing):
            occupancy.setdefault(st.itinerary.final_perch, []).append(
                (st.itinerary.end_s, 10**12, d.id)
            )

    for req in requests:
        reject = lambda code, detail, drone_id=req.drone_id: rejections.append(  # noqa: E731
            Rejection(drone_id, code, detail)
        )
        drone = world.drones.get(req.drone_id)
        if drone is None:
            reject("unknown_drone", f"{req.drone_id} does not exist")
            continue
        if req.drone_id in seen_drones:
            reject("duplicate_drone", "more than one itinerary for the same drone in one batch")
            continue
        seen_drones.add(req.drone_id)
        if req.energy_model_revision != world.energy_model.revision:
            reject(
                "stale_energy_model",
                f"plan assumes {req.energy_model_revision}, "
                f"world runs {world.energy_model.revision}",
            )
            continue
        if req.sensor_model_revision != world.capabilities.revision:
            reject(
                "stale_sensor_model",
                f"plan assumes {req.sensor_model_revision}, "
                f"world runs {world.capabilities.revision}",
            )
            continue
        started = _start_boundary(drone, world)
        if isinstance(started, str):
            reject("drone_unavailable", started)
            continue
        start_s, airborne, current_site, mode = started
        steps = req.steps
        if not steps:
            reject("empty_itinerary", "an itinerary must end with the drone perched")
            continue
        if mode == "continue" and steps[0].start_s != start_s and not _cut_allowed(
            drone, world, steps
        ):
            st = drone.state
            assert isinstance(st, Executing)
            reject(
                "bad_start",
                f"step 0 starts at {format_clock(steps[0].start_s / 60)} but step "
                f"{st.step_index} of the running itinerary ends at {format_clock(start_s / 60)}",
            )
            continue
        for a, b in pairwise(steps):
            if b.start_s != a.end_s:
                reject("non_contiguous", f"step {b.index} does not follow step {a.index}")
                break
        else:
            battery = drone.battery
            if mode == "continue" and (preamble := _cut_preamble(drone, world)) is not None:
                battery, _ = preamble
            here0 = (
                position_at(drone.state.step, 1.0)
                if mode == "continue"
                else drone.position
            )
            plan = _validate_steps(
                world,
                geometry,
                req,
                drone,
                steps,
                start_s,
                airborne,
                current_site,
                here0,
                battery,
                claimed_tasks,
                occupancy,
                reject,
            )
            if plan is not None:
                plans.append(plan)
    _check_occupancy(occupancy, rejections)
    if rejections:
        return Validation((), tuple(rejections))
    return Validation(tuple(plans), ())


def _check_occupancy(occupancy: dict, rejections: list[Rejection]) -> None:
    for site, intervals in occupancy.items():
        ordered = sorted(intervals)
        for (_, e0, d0), (s1, _, d1) in pairwise(ordered):
            if d0 != d1 and s1 < e0:
                rejections.append(
                    Rejection(
                        None,
                        "site_occupied",
                        f"{d0} and {d1} both perch on {site.circuit_id} at "
                        f"{site.point.offset_mm} mm with overlapping intervals",
                    )
                )


def _validate_steps(
    world: World,
    geometry: Geometry,
    req: SortieRequest,
    drone: Drone,
    steps,
    start_s,
    airborne: bool,
    current_site: LandingSite | None,
    here: LonLat,
    battery: BatteryState,
    claimed_tasks: dict[str, str],
    occupancy: dict,
    reject,
) -> SortiePlan | None:
    site = current_site
    perched_since: int | None = None
    valid = True
    for i, step in enumerate(steps):
        op = step.operation
        if step.end_s <= step.start_s:
            reject("bad_step", f"step {i} needs a positive duration")
            valid = False
            break
        if step.end_s > DEADLINE_S:
            reject(
                "deadline", f"step {i} completes at {format_clock(step.end_s / 60)}, after T+04:00"
            )
            valid = False
            break
        match op:
            case Takeoff(op_site, at):
                if airborne:
                    reject("state", f"step {i}: cannot take off while airborne")
                    valid = False
                    break
                if op_site != site:
                    reject("state", f"step {i}: takeoff from a site the drone is not perched on")
                    valid = False
                    break
                if at != geometry.position_of(op_site.point):
                    reject("geometry", f"step {i}: takeoff position contradicts the mapped path")
                    valid = False
                    break
                if perched_since is not None:
                    occupancy.setdefault(op_site, []).append(
                        (perched_since, step.start_s, drone.id)
                    )
                    perched_since = None
                here, airborne, site = at, True, None
            case Land(op_site, at):
                if not airborne:
                    reject("state", f"step {i}: cannot land while perched")
                    valid = False
                    break
                if at != geometry.position_of(op_site.point):
                    reject("geometry", f"step {i}: landing position contradicts the mapped path")
                    valid = False
                    break
                if perched_since is not None:
                    occupancy.setdefault(op_site, []).append((perched_since, 10**12, drone.id))
                    perched_since = None
                perched_since = step.start_s
                here, airborne, site = at, False, op_site
            case Scan(acquisition, task_id):
                if not airborne:
                    reject("state", f"step {i}: a scan needs an airborne drone")
                    valid = False
                    break
                error = _task_check(world, req, task_id, acquisition, claimed_tasks, i)
                if error:
                    reject(*error)
                    valid = False
                    break
                if i > 0 and here != acquisition.trajectory.poses[0].position:
                    reject(
                        "geometry",
                        f"step {i}: the scan starts where no transit delivered the drone",
                    )
                    valid = False
                    break
                here = acquisition.trajectory.poses[-1].position
            case GroundSample(acquisitions, op_site, task_ids, at):
                if airborne or op_site != site:
                    reject(
                        "state", f"step {i}: ground sampling needs the drone perched on its site"
                    )
                    valid = False
                    break
                if len(acquisitions) != len(task_ids):
                    reject("bad_step", f"step {i}: one task id per acquisition required")
                    valid = False
                    break
                for member, task_id in zip(acquisitions, task_ids, strict=True):
                    error = _task_check(world, req, task_id, member, claimed_tasks, i)
                    if error:
                        reject(*error)
                        valid = False
                        break
                if not valid:
                    break
                if at != geometry.position_of(op_site.point):
                    reject("geometry", f"step {i}: sampling position contradicts the mapped path")
                    valid = False
                    break
                here = at
            case Charge(op_site, charge_s, at):
                if airborne or op_site != site:
                    reject("state", f"step {i}: charging needs the drone perched on its site")
                    valid = False
                    break
                if charge_s <= 0:
                    reject("bad_step", f"step {i}: a charge step needs a positive duration")
                    valid = False
                    break
                ch = world.knowledge.chargeability.get(op_site.circuit_id)
                if ch is None or ch.state is not ChargeState.CONFIRMED:
                    uncertain = (
                        ch is not None
                        and ch.state is ChargeState.UNKNOWN
                        and world.landing_policy.allow_uncertain
                    )
                    if not uncertain:
                        reject(
                            "charge_unavailable",
                            f"step {i}: no confirmed charging on {op_site.circuit_id}"
                            + (f" ({ch.reason})" if ch else ""),
                        )
                        valid = False
                        break
                elif ch.expires_s is not None and ch.expires_s <= step.start_s:
                    reject(
                        "charge_unavailable",
                        f"step {i}: charging evidence expires at "
                        f"{format_clock(ch.expires_s / 60)}, before the charge starts at "
                        f"{format_clock(step.start_s / 60)}",
                    )
                    valid = False
                    break
                if at != geometry.position_of(op_site.point):
                    reject("geometry", f"step {i}: charge position contradicts the mapped path")
                    valid = False
                    break
                here = at
            case Transit(_, destination):
                if not airborne:
                    reject("state", f"step {i}: transit needs an airborne drone")
                    valid = False
                    break
                expected = leg_seconds(here, destination, drone.speed_km_per_min)
                if step.duration_s < expected:
                    reject(
                        "geometry",
                        f"step {i}: transit takes {expected} s over the mapped path, "
                        f"the plan claims {step.duration_s} s",
                    )
                    valid = False
                    break
                here, airborne = destination, True
            case Wait(op_airborne, at):
                if op_airborne != airborne:
                    reject("state", f"step {i}: wait state contradicts the drone state")
                    valid = False
                    break
                here = at
        kind = _kind(op)
        charging = _charge_observed(world, op)
        try:
            battery, _ = integrate(
                battery, kind, step.duration_s, charging_observed=charging, model=world.energy_model
            )
        except ValueError as e:
            reject("energy", f"step {i}: {e} during {kind.value}")
            valid = False
            break
    if not valid:
        return None
    if airborne:
        reject("not_perched", "the itinerary must end with the drone perched")
        return None
    if site != req.final_perch:
        reject(
            "bad_final_perch",
            f"the itinerary ends perched on {site.circuit_id} at {site.point.offset_mm} mm, "
            "not the requested final perch",
        )
        return None
    if perched_since is not None:
        occupancy.setdefault(req.final_perch, []).append(
            (perched_since, steps[-1].end_s, drone.id)
        )
    itinerary = Itinerary(tuple(steps), req.final_perch)
    return SortiePlan(req.drone_id, itinerary)


def _kind(op) -> OperationKind:
    if isinstance(op, Wait):
        return OperationKind.WAIT_AIRBORNE if op.airborne else OperationKind.WAIT_PERCHED
    return OPERATION_KIND[type(op)]


def _charge_observed(world: World, op) -> bool:
    if not isinstance(op, Charge):
        return False
    ch = world.knowledge.chargeability.get(op.site.circuit_id)
    return ch is not None and ch.state is ChargeState.CONFIRMED


def _task_check(world: World, req: SortieRequest, task_id: str, acquisition, claimed, i):
    """Method fidelity and ownership. Returns (code, detail) or None."""
    task = world.tasks.get(task_id)
    if task is None:
        return ("bad_task", f"step {i}: unknown task {task_id}")
    if task.status not in (TaskStatusV2.PENDING, TaskStatusV2.RESERVED, TaskStatusV2.MEASURING):
        return ("task_owned", f"step {i}: {task_id} is {task.status}")
    if task.status is TaskStatusV2.MEASURING and task.assigned_drone_id != req.drone_id:
        return ("task_owned", f"step {i}: {task_id} is being measured by {task.assigned_drone_id}")
    if task.status is not TaskStatusV2.MEASURING and task.assigned_drone_id not in (
        None,
        req.drone_id,
    ):
        return ("task_owned", f"step {i}: {task_id} is owned by {task.assigned_drone_id}")
    if claimed.get(task_id, req.drone_id) != req.drone_id:
        return ("task_owned", f"step {i}: {task_id} already goes to {claimed[task_id]}")
    claimed[task_id] = req.drone_id
    request = world.requests.get(task.request_id)
    if request is None or request.status is not RequestStatus.ACTIVE:
        return ("stale_request", f"step {i}: task {task_id} serves no active request")
    if task.acquisition != acquisition:
        return (
            "method_substitution",
            f"step {i}: the executed acquisition differs from task {task_id}'s compiled "
            "acquisition; the requested method may not be substituted",
        )
    return None


def apply_sorties(world: World, physics, plans: tuple[SortiePlan, ...]) -> None:
    """Commit validated plans. Call only with `Validation.plans` of an `ok` validation.

    A cut continuation finalizes the in-progress scan first (partial reading, elapsed energy)
    through a settle, then every new itinerary starts."""
    from grid_ops.world import settle

    cutting = []
    for plan in plans:
        drone = world.drones[plan.drone_id]
        st = drone.state
        if (
            isinstance(st, Executing)
            and isinstance(st.step.operation, Scan)
            and plan.itinerary.steps[0].start_s == world.clock_s
            and world.clock_s < st.step.end_s
        ):
            if world.clock_s > st.step.start_s:
                shortened = replace(st.step, end_s=world.clock_s)
                steps = (*st.itinerary.steps[: st.step_index], shortened)
                drone.state = Executing(replace(st.itinerary, steps=steps), st.step_index)
                cutting.append(drone.id)
            else:
                task = world.tasks.get(st.step.operation.task_id)
                if task is not None and task.status is TaskStatusV2.MEASURING:
                    world.set_task(
                        replace(task, status=TaskStatusV2.PENDING, assigned_drone_id=None)
                    )
    if cutting:
        settle(world, physics)
    for plan in plans:
        drone = world.drones[plan.drone_id]
        keep = {tid for step in plan.itinerary.steps for tid in _step_task_ids(step)}
        for task in list(world.tasks.values()):
            if (
                task.assigned_drone_id == drone.id
                and task.status in (TaskStatusV2.RESERVED, TaskStatusV2.MEASURING)
                and task.id not in keep
            ):
                world.set_task(replace(task, status=TaskStatusV2.PENDING, assigned_drone_id=None))
        for task_id in keep:
            task = world.tasks[task_id]
            if task.status is TaskStatusV2.PENDING:
                world.set_task(
                    replace(task, status=TaskStatusV2.RESERVED, assigned_drone_id=drone.id)
                )
        start_itinerary(world, plan.drone_id, plan.itinerary)


def _step_task_ids(step: ScheduledStep) -> tuple[str, ...]:
    match step.operation:
        case Scan(task_id=task_id):
            return (task_id,)
        case GroundSample(task_ids=task_ids):
            return task_ids
    return ()


def current_requests(world: World) -> tuple[SortieRequest, ...]:
    """The validated continuation of every executing drone, as validatable requests.

    The request starts at the current step's end with renumbered steps; the drone's own current
    step completes on its own and is not part of the request."""
    out = []
    for d in world.drones.values():
        st = d.state
        if not isinstance(st, Executing):
            continue
        remaining = st.itinerary.steps[st.step_index + 1 :]
        if not remaining:
            continue
        renumbered = tuple(replace(step, index=i) for i, step in enumerate(remaining))
        out.append(
            SortieRequest(
                d.id,
                renumbered,
                st.itinerary.final_perch,
                world.revision,
                world.energy_model.revision,
                world.capabilities.revision,
            )
        )
    return tuple(out)
