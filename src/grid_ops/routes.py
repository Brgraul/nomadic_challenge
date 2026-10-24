"""Sortie validation and atomic application.

`validate_sorties` is the independent acceptance check: it recomputes every route from
authoritative geometry and live drone state (never from a solver's or caller's claimed times),
checks every drone in the request, and returns either full plans or every rejection reason.
`apply_sorties` mutates the World only with plans that validated, so a bad later sortie leaves
every earlier drone untouched.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from grid_ops import knowledge
from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import Geometry, LandingSite, LinePoint
from grid_ops.grid import LonLat
from grid_ops.world import (
    DEADLINE_S,
    DWELL_S,
    Charging,
    Drone,
    Flying,
    Measurement,
    MeasurementTask,
    Measuring,
    Ready,
    Route,
    Stop,
    Stranded,
    TaskStatus,
    World,
    format_clock,
    leg_seconds,
)


@dataclass(frozen=True)
class StopRequest:
    point: LinePoint
    kind: Measurement
    purpose: str = "operator measurement"
    repeat_of_reading_id: str | None = None
    task_id: str | None = None  # reuse an existing unowned or already-owned attempt


@dataclass(frozen=True)
class SortieRequest:
    drone_id: str
    stops: tuple[StopRequest, ...]
    landing: LandingSite


@dataclass(frozen=True)
class StopTiming:
    arrive_s: int
    done_s: int


@dataclass(frozen=True)
class SortiePlan:
    drone_id: str
    stops: tuple[Stop, ...]
    new_tasks: tuple[MeasurementTask, ...]
    reuse_task_ids: tuple[str, ...]
    timings: tuple[StopTiming, ...]
    landing: LandingSite
    landing_position: LonLat
    landing_arrive_s: int
    start_s: int
    battery_left_s: int
    locked_dwell_s: int
    route: Route
    first_leg_s: int


@dataclass(frozen=True)
class Rejection:
    drone_id: str | None
    code: str
    detail: str

    def __str__(self) -> str:
        who = f"{self.drone_id}: " if self.drone_id else ""
        return f"{who}{self.code}: {self.detail}"


@dataclass(frozen=True)
class Validation:
    plans: tuple[SortiePlan, ...]
    rejections: tuple[Rejection, ...]

    @property
    def ok(self) -> bool:
        return not self.rejections


def _start_state(drone: Drone, now_s: int) -> tuple[LonLat, int, int, int] | str:
    """(position, start_s, battery_s, locked_dwell_s) for the part of the route still to plan."""
    match drone.state:
        case Ready():
            return drone.position, now_s, drone.battery_s, 0
        case Flying():
            return drone.position, now_s, drone.battery_s, 0
        case Measuring(stop=stop, until_s=until_s):
            locked = until_s - now_s
            return stop.position, until_s, drone.battery_s - locked, locked
        case Charging() | Stranded():
            return f"{drone.phase} drones keep their perch and cannot take a sortie"
    return "unknown state"


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
    # exact perches that someone will still occupy after this batch
    occupied: dict[LandingSite, str] = {}
    for d in world.drones.values():
        match d.state:
            case Charging(perch=p) | Stranded(perch=p):
                occupied[p] = d.id
    planned_landings: dict[LandingSite, str] = {}
    next_number = world.next_task_number

    for req in requests:
        reject = lambda code, detail, drone_id=req.drone_id: rejections.append(  # noqa: E731
            Rejection(drone_id, code, detail)
        )
        drone = world.drones.get(req.drone_id)
        if drone is None:
            reject("unknown_drone", f"{req.drone_id} does not exist")
            continue
        if req.drone_id in seen_drones:
            reject("duplicate_drone", "more than one sortie for the same drone in one decision")
            continue
        seen_drones.add(req.drone_id)
        start = _start_state(drone, world.clock_s)
        if isinstance(start, str):
            reject("drone_unavailable", start)
            continue
        position, start_s, battery_s, locked_dwell_s = start

        # landing
        landing = req.landing
        landing_position: LonLat | None = None
        try:
            landing_position = geometry.position_of(landing.point)
        except ValueError as e:
            reject("bad_landing", str(e))
        circuit = circuits.circuits.get(landing.circuit_id)
        if circuit is None or circuit.segment_id != landing.point.segment_id:
            reject(
                "bad_landing", f"circuit {landing.circuit_id} is not on {landing.point.segment_id}"
            )
        holder = occupied.get(landing)
        if holder is not None and holder != req.drone_id:
            reject(
                "landing_occupied",
                f"{landing.circuit_id} at {landing.point.offset_km:.3f} km is occupied by {holder}",
            )
        other = planned_landings.get(landing)
        if other is not None and other != req.drone_id:
            reject("landing_occupied", f"{other} already plans to land at that exact site")
        for other_drone in world.drones.values():
            st = other_drone.state
            if other_drone.id != req.drone_id and isinstance(st, Ready) and st.perch == landing:
                reject(
                    "landing_occupied", f"{other_drone.id} is perched there; handover is not proved"
                )
            if (
                other_drone.id != req.drone_id
                and isinstance(st, Flying | Measuring)
                and st.route.perch == landing
            ):
                reject("landing_occupied", f"{other_drone.id} is already routed to land there")
        if isinstance(drone.state, Ready) and not req.stops and landing == drone.state.perch:
            reject("no_flight", "a Ready drone waits at its perch instead of flying nowhere")
        planned_landings[landing] = req.drone_id

        # stops
        stops: list[Stop] = []
        new_tasks: list[MeasurementTask] = []
        reuse: list[str] = []
        timings: list[StopTiming] = []
        t = start_s
        here = position
        valid_route = landing_position is not None
        first_leg = 0
        for i, sr in enumerate(req.stops):
            try:
                pos = geometry.position_of(sr.point)
            except ValueError as e:
                reject("bad_stop", f"stop {i + 1}: {e}")
                valid_route = False
                continue
            if sr.kind not in drone.sensors:
                reject(
                    "sensor_capability", f"stop {i + 1}: {req.drone_id} cannot measure {sr.kind}"
                )
                valid_route = False
            if sr.repeat_of_reading_id is not None:
                if sr.repeat_of_reading_id not in world.readings:
                    reject("bad_repeat", f"stop {i + 1}: unknown reading {sr.repeat_of_reading_id}")
                    valid_route = False
                if not sr.purpose.strip():
                    reject("bad_repeat", f"stop {i + 1}: a repeat must state its purpose")
                    valid_route = False
            if sr.task_id is not None:
                existing = world.tasks.get(sr.task_id)
                if existing is None:
                    reject("bad_task", f"stop {i + 1}: unknown task {sr.task_id}")
                    valid_route = False
                elif existing.status not in (TaskStatus.PENDING, TaskStatus.RESERVED) or (
                    existing.assigned_drone_id not in (None, req.drone_id)
                ):
                    reject(
                        "task_owned",
                        f"stop {i + 1}: {sr.task_id} is {existing.status}"
                        f" and owned by {existing.assigned_drone_id}",
                    )
                    valid_route = False
                elif sr.task_id in claimed_tasks:
                    reject(
                        "task_owned",
                        f"stop {i + 1}: {sr.task_id} already goes to {claimed_tasks[sr.task_id]}",
                    )
                    valid_route = False
                else:
                    claimed_tasks[sr.task_id] = req.drone_id
                    reuse.append(sr.task_id)
            leg = leg_seconds(here, pos, drone.speed_km_per_min)
            if i == 0:
                first_leg = leg
            arrive = t + leg
            done = arrive + DWELL_S[sr.kind]
            if done > DEADLINE_S:
                reject(
                    "deadline",
                    f"stop {i + 1} completes at {format_clock(done / 60)}, after T+04:00",
                )
                valid_route = False
            timings.append(StopTiming(arrive, done))
            if sr.task_id is None:
                task_id = f"T{next_number:04d}"
                next_number += 1
            else:
                task_id = sr.task_id
            site = geometry.site(sr.point)
            if sr.task_id is None:
                new_tasks.append(
                    MeasurementTask(
                        id=task_id,
                        site_id=site.id,
                        point=sr.point,
                        kind=sr.kind,
                        purpose=sr.purpose,
                        status=TaskStatus.RESERVED,
                        assigned_drone_id=req.drone_id,
                        repeat_of_reading_id=sr.repeat_of_reading_id,
                    )
                )
            stops.append(Stop(task_id, sr.point, pos, sr.kind, DWELL_S[sr.kind]))
            t, here = done, pos

        if landing_position is None:
            continue
        last_leg = leg_seconds(here, landing_position, drone.speed_km_per_min)
        if not req.stops:
            first_leg = last_leg
        landing_arrive = t + last_leg
        if landing_arrive > DEADLINE_S:
            reject("deadline", f"landing at {format_clock(landing_arrive / 60)} is after T+04:00")
            valid_route = False
        left = battery_s - (landing_arrive - start_s)
        if left < 0:
            reject(
                "battery",
                f"needs {(landing_arrive - start_s) / 60:.1f} min of airtime, "
                f"{battery_s / 60:.1f} min available",
            )
            valid_route = False
        if landing_position is not None:
            verdict = knowledge.assess_landing(world, landing, landing_arrive)
            if not verdict.eligible:
                reject(
                    "landing_ineligible",
                    f"{landing.circuit_id} under {verdict.policy}: " + "; ".join(verdict.reasons),
                )
                valid_route = False
        if not valid_route:
            continue
        plans.append(
            SortiePlan(
                drone_id=req.drone_id,
                stops=tuple(stops),
                new_tasks=tuple(new_tasks),
                reuse_task_ids=tuple(reuse),
                timings=tuple(timings),
                landing=landing,
                landing_position=landing_position,
                landing_arrive_s=landing_arrive,
                start_s=start_s,
                battery_left_s=left,
                locked_dwell_s=locked_dwell_s,
                route=Route(tuple(stops), landing, landing_position),
                first_leg_s=first_leg,
            )
        )
    if rejections:
        return Validation((), tuple(rejections))
    return Validation(tuple(plans), ())


def apply_sorties(world: World, plans: tuple[SortiePlan, ...]) -> None:
    """Commit validated plans. Call only with `Validation.plans` of an `ok` validation."""
    for plan in plans:
        drone = world.drones[plan.drone_id]
        new_ids = {t.id for t in plan.new_tasks}
        keep = {s.task_id for s in plan.stops}
        # release unstarted reservations this drone no longer plans to fly
        for task in list(world.tasks.values()):
            if (
                task.assigned_drone_id == drone.id
                and task.status is TaskStatus.RESERVED
                and task.id not in keep
            ):
                world.set_task(replace(task, status=TaskStatus.PENDING, assigned_drone_id=None))
        for task in plan.new_tasks:
            world.set_task(task)
        world.next_task_number += len(new_ids)
        for task_id in keep - new_ids:
            world.set_task(
                replace(
                    world.tasks[task_id], status=TaskStatus.RESERVED, assigned_drone_id=drone.id
                )
            )
        match drone.state:
            case Measuring(stop=stop, started_s=started, until_s=until):
                drone.state = Measuring(stop, started, until, plan.route)
            case _:
                drone.state = Flying(
                    drone.position,
                    world.clock_s,
                    world.clock_s + plan.first_leg_s,
                    plan.route,
                )
