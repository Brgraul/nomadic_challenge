"""Public simulation state and exact transition processing.

Everything here is pure: `advance_to` and the itinerary functions take a World and mutate it,
with no threads or I/O. The Simulation owns the only live World and applies these functions on
its own thread. Time is integer seconds; every duration uses the one conservative rounding rule
in `leg_seconds`, shared by optimisation, validation and simulation.

A drone's authoritative schedule is its `Itinerary`: an ordered tuple of `ScheduledStep`, each
carrying one operation with resolved positions and exact bounds. Energy is committed only at
step boundaries through `energy.integrate`; a landing never adds energy and only an observed
charging handshake does. Hidden physics reaches this module only through the `Physics` protocol
(`is_chargeable`, `acquire`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import StrEnum
from itertools import pairwise
from typing import Protocol

from grid_ops import knowledge
from grid_ops.conclusion import Investigation
from grid_ops.energy import (
    DEFAULT_ENERGY_MODEL,
    BatteryState,
    EnergyModel,
    OperationKind,
    integrate,
)
from grid_ops.geometry import LandingSite, LinePoint, Site, haversine_km
from grid_ops.grid import LonLat
from grid_ops.knowledge import ChargeState, FaultAssessment, KnowledgeBook, LandingPolicy
from grid_ops.scenario import ScenarioMode, SensorModel
from grid_ops.sensing import (
    AcceptedRequest,
    AcquisitionSample,
    ContactCurrent,
    ExecutedAcquisition,
    LidarPass,
    MeasurementTaskV2,
    Reading,
    SensorCapabilities,
    TaskStatusV2,
    TdrTest,
)

DEADLINE_S = 4 * 60 * 60
# Assumption: cruise speed of 60 km/h, so a full battery (1,080,000 units) covers 30 airborne
# minutes under the normalized energy law in `energy.py`.
SPEED_KM_PER_MINUTE = 1.0
LAND_S = 30
TAKEOFF_S = 30
MAX_SETTLE_ITERATIONS = 10_000

# Assumption: starting perches, one per drone, all fully charged at T+00:00.
START_SEGMENTS = {
    "DRONE_1": "LINE_040",
    "DRONE_2": "LINE_025",
    "DRONE_3": "LINE_026",
    "DRONE_4": "LINE_002",
    "DRONE_5": "LINE_056",
}


class CommandRejected(Exception):
    pass


class LineStatus(StrEnum):
    UNKNOWN = "unknown"
    LIVE = "live"
    DE_ENERGIZED = "de_energized"
    SUSPECTED = "suspected"
    FAULT_CONFIRMED = "fault_confirmed"
    CLEARED = "cleared"


# -- scheduled operations ------------------------------------------------------------------------


@dataclass(frozen=True)
class Transit:
    origin: LonLat
    destination: LonLat


@dataclass(frozen=True)
class Scan:
    acquisition: LidarPass
    task_id: str


@dataclass(frozen=True)
class Land:
    site: LandingSite
    at: LonLat


@dataclass(frozen=True)
class Takeoff:
    site: LandingSite
    at: LonLat


@dataclass(frozen=True)
class GroundSample:
    acquisitions: tuple[ContactCurrent | TdrTest, ...]
    site: LandingSite
    task_ids: tuple[str, ...]
    at: LonLat


@dataclass(frozen=True)
class Charge:
    site: LandingSite
    charge_s: int
    at: LonLat


@dataclass(frozen=True)
class Wait:
    airborne: bool
    at: LonLat


Operation = Transit | Scan | Land | Takeoff | GroundSample | Charge | Wait

OPERATION_KIND = {
    Transit: OperationKind.TRANSIT,
    Scan: OperationKind.SCAN,
    Land: OperationKind.LAND,
    Takeoff: OperationKind.TAKEOFF,
    GroundSample: OperationKind.SAMPLE,
    Charge: OperationKind.CHARGE,
}


@dataclass(frozen=True)
class ScheduledStep:
    index: int
    start_s: int
    end_s: int
    operation: Operation

    def __post_init__(self) -> None:
        if self.end_s <= self.start_s:
            raise ValueError("a scheduled step needs a positive duration")
        if self.index < 0:
            raise ValueError("step index must be nonnegative")

    @property
    def duration_s(self) -> int:
        return self.end_s - self.start_s


@dataclass(frozen=True)
class Itinerary:
    """The single executable schedule of one drone. Steps are contiguous and monotonic."""

    steps: tuple[ScheduledStep, ...]
    final_perch: LandingSite

    def __post_init__(self) -> None:
        if not self.steps:
            raise ValueError("an itinerary needs at least one step")
        for i, step in enumerate(self.steps):
            if step.index != i:
                raise ValueError("step indices must be 0..n-1 in order")
        for a, b in pairwise(self.steps):
            if b.start_s != a.end_s:
                raise ValueError("itinerary steps must be contiguous")

    @property
    def end_s(self) -> int:
        return self.steps[-1].end_s

    def remaining(self, step_index: int) -> tuple[ScheduledStep, ...]:
        return self.steps[step_index:]


# -- drone state ---------------------------------------------------------------------------------


@dataclass(frozen=True)
class Perched:
    perch: LandingSite


@dataclass(frozen=True)
class Stranded:
    """Perched where departure is genuinely infeasible; charging alone never causes this."""

    perch: LandingSite


@dataclass(frozen=True)
class Executing:
    itinerary: Itinerary
    step_index: int

    @property
    def step(self) -> ScheduledStep:
        return self.itinerary.steps[self.step_index]


DroneState = Perched | Stranded | Executing


@dataclass
class Drone:
    id: str
    position: LonLat  # exact at world.clock_s
    battery: BatteryState
    state: DroneState
    speed_km_per_min: float = SPEED_KM_PER_MINUTE
    history: tuple[str, ...] = ()  # completed task ids, in order

    @property
    def phase(self) -> str:
        return type(self.state).__name__.lower()

    @property
    def battery_units(self) -> int:
        return self.battery.remaining_units


def leg_seconds(a: LonLat, b: LonLat, speed_km_per_min: float) -> int:
    """Conservative travel time: ceil of the exact seconds; zero only for zero distance."""
    d = haversine_km(a, b)
    if d <= 0:
        return 0
    return math.ceil(60 * d / speed_km_per_min)


def scan_duration_s(acquisition: LidarPass) -> int:
    return acquisition.trajectory.duration_s


def ground_duration_s(acquisitions: tuple[ContactCurrent | TdrTest, ...]) -> int:
    return max(
        a.integration_s if isinstance(a, ContactCurrent) else a.duration_s for a in acquisitions
    )


def _lerp(a: LonLat, b: LonLat, f: float) -> LonLat:
    return (a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f)


def _pose_position(poses, f: float) -> LonLat:
    x = f * (len(poses) - 1)
    i = min(int(x), len(poses) - 2)
    frac = x - i
    return _lerp(poses[i].position, poses[i + 1].position, frac)


def position_at(step: ScheduledStep, fraction: float) -> LonLat:
    op = step.operation
    match op:
        case Transit(origin, destination):
            return _lerp(origin, destination, fraction)
        case Scan(acquisition=acq):
            return _pose_position(acq.trajectory.poses, fraction)
        case Land(at=at) | Takeoff(at=at) | GroundSample(at=at) | Charge(at=at) | Wait(at=at):
            return at
    raise ValueError(f"unknown operation {type(op).__name__}")


# -- evidence ------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ReadingDisposition:
    """An explicit, reversible decision about how a reading is used. Readings never change."""

    reading_id: str
    action: str  # EXCLUDED | REINSTATED | EPOCH_REASSIGNED
    reason: str
    clock_s: int
    author: str
    from_epoch: str | None = None
    to_epoch: str | None = None


@dataclass(frozen=True)
class Event:
    clock_s: int
    kind: str
    drone_id: str | None
    detail: str


@dataclass(frozen=True)
class LogEntry:
    clock: float
    author: str
    text: str


@dataclass(frozen=True)
class RejectionRecord:
    clock_s: int
    author: str
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class ScheduledReport:
    """A public switching report that becomes known at a scripted time."""

    clock_s: int
    segment_id: str
    text: str


class Physics(Protocol):
    """Hidden physical truth (`grid_ops.truth`): answers acquisitions, exposes nothing."""

    def is_chargeable(self, site: LandingSite) -> bool: ...

    def acquire(self, executed: ExecutedAcquisition) -> AcquisitionSample: ...


@dataclass
class World:
    clock_s: int
    drones: dict[str, Drone]
    belief: dict[str, LineStatus]
    sites: dict[str, Site] = field(default_factory=dict)
    tasks: dict[str, MeasurementTaskV2] = field(default_factory=dict)
    readings: dict[str, Reading] = field(default_factory=dict)
    requests: dict[str, AcceptedRequest] = field(default_factory=dict)
    requests_version: int = 0
    rejections: list[RejectionRecord] = field(default_factory=list)
    log: list[LogEntry] = field(default_factory=list)
    next_task_number: int = 1
    next_acquisition_number: int = 1
    scenario_mode: ScenarioMode = ScenarioMode.INCIDENT_DEMO
    scenario_seed: int = 0
    brief: tuple[str, ...] = ()
    sensor_model: SensorModel = field(default_factory=SensorModel)
    capabilities: SensorCapabilities = field(default_factory=SensorCapabilities)
    energy_model: EnergyModel = DEFAULT_ENERGY_MODEL
    circuits_by_segment: dict = field(default_factory=dict)
    known_switching: dict[str, int] = field(default_factory=dict)  # segment -> reported at (s)
    pending_reports: list[ScheduledReport] = field(default_factory=list)
    knowledge: KnowledgeBook = field(default_factory=KnowledgeBook)
    landing_policy: LandingPolicy = field(default_factory=LandingPolicy)
    hazards: list[FaultAssessment] = field(default_factory=list)
    coverage: list = field(default_factory=list)  # lidar coverage records
    interpretations: dict = field(default_factory=dict)  # reading id -> lidar interpretation
    segment_lengths: dict[str, int] = field(default_factory=dict)
    estimate: object | None = None  # historical field; the three-method runtime never fills it
    estimation_solves: int = 0
    estimation_log: list[str] = field(default_factory=list)
    excluded_readings: set[str] = field(default_factory=set)
    epoch_overrides: dict[str, str] = field(default_factory=dict)
    dispositions: list = field(default_factory=list)  # ReadingDisposition history
    consistency: object | None = None  # historical field; the three-method runtime never fills it
    envelope: object | None = None  # historical field; the three-method runtime never fills it
    revision: int = 0  # increments each time a new evidence snapshot is published
    events: list = field(default_factory=list)  # (event id, Event) of every observable event
    operator_events: list = field(default_factory=list)  # (id, text) of operator decisions
    hypotheses: dict = field(default_factory=dict)  # accepted hypotheses
    proposals: list = field(default_factory=list)  # ProposedFinding records
    hypothesis_revisions: list = field(default_factory=list)  # accepted HypothesisRevisionRecord
    hypotheses_version: int = 0
    manager_turns: list = field(default_factory=list)
    accepted_out_of_service: frozenset = frozenset()  # accepted model: segments assumed open
    staged_models: dict = field(default_factory=dict)  # requested candidates, never active
    hold: object | None = None  # decisions.Hold while an acceptance cycle is open
    next_cycle: int = 1
    decision_outcomes: dict = field(default_factory=dict)  # decision id -> DecisionOutcome
    needs: dict = field(default_factory=dict)  # request/task lifecycle records
    unavailable_work: list = field(default_factory=list)
    task_log: list = field(default_factory=list)
    score_preview: list = field(default_factory=list)  # per-task standalone score rows
    route_passes: list = field(default_factory=list)  # fleet.RoutePass history
    coordinator_turns: list = field(default_factory=list)  # async manager turns (latest state each)
    planner_previews: list = field(default_factory=list)  # previews that authorise nothing
    investigation: Investigation = field(default_factory=Investigation)

    @property
    def clock(self) -> float:
        """Simulation time in minutes."""
        return self.clock_s / 60

    @property
    def finished(self) -> bool:
        return self.clock_s >= DEADLINE_S

    def record(self, author: str, text: str) -> None:
        self.log.append(LogEntry(self.clock, author, text))

    def footprint_of(self, point: LinePoint):
        """Nominal inspection footprint of a stop, clipped to its segment."""
        from grid_ops.geometry import INSPECTION_RANGE_KM, MM_PER_KM, Interval

        half = round(INSPECTION_RANGE_KM * MM_PER_KM)
        length = self.segment_lengths[point.segment_id]
        return Interval(
            point.segment_id, max(0, point.offset_mm - half), min(length, point.offset_mm + half)
        )

    def set_task(self, task: MeasurementTaskV2) -> None:
        self.tasks[task.id] = task


def format_clock(minutes: float) -> str:
    total = int(minutes * 60 + 1e-6)
    return f"T+{total // 3600:02d}:{total % 3600 // 60:02d}"


def distance_km(a: LonLat, b: LonLat) -> float:
    return haversine_km(a, b)


def initial_world(network, geometry) -> World:
    drones = {}
    for drone_id, seg_id in START_SEGMENTS.items():
        point = LinePoint(seg_id, geometry.length_mm(seg_id) // 2)
        perch = LandingSite(point, f"{seg_id}/C1")
        drones[drone_id] = Drone(
            drone_id, geometry.position_of(point), BatteryState(), Perched(perch)
        )
    belief = dict.fromkeys(network.segments, LineStatus.UNKNOWN)
    return World(clock_s=0, drones=drones, belief=belief)


def seed_starting_handshakes(world: World) -> None:
    """Each starting perch has an explicit successful-handshake observation at T+00:00."""
    for d in world.drones.values():
        perch = d.state.perch
        knowledge.ingest_handshake(world, perch.circuit_id, True, d.id, charging=False)


# -- time advance ---------------------------------------------------------------------------------


def next_transition(world: World) -> int | None:
    times = []
    for drone in world.drones.values():
        match drone.state:
            case Executing(itinerary=it, step_index=i):
                times.append(it.steps[i].end_s)
    times += [r.clock_s for r in world.pending_reports]
    expiry = knowledge.next_expiry(world)
    if expiry is not None:
        times.append(expiry)
    return min(times) if times else None


def _move_to(world: World, t: int) -> None:
    for drone in world.drones.values():
        st = drone.state
        if isinstance(st, Executing):
            step = st.step
            span = step.end_s - step.start_s
            f = min(max((t - step.start_s) / span, 0.0), 1.0) if span else 1.0
            drone.position = position_at(step, f)
    world.clock_s = t


def advance_to(
    world: World, physics: Physics, until_s: int, *, stop_on_event: bool = False
) -> list[Event]:
    """Advance exactly to the earliest transition each step, processing every due transition to a
    fixed point. Returns the observable events; with `stop_on_event`, returns right after the
    first timestamp that produced any."""
    until_s = min(until_s, DEADLINE_S)
    events: list[Event] = []
    while True:
        events += settle(world, physics)
        if stop_on_event and events:
            return events
        nxt = next_transition(world)
        if nxt is None or nxt > until_s:
            if until_s > world.clock_s:
                _move_to(world, until_s)
            return events
        _move_to(world, nxt)


def advance(world: World, physics: Physics, minutes: float) -> list[Event]:
    return advance_to(world, physics, world.clock_s + round(minutes * 60))


def settle(world: World, physics: Physics) -> list[Event]:
    """Process all transitions due at world.clock_s, in the declared phase order, to a fixed
    point. A cycle that makes no progress is a bug, not an infinite loop."""
    events: list[Event] = []
    for _ in range(MAX_SETTLE_ITERATIONS):
        changed = (
            _publish_reports(world, events)
            | _advance_drones(world, physics, events)
            | _expire_evidence(world, events)
        )
        if not changed:
            return events
    raise RuntimeError("transition cycle made no progress")


def start_itinerary(world: World, drone_id: str, itinerary: Itinerary) -> None:
    """Put a drone on its itinerary and mark its first acquisition step in progress."""
    drone = world.drones[drone_id]
    drone.state = Executing(itinerary, 0)
    _mark_step_tasks(world, itinerary.steps[0], TaskStatusV2.MEASURING)


def _mark_step_tasks(world: World, step: ScheduledStep, status: TaskStatusV2) -> None:
    for task_id in _step_task_ids(step):
        task = world.tasks.get(task_id)
        if task is not None and task.status in (TaskStatusV2.PENDING, TaskStatusV2.RESERVED):
            world.set_task(replace(task, status=status))


def _step_task_ids(step: ScheduledStep) -> tuple[str, ...]:
    match step.operation:
        case Scan(task_id=task_id):
            return (task_id,)
        case GroundSample(task_ids=task_ids):
            return task_ids
    return ()


def _drones(world: World):
    return sorted(world.drones.values(), key=lambda d: d.id)


def _publish_reports(world: World, events: list[Event]) -> bool:
    due = [r for r in world.pending_reports if r.clock_s <= world.clock_s]
    for r in sorted(due, key=lambda r: (r.clock_s, r.segment_id)):
        world.pending_reports.remove(r)
        world.known_switching[r.segment_id] = r.clock_s
        world.record("report", r.text)
        knowledge.ingest_report(world, r.segment_id, r.clock_s, r.text)
        events.append(Event(world.clock_s, "switching_report", None, r.segment_id))
    return bool(due)


def _expire_evidence(world: World, events: list[Event]) -> bool:
    book = world.knowledge
    due = [
        e
        for e in book.evidence.values()
        if e.expires_s is not None
        and e.expires_s <= world.clock_s
        and e.id not in book.expired_notified
        and not e.renewed_while_charging
    ]
    if not due:
        return False
    for e in sorted(due, key=lambda e: e.id):
        book.expired_notified.add(e.id)
        events.append(Event(world.clock_s, "evidence_expired", None, e.id))
    knowledge.recompute(world)
    world.record(
        "evidence", f"{len(due)} evidence item(s) expired; circuit eligibility reassessed."
    )
    return True


def _commit_energy(world: World, drone: Drone, step: ScheduledStep) -> None:
    op = step.operation
    if isinstance(op, Wait):
        kind = OperationKind.WAIT_AIRBORNE if op.airborne else OperationKind.WAIT_PERCHED
    else:
        kind = OPERATION_KIND[type(op)]
    if isinstance(op, Charge):
        charge = world.knowledge.chargeability.get(op.site.circuit_id)
        observed = charge is not None and charge.state is ChargeState.CONFIRMED
        try:
            drone.battery, _ = integrate(
                drone.battery,
                kind,
                step.duration_s,
                charging_observed=observed,
                model=world.energy_model,
            )
        except ValueError:
            raise RuntimeError(
                f"{drone.id} cannot sustain the ground load while charging on "
                f"{op.site.circuit_id}; route validation failed"
            ) from None
        return
    try:
        drone.battery, _ = integrate(drone.battery, kind, step.duration_s, model=world.energy_model)
    except ValueError:
        raise RuntimeError(
            f"{drone.id} energy exhausted during {kind.value}; route validation failed"
        ) from None


def _advance_drones(world: World, physics: Physics, events: list[Event]) -> bool:
    changed = False
    for drone in _drones(world):
        while isinstance(drone.state, Executing):
            st = drone.state
            step = st.step
            if step.end_s > world.clock_s:
                break
            _complete_step(world, physics, drone, st, step, events)
            changed = True
    return changed


def _complete_step(
    world: World, physics: Physics, drone: Drone, st: Executing, step: ScheduledStep, events
) -> None:
    op = step.operation
    _commit_energy(world, drone, step)
    itinerary = st.itinerary
    last = st.step_index + 1 >= len(itinerary.steps)
    match op:
        case Charge(site):
            charge = world.knowledge.chargeability.get(site.circuit_id)
            observed = charge is not None and charge.state is ChargeState.CONFIRMED
            if observed:
                events.append(Event(world.clock_s, "charged", drone.id, site.circuit_id))
                world.record(drone.id, f"Charged on {site.circuit_id}.")
            else:
                events.append(Event(world.clock_s, "charge_failed", drone.id, site.circuit_id))
                world.record(
                    drone.id, f"Charging unavailable on {site.circuit_id}; staying perched."
                )
                drone.state = Perched(site)
                return
        case Scan(acquisition=acquisition, task_id=task_id):
            _publish_acquisition(world, physics, drone, step, acquisition, task_id, None, events)
        case GroundSample(acquisitions=acquisitions, site=site, task_ids=task_ids):
            for acquisition, task_id in zip(acquisitions, task_ids, strict=True):
                landed = getattr(acquisition, "conductor", None)
                _publish_acquisition(
                    world, physics, drone, step, acquisition, task_id, landed, events
                )
        case Land(site):
            handshake = physics.is_chargeable(site)
            nxt = itinerary.steps[st.step_index + 1] if not last else None
            charge_next = (
                nxt is not None
                and isinstance(nxt.operation, Charge)
                and nxt.operation.site == site
            )
            knowledge.ingest_handshake(
                world, site.circuit_id, handshake, drone.id, charging=charge_next
            )
            detail = f"{site.circuit_id} handshake {'succeeded' if handshake else 'failed'}"
            events.append(Event(world.clock_s, "landed", drone.id, detail))
            world.record(drone.id, f"Landed on {site.circuit_id}; {detail}.")
        case Takeoff(site):
            world.record(drone.id, f"Took off from {site.circuit_id}.")
        case Transit():
            pass
        case Wait():
            pass
    if last:
        drone.state = Perched(itinerary.final_perch)
    else:
        nxt = itinerary.steps[st.step_index + 1]
        drone.state = Executing(itinerary, st.step_index + 1)
        _mark_step_tasks(world, nxt, TaskStatusV2.MEASURING)


def _payload_summary(payload) -> str:
    from grid_ops.sensing import CurrentPayload, LidarPayload, TdrPayload

    match payload:
        case LidarPayload():
            covered = sum(i.end_mm - i.start_mm for i in payload.inspected)
            clear = "none" if payload.clearance_m is None else f"{payload.clearance_m:.2f} m"
            return f"inspected {covered / 1e6:.3f} km, min clearance {clear}"
        case TdrPayload():
            return f"{len(payload.echoes)} echo(es), coupling {payload.coupling_quality}"
        case CurrentPayload():
            return f"rms {payload.rms_a:.1f} A +/- {payload.uncertainty_a:.1f}"
    return "sample"


def _publish_acquisition(
    world: World,
    physics: Physics,
    drone: Drone,
    step: ScheduledStep,
    acquisition,
    task_id: str,
    landed_conductor,
    events,
) -> None:
    task = world.tasks.get(task_id)
    request_ids = (task.request_id,) if task is not None else ()
    duration = (
        acquisition.trajectory.duration_s
        if isinstance(acquisition, LidarPass)
        else (
            acquisition.integration_s
            if isinstance(acquisition, ContactCurrent)
            else acquisition.duration_s
        )
    )
    fraction = 1.0 if duration <= 0 else min(1.0, max(0.0, step.duration_s / duration))
    acq_id = f"ACQ{world.next_acquisition_number:04d}"
    world.next_acquisition_number += 1
    executed = ExecutedAcquisition(
        acq_id,
        acquisition,
        step.start_s,
        step.end_s,
        drone.id,
        landed_conductor=landed_conductor,
        request_ids=request_ids,
        valid_isolation_evidence_ids=getattr(acquisition, "isolation_evidence_ids", ()),
        completed_fraction=fraction,
    )
    sample = physics.acquire(executed)
    reading_id = f"R{len(world.readings) + 1:04d}"
    reading = Reading(
        id=reading_id,
        task_id=task_id,
        request_ids=request_ids,
        drone_id=drone.id,
        acquisition=acquisition,
        acquired_start_s=step.start_s,
        acquired_end_s=step.end_s,
        payload=sample.payload,
        quality=sample.quality,
        sensor_model_revision=sample.sensor_model_revision,
    )
    world.readings[reading_id] = reading
    if task is not None:
        world.set_task(replace(task, status=TaskStatusV2.COMPLETED, reading_id=reading_id))
    drone.history += (task_id,)
    method = acquisition.method.value if hasattr(acquisition.method, "value") else "acquisition"
    world.record(drone.id, f"Reading {reading_id} ({method}): {_payload_summary(sample.payload)}")
    events.append(Event(world.clock_s, "reading", drone.id, reading_id))
