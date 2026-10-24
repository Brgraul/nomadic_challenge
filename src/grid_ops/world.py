"""Public simulation state and exact transition processing.

Everything here is pure: `advance_to` and the route functions take a World and mutate it, with no
threads or I/O. The Simulation owns the only live World and applies these functions on its own
thread. Time is integer seconds; every duration uses the one conservative rounding rule in
`leg_seconds`, shared by optimisation, validation and simulation.

Hidden physics reaches this module only through the `Physics` protocol.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import StrEnum
from typing import Protocol

from grid_ops import assessment, knowledge
from grid_ops.conclusion import Investigation
from grid_ops.geometry import LandingSite, LinePoint, Site, haversine_km
from grid_ops.grid import LonLat
from grid_ops.knowledge import FaultAssessment, KnowledgeBook, LandingPolicy
from grid_ops.scenario import Covariance, ScenarioMode, SensorModel

FLIGHT_S = 30 * 60
CHARGE_S = 3 * 60 * 60
DEADLINE_S = 4 * 60 * 60
# Assumption: cruise speed of 60 km/h, so a full flight covers 30 km.
SPEED_KM_PER_MINUTE = 1.0
FLIGHT_MINUTES = FLIGHT_S / 60
CHARGE_MINUTES = CHARGE_S / 60
MAX_SETTLE_ITERATIONS = 10_000

# Assumption: starting perches, one per drone, all fully charged at T+00:00.
START_SEGMENTS = {
    "DRONE_1": "LINE_040",
    "DRONE_2": "LINE_025",
    "DRONE_3": "LINE_026",
    "DRONE_4": "LINE_002",
    "DRONE_5": "LINE_056",
}


class Measurement(StrEnum):
    VOLTAGE_PRESENCE = "voltage_presence"  # E-field: is the circuit live?
    CURRENT = "current"
    CONDUCTOR_TEMP = "conductor_temp"  # hot joints, only visible on a loaded line
    CLEARANCE = "clearance"  # sag / vegetation distance
    MOTION = "motion"  # galloping, clashing


DWELL_MINUTES = {
    Measurement.VOLTAGE_PRESENCE: 1,
    Measurement.CURRENT: 2,
    Measurement.CONDUCTOR_TEMP: 3,
    Measurement.CLEARANCE: 3,
    Measurement.MOTION: 4,
}
DWELL_S = {kind: minutes * 60 for kind, minutes in DWELL_MINUTES.items()}


class LineStatus(StrEnum):
    UNKNOWN = "unknown"
    LIVE = "live"
    DE_ENERGIZED = "de_energized"
    SUSPECTED = "suspected"
    FAULT_CONFIRMED = "fault_confirmed"
    CLEARED = "cleared"


class TaskStatus(StrEnum):
    PENDING = "pending"
    RESERVED = "reserved"
    MEASURING = "measuring"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAILED = "failed"


@dataclass(frozen=True)
class SensorSample:
    """What the hidden physics returns for one observation: numbers only, no identities."""

    kind: Measurement
    values: tuple[tuple[str, float], ...]
    conditions: tuple[tuple[str, float], ...] = ()  # visibility, excitation, gust flags
    inspected: tuple[tuple[int, int], ...] = ()  # actual inspected sub-intervals, mm


def observation_identity(kind: Measurement, point: LinePoint) -> str:
    """Canonical observation identity: electrical quantities are one observation per segment
    aggregate whatever the offset; local readings keep their exact site."""
    if kind in (Measurement.VOLTAGE_PRESENCE, Measurement.CURRENT):
        return f"electrical:{point.segment_id}"
    return f"local:{point.segment_id}@{point.offset_mm}mm"


class Physics(Protocol):
    """Hidden physical truth (`grid_ops.truth`): answers questions, exposes nothing."""

    def is_chargeable(self, site: LandingSite) -> bool: ...

    def sense(
        self, point: LinePoint, kind: Measurement, clock_s: int, identity: str, repeat: int
    ) -> SensorSample: ...


class CommandRejected(Exception):
    pass


# -- drone state ------------------------------------------------------------------------------


@dataclass(frozen=True)
class Stop:
    task_id: str
    point: LinePoint
    position: LonLat  # resolved once from authoritative geometry
    kind: Measurement
    dwell_s: int


@dataclass(frozen=True)
class Route:
    stops: tuple[Stop, ...]  # task stops still to visit, in order
    perch: LandingSite
    perch_position: LonLat

    @property
    def target(self) -> LonLat:
        return self.stops[0].position if self.stops else self.perch_position


@dataclass(frozen=True)
class Ready:
    perch: LandingSite


@dataclass(frozen=True)
class Flying:
    origin: LonLat
    departed_s: int
    arrives_s: int
    route: Route


@dataclass(frozen=True)
class Measuring:
    stop: Stop
    started_s: int
    until_s: int
    route: Route  # what comes after this stop


@dataclass(frozen=True)
class Charging:
    perch: LandingSite
    until_s: int


@dataclass(frozen=True)
class Stranded:
    """Landed where charging power is unavailable; charging starts over if it returns."""

    perch: LandingSite


DroneState = Ready | Flying | Measuring | Charging | Stranded
ALL_SENSORS = frozenset(Measurement)


@dataclass
class Drone:
    id: str
    position: LonLat  # exact at world.clock_s
    battery_s: int  # airtime left, exact at world.clock_s
    state: DroneState
    speed_km_per_min: float = SPEED_KM_PER_MINUTE
    sensors: frozenset[Measurement] = ALL_SENSORS
    history: tuple[str, ...] = ()  # completed task ids, in order

    @property
    def phase(self) -> str:
        return type(self.state).__name__.lower()

    @property
    def battery_min(self) -> float:
        return self.battery_s / 60


# -- evidence ---------------------------------------------------------------------------------


@dataclass(frozen=True)
class MeasurementTask:
    """One attempt. A repeat is a new attempt linked to the earlier evidence."""

    id: str
    site_id: str
    point: LinePoint
    kind: Measurement
    purpose: str
    status: TaskStatus
    assigned_drone_id: str | None = None
    repeat_of_reading_id: str | None = None
    urgency: int = 3
    reading_id: str | None = None
    note: str = ""
    need_ids: tuple[str, ...] = ()  # the active needs that justify this attempt
    group_id: str = ""  # canonical electrical group, or exact local site identity


@dataclass(frozen=True)
class Reading:
    """Immutable observation. Numbers are authoritative; `summary` is for humans only."""

    id: str
    task_id: str
    site_id: str
    drone_id: str
    point: LinePoint
    kind: Measurement
    clock_s: int
    operating_epoch: str
    payload: tuple[tuple[str, float], ...]
    sensor_model_revision: str
    summary: str
    covariance: Covariance | None = None
    conditions: tuple[tuple[str, float], ...] = ()
    inspected: tuple[tuple[int, int], ...] = ()
    repeat_index: int = 0


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


@dataclass
class World:
    clock_s: int
    drones: dict[str, Drone]
    belief: dict[str, LineStatus]
    sites: dict[str, Site] = field(default_factory=dict)
    tasks: dict[str, MeasurementTask] = field(default_factory=dict)
    readings: dict[str, Reading] = field(default_factory=dict)
    rejections: list[RejectionRecord] = field(default_factory=list)
    log: list[LogEntry] = field(default_factory=list)
    next_task_number: int = 1
    scenario_mode: ScenarioMode = ScenarioMode.INCIDENT_DEMO
    scenario_seed: int = 0
    brief: tuple[str, ...] = ()
    sensor_model: SensorModel = field(default_factory=SensorModel)
    circuits_by_segment: dict = field(default_factory=dict)
    known_switching: dict[str, int] = field(default_factory=dict)  # segment -> reported at (s)
    pending_reports: list[ScheduledReport] = field(default_factory=list)
    knowledge: KnowledgeBook = field(default_factory=KnowledgeBook)
    landing_policy: LandingPolicy = field(default_factory=LandingPolicy)
    hazards: list[FaultAssessment] = field(default_factory=list)
    coverage: list = field(default_factory=list)  # InspectionCoverage records
    interpretations: dict = field(default_factory=dict)  # reading id -> ReadingInterpretation
    segment_lengths: dict[str, int] = field(default_factory=dict)
    estimate: object | None = None  # GridEstimate | EstimateUnavailable (public, reduced DC)
    estimation_solves: int = 0
    estimation_log: list[str] = field(default_factory=list)
    excluded_readings: set[str] = field(default_factory=set)
    epoch_overrides: dict[str, str] = field(default_factory=dict)
    dispositions: list = field(default_factory=list)  # ReadingDisposition history
    consistency: object | None = None  # ConsistencyReport
    envelope: object | None = None  # DiscrepancyEnvelope (public, versioned data)
    revision: int = 0  # increments each time a new evidence snapshot is published
    events: list = field(default_factory=list)  # (event id, Event) of every observable event
    operator_events: list = field(default_factory=list)  # (id, text) of operator decisions
    hypotheses: dict = field(default_factory=dict)  # accepted hypotheses (ticket 10)
    proposals: list = field(default_factory=list)  # ProposedFinding records
    hypothesis_revisions: list = field(default_factory=list)  # accepted HypothesisRevisionRecord
    hypotheses_version: int = 0
    manager_turns: list = field(default_factory=list)
    accepted_out_of_service: frozenset = frozenset()  # accepted model: segments assumed open
    staged_models: dict = field(default_factory=dict)  # requested candidates, never active
    hold: object | None = None  # decisions.Hold while an acceptance cycle is open
    next_cycle: int = 1
    decision_outcomes: dict = field(default_factory=dict)  # decision id -> DecisionOutcome
    needs: dict = field(default_factory=dict)  # MeasurementNeed by id (active and retired)
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

    def set_task(self, task: MeasurementTask) -> None:
        self.tasks[task.id] = task


def format_clock(minutes: float) -> str:
    total = int(minutes * 60 + 1e-6)
    return f"T+{total // 3600:02d}:{total % 3600 // 60:02d}"


def distance_km(a: LonLat, b: LonLat) -> float:
    return haversine_km(a, b)


def leg_seconds(a: LonLat, b: LonLat, speed_km_per_min: float) -> int:
    """Conservative travel time: ceil of the exact seconds; zero only for zero distance."""
    d = haversine_km(a, b)
    if d <= 0:
        return 0
    return math.ceil(60 * d / speed_km_per_min)


def initial_world(network, geometry) -> World:
    drones = {}
    for drone_id, seg_id in START_SEGMENTS.items():
        point = LinePoint(seg_id, geometry.length_mm(seg_id) // 2)
        perch = LandingSite(point, f"{seg_id}/C1")
        drones[drone_id] = Drone(drone_id, geometry.position_of(point), FLIGHT_S, Ready(perch))
    belief = dict.fromkeys(network.segments, LineStatus.UNKNOWN)
    return World(clock_s=0, drones=drones, belief=belief)


def seed_starting_handshakes(world: World) -> None:
    """Each starting perch has an explicit successful-handshake observation at T+00:00."""
    for d in world.drones.values():
        knowledge.ingest_handshake(world, d.state.perch.circuit_id, True, d.id, charging=False)


# -- time advance -----------------------------------------------------------------------------


def next_transition(world: World) -> int | None:
    times = []
    for drone in world.drones.values():
        match drone.state:
            case Flying(arrives_s=t) | Measuring(until_s=t) | Charging(until_s=t):
                times.append(t)
    times += [r.clock_s for r in world.pending_reports]
    expiry = knowledge.next_expiry(world)
    if expiry is not None:
        times.append(expiry)
    return min(times) if times else None


def _move_to(world: World, t: int) -> None:
    dt = t - world.clock_s
    for drone in world.drones.values():
        match drone.state:
            case Flying(origin, departed_s, arrives_s, route):
                if arrives_s > departed_s:
                    f = min(max((t - departed_s) / (arrives_s - departed_s), 0.0), 1.0)
                    drone.position = (
                        origin[0] + (route.target[0] - origin[0]) * f,
                        origin[1] + (route.target[1] - origin[1]) * f,
                    )
                drone.battery_s -= dt
            case Measuring():
                drone.battery_s -= dt
        if drone.battery_s < 0:
            raise RuntimeError(f"{drone.id} battery exhausted in flight; route validation failed")
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
            | _charging_and_arrivals(world, physics, events)
            | _sensing(world, physics, events)
            | _charge_and_availability(world, physics, events)
            | _expire_evidence(world, events)
        )
        if not changed:
            return events
    raise RuntimeError("transition cycle made no progress")


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


def _drones(world: World):
    return sorted(world.drones.values(), key=lambda d: d.id)


def _arrive(world: World, physics: Physics, drone: Drone, events: list[Event]) -> None:
    state = drone.state
    assert isinstance(state, Flying)
    route = state.route
    drone.position = route.target
    if route.stops:
        stop = route.stops[0]
        rest = replace(route, stops=route.stops[1:])
        drone.state = Measuring(stop, world.clock_s, world.clock_s + stop.dwell_s, rest)
        task = world.tasks[stop.task_id]
        world.set_task(replace(task, status=TaskStatus.MEASURING))
        events.append(Event(world.clock_s, "measuring", drone.id, stop.task_id))
        world.record(
            drone.id,
            f"Arrived at {stop.point.segment_id} {stop.point.offset_km:.3f} km; "
            f"measuring {stop.kind} for {stop.dwell_s // 60} min.",
        )
        return
    perch = route.perch
    if physics.is_chargeable(perch):
        drone.state = Charging(perch, world.clock_s + CHARGE_S)
        knowledge.ingest_handshake(world, perch.circuit_id, True, drone.id, charging=True)
        text = (
            f"Landed on {perch.circuit_id}; charging until "
            f"{format_clock((world.clock_s + CHARGE_S) / 60)}."
        )
        events.append(Event(world.clock_s, "landed_charging", drone.id, perch.circuit_id))
    else:
        drone.state = Stranded(perch)
        knowledge.ingest_handshake(world, perch.circuit_id, False, drone.id, charging=False)
        text = f"Landed on {perch.circuit_id}; no charging power, stranded."
        events.append(Event(world.clock_s, "landed_stranded", drone.id, perch.circuit_id))
    world.record(drone.id, text)


def _charging_and_arrivals(world: World, physics: Physics, events: list[Event]) -> bool:
    changed = False
    for drone in _drones(world):
        match drone.state:
            case Charging(perch) if not physics.is_chargeable(perch):
                drone.state = Stranded(perch)
                knowledge.ingest_handshake(world, perch.circuit_id, False, drone.id, charging=False)
                world.record(drone.id, f"Charging stopped on {perch.circuit_id}.")
                events.append(Event(world.clock_s, "charging_stopped", drone.id, perch.circuit_id))
                changed = True
            case Flying(arrives_s=arrives) if arrives <= world.clock_s:
                _arrive(world, physics, drone, events)
                changed = True
    return changed


def _sensing(world: World, physics: Physics, events: list[Event]) -> bool:
    changed = False
    for drone in _drones(world):
        state = drone.state
        if not (isinstance(state, Measuring) and state.until_s <= world.clock_s):
            continue
        stop = state.stop
        identity = observation_identity(stop.kind, stop.point)
        repeat = sum(
            1
            for r in world.readings.values()
            if r.kind == stop.kind and observation_identity(r.kind, r.point) == identity
        )
        sample = physics.sense(stop.point, stop.kind, world.clock_s, identity, repeat)
        reading_id = f"R{len(world.readings) + 1:04d}"
        task = world.tasks[stop.task_id]
        covariance = None
        if stop.kind in (Measurement.VOLTAGE_PRESENCE, Measurement.CURRENT):
            covariance = world.sensor_model.covariance(
                stop.kind, world.circuits_by_segment[stop.point.segment_id]
            )
        reading = Reading(
            id=reading_id,
            task_id=stop.task_id,
            site_id=task.site_id,
            drone_id=drone.id,
            point=stop.point,
            kind=stop.kind,
            clock_s=world.clock_s,
            operating_epoch="E0",
            payload=sample.values,
            sensor_model_revision="synthetic-v0",
            summary=", ".join(f"{k}={v:.3g}" for k, v in sample.values),
            covariance=covariance,
            conditions=sample.conditions,
            inspected=sample.inspected,
            repeat_index=repeat,
        )
        world.readings[reading_id] = reading
        knowledge.ingest_reading(world, reading)
        if stop.kind not in (Measurement.VOLTAGE_PRESENCE, Measurement.CURRENT):
            assessment.ingest_local_reading(
                world,
                reading,
                world.footprint_of(stop.point),
            )
        world.set_task(replace(task, status=TaskStatus.COMPLETED, reading_id=reading_id))
        drone.history += (stop.task_id,)
        world.record(drone.id, f"Reading {reading_id} ({stop.kind}): {reading.summary}")
        events.append(Event(world.clock_s, "reading", drone.id, reading_id))
        route = state.route
        drone.state = Flying(
            stop.position,
            world.clock_s,
            world.clock_s + leg_seconds(stop.position, route.target, drone.speed_km_per_min),
            route,
        )
        changed = True
    return changed


def _charge_and_availability(world: World, physics: Physics, events: list[Event]) -> bool:
    changed = False
    for drone in _drones(world):
        match drone.state:
            case Charging(perch, until_s) if until_s <= world.clock_s:
                drone.state = Ready(perch)
                drone.battery_s = FLIGHT_S
                knowledge.end_charge_telemetry(world, perch.circuit_id)
                world.record(drone.id, f"Fully charged on {perch.circuit_id}.")
                events.append(Event(world.clock_s, "charged", drone.id, perch.circuit_id))
                changed = True
            case Stranded(perch) if physics.is_chargeable(perch):
                drone.state = Charging(perch, world.clock_s + CHARGE_S)
                knowledge.ingest_handshake(world, perch.circuit_id, True, drone.id, charging=True)
                world.record(drone.id, f"Charging resumed on {perch.circuit_id}.")
                events.append(Event(world.clock_s, "charging_resumed", drone.id, perch.circuit_id))
                changed = True
    return changed
