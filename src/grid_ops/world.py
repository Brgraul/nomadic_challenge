"""Dynamic simulation state: what the operator believes about each line, and where the drones are.

Everything here is pure: `advance` and `dispatch` take a World and mutate it, with no threads or
I/O. The Simulation owns the only live World and applies these functions on its own thread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import StrEnum

from grid_ops.grid import LonLat, Network

FLIGHT_MINUTES = 30.0
CHARGE_MINUTES = 180.0
# Assumption: cruise speed of 60 km/h, so a full flight covers 30 km.
SPEED_KM_PER_MINUTE = 1.0

# Assumption: starting perches, one per drone, all fully charged at T+00:00.
START_SEGMENTS = {
    "DRONE_1": "LINE_040",
    "DRONE_2": "LINE_025",
    "DRONE_3": "LINE_026",
    "DRONE_4": "LINE_002",
    "DRONE_5": "LINE_056",
}


class LineStatus(StrEnum):
    UNKNOWN = "unknown"
    LIVE = "live"
    DE_ENERGIZED = "de_energized"
    SUSPECTED = "suspected"
    FAULT_CONFIRMED = "fault_confirmed"
    CLEARED = "cleared"


@dataclass(frozen=True)
class Ready:
    segment_id: str


@dataclass(frozen=True)
class Flying:
    origin: LonLat
    target: LonLat
    target_segment: str
    departed: float
    duration: float


@dataclass(frozen=True)
class Charging:
    segment_id: str
    until: float


@dataclass(frozen=True)
class Stranded:
    """Landed on a line that cannot recharge it. Charging starts if the line is re-energized."""

    segment_id: str


DroneState = Ready | Flying | Charging | Stranded


@dataclass
class Drone:
    id: str
    position: LonLat
    state: DroneState

    @property
    def phase(self) -> str:
        return type(self.state).__name__.lower()


@dataclass(frozen=True)
class LogEntry:
    clock: float
    author: str
    text: str


@dataclass
class World:
    clock: float
    drones: dict[str, Drone]
    belief: dict[str, LineStatus]
    energized: frozenset[str]
    log: list[LogEntry] = field(default_factory=list)

    def record(self, author: str, text: str) -> None:
        self.log.append(LogEntry(self.clock, author, text))


class CommandRejected(Exception):
    pass


def format_clock(minutes: float) -> str:
    return f"T+{int(minutes // 60):02d}:{int(minutes % 60):02d}"


def distance_km(a: LonLat, b: LonLat) -> float:
    lon1, lat1, lon2, lat2 = map(math.radians, (*a, *b))
    h = (
        math.sin((lat2 - lat1) / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    )
    return 2 * 6371.0 * math.asin(math.sqrt(h))


def initial_world(network: Network, energized: frozenset[str]) -> World:
    drones = {
        drone_id: Drone(drone_id, network.segments[seg_id].midpoint, Ready(seg_id))
        for drone_id, seg_id in START_SEGMENTS.items()
    }
    belief = dict.fromkeys(network.segments, LineStatus.UNKNOWN)
    return World(clock=0.0, drones=drones, belief=belief, energized=energized)


def dispatch(world: World, network: Network, drone_id: str, segment_id: str) -> str:
    drone = world.drones.get(drone_id)
    if drone is None:
        raise CommandRejected(f"Unknown drone {drone_id}.")
    segment = network.segments.get(segment_id)
    if segment is None:
        raise CommandRejected(f"Unknown segment {segment_id}.")
    if not isinstance(drone.state, Ready):
        raise CommandRejected(f"{drone_id} is {drone.phase} and cannot take off.")

    target = segment.midpoint
    km = distance_km(drone.position, target)
    duration = km / SPEED_KM_PER_MINUTE
    if duration > FLIGHT_MINUTES:
        raise CommandRejected(
            f"{segment_id} is {km:.1f} km away; {drone_id} can fly at most "
            f"{FLIGHT_MINUTES * SPEED_KM_PER_MINUTE:.0f} km per flight."
        )
    drone.state = Flying(drone.position, target, segment_id, world.clock, max(duration, 0.1))
    return f"{drone_id} departed for {segment_id} ({km:.1f} km, {duration:.0f} min)."


def advance(world: World, minutes: float) -> None:
    world.clock += minutes
    for drone in world.drones.values():
        match drone.state:
            case Flying(origin, target, segment_id, departed, duration):
                progress = min((world.clock - departed) / duration, 1.0)
                drone.position = (
                    origin[0] + (target[0] - origin[0]) * progress,
                    origin[1] + (target[1] - origin[1]) * progress,
                )
                if progress >= 1.0:
                    _land(world, drone, segment_id)
            case Charging(segment_id, until):
                if segment_id not in world.energized:
                    drone.state = Stranded(segment_id)
                    world.record(drone.id, f"Charging interrupted: {segment_id} lost power.")
                elif world.clock >= until:
                    drone.state = Ready(segment_id)
                    world.record(drone.id, f"Fully charged on {segment_id}.")
            case Stranded(segment_id) if segment_id in world.energized:
                drone.state = Charging(segment_id, world.clock + CHARGE_MINUTES)
                world.record(drone.id, f"{segment_id} re-energized; charging.")


def _land(world: World, drone: Drone, segment_id: str) -> None:
    if segment_id in world.energized:
        drone.state = Charging(segment_id, world.clock + CHARGE_MINUTES)
        world.record(
            drone.id, f"Landed on {segment_id}; charging until {format_clock(drone.state.until)}."
        )
    else:
        drone.state = Stranded(segment_id)
        world.record(drone.id, f"Landed on {segment_id}, which is not energized; stranded.")
