"""Background simulation loop.

The sim thread is the only writer of the World. Agents and the UI send commands through a queue
and read `snapshot`, a deep copy republished after every tick, so readers never need a lock.
"""

from __future__ import annotations

import copy
import queue
import threading
import time
from concurrent.futures import Future
from dataclasses import dataclass

from grid_ops.grid import Network, PowerModel
from grid_ops.world import (
    CommandRejected,
    LineStatus,
    World,
    advance,
    dispatch,
    initial_world,
)

TICK_SECONDS = 0.5


@dataclass(frozen=True)
class Dispatch:
    drone_id: str
    segment_id: str
    author: str


@dataclass(frozen=True)
class SetBelief:
    segment_id: str
    status: LineStatus
    note: str
    author: str


@dataclass(frozen=True)
class SetInService:
    """Ground-truth switching or a fault trip, not something the agents can see directly."""

    segment_id: str
    in_service: bool
    author: str


@dataclass(frozen=True)
class Log:
    author: str
    text: str


Command = Dispatch | SetBelief | SetInService | Log


class Simulation:
    def __init__(self, network: Network, minutes_per_second: float = 1.0) -> None:
        self.network = network
        self.power = PowerModel(network)
        self.minutes_per_second = minutes_per_second
        self.paused = False
        self._commands: queue.Queue[tuple[Command, Future[str]]] = queue.Queue()
        self._stop = threading.Event()

        world = initial_world(network, self.power.energized())
        self._world = world
        self._apply(SetInService("LINE_024", False, "scenario (SYN_REPORT_03)"))
        self.snapshot: World = copy.deepcopy(world)

    def submit(self, command: Command) -> Future[str]:
        future: Future[str] = Future()
        self._commands.put((command, future))
        return future

    def start(self) -> None:
        threading.Thread(target=self._run, name="grid-sim", daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            while not self._commands.empty():
                command, future = self._commands.get_nowait()
                try:
                    future.set_result(self._apply(command))
                except CommandRejected as e:
                    self._world.record(command.author, f"Rejected: {e}")
                    future.set_exception(e)
            now = time.monotonic()
            if not self.paused:
                advance(self._world, (now - last) * self.minutes_per_second)
            last = now
            self.snapshot = copy.deepcopy(self._world)
            time.sleep(TICK_SECONDS)

    def _apply(self, command: Command) -> str:
        world = self._world
        match command:
            case Dispatch(drone_id, segment_id, author):
                result = dispatch(world, self.network, drone_id, segment_id)
            case SetBelief(segment_id, status, note, author):
                if segment_id not in world.belief:
                    raise CommandRejected(f"Unknown segment {segment_id}.")
                world.belief[segment_id] = status
                result = f"{segment_id} marked {status}: {note}"
            case SetInService(segment_id, in_service, author):
                self.power.set_in_service(segment_id, in_service)
                world.energized = self.power.energized()
                state = "in service" if in_service else "out of service"
                result = (
                    f"{segment_id} switched {state}; {len(world.energized)} segments energized."
                )
            case Log(author, text):
                result = text
        world.record(author, result)
        return result
