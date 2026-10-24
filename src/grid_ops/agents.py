"""Master agent that delegates to one sub-agent per drone.

Every agent reads the shared World from `sim.snapshot` and writes to it by submitting commands, so
the simulation thread stays the single writer.
"""

from __future__ import annotations

import os
from collections.abc import Callable

from langchain.agents import create_agent
from langchain.tools import BaseTool, tool

from grid_ops.sim import Dispatch, Log, SetBelief, Simulation
from grid_ops.world import (
    CHARGE_MINUTES,
    FLIGHT_MINUTES,
    SPEED_KM_PER_MINUTE,
    CommandRejected,
    LineStatus,
    distance_km,
    format_clock,
)

MODEL = os.environ.get("GRID_OPS_MODEL", "anthropic:claude-sonnet-5-5")

RULES = (
    f"Each drone flies at most {FLIGHT_MINUTES:.0f} minutes per flight at "
    f"{SPEED_KM_PER_MINUTE * 60:.0f} km/h, then must charge {CHARGE_MINUTES / 60:.0f} hours "
    "before its next flight. It can only charge while perched on an energized line. "
    "Line statuses are the operator's beliefs, not ground truth."
)


def _submit(sim: Simulation, command) -> str:
    try:
        return sim.submit(command).result(timeout=5)
    except CommandRejected as e:
        return f"Rejected: {e}"


def _shared_tools(sim: Simulation, author: str) -> list[BaseTool]:
    network = sim.network

    @tool
    def describe_world() -> str:
        """Current sim clock, every drone's state, and every line status other than unknown."""
        world = sim.snapshot
        lines = [f"Clock {format_clock(world.clock)}."]
        for drone in world.drones.values():
            lines.append(f"{drone.id}: {drone.phase} {drone.state}")
        known = {s: b for s, b in world.belief.items() if b is not LineStatus.UNKNOWN}
        lines.append(f"Known line statuses: {known or 'none'}.")
        return "\n".join(lines)

    @tool
    def describe_segment(segment_id: str) -> str:
        """Static attributes and current believed status of one line segment."""
        segment = network.segments.get(segment_id)
        if segment is None:
            return f"Unknown segment {segment_id}."
        return (
            f"{segment.id}: {segment.voltage_kv:.0f} kV, {segment.length_km:.1f} km, "
            f"{segment.from_node} to {segment.to_node}, {segment.component}, "
            f"midpoint {segment.midpoint}, status {sim.snapshot.belief[segment.id]}."
        )

    @tool
    def set_line_status(segment_id: str, status: LineStatus, note: str) -> str:
        """Record what the operator now believes about a line, with the evidence in `note`."""
        return _submit(sim, SetBelief(segment_id, status, note, author))

    return [describe_world, describe_segment, set_line_status]


def _drone_tools(sim: Simulation, drone_id: str) -> list[BaseTool]:
    network = sim.network

    @tool
    def reachable_segments() -> str:
        """Segments this drone can reach in one flight, nearest first."""
        here = sim.snapshot.drones[drone_id].position
        reach = FLIGHT_MINUTES * SPEED_KM_PER_MINUTE
        options = sorted((distance_km(here, s.midpoint), s.id) for s in network.segments.values())
        return ", ".join(f"{sid} ({km:.1f} km)" for km, sid in options if km <= reach)

    @tool
    def fly_to(segment_id: str) -> str:
        """Fly this drone to the midpoint of a segment and perch there."""
        return _submit(sim, Dispatch(drone_id, segment_id, drone_id))

    return [reachable_segments, fly_to]


def _delegate_tool(sim: Simulation, drone_id: str) -> BaseTool:
    agent = create_agent(
        MODEL,
        tools=[*_shared_tools(sim, drone_id), *_drone_tools(sim, drone_id)],
        system_prompt=(
            f"You control sensor drone {drone_id} in a storm-damaged power grid. {RULES} "
            "Carry out the coordinator's task, then report what you did and what you found."
        ),
    )

    @tool(f"delegate_{drone_id.lower()}", description=f"Give {drone_id} a task.")
    def delegate(task: str) -> str:
        sim.submit(Log("master", f"Task for {drone_id}: {task}"))
        result = agent.invoke({"messages": [{"role": "user", "content": task}]})
        return result["messages"][-1].content

    return delegate


def build_master(sim: Simulation) -> Callable[[str], str]:
    reports = "\n".join(
        f"- {r.id} ({r.timestamp}, confidence {r.confidence}): {r.description}"
        for r in sim.network.reports.values()
    )
    master = create_agent(
        MODEL,
        tools=[
            *_shared_tools(sim, "master"),
            *(_delegate_tool(sim, drone_id) for drone_id in sim.snapshot.drones),
        ],
        system_prompt=(
            "You coordinate sensor drones investigating a power grid after a windstorm. "
            f"{RULES}\n\nReports received so far (approximate, possibly wrong):\n{reports}"
        ),
    )

    def run(instruction: str) -> str:
        result = master.invoke({"messages": [{"role": "user", "content": instruction}]})
        answer = result["messages"][-1].content
        sim.submit(Log("master", answer if isinstance(answer, str) else str(answer)))
        return answer

    return run
