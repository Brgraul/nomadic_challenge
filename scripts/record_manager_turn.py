"""Record live manager turns (startup, then after real readings) against the real simulation.

Requires OPENROUTER_API_KEY. The startup proposal is accepted through a scripted coordinator
turn, a route pass commits the planner's plan, the sim steps until readings publish, and the
LLM turn revises from the new evidence.
"""

from __future__ import annotations

import sys

from grid_ops import agents, manager_tools
from grid_ops.coordinator import Coordinator
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.sim import Simulation
from grid_ops.world import DEADLINE_S


def main() -> int:
    sim = Simulation(load_network())
    sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
    coord = Coordinator(sim)
    startup = coord.run_turn_now()
    print("startup", startup.status, startup.note)
    rp = sim.submit_route_pass("recording measurement turn")
    print("route pass:", rp)
    for _ in range(400):
        if sim.snapshot.readings or sim.snapshot.clock_s >= DEADLINE_S:
            break
        sim.step(min(90, (DEADLINE_S - sim.snapshot.clock_s) / 60))
    triggers = tuple(i for i, e in sim.snapshot.events if e.kind == "reading")
    tb = manager_tools.build_toolbox(sim.snapshot, sim.network, sim.circuits)
    turn = agents.run_llm_turn(tb, trigger_ids=triggers, record=True)
    print("measurement", "valid" if not turn.errors else f"INVALID: {turn.errors[:4]}")
    for r in (turn.output.revisions if turn.output else [])[:3]:
        print(" ", r.hypothesis_id, "|", r.finding_summary[:160])
    return 0


if __name__ == "__main__":
    sys.exit(main())