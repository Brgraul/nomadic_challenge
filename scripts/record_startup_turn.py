"""Record one live startup manager turn (requires OPENROUTER_API_KEY; offline runs stay dry)."""

from __future__ import annotations

from grid_ops import agents, manager_tools
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.sim import Simulation

sim = Simulation(load_network())
sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
tb = manager_tools.build_toolbox(sim.snapshot, sim.network, sim.circuits)
turn = agents.run_llm_turn(tb, record=True)
print("startup", "valid" if not turn.errors else f"INVALID: {turn.errors[:4]}")
for r in (turn.output.revisions if turn.output else [])[:2]:
    print("  ", r.hypothesis_id, "|", r.finding_summary[:140], "|", r.update_reason[:140])