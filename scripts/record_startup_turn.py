"""Record one live startup manager turn."""

from __future__ import annotations

from grid_ops import agents
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.sim import Simulation

sim = Simulation(load_network())
sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
turn = agents.run_llm_turn(sim, record=True)
print("startup", "valid" if not turn.errors else f"INVALID: {turn.errors[:4]}")
