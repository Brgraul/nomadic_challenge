"""Record one live manager turn after real readings (requires OPENROUTER_API_KEY).

A scripted startup turn is accepted, a route pass commits the planner's plan, the sim steps
until readings publish (DRONE_3's contact-current work), then the LLM turn revises from them.
"""

from __future__ import annotations

from grid_ops import agents, manager_tools
from grid_ops.coordinator import Coordinator
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.sim import Simulation
from grid_ops.world import DEADLINE_S

sim = Simulation(load_network())
sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
turn = Coordinator(sim).run_turn_now()
assert turn.status == "accepted", turn.note
print("route pass:", sim.submit_route_pass("recording measurement turn"))
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
