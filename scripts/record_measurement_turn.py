"""Record one live manager turn after real readings (DRONE_2 clearance on LINE_058/LINE_064)."""

from __future__ import annotations

from grid_ops import agents
from grid_ops.geometry import LinePoint
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.sim import CommitSorties, Simulation
from grid_ops.world import Measurement

sim = Simulation(load_network())
sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
perch = sim._world.drones["DRONE_2"].state.perch
stops = tuple(
    StopRequest(LinePoint(seg, mm), Measurement.CLEARANCE)
    for seg, mm in (("LINE_058", 28_000), ("LINE_064", 9_000_000))
)
sim._apply(CommitSorties((SortieRequest("DRONE_2", stops, perch),), "operator"))
sim.step(25)
turn = agents.run_llm_turn(sim, record=True)
print("measurement", "valid" if not turn.errors else f"INVALID: {turn.errors[:4]}")
for r in (turn.output.revisions if turn.output else [])[:3]:
    print(" ", r.hypothesis_id, "|", r.finding_summary[:160])
