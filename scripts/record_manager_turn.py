"""Record live manager turns (startup, then after real readings) against the real simulation."""

from __future__ import annotations

import sys

from grid_ops import agents
from grid_ops.geometry import LinePoint
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.sim import CommitSorties, Simulation
from grid_ops.world import Measurement


def main() -> int:
    sim = Simulation(load_network())
    sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
    turns = []
    turns.append(("startup", agents.run_llm_turn(sim, record=True)))
    perch = sim._world.drones["DRONE_2"].state.perch
    stops = tuple(
        StopRequest(LinePoint(seg, mm), Measurement.CLEARANCE)
        for seg, mm in (("LINE_058", 28_000), ("LINE_064", 9_000_000))
    )
    sim._apply(CommitSorties((SortieRequest("DRONE_2", stops, perch),), "operator"))
    sim.step(25)
    turns.append(("after readings", agents.run_llm_turn(sim, record=True)))
    for name, t in turns:
        print(name, "valid" if not t.errors else f"INVALID: {t.errors[:4]}")
        if t.output:
            for r in t.output.revisions[:2]:
                print("  ", r.hypothesis_id, "|", r.finding_summary[:140], "|", r.update_reason[:140])
    return 0


if __name__ == "__main__":
    sys.exit(main())
