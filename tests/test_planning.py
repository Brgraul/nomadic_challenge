"""Bounded CP-SAT planning on the three-method runtime: honest candidates, hard constraints.

`plan_sorties` is pure: it reads a published snapshot and returns ranked candidate plans, every
one independently revalidated. Locked (in-progress) attempts are never decision variables,
occupied perches and drained batteries leave reasoned omissions, and only validated plans commit.
The exact-mode enumeration match is proven in test_planning_v2.
"""

from __future__ import annotations

import dataclasses

import pytest
from fixtures_three_method import contact_task, seed_handshakes, site_at, two_drone_world
from helpers_p2 import publish, tiny_sim

from grid_ops import planning, scoring
from grid_ops.coordinator import Coordinator
from grid_ops.energy import BatteryState
from grid_ops.geometry import LandingSite, LinePoint
from grid_ops.routes import ScheduledStep, SortieRequest, validate_sorties
from grid_ops.sim import CommitSorties
from grid_ops.world import (
    DEADLINE_S,
    LAND_S,
    TAKEOFF_S,
    Land,
    Scan,
    Stranded,
    Takeoff,
    Transit,
    leg_seconds,
)


def accepted_sim(**kw):
    """A tiny Simulation whose startup proposal (hypotheses + measurement requests) is accepted."""
    sim = tiny_sim(**kw)
    Coordinator(sim).run_turn_now()
    return sim


def plan(sim, **kw):
    return planning.plan_sorties(
        sim.snapshot, sim.geometry, sim.circuits, revision=sim.snapshot.revision, **kw
    )


def sortie_task_ids(req):
    ids = []
    for step in req.steps:
        op = step.operation
        if hasattr(op, "task_id"):
            ids.append(op.task_id)
        if hasattr(op, "task_ids"):
            ids.extend(op.task_ids)
    return ids


def scan_locked_sortie(sim, drone_id, task_id):
    """An itinerary whose scan over a compiled lidar task is in progress (the locked attempt)."""
    w = sim._world
    task = w.tasks[task_id]
    acq = task.acquisition
    drone = w.drones[drone_id]
    entry = acq.trajectory.poses[0].position
    exit_pos = acq.trajectory.poses[-1].position
    perch = LandingSite(
        LinePoint(acq.target.segment_id, acq.target.end_mm), f"{acq.target.segment_id}/C1"
    )
    here = sim.geometry.position_of(drone.state.perch.point)
    t = w.clock_s
    leg = leg_seconds(here, entry, drone.speed_km_per_min)
    steps = [ScheduledStep(0, t, t + TAKEOFF_S, Takeoff(drone.state.perch, here))]
    if leg:
        steps.append(ScheduledStep(1, t + TAKEOFF_S, t + TAKEOFF_S + leg, Transit(here, entry)))
    i = len(steps)
    scan_end = t + TAKEOFF_S + leg + acq.trajectory.duration_s
    steps.append(ScheduledStep(i, t + TAKEOFF_S + leg, scan_end, Scan(acq, task_id)))
    i += 1
    steps.append(ScheduledStep(i, scan_end, scan_end + LAND_S, Land(perch, exit_pos)))
    return SortieRequest(
        drone_id, tuple(steps), perch, w.revision, w.energy_model.revision, w.capabilities.revision
    ), TAKEOFF_S + leg


# -- structure ---------------------------------------------------------------------------------------------------------


def test_ranked_candidates_own_tasks_uniquely_and_validate():
    sim = accepted_sim()
    res = plan(sim)
    assert res.best is not None
    seen: list[str] = []
    for cand in res.ranked:
        assert cand.valid
        ids = [tid for r in cand.sorties for tid in sortie_task_ids(r)]
        assert len(ids) == len(set(ids))  # unique ownership
        landings = [r.final_perch for r in cand.sorties]
        assert len(landings) == len(set(landings))  # one drone per exact site
        check = validate_sorties(sim.snapshot, sim.geometry, sim.circuits, cand.sorties)
        assert check.ok, [str(r) for r in check.rejections]
        seen += ids
    assert all(sim._world.tasks[i].status.value == "PENDING" for i in set(seen))  # planning is pure


def test_drones_wait_when_there_is_no_work_and_never_fly_empty():
    sim = tiny_sim()  # no accepted requests: nothing worth flying for
    res = plan(sim)
    assert res.best is not None and res.best.sorties == ()
    sim2 = accepted_sim()
    for cand in plan(sim2).ranked:
        assert all(r.steps for r in cand.sorties)  # no empty itineraries


def test_locked_attempts_are_counted_and_never_reassigned():
    sim = accepted_sim()
    from grid_ops.sensing import LidarPass

    task = next(t for t in sim._world.tasks.values() if isinstance(t.acquisition, LidarPass))
    req, t_scan_start = scan_locked_sortie(sim, "DRONE_1", task.id)
    sim.submit(CommitSorties((req,), "operator")).result()
    sim.step((t_scan_start + task.acquisition.trajectory.duration_s / 2) / 60)  # mid scan
    assert sim._world.tasks[task.id].status.value == "MEASURING"
    res = plan(sim)
    for cand in res.ranked:
        assert task.id in cand.task_ids  # the locked attempt is counted as served
        assert all(
            task.id not in sortie_task_ids(r) for r in cand.sorties
        )  # never a decision variable
        mine = [r for r in cand.sorties if r.drone_id == "DRONE_1"]
        assert len(mine) <= 1  # at most one executable continuation for the busy drone
    assert res.best is not None


def test_a_corrupt_solver_round_is_never_ranked(monkeypatch):
    sim = accepted_sim()
    real = planning._round_requests

    def corrupt(world, geometry, vehicles, model, solution, revision):
        reqs = real(world, geometry, vehicles, model, solution, revision)
        return tuple(
            dataclasses.replace(r, energy_model_revision="tampered-energy-model") for r in reqs
        )

    monkeypatch.setattr(planning, "_round_requests", corrupt)
    res = plan(sim)
    assert [c.source for c in res.ranked] == ["incumbent"]  # nothing invalid ever ranks


# -- authoritative constraints ---------------------------------------------------------------------------------------------------------


def test_drained_batteries_and_the_deadline_leave_reasoned_unassigned_work():
    sim = accepted_sim()
    w = sim._world
    for d in w.drones.values():
        d.battery = BatteryState(remaining_units=1_200)  # not even a takeoff
    publish(w, sim)
    res = plan(sim)
    assert res.unassigned and all(u.reason for u in res.unassigned)
    assert all(
        "infeasible for every drone" in u.reason
        or "no eligible visit node" in u.reason
        or "left out" in u.reason
        for u in res.unassigned
    )
    for cand in res.ranked:
        check = validate_sorties(sim.snapshot, sim.geometry, sim.circuits, cand.sorties)
        assert check.ok
    w.clock_s = DEADLINE_S - 30
    publish(w, sim)
    res2 = plan(sim)
    for cand in res2.ranked:
        assert all(r.steps[-1].end_s <= DEADLINE_S for r in cand.sorties)


def test_occupied_perches_are_never_planned():
    sim = accepted_sim()
    w = sim._world
    w.drones["DRONE_5"].state = Stranded(w.drones["DRONE_5"].state.perch)
    occupied = w.drones["DRONE_5"].state.perch
    publish(w, sim)
    res = plan(sim)
    for cand in res.ranked:
        assert occupied not in [r.final_perch for r in cand.sorties]


# -- objective, budget and replay ------------------------------------------------------------------------------------------------------


def test_stages_replay_and_round_caps_are_reported():
    sim = accepted_sim()
    res = plan(sim, rounds=5)
    assert max(s.round for s in res.stages) <= 5 and res.replay["rounds"] == 5
    for s in res.stages:
        assert s.status in {"OPTIMAL", "FEASIBLE", "UNKNOWN", "INFEASIBLE", "MODEL_INVALID"}
        if s.status == "FEASIBLE":
            assert s.bound is not None  # a bound accompanies every non-proven value
    assert res.replay["seed"] == 0 and res.replay["workers"] == 1
    assert res.replay["deterministic_budget"] == pytest.approx(planning.DETERMINISTIC_BUDGET)
    assert res.replay["solver"].startswith("ortools ")
    assert res.revision >= 0 and res.stop_reason
    tight = plan(sim, wall_cap_s=0.001)
    assert "wall cap" in tight.stop_reason and tight.ranked  # still returns the incumbent


def test_strict_urgency_precedence_without_multipliers():
    w, geo = two_drone_world()
    seed_handshakes(w)
    contact_task(w, "T-HIGH", site_at(w, "LINE_040", 555_000), "REQ-HIGH", urgency=5)
    contact_task(w, "T-LOW", site_at(w, "LINE_025", 555_000), "REQ-LOW", urgency=2)
    del w.drones["DRONE_2"]  # one vehicle: the cap forces a choice between urgencies
    res = planning.plan_sorties(w, geo, None, cap=1)
    best = res.best
    assert best is not None and best.valid
    assert set(best.task_ids) == {"T-HIGH"}  # urgency-5 work is taken ...
    assert best.score.tiers[scoring.TIERS.index(5)] > 0
    assert best.score.tiers[scoring.TIERS.index(2)] == 0  # ... ahead of urgency-2 work
    t5 = next(s for s in res.stages if s.round == 1 and s.name == "tier-5")
    assert t5.objective is not None and t5.objective >= 0
    for cand in res.ranked:
        assert not cand.score.better_than(best.score)


def test_the_cap_reports_omissions_and_blocks_when_required_work_cannot_fit():
    w, geo = two_drone_world()
    seed_handshakes(w)
    contact_task(w, "T-HIGH", site_at(w, "LINE_040", 555_000), "REQ-HIGH", urgency=5)
    contact_task(w, "T-LOW", site_at(w, "LINE_025", 555_000), "REQ-LOW", urgency=2)
    del w.drones["DRONE_2"]
    res = planning.plan_sorties(w, geo, None, cap=1)
    assert len(res.omitted) == 1 and "cap" in res.omitted[0].reason
    blocked = planning.plan_sorties(w, geo, None, cap=0)
    assert blocked.blocked is not None and blocked.stop_reason == "PlanningBlocked"
    assert all(c.source == "incumbent" for c in blocked.ranked)


def test_candidates_never_rank_below_a_validated_incumbent():
    sim = accepted_sim()
    res = plan(sim)
    incumbent = next(c for c in res.ranked if c.source == "incumbent")
    assert res.best.score.key >= incumbent.score.key
    assert not incumbent.score.better_than(res.best.score)
    keys = [c.score.key for c in res.ranked]
    assert keys == sorted(keys, reverse=True)


def test_only_validated_plans_commit_and_commit_reserves_tasks():
    sim = accepted_sim()
    res = plan(sim)
    best = res.best
    assert best.valid and best.sorties
    sim.submit(CommitSorties(best.sorties, "planner")).result()
    w = sim._world
    for r in best.sorties:
        assert w.drones[r.drone_id].phase == "executing"
        for tid in sortie_task_ids(r):
            assert w.tasks[tid].status.value == "RESERVED"
            assert w.tasks[tid].assigned_drone_id == r.drone_id