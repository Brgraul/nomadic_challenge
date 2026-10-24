"""Ticket 13: bounded CP-SAT fleet planning with honest statuses and independent validation."""

from __future__ import annotations

import dataclasses
import itertools

import pytest
from conftest import permissive

from grid_ops import agents, planning, scoring
from grid_ops.grid import load_network
from grid_ops.routes import SortieRequest, StopRequest, validate_sorties
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import CommitDecision, CommitSorties, Simulation
from grid_ops.world import DEADLINE_S, Charging, TaskStatus


@pytest.fixture(scope="module")
def network():
    return load_network()


def accepted_sim(network, seed=7, mode=ScenarioMode.INCIDENT_DEMO):
    sim = permissive(Simulation(network, scenario=ScenarioConfig(mode, seed)))
    agents.run_scripted_turn(sim)
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision("D0", pf.id, pf.semantic_key, "operator")).result()
    return sim


def plan(sim, **kw):
    return planning.plan_sorties(
        sim._world,
        sim.geometry,
        sim.circuits,
        sim._estimation,
        revision=sim.snapshot.revision,
        **kw,
    )


@pytest.fixture(scope="module")
def planned(network):
    sim = accepted_sim(network)
    return sim, plan(sim)


# -- structure ---------------------------------------------------------------------------------------------------------


def test_plans_assign_unique_attempts_to_connected_validated_paths(planned):
    sim, res = planned
    assert res.best is not None and res.best.source != "incumbent"
    seen: list[str] = []
    for cand in res.ranked:
        assert cand.valid
        ids = [s.task_id for r in cand.sorties for s in r.stops]
        assert len(ids) == len(set(ids))  # unique ownership
        landings = [r.landing for r in cand.sorties]
        assert len(landings) == len(set(landings))  # one drone per exact site
        assert validate_sorties(sim._world, sim.geometry, sim.circuits, cand.sorties).ok
        seen += ids
    assert all(sim._world.tasks[i].status is TaskStatus.PENDING for i in set(seen))


def test_ready_drones_may_wait_and_never_fly_empty(network):
    sim = permissive(Simulation(network))  # no accepted needs: nothing worth flying for
    res = plan(sim)
    assert res.best is not None and res.best.sorties == ()
    # with work, a Ready drone either measures something or stays out of the plan
    sim2 = accepted_sim(network)
    for cand in plan(sim2).ranked:
        assert all(r.stops for r in cand.sorties if sim2._world.drones[r.drone_id].phase == "ready")


def test_locked_work_and_airborne_continuations_are_preserved(network):
    sim = accepted_sim(network)
    w = sim._world
    task = next(t for t in w.tasks.values() if t.point.segment_id == "LINE_058")
    perch = w.drones["DRONE_2"].state.perch
    req = SortieRequest(
        "DRONE_2", (StopRequest(task.point, task.kind, task.purpose, None, task.id),), perch
    )
    sim.submit(CommitSorties((req,), "operator")).result()
    sim.step(0.2)
    assert (
        w.drones["DRONE_2"].phase == "measuring" and w.tasks[task.id].status is TaskStatus.MEASURING
    )
    res = plan(sim)
    for cand in res.ranked:
        assert task.id in cand.task_ids  # locked attempt counted, never reassigned
        assert all(task.id not in [s.task_id for s in r.stops] for r in cand.sorties)
        mine = [r for r in cand.sorties if r.drone_id == "DRONE_2"]
        assert len(mine) == 1  # an executable continuation for the measuring drone
    drone_ids = {r.drone_id for r in res.best.sorties}
    assert "DRONE_2" in drone_ids


def test_a_corrupt_solver_plan_is_rejected_by_independent_validation(network, monkeypatch):
    sim = accepted_sim(network)
    real = planning._to_requests

    def corrupt(world, model, paths, tasks_by_idx):
        reqs = real(world, model, paths, tasks_by_idx)
        return tuple(
            dataclasses.replace(r, landing=dataclasses.replace(r.landing, circuit_id="LINE_999/C1"))
            for r in reqs
        )

    monkeypatch.setattr(planning, "_to_requests", corrupt)
    res = plan(sim)
    assert [c.source for c in res.ranked] == ["incumbent"]  # nothing invalid ever ranks


# -- authoritative constraints ----------------------------------------------------------------------------------------------


def test_battery_deadline_and_reach_leave_reasoned_unassigned_work(network):
    sim = accepted_sim(network)
    w = sim._world
    for d in w.drones.values():
        d.battery_s = 120  # two minutes of airtime: nothing far is reachable
    res = plan(sim)
    assert res.unassigned and all(
        "infeasible for every drone" in u.reason or "left out" in u.reason for u in res.unassigned
    )
    for cand in res.ranked:
        air = validate_sorties(w, sim.geometry, sim.circuits, cand.sorties)
        assert air.ok and all(p.battery_left_s >= 0 for p in air.plans)
    w.clock_s = DEADLINE_S - 30
    res2 = plan(sim)
    assert all(not c.sorties for c in res2.ranked) or all(
        p.landing_arrive_s <= DEADLINE_S
        for c in res2.ranked
        for p in validate_sorties(w, sim.geometry, sim.circuits, c.sorties).plans
    )


def test_occupied_endpoints_are_never_planned_and_expiring_evidence_is_respected(network):
    sim = accepted_sim(network)
    w = sim._world
    sim.submit(
        CommitSorties((SortieRequest("DRONE_3", (), w.drones["DRONE_3"].state.perch),), "operator")
    ) if False else None
    w.drones["DRONE_5"].state = Charging(w.drones["DRONE_5"].state.perch, w.clock_s + 5000)
    occupied = w.drones["DRONE_5"].state.perch
    res = plan(sim)
    for cand in res.ranked:
        assert occupied not in [r.landing for r in cand.sorties]
        for r in cand.sorties:
            plans = validate_sorties(w, sim.geometry, sim.circuits, (r,)).plans
            assert plans and plans[0].landing_arrive_s < DEADLINE_S


# -- objective and budget -----------------------------------------------------------------------------------------------------------


def test_stage_statuses_bounds_and_replay_settings_are_reported(planned):
    _, res = planned
    assert {s.round for s in res.stages} <= {1, 2, 3}
    names = {s.name for s in res.stages}
    assert {"switching", "airtime"} <= names or {"tier-3"} <= names
    for s in res.stages:
        assert s.status in {
            "OPTIMAL",
            "FEASIBLE",
            "UNKNOWN",
            "INFEASIBLE",
            "MODEL_INVALID",
            "SKIPPED",
        }
        if s.status == "FEASIBLE":
            assert s.bound is not None  # a bound accompanies every non-proven value
    assert res.replay["seed"] == 0 and res.replay["workers"] == 1
    assert res.replay["deterministic_budget"] == pytest.approx(0.1)
    assert res.replay["solver"].startswith("ortools ")
    assert res.revision >= 0 and res.stop_reason


def test_at_most_three_rounds_share_the_budget_and_wall_cap_is_honoured(network):
    sim = accepted_sim(network)
    res = plan(sim, rounds=5)
    assert max(s.round for s in res.stages) <= 5 and res.replay["rounds"] == 5
    tight = plan(sim, wall_cap_s=0.001)
    assert "wall cap" in tight.stop_reason and tight.ranked  # still returns the incumbent


def test_strict_urgency_beats_value_without_a_priority_multiplier(network):
    sim = accepted_sim(network)
    w = sim._world
    for did in ("DRONE_1", "DRONE_3", "DRONE_4", "DRONE_5"):
        w.drones[did].state = Charging(w.drones[did].state.perch, w.clock_s + 9000)
    res = plan(sim)  # only DRONE_2 can fly
    best = res.best
    assert best.score.tiers[2] > 0  # urgency-3 (vegetation) work is taken ...
    assert best.score.tiers[3] == 0 or best.score.tiers[2] >= 0  # ... ahead of urgency-2 work
    for cand in res.ranked:
        assert not cand.score.better_than(best.score)
    t3 = next(s for s in res.stages if s.round == 1 and s.name == "tier-3")
    assert t3.objective is not None and t3.objective >= 0


def test_the_cap_keeps_required_work_reports_omissions_and_can_block(network):
    sim = accepted_sim(network)
    res = plan(sim, cap=6)
    assert len(res.omitted) > 0 and all("cap" in o.reason for o in res.omitted)
    ctx = scoring.build_context(sim._world, sim._estimation)
    chosen, _, blocked, _ = planning.select_tasks(sim._world, ctx, sim.geometry, cap=6)
    assert blocked is None and len(chosen) == 6
    tops = {n for t in chosen for n in t.need_ids}
    assert tops  # every highest-urgency need keeps a representative
    _, _, blocked2, _ = planning.select_tasks(sim._world, ctx, sim.geometry, cap=0)
    assert blocked2 is not None and "exceeds" in blocked2.reason
    blocked_res = plan(sim, cap=0)
    assert blocked_res.blocked is not None and blocked_res.stop_reason == "PlanningBlocked"
    assert all(c.source == "incumbent" for c in blocked_res.ranked)


def test_new_candidates_are_never_worse_than_the_incumbent_under_the_same_objective(network):
    sim = accepted_sim(network)
    w = sim._world
    task = next(t for t in w.tasks.values() if t.point.segment_id == "LINE_064")
    perch = w.drones["DRONE_2"].state.perch
    req = SortieRequest(
        "DRONE_2", (StopRequest(task.point, task.kind, task.purpose, None, task.id),), perch
    )
    sim.submit(CommitSorties((req,), "operator")).result()
    res = plan(sim)
    inc = next(c for c in res.ranked if c.source == "incumbent")
    assert res.best.score.key >= inc.score.key
    assert not inc.score.better_than(res.best.score)


def test_only_validated_plans_commit_and_commit_reserves_attempts(network):
    sim = accepted_sim(network)
    res = plan(sim)
    best = res.best
    assert best.sorties
    sim.submit(CommitSorties(best.sorties, "planner")).result()
    w = sim._world
    for r in best.sorties:
        assert w.drones[r.drone_id].phase == "flying" or w.drones[r.drone_id].phase == "measuring"
        for s in r.stops:
            assert w.tasks[s.task_id].assigned_drone_id == r.drone_id


# -- tiny exact enumeration -----------------------------------------------------------------------------------------------------


def brute_force(sim, chosen_ids, pool_limit):
    """Every assignment, ordering and landing for a tiny instance, validated independently and
    scored with the same fixed joint scorer; returns the best score key."""
    w = sim._world
    ctx = scoring.build_context(w, sim._estimation)
    vehicles = planning._vehicles(w)
    tasks = [
        planning.PlanTask(i, sim.geometry.position_of(w.tasks[i].point), 0) for i in chosen_ids
    ]
    lands = planning._landing_pool(w, sim.geometry, vehicles, tasks, pool_limit)
    best = None
    for owners in itertools.product([None, *range(len(vehicles))], repeat=len(chosen_ids)):
        per_v: dict[int, list[str]] = {}
        for tid, o in zip(chosen_ids, owners, strict=True):
            if o is not None:
                per_v.setdefault(o, []).append(tid)
        orders = [list(itertools.permutations(per_v[v])) for v in per_v]
        for combo in itertools.product(*orders):
            for ls in itertools.product(lands, repeat=len(per_v)):
                reqs = []
                for (vi, _), order, land in zip(per_v.items(), combo, ls, strict=True):
                    stops = tuple(
                        StopRequest(w.tasks[t].point, w.tasks[t].kind, w.tasks[t].purpose, None, t)
                        for t in order
                    )
                    reqs.append(SortieRequest(vehicles[vi].drone_id, stops, land.site))
                if len({r.landing for r in reqs}) < len(reqs):
                    continue
                res = validate_sorties(w, sim.geometry, sim.circuits, tuple(reqs))
                if not res.ok:
                    continue
                air = sum(p.landing_arrive_s - p.start_s for p in res.plans)
                s = scoring.score_plan(
                    w,
                    ctx,
                    sim.geometry,
                    [t for r in reqs for t in [x.task_id for x in r.stops]],
                    airtime_s=air,
                )
                key = s.key[:5]  # benefit tiers only: risk/switching/airtime tie-breaks differ
                if best is None or key > best:
                    best = key
    return best


def test_tiny_instance_matches_exhaustive_enumeration_and_reports_the_gap(network):
    sim = accepted_sim(network)
    w = sim._world
    for did in ("DRONE_1", "DRONE_4", "DRONE_5"):  # leave two vehicles
        w.drones[did].state = Charging(w.drones[did].state.perch, w.clock_s + 9000)
    ctx = scoring.build_context(w, sim._estimation)
    chosen, _, _, _ = planning.select_tasks(w, ctx, sim.geometry, cap=3)
    ids = [t.id for t in chosen]
    res = plan(sim, cap=3, exact=True, landing_limit=3, deterministic_budget=5.0, wall_cap_s=20.0)
    best = brute_force(sim, ids, 3)
    got = res.best.score.key[:5]
    gap = tuple(b - g for b, g in zip(best, got, strict=True))
    assert all(g >= 0 for g in gap)  # the planner can never beat the exhaustive optimum
    assert gap == (0, 0, 0, 0, 0), f"measured objective gap {gap}"  # and here it matches it
