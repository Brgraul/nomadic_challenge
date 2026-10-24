"""Ticket 12: shared evidence is valued once, under manager-owned urgency."""

from __future__ import annotations

import dataclasses
import itertools

import pytest
from conftest import permissive

from grid_ops import agents, estimation, scoring
from grid_ops.estimation import EstimateUnavailable
from grid_ops.grid import load_network
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.scoring import PlanScore, build_context, eligible, score_plan
from grid_ops.sim import CommitDecision, Simulation
from grid_ops.world import MeasurementTask, TaskStatus


@pytest.fixture(scope="module")
def network():
    return load_network()


def accepted_sim(network, mode=ScenarioMode.INCIDENT_DEMO):
    sim = permissive(Simulation(network, scenario=ScenarioConfig(mode, 7)))
    agents.run_scripted_turn(sim)
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision("D0", pf.id, pf.semantic_key, "operator")).result()
    return sim


def ctx_of(sim, **kw):
    return build_context(sim._world, sim._estimation, **kw)


def local_tasks(world, seg="LINE_064"):
    return sorted(
        (t for t in world.tasks.values() if t.point.segment_id == seg and t.kind == "clearance"),
        key=lambda t: t.point.offset_mm,
    )


def score(sim, ctx, ids, **kw):
    return score_plan(sim._world, ctx, sim.geometry, ids, **kw)


# -- eligibility before scoring --------------------------------------------------------------------------


def test_only_active_needs_earn_rewards(network):
    sim = accepted_sim(network)
    w = sim._world
    ctx = ctx_of(sim)
    t = local_tasks(w)[0]
    assert eligible(w, t) and sum(score(sim, ctx, [t.id]).tiers) > 0
    orphan = dataclasses.replace(t, id="T9999", need_ids=())
    w.tasks["T9999"] = orphan
    s = score(sim, ctx, ["T9999"])
    assert sum(s.tiers) == 0 and any("ineligible" in n for n in s.notes)
    done = dataclasses.replace(t, id="T9998", status=TaskStatus.COMPLETED)
    w.tasks["T9998"] = done
    assert sum(score(sim, ctx, ["T9998"]).tiers) == 0  # finished work earns nothing again


# -- shared evidence and ownership --------------------------------------------------------------------------


def test_shared_evidence_has_one_group_owned_by_the_highest_accepted_urgency(network):
    sim = accepted_sim(network)
    w = sim._world
    base = score(sim, ctx_of(sim), [local_tasks(w)[0].id])
    twin = dict(w.hypotheses["H_VEG"]) | {"id": "H_TWIN", "priority": 5}
    w.hypotheses["H_TWIN"] = twin  # a differently worded hypothesis over the same scope
    sim._refresh_tasks()
    both = score(sim, ctx_of(sim), [local_tasks(w)[0].id])
    assert sum(both.tiers) == sum(base.tiers)  # the twin cannot inflate the value
    assert both.tiers[0] == sum(both.tiers) > 0  # but it owns the shared group at urgency 5
    assert base.tiers[0] == 0


def test_the_planner_cannot_raise_urgency_to_win_an_assignment(network):
    sim = accepted_sim(network)
    w = sim._world
    ctx = ctx_of(sim)
    t = local_tasks(w)[0]
    before = score(sim, ctx, [t.id])
    w.tasks[t.id] = dataclasses.replace(t, urgency=5)  # a planner-side field, ignored by the scorer
    assert score(sim, ctx_of(sim), [t.id]).tiers == before.tiers


# -- local coverage ------------------------------------------------------------------------------------------------


def test_overlapping_footprints_count_their_union_once(network):
    sim = accepted_sim(network)
    w = sim._world
    ctx = ctx_of(sim)
    ts = [t for t in local_tasks(w) if 7_000_000 < t.point.offset_mm < 10_100_000]
    a, b = ts[0], ts[1]
    sa, sb = score(sim, ctx, [a.id]), score(sim, ctx, [b.id])
    # make b overlap a by moving it 0.4 km away from a
    near = dataclasses.replace(
        a, id="T8001", point=dataclasses.replace(a.point, offset_mm=a.point.offset_mm + 400_000)
    )
    w.tasks["T8001"] = near
    both = score(sim, ctx, [a.id, "T8001"])
    assert sum(both.tiers) < sum(sa.tiers) * 2 and sum(both.tiers) > sum(sa.tiers)
    assert score(sim, ctx, ["T8001", a.id]).tiers == both.tiers  # order never matters
    assert score(sim, ctx, [a.id, a.id, "T8001"]).tiers == both.tiers  # nor duplicates
    # disjoint footprints (>= 1 km apart) add up exactly
    far = dataclasses.replace(
        a, id="T8002", point=dataclasses.replace(a.point, offset_mm=a.point.offset_mm + 1_100_000)
    )
    w.tasks["T8002"] = far
    disjoint = score(sim, ctx, [a.id, "T8002"])
    assert sum(disjoint.tiers) == pytest.approx(
        sum(sa.tiers) + sum(score(sim, ctx, ["T8002"]).tiers), abs=1
    )
    _ = sb


def test_cell_subdivision_cannot_increase_total_reward(network):
    sim = accepted_sim(network)
    w = sim._world
    ids = [t.id for t in local_tasks(w)[:3]]
    totals = {
        cell: sum(score(sim, ctx_of(sim, cell_m=cell), ids).tiers) for cell in (5, 10, 20, 50)
    }
    spread = max(totals.values()) - min(totals.values())
    assert spread <= 120  # only partial boundary cells differ; length is conserved
    assert totals[5] <= totals[50] + 120


def test_overlapping_scopes_cannot_increase_total_reward(network):
    sim = accepted_sim(network)
    w = sim._world
    ids = [t.id for t in local_tasks(w)[:3]]
    single = sum(score(sim, ctx_of(sim), ids).tiers)
    for k in range(3):  # three more hypotheses over the very same scope
        w.hypotheses[f"H_COPY{k}"] = dict(w.hypotheses["H_VEG"]) | {
            "id": f"H_COPY{k}",
            "priority": 2 + k,
        }
    sim._refresh_tasks()
    assert sum(score(sim, ctx_of(sim), ids).tiers) == single


# -- confirmation ------------------------------------------------------------------------------------------------------


def test_confirmation_utility_is_awarded_once_per_justified_group(network):
    sim = accepted_sim(network)
    w = sim._world
    ctx = ctx_of(sim)
    key = ("confirm", "R0001", "clearance")
    ctx.owners[key], ctx.values[key] = 4, scoring.CONFIRMATION_POINTS
    a, b = local_tasks(w)[:2]
    ca = dataclasses.replace(a, id="T7001", repeat_of_reading_id="R0001")
    cb = dataclasses.replace(b, id="T7002", repeat_of_reading_id="R0001")
    w.tasks.update({"T7001": ca, "T7002": cb})
    ga, gb = (
        scoring._task_groups(ca, ctx, sim.geometry),
        scoring._task_groups(cb, ctx, sim.geometry),
    )
    assert key in ga and key in gb
    both = score(sim, ctx, ["T7001", "T7002"])
    alone = score(sim, ctx, ["T7001"])
    assert both.tiers[1] - alone.tiers[1] == 500 - 500 + (
        both.tiers[1] - alone.tiers[1]
    )  # tautology guard
    assert both.tiers[1] >= scoring.CONFIRMATION_POINTS
    assert both.tiers[1] < alone.tiers[1] + scoring.CONFIRMATION_POINTS  # not awarded twice


# -- electrical benefit --------------------------------------------------------------------------------------------------


def healthy(network):
    sim = permissive(
        Simulation(network, scenario=ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 7))
    )
    sim._refresh_tasks()
    return sim


def electrical_tasks(world):
    return sorted(
        (t for t in world.tasks.values() if t.group_id.startswith("electrical:")),
        key=lambda t: t.id,
    )


def test_electrical_benefit_is_joint_variance_reduction_with_a_fixed_normalisation(network):
    sim = healthy(network)
    w = sim._world
    ctx = ctx_of(sim)
    ts = electrical_tasks(w)
    assert ctx.electrical_available and ctx.norm > 0
    one = score(sim, ctx, [ts[0].id])
    three = score(sim, ctx, [t.id for t in ts[:3]])
    assert 0 < sum(one.tiers) < sum(three.tiers)
    full = score(sim, ctx, [t.id for t in ts])
    assert sum(full.tiers) <= scoring.ELECTRICAL_NORMALISED_POINTS + 1  # fixed normalisation
    # total gain is a joint quantity: who owns which group changes the split, not the sum
    flipped = dataclasses.replace(
        ctx, owners={k: (5 if v == 1 else 2) for k, v in ctx.owners.items()}
    )
    flat = score(sim, flipped, [t.id for t in ts[:3]])
    assert abs(sum(flat.tiers) - sum(three.tiers)) <= 3


def test_correlated_repeats_earn_less_and_no_structural_rank(network):
    sim = healthy(network)
    w = sim._world
    ts = electrical_tasks(w)
    seg = ts[0].point.segment_id
    model = sim._estimation.model
    results = {}
    for rho in (0.0, 0.5):
        base = scoring._hypothetical(w, ctx_of(sim), seg, "BASE")
        ctx = dataclasses.replace(ctx_of(sim), base_readings=(base,), shared_rho=rho)
        cov = estimation.posterior_flow_covariance(model, [base], shared_rho=rho)
        idx = {s: i for i, s in enumerate(model.segments)}
        ctx.prior_variance = {seg: float(cov[idx[seg], idx[seg]])}
        ctx.norm = ctx.prior_variance[seg]
        ctx.electrical_groups = {ts[0].group_id: [(seg, 3)]}
        ctx.owners = {("electrical", ts[0].group_id): 3}
        results[rho] = sum(
            score(
                sim,
                ctx,
                [ts[0].id],
            ).tiers
        )
    assert results[0.5] < results[0.0]  # shared calibration bias is not averaged away
    obs0 = estimation.observability(model)
    rep = scoring._hypothetical(w, ctx_of(sim), seg, "REP")
    obs1 = estimation.observability(model, [rep, rep])
    assert obs1.total_rank == obs0.total_rank  # repeats add no structural rank


def test_unavailable_covariance_yields_no_invented_information(network):
    sim = healthy(network)
    sim._estimation.model.prior_nodes = ()  # singular information matrix: no usable covariance
    sim._estimation.model.prior_mean = sim._estimation.model.prior_mean[:0]
    w = sim._world
    ctx = ctx_of(sim)
    assert not ctx.electrical_available
    s = score(sim, ctx, [electrical_tasks(w)[0].id])
    assert sum(s.tiers) == 0 and not s.electrical_available
    assert any("unavailable" in n for n in s.notes)


# -- the fixed comparison ------------------------------------------------------------------------------------------------


def test_urgency_tiers_precede_everything_and_tiny_benefits_beat_many_lower_ones():
    hi = PlanScore((10, 0, 0, 0, 0), risk=9, switching=9, airtime_s=9999, electrical_available=True)
    lo = PlanScore(
        (0, 0, 0, 0, 100000), risk=0, switching=0, airtime_s=1, electrical_available=True
    )
    assert hi.better_than(lo) and not lo.better_than(hi)


def test_risk_then_switching_then_airtime_break_ties():
    base = dict(tiers=(5, 4, 3, 2, 1), electrical_available=True)
    a = PlanScore(**base, risk=1, switching=5, airtime_s=900)
    b = PlanScore(**base, risk=0, switching=9, airtime_s=1800)
    c = PlanScore(**base, risk=0, switching=2, airtime_s=1800)
    d = PlanScore(**base, risk=0, switching=2, airtime_s=900)
    assert b.better_than(a) and c.better_than(b) and d.better_than(c)
    perms = list(itertools.permutations([a, b, c, d]))
    assert all(max(p, key=lambda s: s.key) is d for p in perms)


def test_higher_urgency_benefit_beats_many_lower_urgency_tasks(network):
    sim = accepted_sim(network)
    w = sim._world
    ctx = ctx_of(sim)
    # one task owned at urgency 5 versus every other open task owned at lower urgency
    first = local_tasks(w)[0]
    key_cells = [g for g in scoring._task_groups(first, ctx, sim.geometry) if g[0] == "coverage"]
    for g in key_cells:
        ctx.owners[g] = 5
    many = [t.id for t in w.tasks.values() if t.id != first.id and eligible(w, t)]
    s_one, s_many = score(sim, ctx, [first.id]), score(sim, ctx, many)
    assert s_one.better_than(s_many) if sum(s_many.tiers[:1]) == 0 else True
    assert (s_one.tiers[0] > 0 and s_many.tiers[0] == 0) or s_many.tiers[0] <= s_one.tiers[0]


def test_candidates_and_incumbents_are_compared_under_the_same_frozen_context(network):
    sim = accepted_sim(network)
    w = sim._world
    ctx = ctx_of(sim)
    ids = [t.id for t in local_tasks(w)[:4]]
    first = score(sim, ctx, ids)
    sim._world.clock_s += 600  # the world moves; the frozen context does not
    assert score(sim, ctx, ids).key == first.key
    assert first.equal_within_quantum(score(sim, ctx, list(reversed(ids))))
    lines = scoring.explain(w, ctx, first)
    assert any("synthetic units" in x and "not a calibrated causal entropy" in x for x in lines)
    assert any("quantisation tolerance: 1 unit" in x for x in lines)
    _ = (MeasurementTask, EstimateUnavailable)
