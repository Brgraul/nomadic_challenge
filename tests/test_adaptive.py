"""P3: the reconstruction wired through the real runtime, tools, scoring and conclusion.

The scripted adaptive policy requests the highest-value separating measurement each turn; the
optimizer assigns drones; the hidden episode answers; the posterior moves. Truth is consulted only
here, by the privileged evaluator handle, to check the outcome after the policy has stopped.
"""

from __future__ import annotations

import copy
import json

import pytest
from fixtures_three_method import tiny_network

from grid_ops import agents, knowledge
from grid_ops.coordinator import Coordinator
from grid_ops.energy import BatteryState
from grid_ops.manager_tools import build_toolbox
from grid_ops.outage import StopReason
from grid_ops.scenario import ScenarioConfig
from grid_ops.sensing import ContactCurrent, TaskStatusV2
from grid_ops.sim import Simulation
from grid_ops.world import DEADLINE_S


def adaptive(tb, trigger_ids=()):
    return agents.run_adaptive_scripted_turn(tb, trigger_ids, conclude=True)


def run_loop(sim, co, rounds=12):
    co.run_turn_now()
    for _ in range(rounds):
        w = sim.snapshot
        if w.investigation.status.value != "OPEN" or w.clock_s >= DEADLINE_S:
            break
        before = len(w.readings)
        sim.step(0)
        sim.submit_route_pass("test")
        for _ in range(120):
            sim.step(1)
            now = sim.snapshot
            if len(now.readings) > before and all(
                d.phase == "perched" for d in now.drones.values()
            ):
                break
        if len(sim.snapshot.readings) == before:
            sim.step(0)
            co.run_turn_now(())
            break
        triggers = tuple(i for i, e in sim.snapshot.events if e.kind == "reading")
        co.run_turn_now(triggers)
    return sim.snapshot


@pytest.mark.parametrize("seed", [7, 8, 12])
def test_the_closed_loop_concludes_with_the_hidden_fault_inside_the_supported_interval(seed):
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=seed))
    w = run_loop(sim, Coordinator(sim, manager_fn=adaptive))
    assert w.investigation.status.value == "CONCLUDED", (w.outage_stop, w.investigation)
    fact = next(f for f in w.outage_belief.supported_facts if f.kind == "fault_interval")
    assert fact.supported
    truth = sim.evaluator_truth().manifest()["fault"]
    assert truth["circuit_id"].startswith(fact.interval.segment_id)
    assert fact.interval.start_mm <= truth["offset_mm"] <= fact.interval.end_mm
    # a handful of acquisitions, no recharge wait: the clock only advanced by flying
    assert len(w.readings) <= 8
    assert w.clock_s < 3600
    conclusion = w.investigation.conclusion
    assert set(conclusion["causal_link_evidence_ids"]) <= set(w.readings)
    assert conclusion["remaining_uncertainty"]


def test_the_policy_requests_only_the_three_methods_and_reasons_from_reconstruction_evidence():
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=7))
    co = Coordinator(sim, manager_fn=adaptive)
    co.run_turn_now()
    w = sim.snapshot
    assert w.requests and all(
        str(a.request.method) in ("lidar", "tdr", "contact_current") for a in w.requests.values()
    )
    assert all(a.request.hypothesis_ids == ("H_FAULT",) for a in w.requests.values())
    assert 1 <= len(w.requests) <= 3
    turn = w.coordinator_turns[-1]
    assert turn.status == "accepted"


def test_repeated_samples_are_not_requested_again():
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=12))
    co = Coordinator(sim, manager_fn=adaptive)
    run_loop(sim, co)
    keys = [
        (
            str(r.acquisition.method),
            r.acquisition.conductor.circuit_id,
            r.acquisition.conductor.point.offset_mm,
        )
        for r in sim.snapshot.readings.values()
        if hasattr(r.acquisition, "conductor")
    ]
    assert len(keys) == len(set(keys))


def test_a_fleet_with_no_energy_stops_as_unresolved_not_as_a_cause():
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=7))
    for d in sim._world.drones.values():
        d.battery = BatteryState(remaining_units=2_000)
    # no drone has charging evidence, so nothing may rely on a recharge to reach a target
    sim._world.knowledge.evidence.clear()
    knowledge.recompute(sim._world)
    sim.snapshot = copy.deepcopy(sim._world)
    co = Coordinator(sim, manager_fn=adaptive)
    co.run_turn_now()
    sim.step(0)
    sim.submit_route_pass("test")
    sim.step(1)
    w = sim.snapshot
    assert not w.readings
    assert w.outage_stop.reason is StopReason.NO_SAFE_EXPERIMENT
    turn = co.run_turn_now(())
    assert turn.status == "accepted", turn.note
    inv = sim.snapshot.investigation
    assert inv.conclusion is None and "not established" in " ".join(h for _, h in inv.history)
    assert inv.unresolved


def test_a_cause_cannot_be_concluded_while_the_reconstruction_is_unsupported():
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=7))
    co = Coordinator(
        sim,
        manager_fn=lambda tb, trigger_ids=(): agents.run_adaptive_scripted_turn(
            tb, trigger_ids, conclude=True
        ),
    )
    co.run_turn_now()
    w = sim.snapshot
    assert w.outage_stop.reason is not StopReason.SUPPORTED
    tb = build_toolbox(w, sim.network, sim.circuits)
    out = agents.run_adaptive_scripted_turn(tb, (), conclude=True).output
    assert out.conclusion is None or out.conclusion.kind == "unresolved_ambiguous"


# -- the manager's view ----------------------------------------------------------------------------------------------


def test_the_reconstruction_tool_reports_facts_alternatives_and_limits_without_truth():
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=7))
    tb = build_toolbox(sim.snapshot, sim.network, sim.circuits)
    text = tb.reconstruction()
    payload = json.loads(text[text.index("{") :])
    assert payload["model_version"] and payload["limits"]
    assert payload["stop_assessment"]["state"] == "continue"
    assert "computed by the system" in payload["note"]
    assert {"regions", "supported_facts", "alternatives_not_yet_separated", "model_checks"} <= set(
        payload
    )
    private = sim.evaluator_truth().manifest()
    for token in (
        str(private["episode_spec"]["episode_seed"]),
        "episode_seed",
        "noise_seed",
        "newly_unsupplied",
        "FaultEvent",
    ):
        assert token not in text


def test_the_experiment_tool_is_bounded_ranked_and_leaves_the_choice_to_the_manager():
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=7))
    tb = build_toolbox(sim.snapshot, sim.network, sim.circuits)
    text = tb.suggest_experiments()
    payload = json.loads(text[text.index("{") :])
    rows = payload["experiments"]
    assert 0 < len(rows) <= sim.outage_model.caps.max_alternatives
    bits = [r["expected_discrimination_bits"] for r in rows]
    assert bits == sorted(bits, reverse=True)
    assert {r["method"] for r in rows} <= {"lidar", "tdr", "contact_current"}
    assert "You choose method and scope" in payload["note"]


def test_the_language_model_is_given_the_reconstruction_tools_and_the_no_invented_scores_rule():
    assert (
        "reconstruction" in agents.SYSTEM and "never state a posterior probability" in agents.SYSTEM
    )
    names = {
        t.name
        for t in agents._tools(
            build_toolbox(Simulation(tiny_network()).snapshot, tiny_network(), None)
        )
    }
    assert {"reconstruction", "suggest_experiments"} <= names


# -- the planner is valued by the posterior, not by a fixed reward --------------------------------------------


def test_open_tasks_are_valued_by_expected_discrimination_and_cheaper_plans_win():
    sim = Simulation(tiny_network(), scenario=ScenarioConfig(seed=7))
    co = Coordinator(sim, manager_fn=adaptive)
    co.run_turn_now()
    w = sim.snapshot
    pending = [t for t in w.tasks.values() if t.status is TaskStatusV2.PENDING]
    assert pending and set(w.experiment_values) >= {t.id for t in pending}
    assert any(v > 0 for v in w.experiment_values.values())
    assert w.outage_timing["belief_update_s"] < 0.25
    assert w.outage_timing["request_value_s"] < 1.0
    current = [t for t in pending if isinstance(t.acquisition, ContactCurrent)]
    assert all(t.id in w.experiment_values for t in current)


def test_a_dash_between_numbers_is_a_range_not_a_minus_sign():
    from grid_ops.proposals import quoted_numbers

    assert quoted_numbers("fault between 0.298-0.744 km and 12 m") == [("0.744", "km"), ("12", "m")]
    en_dash = chr(0x2013)
    assert quoted_numbers(f"spans 0.2 {en_dash} 0.7 km") == [("0.7", "km")]
    assert quoted_numbers("an offset of -3 m") == [("-3", "m")]
