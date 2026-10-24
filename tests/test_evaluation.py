"""Ticket 18: headless policy comparison with truth-isolated scoring and replay."""

from __future__ import annotations

import json
import subprocess
import sys

import pytest

from grid_ops import evaluation
from grid_ops.grid import load_network


@pytest.fixture(scope="module")
def network():
    return load_network()


def test_headless_run_scores_incident_by_evidence_batch_and_writes_results(network, tmp_path):
    r = evaluation.run("incident_demo", "agent", 7, minutes=120, network=network)
    path = evaluation.write(r, tmp_path)
    data = json.loads(path.read_text())
    assert data["batches"] and all("sim_s" in b and "incident" in b for b in data["batches"])
    inc = data["final"]["incident"]
    assert inc["of"] == 3 and set(inc) >= {
        "causal_diagnosis",
        "material_alternatives",
        "anomalies",
        "scoped_actions",
        "post_completion_readings",
    }
    ops = data["operations"]
    assert {"minutes_in_phase", "repeats", "landing_failures", "urgent_unassigned"} <= set(ops)
    assert {"fallback_frequency", "solver_bounds", "tiny_case_gap"} <= set(ops)
    assert [b["sim_s"] for b in data["batches"]] == sorted(b["sim_s"] for b in data["batches"])


def test_replay_record_keeps_solver_budgets_and_labels_wall_variability(network):
    r = evaluation.run("incident_demo", "agent", 7, minutes=60, network=network)
    rp = r.replay
    assert rp["solver"].startswith("ortools") and rp["budgets"]["wall_cap_s"] <= 1.5
    assert "wall" in rp["wall_watchdog_variability"] and "manager_turns" in rp
    assert r.declared["differences"]["agent"] and "same landing rule" in " ".join(
        r.declared["shared"]
    )


def test_policies_share_initialization_and_are_reproducible_per_seed(network):
    a = evaluation.run("incident_demo", "greedy", 8, minutes=90, network=network)
    b = evaluation.run("incident_demo", "greedy", 8, minutes=90, network=network)
    assert a.final == b.final and a.batches == b.batches  # observation-keyed noise: paired seeds
    r = evaluation.run("incident_demo", "random", 8, minutes=90, network=network)
    assert r.mode == a.mode and r.seed == a.seed


def test_lookahead_baseline_plans_two_stop_sorties_unlike_the_myopic_policies(network):
    sim_pols = {}
    for name in ("greedy", "lookahead"):
        r = evaluation.run("incident_demo", name, 7, minutes=60, network=network)
        sim_pols[name] = r
    sizes = {k: [len(t["tasks"]) for t in v.replay["manager_turns"]] for k, v in sim_pols.items()}
    assert sizes["greedy"] and set(sizes["greedy"]) == {1}
    assert 2 in sizes["lookahead"]


def test_healthy_scoring_reports_error_observability_and_unsupported_intervals(network):
    r = evaluation.run("healthy_benchmark", "agent", 7, minutes=60, network=network)
    h = r.final["healthy"]
    assert {"flow_error_mw", "observability", "calibration", "benchmark_checks"} <= set(h)
    assert {c["name"] for c in h["benchmark_checks"]} == {
        "observability",
        "uncertainty",
        "consistency",
        "detectability",
    }
    assert "unsupported_intervals" in h["calibration"] and "unresolved_work" in h


def test_scores_never_reach_public_state_or_decision_payloads(network):
    from grid_ops.scenario import ScenarioConfig
    from grid_ops.sim import Simulation

    sim = Simulation(network, scenario=ScenarioConfig())
    sim.step(5)
    score = evaluation.score_state(sim)
    assert score["incident"]["of"] == 3
    text = json.dumps(sim.snapshot, default=str)
    assert "time_to_detection" not in text and "false_hazards" not in text
    assert not hasattr(sim.snapshot, "score") and "evaluation" not in vars(sim)


def test_cli_entry_point_runs_end_to_end(tmp_path):
    out = subprocess.run(
        [
            sys.executable,
            "-m",
            "grid_ops.evaluation",
            "--policy",
            "greedy",
            "--seed",
            "9",
            "--minutes",
            "45",
            "--out",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert out.returncode == 0, out.stderr[-500:]
    written = tmp_path / "incident_demo" / "greedy-seed9.json"
    assert written.exists() and json.loads(written.read_text())["policy"] == "greedy"
