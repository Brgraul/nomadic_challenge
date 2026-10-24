"""Headless evaluation: policies, truth-isolated scoring and replayable results.

This is the only module besides `sim` that may look at hidden truth, and only *after* the fact:
policies see the same public `Simulation` interface the console uses, scoring reads the hidden
field once per evidence batch into a separate results list, and nothing computed here is ever
written into public World state, tool outputs or decision payloads.

    uv run python -m grid_ops.evaluation --mode incident_demo --policy agent --seed 7
    uv run python -m grid_ops.evaluation --mode incident_demo --compare --seeds 7 8 9
"""

from __future__ import annotations

import argparse
import contextlib
import json
import random
import statistics
import time
from dataclasses import dataclass, field
from pathlib import Path

from grid_ops import agents, conclusion, planning, resilience, scoring
from grid_ops.grid import load_network
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import CommitDecision, SetLandingPolicy, Simulation
from grid_ops.world import (
    CommandRejected,
    Ready,
    TaskStatus,
    leg_seconds,
)


def energized_segments(sim: Simulation) -> frozenset[str]:
    return sim.evaluator_truth().energized()


DEADLINE_MIN = 240
POLICIES = ("random", "greedy", "lookahead", "agent")
RESULTS_DIR = Path(__file__).resolve().parents[2] / "results"
KIND_OF = {"vegetation": "clearance", "hot_joint": "conductor_temp", "motion": "motion"}

DECLARED = {
    "shared": [
        "same scenario, seed, sensor model and observation-keyed noise",
        "same accepted initial hypotheses (incident) or reconstruction needs (healthy)",
        "same task set and attempt cap",
        "same landing rule (uncertain endpoints permitted, hazards forbidden) and T+04:00 deadline",
        "one decision per ready drone; routes validated by the independent validator",
    ],
    "differences": {
        "random": "one random eligible task per sortie, landing at the drone's perch",
        "greedy": "nearest eligible task per sortie, landing at the drone's perch",
        "lookahead": "best two-task sortie by the joint scorer, landing at the drone's perch",
        "agent": "scripted manager follow-ups, joint CP-SAT planner and reactive route passes "
        "(plus a route pass every 5 min while drones are idle and tasks are open)",
    },
}


# -- policies (public interface only) -------------------------------------------------------


def _open_tasks(sim):
    w = sim.snapshot
    return [
        t for t in w.tasks.values() if t.status is TaskStatus.PENDING and scoring.eligible(w, t)
    ]


def _stop(task) -> StopRequest:
    return StopRequest(task.point, task.kind)


class BaselinePolicy:
    """Myopic dispatch of ready drones, with no manager and no reactive replanning."""

    def __init__(self, name: str, seed: int) -> None:
        self.name = name
        self.rng = random.Random(f"{seed}|{name}")
        self.refused: set[tuple[str, str]] = set()
        self.turns: list[dict] = []  # sorties submitted: (sim_s, drone, task ids)

    def start(self, sim) -> None:
        sim.auto_replan = False

    def act(self, sim) -> None:
        w = sim.snapshot
        for did, drone in sorted(w.drones.items()):
            if not isinstance(drone.state, Ready):
                continue
            tasks = [t for t in _open_tasks(sim) if (did, t.id) not in self.refused]
            if not tasks:
                continue
            for pick in self._candidates(sim, drone, tasks):
                req = SortieRequest(did, tuple(_stop(t) for t in pick), drone.state.perch)
                try:
                    from grid_ops.sim import CommitSorties

                    sim.submit(CommitSorties((req,), f"policy-{self.name}")).result()
                    self.turns.append(
                        {"sim_s": w.clock_s, "drone": did, "tasks": [t.id for t in pick]}
                    )
                    break
                except CommandRejected:
                    for t in pick:
                        self.refused.add((did, t.id))
            w = sim.snapshot

    def _candidates(self, sim, drone, tasks):
        speed = drone.speed_km_per_min
        near = sorted(
            tasks,
            key=lambda t: leg_seconds(drone.position, sim.geometry.position_of(t.point), speed),
        )
        if self.name == "random":
            self.rng.shuffle(tasks)
            return [(t,) for t in tasks[:4]]
        if self.name == "greedy":
            return [(t,) for t in near[:4]]
        # two-sortie lookahead: choose the pair the joint scorer values most
        ctx = scoring.build_context(sim.snapshot, sim._estimation)
        pool = near[:8]
        best = []
        for a in pool:
            for b in pool:
                if a.id != b.id:
                    s = scoring.score_plan(sim.snapshot, ctx, sim.geometry, [a.id, b.id])
                    best.append((s.key, (a, b)))
        best.sort(key=lambda x: x[0], reverse=True)
        return [pair for _, pair in best[:3]] + [(t,) for t in near[:2]]


class AgentPolicy:
    """Scripted manager plus the production planner; optionally a live LLM manager."""

    name = "agent"

    def __init__(self, seed: int, llm: bool = False) -> None:
        self.llm = llm
        self.seen_readings = 0
        self.turns: list[dict] = []

    def start(self, sim) -> None:
        sim.auto_replan = True
        self.last_poke_s = -(10**9)

    def act(self, sim) -> None:
        w = sim.snapshot
        idle = any(isinstance(d.state, Ready) for d in w.drones.values())
        if idle and _open_tasks(sim) and w.clock_s - self.last_poke_s >= 300:
            # New tasks are not a route-pass trigger, so an idle fleet is nudged (declared)
            self.last_poke_s = w.clock_s
            sim.submit_route_pass("evaluation: idle drones and open tasks")
            w = sim.snapshot
        if w.scenario_mode is not ScenarioMode.INCIDENT_DEMO:
            return
        if len(w.readings) == self.seen_readings or w.investigation.status.value != "OPEN":
            return
        self.seen_readings = len(w.readings)
        if self.llm and agents.llm_available():
            res = agents.run_llm_turn(sim, "Review the new readings.", (), True, 2)
        else:
            res = agents.run_scripted_turn(sim, conclude=True)
        self.turns.append(
            {
                "sim_s": sim.snapshot.clock_s,
                "finding": res.finding_id,
                "errors": res.errors,
                "transcript": res.transcript if self.llm else [],
            }
        )
        _accept(sim, res.finding_id)


_decision_no = [0]


def _accept(sim, finding_id) -> None:
    if finding_id is None:
        return
    pf = next((p for p in sim.snapshot.proposals if p.id == finding_id), None)
    if pf is None:
        return
    _decision_no[0] += 1
    with contextlib.suppress(CommandRejected):
        sim.submit(
            CommitDecision(f"EV{_decision_no[0]:04d}", pf.id, pf.semantic_key, "evaluation")
        ).result()


def make_policy(name: str, seed: int, llm: bool = False):
    if name == "agent":
        return AgentPolicy(seed, llm)
    if name in ("random", "greedy", "lookahead"):
        return BaselinePolicy(name, seed)
    raise ValueError(f"unknown policy {name!r}")


# -- truth-isolated scoring -----------------------------------------------------------------


def _overlap(a_seg, a0, a1, b_seg, b0, b1) -> bool:
    return a_seg == b_seg and a0 < b1 and b0 < a1


def score_state(sim: Simulation) -> dict:
    """Read hidden truth once and score the public record. Returned dicts live only in results."""
    w, truth = sim.snapshot, sim.evaluator_truth()
    out: dict = {"sim_s": w.clock_s, "readings": len(w.readings)}
    if w.scenario_mode is ScenarioMode.INCIDENT_DEMO:
        out["incident"] = _incident_score(w, truth)
    else:
        out["healthy"] = _healthy_score(w, truth, sim)
    return out


def _incident_score(w, truth) -> dict:
    anomalies = []
    for a in truth.local.anomalies:
        iv = a.interval
        hit = None
        for hz in w.hazards:
            rid = hz.evidence_ids[0] if hz.evidence_ids else None
            r = w.readings.get(rid)
            if (
                r
                and str(r.kind) == KIND_OF[a.kind]
                and _overlap(
                    hz.segment_id, hz.start_mm, hz.end_mm, iv.segment_id, iv.start_mm, iv.end_mm
                )
            ) and (hit is None or r.clock_s < hit.clock_s):
                hit = r
        before = (
            sorted(r.clock_s for r in w.readings.values() if hit and r.clock_s <= hit.clock_s)
            if hit
            else []
        )
        anomalies.append(
            {
                "kind": a.kind,
                "segment": iv.segment_id,
                "detected": hit is not None,
                "time_to_detection_s": hit.clock_s if hit else None,
                "measurements_to_detection": len(before) if hit else None,
            }
        )
    false_hazards = 0
    for hz in w.hazards:
        if not any(
            _overlap(
                hz.segment_id,
                hz.start_mm,
                hz.end_mm,
                a.interval.segment_id,
                a.interval.start_mm,
                a.interval.end_mm,
            )
            for a in truth.local.anomalies
        ):
            false_hazards += 1
    inv = w.investigation
    hyps = list(w.hypotheses.values())
    after = [
        r
        for r in w.readings.values()
        if inv.concluded_s is not None and r.clock_s > inv.concluded_s
    ]
    return {
        "causal_diagnosis": {
            "status": str(inv.status),
            "concluded_s": inv.concluded_s,
            "statement": (inv.conclusion or {}).get("causal_statement"),
        },
        "material_alternatives": {
            "hypotheses": len(hyps),
            "supported": sum(1 for h in hyps if h["assessment"] == "SUPPORTED"),
            "refuted": sum(1 for h in hyps if h["assessment"] == "REFUTED"),
        },
        "anomalies": anomalies,
        "detected": sum(1 for a in anomalies if a["detected"]),
        "of": len(anomalies),
        "scoped_actions": {"hazards_raised": len(w.hazards), "false_hazards": false_hazards},
        "post_completion_readings": len(after),
    }


def _healthy_score(w, truth, sim) -> dict:
    est = w.estimate
    if est is None or not hasattr(est, "flows"):
        return {"flow_error": None, "note": "estimate unavailable"}
    errs, unsupported = [], 0
    for seg, f in est.flows.items():
        true_p = sum(
            truth.circuit_states[c.id].p_from_mw for c in sim.circuits.by_segment.get(seg, ())
        )
        errs.append((seg, abs(f.p_mw - true_p), f.observable))
        if f.uncertainty_radius_95_mw is None:
            unsupported += 1
    obs = [e for _, e, o in errs if o]
    covered = [
        1.0
        for seg, e, _ in errs
        if (
            est.flows[seg].uncertainty_radius_95_mw is not None
            and e <= est.flows[seg].uncertainty_radius_95_mw
        )
    ]
    n_radius = len(errs) - unsupported
    return {
        "flow_error_mw": {
            "mean_all": statistics.fmean(e for _, e, _ in errs) if errs else None,
            "mean_observed": statistics.fmean(obs) if obs else None,
            "max": max((e for _, e, _ in errs), default=None),
        },
        "observability": {"observable": len(est.observable_segments), "flows": len(errs)},
        "calibration": {
            "intervals_supported": n_radius,
            "truth_inside_95": len(covered),
            "unsupported_intervals": unsupported,
        },
        "benchmark_checks": [
            {"name": c.name, "met": c.met, "detail": c.detail}
            for c in conclusion.benchmark_checks(w)
        ],
        "unresolved_work": sorted(
            t.id for t in w.tasks.values() if t.status in (TaskStatus.PENDING, TaskStatus.RESERVED)
        ),
    }


# -- running --------------------------------------------------------------------------------


@dataclass
class RunResult:
    mode: str
    policy: str
    seed: int
    batches: list[dict] = field(default_factory=list)
    final: dict = field(default_factory=dict)
    operations: dict = field(default_factory=dict)
    replay: dict = field(default_factory=dict)
    declared: dict = field(default_factory=lambda: DECLARED)

    def to_json(self) -> str:
        return json.dumps(self.__dict__, indent=2, default=str)


def run(
    mode: str = "incident_demo",
    policy: str = "agent",
    seed: int = 7,
    *,
    minutes: int = DEADLINE_MIN,
    llm: bool = False,
    network=None,
) -> RunResult:
    network = network or load_network()
    sim = Simulation(network, scenario=ScenarioConfig(ScenarioMode(mode), seed))
    sim.submit(SetLandingPolicy(True, "evaluation")).result()
    pol = make_policy(policy, seed, llm)
    if sim.snapshot.scenario_mode is ScenarioMode.INCIDENT_DEMO:  # shared initialization
        res = agents.run_scripted_turn(sim)
        _accept(sim, res.finding_id)
    pol.start(sim)
    result = RunResult(mode, policy, seed)
    phase_min = {n: {} for n in sim.snapshot.drones}
    last_batch = -1
    wall0 = time.perf_counter()
    for _ in range(minutes):
        pol.act(sim)
        sim.step(1)
        w = sim.snapshot
        for did, d in w.drones.items():
            ph = d.phase
            phase_min[did][ph] = phase_min[did].get(ph, 0) + 1
        if len(w.readings) != last_batch:
            last_batch = len(w.readings)
            result.batches.append(score_state(sim))  # scores stay in the results, never in World
        if w.finished:
            break
    w = sim.snapshot
    result.final = score_state(sim)
    passes = w.route_passes
    rep = resilience.latency_report(passes)
    preview = sim.preview_plan() if w.tasks else None
    result.operations = {
        "minutes_in_phase": phase_min,
        "readings": len(w.readings),
        "repeats": sum(1 for t in w.tasks.values() if t.repeat_of_reading_id),
        "landing_failures": sum(1 for _, e in w.events if e.kind == "landed_stranded"),
        "urgent_unassigned": sorted(
            t.id
            for t in w.tasks.values()
            if t.status in (TaskStatus.PENDING, TaskStatus.RESERVED) and t.urgency >= 4
        ),
        "route_passes": len(passes),
        "fallback_frequency": (len(rep.fallbacks) / len(passes)) if passes else 0.0,
        "latency": rep.summary(),
        "solver_bounds": [
            {"stage": s.name, "status": s.status, "objective": s.objective, "bound": s.bound}
            for s in (preview.stages if preview else ())
        ],
        "tiny_case_gap": "verified exact (gap 0) by tests/test_planning.py brute-force check",
    }
    result.replay = {
        "solver": planning.SOLVER_VERSION,
        "budgets": {
            "deterministic": planning.DETERMINISTIC_BUDGET,
            "wall_cap_s": planning.WALL_CAP_S,
            "response_budget_s": planning.RESPONSE_BUDGET_S,
        },
        "wall_watchdog_variability": (
            "wall-clock caps and watchdog kills can change a plan between machines; "
            "deterministic-time limits are replayable, wall caps are not"
        ),
        "manager_turns": getattr(pol, "turns", []),
        "llm": bool(llm and agents.llm_available()),
        "model": agents.MODEL if llm else None,
        "wall_s": round(time.perf_counter() - wall0, 2),
    }
    return result


def write(result: RunResult, out: Path = RESULTS_DIR) -> Path:
    d = out / result.mode
    d.mkdir(parents=True, exist_ok=True)
    path = d / f"{result.policy}-seed{result.seed}.json"
    path.write_text(result.to_json())
    return path


def compare(mode: str, policies, seeds, *, minutes=DEADLINE_MIN, llm=False, out=RESULTS_DIR):
    """Paired seeds: every policy faces the same scenario and observation-keyed noise."""
    network = load_network()
    rows: dict[str, list[dict]] = {p: [] for p in policies}
    for seed in seeds:
        for p in policies:
            r = run(mode, p, seed, minutes=minutes, llm=llm, network=network)
            write(r, out)
            rows[p].append(_headline(r))
    return rows


def _headline(r: RunResult) -> dict:
    f = r.final
    if "incident" in f:
        inc = f["incident"]
        times = [a["time_to_detection_s"] for a in inc["anomalies"] if a["detected"]]
        return {
            "seed": r.seed,
            "detected": inc["detected"],
            "of": inc["of"],
            "mean_time_to_detection_min": (statistics.fmean(times) / 60) if times else None,
            "false_hazards": inc["scoped_actions"]["false_hazards"],
            "post_completion_readings": inc["post_completion_readings"],
            "status": inc["causal_diagnosis"]["status"],
            "readings": r.operations["readings"],
        }
    h = f["healthy"]
    return {
        "seed": r.seed,
        "mean_flow_error_mw": (h.get("flow_error_mw") or {}).get("mean_all"),
        "observable": (h.get("observability") or {}).get("observable"),
        "unsupported_intervals": (h.get("calibration") or {}).get("unsupported_intervals"),
        "unresolved": len(h.get("unresolved_work", [])),
        "readings": r.operations["readings"],
    }


def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="grid-ops-eval", description=__doc__)
    ap.add_argument("--mode", default="incident_demo", choices=[m.value for m in ScenarioMode])
    ap.add_argument("--policy", default="agent", choices=POLICIES)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--seeds", type=int, nargs="*")
    ap.add_argument("--compare", action="store_true")
    ap.add_argument("--minutes", type=int, default=DEADLINE_MIN)
    ap.add_argument("--llm", action="store_true", help="use the live OpenRouter manager")
    ap.add_argument("--out", type=Path, default=RESULTS_DIR)
    a = ap.parse_args(argv)
    if a.compare:
        rows = compare(
            a.mode, POLICIES, a.seeds or [a.seed], minutes=a.minutes, llm=a.llm, out=a.out
        )
        print(json.dumps(rows, indent=2, default=str))
    else:
        r = run(a.mode, a.policy, a.seed, minutes=a.minutes, llm=a.llm)
        print(write(r, a.out))
        print(json.dumps(_headline(r), indent=2, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
