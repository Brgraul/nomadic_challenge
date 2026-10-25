"""Paired policy ablation over hidden episodes: does value-ranked selection reduce acquisitions?

All policies draw from the same bounded pool of candidate experiments the public model proposes
and use the same optimizer, fleet, energy law and stopping rule. Only the order in which the
pool is turned into requests differs:

  value    the jointly conditioned expected-discrimination batch (the declared objective)
  fixed    the pool in a fixed geographic order, ignoring value
  random   a seeded shuffle of the pool
  value-routing-only   the value-ranked requests, scheduled without belief values (flat awards)

Quality is checked after each policy has stopped, against the privileged answer key.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import sys
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "scripts"))

from fixtures_three_method import tiny_network  # noqa: E402
from outage_lanes import closed_loop  # noqa: E402

from grid_ops import agents  # noqa: E402
from grid_ops.grid import load_network  # noqa: E402
from grid_ops.outage import MIN_USEFUL_BITS  # noqa: E402

BATCH = 3
POLICIES = ("value", "fixed", "random", "value-routing-only")


def policy(kind: str, seed: int):
    rng = random.Random(seed)

    def manager(tb, trigger_ids=()):
        props = list(tb.world.experiment_proposals)
        if kind == "fixed":
            props.sort(key=lambda p: (p.segment_id, p.method, p.circuit_id or "", p.start_mm))
        elif kind == "random":
            rng.shuffle(props)
        if kind != "value":
            usable = [p for p in props if p.eig_bits >= MIN_USEFUL_BITS]
            keep = {id(p): i + 1 for i, p in enumerate(usable[:BATCH])}
            props = [replace(p, batch_order=keep.get(id(p), 0)) for p in props]
            tb.world.experiment_proposals = tuple(props)
        return agents.run_adaptive_scripted_turn(tb, trigger_ids, conclude=True)

    return manager


def episode(kind: str, network, seed: int, rounds: int) -> dict:
    routing_only = kind == "value-routing-only"
    sim, _ = closed_loop(
        policy("value" if routing_only else kind, seed),
        seed,
        rounds,
        network,
        belief_valued=not routing_only,
    )
    w = sim.snapshot
    truth = sim.evaluator_truth().manifest()["fault"]
    fact = next((f for f in w.outage_belief.supported_facts if f.kind == "fault_interval"), None)
    concluded = w.investigation.status.value == "CONCLUDED"
    covers = bool(
        fact
        and fact.supported
        and fact.interval.start_mm <= truth["offset_mm"] <= fact.interval.end_mm
    )
    return {
        "seed": seed,
        "policy": kind,
        "concluded": concluded,
        "correct": concluded and covers,
        "false_certainty": concluded and not covers,
        "unresolved": not concluded,
        "acquisitions": len(w.readings),
        "invalid_acquisitions": sum(1 for r in w.readings.values() if not r.quality.valid),
        "clock_min": round(w.clock_s / 60, 1),
        "stop": str(w.outage_stop.reason),
    }


def mean_ci(values: list[float]) -> tuple[float, float, float]:
    """Mean with a normal-approximation 95% interval (indicative at these sample sizes)."""
    if not values:
        return float("nan"), float("nan"), float("nan")
    m = statistics.fmean(values)
    if len(values) < 2:
        return m, m, m
    half = 1.96 * statistics.stdev(values) / math.sqrt(len(values))
    return m, m - half, m + half


def summarize(rows: list[dict]) -> dict:
    out = {}
    by_policy = {k: [r for r in rows if r["policy"] == k] for k in POLICIES}
    for kind, rs in by_policy.items():
        out[kind] = {
            "episodes": len(rs),
            "correct": sum(r["correct"] for r in rs),
            "false_certainty": sum(r["false_certainty"] for r in rs),
            "unresolved": sum(r["unresolved"] for r in rs),
            "acquisitions_mean_ci": [round(x, 2) for x in mean_ci([r["acquisitions"] for r in rs])],
            "clock_min_mean_ci": [round(x, 1) for x in mean_ci([r["clock_min"] for r in rs])],
        }
    paired = {}
    for other in POLICIES[1:]:
        diffs = []
        for a, b in zip(by_policy["value"], by_policy[other], strict=True):
            if a["correct"] and b["correct"]:  # matched quality: both reached the right answer
                diffs.append(b["acquisitions"] - a["acquisitions"])
        paired[f"{other}_minus_value_acquisitions"] = {
            "matched_pairs": len(diffs),
            "mean_ci": [round(x, 2) for x in mean_ci(diffs)],
        }
    out["paired_at_matched_quality"] = paired
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--tiny", type=int, default=30)
    parser.add_argument("--full", type=int, default=0)
    parser.add_argument("--rounds", type=int, default=14)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    result = {}
    for label, count, network in (
        ("tiny", args.tiny, tiny_network),
        ("full", args.full, load_network),
    ):
        rows = []
        for seed in range(1, count + 1):
            for kind in POLICIES:
                rows.append(episode(kind, network(), seed, args.rounds))
        if rows:
            result[label] = {"rows": rows, "summary": summarize(rows)}
            print(label, json.dumps(result[label]["summary"], indent=1))
    (args.out / "ablation.json").write_text(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
