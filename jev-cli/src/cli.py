"""
jev-eval  —  Nomadic crew-dispatch evaluator

Usage:
  python -m src.cli --file hypotheses.json
  python -m src.cli --inline '{"segment_id": "LINE_026", ...}'
  echo '<json>' | python -m src.cli

Output:
  Ranked table of segments with decision + evidence scores.
  Exit code 0 always; individual errors printed per segment.
"""

import argparse
import json
import sys
from dataclasses import asdict

from .evaluate import evaluate
from .types import (
    Decision,
    Report,
    SegmentHypothesis,
    SensorReadings,
    VegetationContext,
)


def _parse_hypothesis(raw: dict) -> SegmentHypothesis:
    reports = [
        Report(
            id=r["id"],
            type=r["type"],
            description=r["description"],
            confidence=r["confidence"],
        )
        for r in raw.get("reports", [])
    ]

    sr = raw.get("sensor_readings", {})
    sensor = SensorReadings(
        vibration_anomaly=sr.get("vibration_anomaly"),
        sag_angle_anomaly=sr.get("sag_angle_anomaly"),
        pd_detected=sr.get("pd_detected"),
        line_live=sr.get("line_live"),
        baseline_comparison=sr.get("baseline_comparison"),
    )

    vc = raw.get("vegetation", {})
    vegetation = VegetationContext(
        forest_cover_nearby=vc.get("forest_cover_nearby", False),
        estimated_tree_height_m=vc.get("estimated_tree_height_m"),
        estimated_line_clearance_m=vc.get("estimated_line_clearance_m"),
        wind_alignment=vc.get("wind_alignment", "unknown"),
        tree_species_fragility=vc.get("tree_species_fragility", "unknown"),
    )

    return SegmentHypothesis(
        segment_id=raw["segment_id"],
        voltage_kv=raw["voltage_kv"],
        length_km=raw["length_km"],
        line_is_live=raw["line_is_live"],
        in_storm_footprint=raw.get("in_storm_footprint", False),
        reports=reports,
        sensor_readings=sensor,
        vegetation=vegetation,
        notes=raw.get("notes"),
    )


DECISION_COLORS = {
    Decision.DISPATCH_CREW: "\033[91m",   # red
    Decision.MEASURE_MORE:  "\033[93m",   # yellow
    Decision.DEPRIORITIZE:  "\033[92m",   # green
}
RESET = "\033[0m"


def _print_table(results):
    # Sort by combined_probability descending
    results.sort(key=lambda r: r.combined_probability, reverse=True)

    header = f"{'Segment':<14} {'P(crew)':<9} {'Evidence':<10} {'Veg':>5} {'Sensor':>7} {'Report':>7} {'Innocent':>9}  Decision"
    print()
    print(header)
    print("-" * len(header))

    for r in results:
        color = DECISION_COLORS.get(r.decision, "")
        e = r.evidence
        print(
            f"{r.segment_id:<14} "
            f"{r.combined_probability:<9.3f} "
            f"{r.evidence_count:<10} "
            f"{e.vegetation_risk:>5.2f} "
            f"{e.sensor_anomaly:>7.2f} "
            f"{e.report_credibility:>7.2f} "
            f"{e.innocent_explanation:>9.2f}  "
            f"{color}{r.decision.value}{RESET}"
        )
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate grid segment hypotheses for crew dispatch using Jev."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--file", "-f", help="Path to JSON file (array or single object).")
    group.add_argument("--inline", "-i", help="Inline JSON string (array or single object).")
    parser.add_argument(
        "--json-out", action="store_true", help="Print full JSON results to stdout."
    )
    args = parser.parse_args()

    if args.file:
        with open(args.file) as fh:
            raw = json.load(fh)
    elif args.inline:
        raw = json.loads(args.inline)
    elif not sys.stdin.isatty():
        raw = json.load(sys.stdin)
    else:
        parser.print_help()
        sys.exit(1)

    if isinstance(raw, dict):
        raw = [raw]

    hypotheses = [_parse_hypothesis(h) for h in raw]

    print(f"Evaluating {len(hypotheses)} segment(s) with Jev…")
    results = []
    for h in hypotheses:
        print(f"  [{h.segment_id}] calling Jev…", end=" ", flush=True)
        try:
            result = evaluate(h)
            results.append(result)
            print("done.")
        except Exception as exc:
            print(f"ERROR: {exc}")

    if args.json_out:
        print(json.dumps([asdict(r) for r in results], indent=2, default=str))
    else:
        _print_table(results)
        for r in results:
            print(r.reasoning)
            print()


if __name__ == "__main__":
    main()
