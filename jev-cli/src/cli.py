"""
jev-eval  —  Nomadic crew-dispatch evaluator (vegetation focus)

Usage:
  python -m src.cli --file hypotheses.json
  python -m src.cli --inline '{"segment_id": "LINE_026", ...}'
  echo '<json>' | python -m src.cli
"""

import argparse
import json
import sys
from dataclasses import asdict

from .evaluate import evaluate
from .types import (
    Decision, ImpactContext, Report, SegmentHypothesis, SensorReadings, VegetationRisk,
)


def _parse_hypothesis(raw: dict) -> SegmentHypothesis:
    reports = [
        Report(id=r["id"], type=r["type"], description=r["description"], confidence=r["confidence"])
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

    vr = raw.get("vegetation", {})
    vegetation = VegetationRisk(
        forest_overlap_pct=vr.get("forest_overlap_pct", 0.0),
        nearest_forest_m=vr.get("nearest_forest_m"),
        max_canopy_height_m=vr.get("max_canopy_height_m"),
        mean_canopy_height_m=vr.get("mean_canopy_height_m"),
        estimated_clearance_m=vr.get("estimated_clearance_m"),
        dominant_species=vr.get("dominant_species", "unknown"),
        wind_exposure_score=vr.get("wind_exposure_score", 0.0),
    )

    ic = raw.get("impact", {})
    impact = ImpactContext(
        downstream_households=ic.get("downstream_households", 0),
        downstream_population=ic.get("downstream_population", 0),
        downstream_power_plants_mw=ic.get("downstream_power_plants_mw", 0.0),
        downstream_renewable_plants=ic.get("downstream_renewable_plants", 0),
        downstream_hospitals=ic.get("downstream_hospitals", 0),
        downstream_water_works=ic.get("downstream_water_works", 0),
        downstream_care_homes=ic.get("downstream_care_homes", 0),
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
        impact=impact,
        notes=raw.get("notes"),
    )


COLORS = {
    Decision.DISPATCH_CREW: "\033[91m",
    Decision.MEASURE_MORE:  "\033[93m",
    Decision.DEPRIORITIZE:  "\033[92m",
}
RESET = "\033[0m"


def _print_table(results):
    results.sort(key=lambda r: r.dispatch_priority, reverse=True)
    header = (
        f"{'Segment':<14} {'P(tree)':<9} {'P(impact)':<11} {'Priority':<10} "
        f"{'Report':>7} {'Innocent':>9}  Decision"
    )
    print()
    print(header)
    print("-" * len(header))
    for r in results:
        color = COLORS.get(r.decision, "")
        e = r.evidence
        print(
            f"{r.segment_id:<14} "
            f"{r.vegetation_contact_probability:<9.2f} "
            f"{r.impact_probability:<11.2f} "
            f"{r.dispatch_priority:<10.3f} "
            f"{e.report_credibility:>7.2f} "
            f"{e.innocent_explanation:>9.2f}  "
            f"{color}{r.decision.value}{RESET}"
        )
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate grid segments for vegetation-caused faults and crew dispatch."
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--file", "-f", help="JSON file (array or single object).")
    group.add_argument("--inline", "-i", help="Inline JSON string.")
    parser.add_argument("--json-out", action="store_true", help="Print full JSON results.")
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

    print(f"Evaluating {len(hypotheses)} segment(s)…")
    results = []
    for h in hypotheses:
        print(f"  [{h.segment_id}] calling Gemini…", end=" ", flush=True)
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
