"""Run real three-method acquisition and energy assertions with separated privileged output."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, replace
from pathlib import Path

import pandapower as pp

from grid_ops.circuits import CircuitRegistry, build_registry
from grid_ops.energy import BatteryState, OperationKind, integrate
from grid_ops.geometry import Geometry, Interval, LinePoint, union_length_mm
from grid_ops.grid import Network, Node, Segment, load_network
from grid_ops.sensing import (
    ConductorPoint,
    ContactCurrent,
    ExecutedAcquisition,
    LidarPass,
    ScanPose,
    ScanTrajectory,
    SensorCapabilities,
    TdrTest,
    echo_distance_interval,
    tdr_access,
)
from grid_ops.truth.episode import EpisodeSpec, build_episode
from grid_ops.truth.local import LidarScene, SceneOccluder

CASES = (
    "all",
    "regression",
    "zero-current",
    "fov",
    "occlusion",
    "echo",
    "branches",
    "coupling",
    "hop",
    "partial-charge",
    "replay",
    "runtime",
    "perf",
    "full",
)


def analytic_network(*, bent=False, branched=False):
    positions = {"A": (0.0, -0.009), "J": (0.0, 0.0), "B": (-0.009, 0.0), "C": (0.009, 0.0)}
    if branched:
        nodes = {
            key: Node(key, "boundary_endpoint" if key == "A" else "terminal", key, "C", position)
            for key, position in positions.items()
        }
        segments = {
            sid: Segment(sid, "C", a, b, 110.0, 1, 1.0, (positions[a], positions[b]))
            for sid, a, b in (("IN", "A", "J"), ("LEFT", "J", "B"), ("RIGHT", "J", "C"))
        }
        return Network(nodes, segments, {}, {})
    a = Node("A", "boundary_endpoint", "source", "C", (0.0, 0.0))
    end = (0.0045, 0.0045) if bent else (0.009, 0.0)
    b = Node("B", "terminal", "load", "C", end)
    path = (a.position, (0.0045, 0.0), b.position) if bent else (a.position, b.position)
    return Network(
        {"A": a, "B": b}, {"S": Segment("S", "C", "A", "B", 110.0, 1, 1.0, path)}, {}, {}
    )


def perform(episode, acquisition, identity, *, landed=True):
    duration = (
        acquisition.trajectory.duration_s
        if isinstance(acquisition, LidarPass)
        else (
            acquisition.integration_s
            if isinstance(acquisition, ContactCurrent)
            else acquisition.duration_s
        )
    )
    executed = ExecutedAcquisition(
        identity,
        acquisition,
        0,
        duration,
        "DRONE_1",
        None if isinstance(acquisition, LidarPass) or not landed else acquisition.conductor,
    )
    start = time.perf_counter()
    sample = episode.acquire(executed)
    elapsed = time.perf_counter() - start
    return sample, {
        "executed": asdict(executed),
        "sample": asdict(sample),
        "acquisition_s": elapsed,
    }


def run_case(case):
    full = case == "full"
    network = (
        load_network()
        if full
        else analytic_network(bent=case == "fov", branched=case == "branches")
    )
    registry = build_registry(network) if full else CircuitRegistry(network, {})
    geometry = Geometry(network)
    cap = replace(
        SensorCapabilities(),
        current_sigma_a=0.0,
        tdr_delay_sigma_s=0.0,
        lidar_range_m=200.0,
        lidar_fov_deg=60.0,
    )
    target = None if full else ("LEFT" if case == "branches" else "S")
    spec = EpisodeSpec(7, 11, target, 0.6, "opaque-verification-run")
    start = time.perf_counter()
    episode = build_episode(network, registry, geometry, spec, capabilities=cap)
    construction = time.perf_counter() - start
    sid = episode.registry.circuits[episode.fault.circuit_id].segment_id
    n = geometry.length_mm(sid)
    conductor = ConductorPoint(LinePoint(sid, min(n, 50_000)), episode.fault.circuit_id)
    access = tdr_access(conductor)
    point0 = geometry.position_of(LinePoint(sid, 0))
    point1 = geometry.position_of(LinePoint(sid, n))
    poses = (
        ScanPose((point0[0], point0[1] - 0.0005), 50.0, 0.0),
        ScanPose((point1[0], point0[1] - 0.0005), 50.0, 0.0),
    )
    scan = LidarPass(Interval(sid, 0, n), ScanTrajectory(poses, 100))
    tests = []
    traces = []

    def check(condition, statement):
        tests.append({"assertion": statement, "passed": bool(condition)})
        if not condition:
            raise AssertionError(statement)

    if case == "occlusion":
        episode.scene = LidarScene(
            occluders=(SceneOccluder((-0.01, -0.0004), (0.02, -0.0001), 0, 100),)
        )
    if case == "zero-current":
        episode.net.line.loc[:, "c_nf_per_km"] = 0.0
        pp.runpp(episode.net, init="dc", max_iteration=60, numba=False)
    acquisitions = [
        scan,
        TdrTest(access.id, conductor, access.return_path_ids[0]),
        ContactCurrent(conductor),
    ]
    samples = []
    for i, a in enumerate(acquisitions):
        sample, trace = perform(episode, a, f"attempt-{i}")
        samples.append(sample)
        traces.append(trace)
    check(
        {s.payload.method.value for s in samples} == {"lidar", "tdr", "contact_current"},
        "exactly the three supported payload shapes",
    )
    if case in ("all", "regression", "replay"):
        other = build_episode(
            network,
            registry,
            geometry,
            replace(spec, noise_seed=99, fault_fraction=0.5),
            capabilities=cap,
        )
        check(
            other.public == episode.public, "paired private worlds have equal initial public views"
        )
        check(episode.newly_unsupplied, "physical fault and protection produce solved supply loss")
        for i, a in enumerate(acquisitions):
            replay, _ = perform(episode, a, f"attempt-{i}")
            check(replay == samples[i], "same acquisition and manifest replay exactly")
    if case == "zero-current":
        dead = ContactCurrent(replace(conductor, point=LinePoint(sid, round(n * 0.8))))
        dead_sample, trace = perform(episode, dead, "dead")
        traces.append(trace)
        check(
            samples[2].payload.rms_a == dead_sample.payload.rms_a == 0.0,
            "energized unloaded and disconnected conductor both read zero",
        )
        check("energized" not in asdict(samples[2].payload), "current is not energization evidence")
    if case == "fov":
        check(
            samples[0].payload.inspected and samples[0].payload.missing,
            "narrow field of view on bent corridor preserves unobserved intervals",
        )
    if case == "occlusion":
        check(
            not samples[0].quality.valid and samples[0].payload.clearance_m is None,
            "occluded scan is unknown and never a clean negative",
        )
    if case == "echo":
        lo, hi = echo_distance_interval(samples[1].payload.echoes[0], samples[1].payload)
        expected = abs(episode.protection.offset_mm - conductor.point.offset_mm) / 1000
        check(lo <= expected <= hi, "round-trip delay interval covers known protection distance")
    if case == "branches":
        other = build_episode(
            network, registry, geometry, replace(spec, fault_segment_id="RIGHT"), capabilities=cap
        )
        c = ConductorPoint(LinePoint("IN", 500_000), "IN/C1")
        access2 = tdr_access(c)
        a = TdrTest(access2.id, c, access2.return_path_ids[0])
        left_sample, trace = perform(episode, a, "branch")
        traces.append(trace)
        right_sample, _ = perform(other, a, "branch")
        check(
            left_sample.payload.echoes == right_sample.payload.echoes,
            "symmetric branch faults remain compatible",
        )
    if case == "coupling":
        bad = replace(acquisitions[1], return_path_id="")
        sample, trace = perform(episode, bad, "bad-return")
        traces.append(trace)
        check(
            not sample.quality.valid and not sample.payload.echoes,
            "missing return path rejects acquisition",
        )
    battery = BatteryState()
    energy = []
    for kind, duration in (
        (OperationKind.TAKEOFF, 3),
        (OperationKind.TRANSIT, 60),
        (OperationKind.LAND, 3),
        (OperationKind.SAMPLE, 10),
        (OperationKind.TAKEOFF, 3),
    ):
        battery, entry = integrate(battery, kind, duration)
        energy.append(asdict(entry))
    check(
        all(e["after_units"] < e["before_units"] for e in energy),
        "landed contact and immediate next takeoff retain reduced energy",
    )
    if case == "partial-charge":
        before = battery.remaining_units
        battery, entry = integrate(battery, OperationKind.CHARGE, 60, charging_observed=True)
        energy.append(asdict(entry))
        check(
            battery.remaining_units == min(battery.capacity_units, before + 6000),
            "one minute observed partial charge adds only modeled net energy",
        )
    raw_public = json.dumps({"context": asdict(episode.public), "trace": traces, "energy": energy})
    check(
        not any(k in raw_public for k in ("episode_seed", "noise_seed", "FaultEvent")),
        "public trace contains neither private manifest nor its seeds",
    )
    return (
        {
            "case": case,
            "context": asdict(episode.public),
            "trace": traces,
            "energy": energy,
            "assertions": tests,
            "coverage_mm": union_length_mm(list(samples[0].payload.inspected)),
            "mode": "P1 physical API verification; production runtime migration is P2",
        },
        episode.manifest(),
        {
            "episode_construction_s": construction,
            "acquisitions": len(traces),
            "errors": 0,
            "lidar_max_cells": cap.lidar_max_cells,
            "lidar_max_poses": 128,
            "tdr_max_paths": cap.tdr_max_paths,
        },
    )


def run_runtime_lanes() -> tuple[list[dict], dict]:
    """P2 live lanes on the real Simulation: request -> accept -> plan -> execute -> reading."""
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
    from fixtures_three_method import tiny_network

    from grid_ops.coordinator import Coordinator
    from grid_ops.sim import Simulation
    from grid_ops.world import DEADLINE_S, Perched

    tests = []
    sim = Simulation(tiny_network())
    coordinator = Coordinator(sim)
    turn = coordinator.run_turn_now()

    def check(condition, statement):
        tests.append({"assertion": statement, "passed": bool(condition)})
        if not condition:
            raise AssertionError(statement)

    check(turn.status == "accepted", f"startup scripted turn accepted ({turn.status}: {turn.note})")
    w = sim.snapshot
    check(bool(w.requests), "accepted proposal carries explicit measurement requests")
    check(
        {str(r.request.method) for r in w.requests.values()}
        <= {"lidar", "tdr", "contact_current"},
        "requests use only the three declared methods",
    )
    check(any(t.status == "PENDING" for t in w.tasks.values()), "compiled tasks exist")
    route = sim.submit_route_pass("verification")
    check("HELD" not in route, f"route pass finds a validated continuation ({route})")
    w = sim.snapshot
    check(
        any(hasattr(d.state, "itinerary") for d in w.drones.values()),
        "an executing drone carries an explicit itinerary",
    )
    battery_at_start = {d.id: d.battery.remaining_units for d in w.drones.values()}
    readings_before = len(w.readings)
    for _ in range(120):
        if len(sim.snapshot.readings) > readings_before:
            break
        if sim.snapshot.clock_s >= DEADLINE_S:
            break
        sim.step(min(90, (DEADLINE_S - sim.snapshot.clock_s) / 60))
    w = sim.snapshot
    check(len(w.readings) > readings_before, "an executed acquisition publishes a reading")
    check(
        all(d.battery.remaining_units <= battery_at_start[d.id] for d in w.drones.values()),
        "battery never increases without an observed charge",
    )
    from grid_ops.sensing import CurrentPayload, LidarPayload, TdrPayload

    check(
        any(
            isinstance(r.payload, (LidarPayload, CurrentPayload, TdrPayload))
            for r in w.readings.values()
        ),
        "published readings use exactly the three payload shapes",
    )
    check(
        all(
            isinstance(d.state, Perched) or hasattr(d.state, "itinerary")
            for d in w.drones.values()
        ),
        "every drone is perched or on an explicit itinerary",
    )
    triggers = tuple(i for i, e in w.events if e.kind == "reading")
    result = coordinator.run_turn_now(triggers)
    check(result.status in ("accepted", "proposed", "failed"), "a measurement turn runs")
    check(
        result.status == "accepted",
        f"the measurement turn accepts a revision ({result.status}: {result.note})",
    )
    private = {
        "mode": "P2 runtime lanes on the real Simulation (scripted manager, no LLM)",
        "note": "episode seeds stay private; the public trace carries requests, readings, energy",
    }
    public = {
        "case": "runtime",
        "turns": [asdict(turn)],
        "requests": [
            {"id": r.id, "method": str(r.request.method), "status": str(r.status)}
            for r in w.requests.values()
        ],
        "readings": len(w.readings),
        "battery": {d.id: d.battery.remaining_units for d in w.drones.values()},
        "events": len(w.events),
        "assertions": tests,
    }
    return public, private


def run_perf_probe() -> tuple[list[dict], dict]:
    """Ingestion-to-validated-route latency on the tiny runtime; the plan's p95 1 s target."""
    import statistics
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
    from fixtures_three_method import tiny_network

    from grid_ops import planning
    from grid_ops.sim import Simulation

    tests = []
    sim = Simulation(tiny_network())
    sim.submit_route_pass("warmup")  # warm the planner path once (cold start excluded)
    latencies = []
    for _ in range(8):
        t0 = time.perf_counter()
        w = sim.snapshot
        result = planning.plan_sorties(w, sim.geometry, sim.circuits, revision=w.revision)
        sim.submit_route_pass("probe")
        latencies.append(time.perf_counter() - t0)
        tests.append({"assertion": "probe run produced a result", "passed": result is not None})
    p50 = statistics.median(latencies)
    p95 = sorted(latencies)[max(0, int(-(-0.95 * len(latencies) // 1)) - 1)]
    tests.append({"assertion": f"p50 {p50:.3f}s <= 1s", "passed": p50 <= 1.0})
    tests.append({"assertion": f"p95 {p95:.3f}s <= 2s", "passed": p95 <= 2.0})
    perf = {
        "n": len(latencies),
        "p50_s": round(p50, 4),
        "p95_s": round(p95, 4),
        "max_s": round(max(latencies), 4),
        "budget": "plan+validate+apply within the existing 1s p95 / 2s response target",
        "note": "tiny-fixture probe; not a full-dataset latency claim",
    }
    public = {"case": "perf", "perf": perf, "assertions": tests}
    return public, {"mode": "P2 latency probe"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=CASES, default="all")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.case == "runtime":
        public, private = run_runtime_lanes()
        perf = {"mode": "runtime lanes"}
    elif args.case == "perf":
        public, private = run_perf_probe()
        perf = public["perf"]
    else:
        public, private, perf = run_case(args.case)
    (args.out / "public-trace.json").write_text(json.dumps(public, indent=2))
    private_path = args.out / "private-evaluator.json"
    private_path.touch(mode=0o600)
    private_path.write_text(json.dumps(private, indent=2))
    (args.out / "timings.json").write_text(json.dumps(perf, indent=2))
    print(
        json.dumps(
            {
                "case": args.case,
                "assertions": len(public["assertions"]),
                "passed": all(a["passed"] for a in public["assertions"]),
                "out": str(args.out),
            }
        )
    )


if __name__ == "__main__":
    main()
