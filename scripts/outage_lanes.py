"""P3 live lanes: reconstruction, experiment value and the closed measurement loop.

Each lane drives the real public interfaces (the hidden episode answers acquisitions, the public
model reconstructs, the real planner and simulation fly) and writes, beside its terminal
screenshot, a public trace and a separate privileged evaluator file. Nothing here mocks sensing,
energy or optimization; fixtures that need an exact reading publish one through the same Reading
type a drone would.
"""

from __future__ import annotations

import copy
import json
import math
import statistics
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))

from fixtures_three_method import tiny_network  # noqa: E402
from test_outage import Lab, fact, weights_of  # noqa: E402

from grid_ops import agents, outage, planning  # noqa: E402
from grid_ops.circuits import CircuitRegistry, build_registry  # noqa: E402
from grid_ops.coordinator import Coordinator  # noqa: E402
from grid_ops.geometry import Geometry, LinePoint  # noqa: E402
from grid_ops.grid import load_network  # noqa: E402
from grid_ops.outage import StopReason, assess_stop, build_model  # noqa: E402
from grid_ops.routes import validate_sorties  # noqa: E402
from grid_ops.scenario import ScenarioConfig  # noqa: E402
from grid_ops.sensing import ConductorPoint, SensorCapabilities  # noqa: E402
from grid_ops.sim import Simulation  # noqa: E402
from grid_ops.truth.episode import EpisodeSpec, build_episode  # noqa: E402
from grid_ops.world import DEADLINE_S  # noqa: E402

LANES = {
    1: ("regression", "Regression against trunk: isolation holds, no legacy claims"),
    2: ("open-conductor", "Tiny identifiable open conductor is supported"),
    3: ("current-ambiguity", "Two zero-current explanations are retained"),
    4: ("tdr-ambiguity", "Symmetric TDR branches stay ambiguous; next request distinguishes"),
    5: ("confounder", "An unrelated hazard does not conclude causation"),
    6: ("repeats", "Redundant correlated repeats do not inflate confidence"),
    7: ("mismatch", "Out-of-family multi-fault evidence refuses certainty"),
    8: ("efficiency", "Partial charging versus no-charge hopping at the tiny optimum"),
    9: ("budget", "Exhausted candidate or route budget stays visible"),
    10: ("closed-loop", "Request, measurement, revised request in one linked trace"),
}


@dataclass
class Lane:
    n: int
    out: Path
    name: str = ""
    title: str = ""
    assertions: list[dict] = field(default_factory=list)
    facts: dict = field(default_factory=dict)
    private: dict = field(default_factory=dict)
    started: float = field(default_factory=time.perf_counter)

    def __post_init__(self):
        self.name, self.title = LANES[self.n]
        self.dir = self.out / f"lane-{self.n}"
        self.dir.mkdir(parents=True, exist_ok=True)

    def check(self, condition, statement: str) -> bool:
        self.assertions.append({"assertion": statement, "passed": bool(condition)})
        return bool(condition)

    @property
    def passed(self) -> bool:
        return all(a["passed"] for a in self.assertions)

    def finish(self) -> dict:
        elapsed = time.perf_counter() - self.started
        public = {
            "lane": self.n,
            "name": self.name,
            "title": self.title,
            "assertions": self.assertions,
            "facts": self.facts,
            "elapsed_s": round(elapsed, 3),
        }
        (self.dir / "public-trace.json").write_text(json.dumps(public, indent=2, default=str))
        private = self.dir / "private-evaluator.json"
        private.touch(mode=0o600)
        private.write_text(json.dumps(self.private, indent=2, default=str))
        lines = [f"LANE {self.n}  {self.title}", ""]
        lines += [f"{'PASS' if a['passed'] else 'FAIL'}  {a['assertion']}" for a in self.assertions]
        lines += ["", *[f"{k}: {v}" for k, v in self.facts.items()]]
        lines += ["", f"elapsed {elapsed:.2f} s   overall {'PASS' if self.passed else 'FAIL'}"]
        (self.dir / "assertions.log").write_text("\n".join(lines))
        render_png(self.dir / f"{self.name}.png", lines)
        return public


def render_png(path: Path, lines: list[str]) -> None:
    """A terminal-style image of the assertion log. It supplements, never replaces, the trace."""
    from PIL import Image, ImageDraw, ImageFont

    font = None
    for candidate in ("/System/Library/Fonts/Menlo.ttc", "/System/Library/Fonts/Monaco.ttf"):
        try:
            font = ImageFont.truetype(candidate, 15)
            break
        except OSError:
            continue
    font = font or ImageFont.load_default()
    wrapped: list[str] = []
    for line in lines:
        while len(line) > 118:
            wrapped.append(line[:118])
            line = "      " + line[118:]
        wrapped.append(line)
    height = 24 + 20 * len(wrapped)
    image = Image.new("RGB", (1000, height), (18, 20, 24))
    draw = ImageDraw.Draw(image)
    for i, line in enumerate(wrapped):
        colour = (210, 214, 220)
        if line.startswith("PASS"):
            colour = (110, 200, 130)
        elif line.startswith("FAIL"):
            colour = (240, 100, 100)
        elif line.startswith("LANE"):
            colour = (120, 170, 250)
        draw.text((14, 12 + 20 * i), line, fill=colour, font=font)
    image.save(path)


def supported_text(belief) -> str:
    return "; ".join(
        f"{f.kind}:{f.subject}={f.probability:.2f}{'*' if f.supported else ''}"
        for f in belief.supported_facts
        if f.kind != "observed_current"
    )


# -- lane 1 --------------------------------------------------------------------------------------


def trunk_probe(trunk: Path | None) -> dict:
    """Does the P2 trunk expose a public outage posterior? Asked of its source tree."""
    if trunk is None:
        return {"checked": False, "reason": "no trunk worktree supplied"}
    code = (
        "import sys, json; sys.path.insert(0, 'tests'); "
        "import importlib.util as u; "
        "print(json.dumps({'outage_module': u.find_spec('grid_ops.outage') is not None, "
        "'world_field': 'outage_belief' in __import__('grid_ops.world', fromlist=['World'])"
        ".World.__dataclass_fields__}))"
    )
    env = {"PYTHONPATH": str(trunk / "src"), "PATH": "/usr/bin:/bin"}
    import os

    env = {**os.environ, "PYTHONPATH": str(trunk / "src")}
    proc = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, cwd=trunk, env=env
    )
    try:
        return {"checked": True, **json.loads(proc.stdout.strip().splitlines()[-1])}
    except (json.JSONDecodeError, IndexError):
        return {"checked": True, "error": (proc.stderr or proc.stdout)[-300:]}


def lane1(out: Path, trunk: Path | None) -> dict:
    lane = Lane(1, out)
    import grid_ops.sim as sim_module

    seeds = iter(((111, 222), (333, 444)))
    original = sim_module._private_seeds
    sim_module._private_seeds = lambda policy_seed: next(seeds)
    try:
        a, b = Simulation(tiny_network()), Simulation(tiny_network())
    finally:
        sim_module._private_seeds = original
    wa, wb = a.snapshot, b.snapshot
    lane.check(wa.outage_belief == wb.outage_belief, "paired hidden worlds: equal public belief")
    lane.check(wa.experiment_proposals == wb.experiment_proposals, "equal ranked experiments")
    lane.check(
        a.evaluator_truth().manifest()["fault"] != b.evaluator_truth().manifest()["fault"]
        or a.evaluator_truth().manifest() != b.evaluator_truth().manifest(),
        "the hidden episodes themselves differ (privileged check)",
    )
    lane.check(
        wa.estimate is None and wa.consistency is None and wa.envelope is None,
        "legacy MW estimate, consistency and envelope are absent in the three-method mode",
    )
    blob = repr(wa).lower()
    lane.check(
        not any(token in blob for token in ("p_mw", "q_mvar", "episode_seed", "noise_seed")),
        "the public snapshot carries no MW/Mvar payload and no private seed",
    )
    lane.check(
        not any(
            f.supported
            for f in wa.outage_belief.supported_facts
            if f.kind in ("fault_segment", "fault_interval", "faulted_circuit", "service_state")
        ),
        "no fault or service claim is supported before any measurement",
    )
    lane.check(
        {f.kind for f in wa.outage_belief.supported_facts if f.supported}
        <= {"charge_availability"},
        "only the declared starting-perch charge evidence is a supported fact",
    )
    probe = trunk_probe(trunk)
    lane.facts["trunk"] = probe
    if probe.get("checked") and "outage_module" in probe:
        lane.check(
            probe["outage_module"] is False and probe["world_field"] is False,
            "trunk (P2) has no public outage posterior: this is new in the head",
        )
    lane.private = {"episodes": [x.evaluator_truth().manifest() for x in (a, b)]}
    return lane.finish()


# -- lane 2 --------------------------------------------------------------------------------------


def lane2(out: Path) -> dict:
    lane = Lane(2, out)
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.current("LINE_026/C1", 0.5)
    lab.tdr("LINE_026/C1", 0.35)
    lab.tdr("LINE_026/C1", 0.75)
    belief = lab.belief()
    interval = fact(belief, "fault_interval")
    truth = lab.episode.fault.offset_mm
    lane.check(interval is not None and interval.supported, "fault interval is supported")
    if interval is not None:
        lane.check(
            interval.interval.start_mm <= truth <= interval.interval.end_mm,
            "the hidden fault lies inside the supported interval (privileged check)",
        )
        cells = (interval.interval.end_mm - interval.interval.start_mm) / max(
            1, lab.model.branches["LINE_026"].length_mm * 0.6 / lab.model.cells_per_branch
        )
        lane.check(cells <= lab.model.goal.max_interval_cells + 0.01, "width within the goal")
        lane.facts["interval"] = f"{interval.interval.start_mm}-{interval.interval.end_mm} mm"
    stop = assess_stop(lab.model, belief, best_eig_bits=1.0)
    lane.check(stop.reason is StopReason.SUPPORTED, f"stop assessment: {stop.reason}")
    lane.check("declared model" in stop.detail, "the claim is stated as conditional")
    lane.facts["belief"] = supported_text(belief)
    lane.private = {"truth": lab.episode.manifest()}
    return lane.finish()


# -- lane 3 --------------------------------------------------------------------------------------


def lane3(out: Path) -> dict:
    lane = Lane(3, out)
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.current("LINE_026/C1", 0.5)
    belief = lab.belief()
    weights = weights_of(belief)
    tripped = sum(w for k, w in weights.items() if k.startswith("F:LINE_026"))
    unloaded = weights["U:LINE_026"]
    lane.check(unloaded > 0.02 and tripped > 0.02, "both explanations keep posterior mass")
    service = fact(belief, "service_state")
    lane.check(service is not None and not service.supported, "disconnected is not certified")
    lane.check("never certifies" in service.statement, "the statement says why")
    classes = [a.label for a in belief.unresolved_classes]
    lane.check(any("live_unloaded" in c for c in classes), "live-unloaded is a named alternative")
    lane.facts["mass"] = f"disconnected {tripped:.3f}  live-unloaded {unloaded:.3f}"
    lane.private = {"truth": lab.episode.manifest()}
    return lane.finish()


# -- lane 4 --------------------------------------------------------------------------------------


def analytic_branched() -> tuple:
    sys.path.insert(0, str(ROOT / "scripts"))
    import verify_outage

    network = verify_outage.analytic_network(branched=True)
    return network, CircuitRegistry(network, {}), Geometry(network)


def lane4(out: Path) -> dict:
    lane = Lane(4, out)
    network, registry, geometry = analytic_branched()
    model = build_model(network, registry, geometry)
    cap = SensorCapabilities()
    episodes = {}
    for side in ("LEFT", "RIGHT"):
        episodes[side] = build_episode(
            network, registry, geometry, EpisodeSpec(7, 11, side, 0.6, "run"), capabilities=cap
        )

    def take_tdr(ep, circuit, offset):
        from grid_ops.sensing import ExecutedAcquisition, Reading, TdrTest, tdr_access

        cp = ConductorPoint(LinePoint(circuit.split("/")[0], offset), circuit)
        access = tdr_access(cp)
        acq = TdrTest(access.id, cp, access.return_path_ids[0])
        sample = ep.acquire(ExecutedAcquisition("A1", acq, 0, 30, "D", cp))
        return Reading("A1", "T", (), "D", acq, 0, 30, sample.payload, sample.quality), acq

    beliefs = {}
    for side, ep in episodes.items():
        reading, _ = take_tdr(ep, "IN/C1", 500_000)
        lane.check(reading.quality.valid, f"{side}: the TDR from the shared inlet is valid")
        beliefs[side] = outage.reconstruct(model, {"A1": reading}, ())
    left = sum(w.weight for w in beliefs["LEFT"].candidate_weights if "LEFT" in w.candidate_id)
    right = sum(w.weight for w in beliefs["LEFT"].candidate_weights if "RIGHT" in w.candidate_id)
    lane.check(
        abs(left - right) < 0.05, f"symmetric branches stay compatible ({left:.2f}/{right:.2f})"
    )
    lane.check(
        weights_of(beliefs["LEFT"]) == weights_of(beliefs["RIGHT"]),
        "the two hidden worlds give the same public belief after the symmetric test",
    )
    reading, _ = take_tdr(episodes["LEFT"], "IN/C1", 500_000)
    props, _ = outage.propose_experiments(model, beliefs["LEFT"], {"A1": reading})
    best = props[0]
    lane.check(best.eig_bits > 0.5, f"next experiment adds {best.eig_bits:.2f} bits")
    lane.check(
        not (best.method == "tdr" and best.segment_id == "IN"),
        "the next request is not a repeat of the symmetric inlet test",
    )
    lane.check(best.segment_id in ("LEFT", "RIGHT"), f"it targets a branch ({best.segment_id})")
    lane.facts["top_proposal"] = (
        f"{best.method} {best.segment_id} {best.circuit_id} {best.start_mm}"
    )
    # execute exactly the proposed acquisition; the truth is consulted only by the episode itself
    from grid_ops.sensing import ContactCurrent, ExecutedAcquisition, Reading

    if best.method == "tdr":
        reading2, _ = take_tdr(episodes["LEFT"], best.circuit_id, best.start_mm)
    else:
        cp = ConductorPoint(LinePoint(best.segment_id, best.start_mm), best.circuit_id)
        acq2 = ContactCurrent(cp)
        sample = episodes["LEFT"].acquire(ExecutedAcquisition("A1", acq2, 0, 10, "D", cp))
        reading2 = Reading("A2", "T", (), "D", acq2, 0, 10, sample.payload, sample.quality)
    reading2 = replace(reading2, id="A2")
    lane.check(reading2.quality.valid, f"the proposed {best.method} is a valid acquisition")
    after = outage.reconstruct(model, {"A1": reading, "A2": reading2}, ())
    fault_side = after.regions
    mass = {r.segment_id: r.fault_mass for r in fault_side}
    lane.check(max(mass.values()) > 0.9, f"one branch now carries the fault mass {mass}")
    lane.private = {"truth": episodes["LEFT"].manifest()}
    return lane.finish()


# -- lane 5 --------------------------------------------------------------------------------------


def lane5(out: Path) -> dict:
    lane = Lane(5, out)
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.current("LINE_026/C1", 0.5)
    other = lab.geometry.length_mm("LINE_002")
    unrelated = lab.scan("LINE_002", round(other * 0.4), round(other * 0.6))
    # the hidden scene hazard sits at the fault; publish an unrelated low-clearance reading
    forged = replace(unrelated, payload=replace(unrelated.payload, clearance_m=0.3), id="HZ-ROAD")
    lab.readings = {k: v for k, v in lab.readings.items() if k != unrelated.id}
    lab.readings[forged.id] = forged
    belief = lab.belief()
    lane.check(forged.payload.clearance_m < outage.HAZARD_CLEARANCE_M, "a hazard was detected")
    interval = fact(belief, "fault_interval")
    lane.check(
        interval is None or not interval.supported, "the hazard alone supports no fault interval"
    )
    stop = assess_stop(lab.model, belief, best_eig_bits=1.0)
    lane.check(stop.reason is not StopReason.SUPPORTED, f"stop assessment stays {stop.reason}")
    mass = {r.segment_id: r.fault_mass for r in belief.regions}
    lane.check(mass.get("LINE_002", 0) < 0.5, f"the hazard's segment is not concluded ({mass})")
    lane.facts["mass"] = {k: round(v, 3) for k, v in mass.items()}
    lane.private = {"truth": lab.episode.manifest()}
    return lane.finish()


# -- lane 6 --------------------------------------------------------------------------------------


def lane6(out: Path) -> dict:
    lane = Lane(6, out)
    repeated, distinct = Lab(segment="LINE_026"), Lab(segment="LINE_026")
    for _ in range(6):
        repeated.forge_current("LINE_002/C1", 0.5, 90.0)
    for fraction in (0.1, 0.25, 0.4, 0.55, 0.7, 0.85):
        distinct.forge_current("LINE_002/C1", fraction, 90.0)

    def entropy(ws):
        total = sum(ws)
        return -sum(w / total * math.log(w / total) for w in ws if w > 0)

    h_rep = entropy(repeated.belief().regime_weights)
    h_dis = entropy(distinct.belief().regime_weights)
    lane.check(
        h_rep > h_dis,
        f"six repeats keep more uncertainty ({h_rep:.3f}) than six points ({h_dis:.3f})",
    )
    obs = outage._group_obs(repeated.model, repeated.readings).currents
    lane.check(len(obs) == 1, "six repeats are one observation")
    lane.check(
        obs[0]["sigma"]
        >= math.sqrt(repeated.capabilities.systematic_fraction)
        * repeated.capabilities.current_sigma_a,
        "the shared systematic error floor is kept",
    )
    scan = Lab(segment="LINE_026", fraction=0.3)
    lo, hi = round(scan.length * 0.55), round(scan.length * 0.75)
    scan.scan("LINE_026", lo, hi)
    once = weights_of(scan.belief())
    scan.scan("LINE_026", lo, hi)
    scan.scan("LINE_026", lo, hi)
    lane.check(
        weights_of(scan.belief()) == once
        or all(math.isclose(weights_of(scan.belief())[k], once[k], rel_tol=1e-9) for k in once),
        "identical negative scans add no independent evidence",
    )
    proposals_before = outage.value_acquisitions(
        scan.model,
        scan.belief(),
        [("again", scan.readings[sorted(scan.readings)[0]].acquisition)],
        scan.readings,
    )
    lane.check(proposals_before.values[0].eig_bits < 0.05, "a repeat scan has almost no value")
    lane.facts["regime_entropy"] = f"repeats {h_rep:.3f} vs distinct {h_dis:.3f}"
    return lane.finish()


# -- lane 7 --------------------------------------------------------------------------------------


def lane7(out: Path) -> dict:
    lane = Lane(7, out)
    lab = Lab(segment="LINE_026", fraction=0.6)
    for circuit in ("LINE_026/C1", "LINE_002/C1", "LINE_056/C1"):
        lab.forge_current(circuit, 0.5, 0.0)
    belief = lab.belief()
    lane.check(belief.model_checks.mismatch, "the model-mismatch check fires")
    lane.check(belief.residual_weight > 0.5, f"residual mass {belief.residual_weight:.2f}")
    lane.check(
        not any(f.supported for f in belief.supported_facts if f.kind != "observed_current"),
        "no supported claim is issued",
    )
    stop = assess_stop(lab.model, belief, best_eig_bits=2.0)
    lane.check(stop.reason is StopReason.MODEL_MISMATCHED, f"stop: {stop.reason}")
    lane.check(
        any("limited power" in n for n in belief.model_checks.notes),
        "the limit of the check is stated: it does not detect every unmodelled cause",
    )
    # the same detector on single-fault evidence stays quiet
    quiet = Lab(segment="LINE_026", fraction=0.6)
    quiet.current("LINE_026/C1", 0.5)
    lane.check(not quiet.belief().model_checks.mismatch, "an in-family case is not flagged")
    lane.facts["undetected_by_design"] = (
        "a second fault elsewhere that no reading touches is invisible to this check"
    )
    return lane.finish()


# -- lane 8 --------------------------------------------------------------------------------------


def lane8(out: Path) -> dict:
    from fixtures_three_method import contact_task, seed_handshakes, site_at, two_drone_world

    from grid_ops.energy import BatteryState
    from grid_ops.world import Charge

    lane = Lane(8, out)
    w, geo = two_drone_world()
    seed_handshakes(w)
    del w.drones["DRONE_2"]
    w.drones["DRONE_1"].battery = BatteryState(remaining_units=15_000)
    contact_task(w, "T1", site_at(w, "LINE_040", 555_000), "REQ-1")
    result = planning.plan_sorties(w, geo, None)
    best = result.best
    lane.check(best is not None and "T1" in best.task_ids, "the task is planned")
    charges = [
        s.operation.charge_s
        for r in best.sorties
        for s in r.steps
        if isinstance(s.operation, Charge)
    ]
    lane.check(len(charges) == 1, "exactly one partial charge, taken before the first takeoff")
    chosen = charges[0] if charges else 0
    check = validate_sorties(w, geo, None, best.sorties)
    lane.check(check.ok, "the plan replays on the energy law and geometry independently")
    # brute force over every declared charge option on the same single-drone instance
    feasible = []
    for option in planning.CHARGE_OPTIONS_S:
        trial = copy.deepcopy(w)
        trial.drones["DRONE_1"].battery = BatteryState(remaining_units=15_000)
        veh = planning._vehicles(trial, geo)[0]
        if option and option not in veh.charge_options_s:
            continue
        scans, perches = planning._build_nodes(trial, geo, (veh,), [trial.tasks["T1"]], exact=True)
        for j in range(len(perches)):
            req = planning._build_sortie(
                trial,
                geo,
                veh,
                0,
                scans,
                perches,
                (("p", j),),
                lambda v, k, o=option: o if k == -1 else 0,
                trial.revision,
            )
            if req is not None and validate_sorties(trial, geo, None, (req,)).ok:
                feasible.append(option)
    optimum = min(feasible) if feasible else None
    lane.check(optimum is not None, "a no-wait-beyond-need plan exists in the brute-force set")
    lane.check(
        chosen == optimum, f"chosen charge {chosen} s equals the brute-force optimum {optimum} s"
    )
    lane.check(chosen < planning.CHARGE_OPTIONS_S[-1], "no full-charge wait was imposed")
    no_charge = copy.deepcopy(w)
    veh = planning._vehicles(no_charge, geo)[0]
    scans, perches = planning._build_nodes(
        no_charge, geo, (veh,), [no_charge.tasks["T1"]], exact=True
    )
    req = planning._build_sortie(
        no_charge, geo, veh, 0, scans, perches, (("p", 0),), lambda v, k: 0, no_charge.revision
    )
    lane.check(
        req is None or not validate_sorties(no_charge, geo, None, (req,)).ok,
        "hopping without charging is infeasible from this energy",
    )
    lane.facts["chosen_charge_s"] = chosen
    lane.facts["feasible_options_s"] = sorted(set(feasible))
    return lane.finish()


# -- lane 9 --------------------------------------------------------------------------------------


def lane9(out: Path) -> dict:
    lane = Lane(9, out)
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.model = build_model(
        lab.network, lab.registry, lab.geometry, caps=outage.OutageCaps(max_candidates=4)
    )
    lab.current("LINE_026/C1", 0.5)
    lab.tdr("LINE_026/C1", 0.3)
    lab.tdr("LINE_026/C1", 0.75)
    belief = lab.belief()
    lane.check(
        lab.model.truncated and lab.model.omitted_segments, f"omitted: {lab.model.omitted_segments}"
    )
    lane.check(
        belief.truncated and any("truncated" in x for x in belief.limits),
        "limits publish the truncation",
    )
    stop = assess_stop(lab.model, belief, best_eig_bits=0.0)
    lane.check(stop.reason is not StopReason.SUPPORTED, f"no completion: {stop.reason}")
    capped = Lab(segment="LINE_026", fraction=0.6)
    capped.model.caps = outage.OutageCaps(max_alternatives=2)
    pool, _ = outage.experiment_pool(capped.model, capped.belief())
    values = outage.value_acquisitions(capped.model, capped.belief(), pool)
    lane.check(
        len(values.omitted) > 0, f"{len(values.omitted)} acquisitions omitted by the value cap"
    )
    lane.check(
        len({v.key for v in values.values if v.eig_bits >= 0}) <= len(pool),
        "omissions are listed, not silently dropped",
    )
    from fixtures_three_method import contact_task, seed_handshakes, site_at, two_drone_world

    w, geo = two_drone_world()
    seed_handshakes(w)
    contact_task(w, "T1", site_at(w, "LINE_040", 100_000), "REQ-1")
    contact_task(w, "T2", site_at(w, "LINE_025", 100_000), "REQ-2")
    contact_task(w, "T3", site_at(w, "LINE_040", 100_001), "REQ-3")
    plan = planning.plan_sorties(w, geo, None, cap=2)
    lane.check(plan.blocked is not None or plan.omitted, "route budget over-capacity is reported")
    lane.facts["route_budget"] = (
        plan.blocked.reason if plan.blocked else [o.reason for o in plan.omitted]
    )
    lane.private = {"truth": lab.episode.manifest()}
    return lane.finish()


# -- lane 10 -------------------------------------------------------------------------------------


def closed_loop(
    manager_fn, seed: int, rounds: int, network, wait_s: int = 120, belief_valued: bool = True
):
    """Run request -> accept -> plan -> fly -> reading -> revised request on the real runtime."""
    sim = Simulation(network, scenario=ScenarioConfig(seed=seed))
    sim.belief_valued_scheduling = belief_valued
    co = Coordinator(sim, manager_fn=manager_fn)
    trace = []
    turn = co.run_turn_now()
    trace.append({"round": 0, "turn": asdict(turn)})
    idle = 0
    for rnd in range(1, rounds + 1):
        w = sim.snapshot
        if w.investigation.status.value != "OPEN" or w.clock_s >= DEADLINE_S:
            break
        before = len(w.readings)
        sim.step(0)
        route = sim.submit_route_pass("lane")
        for _ in range(wait_s):
            sim.step(1)
            now = sim.snapshot
            if len(now.readings) > before and all(
                d.phase == "perched" for d in now.drones.values()
            ):
                break
        w = sim.snapshot
        new = [r for r in w.readings.values() if int(r.id[1:]) > before]
        row = {
            "round": rnd,
            "route": route[:80],
            "new_readings": [
                {
                    "id": r.id,
                    "method": str(r.acquisition.method),
                    "task": r.task_id,
                    "requests": list(r.request_ids),
                    "valid": r.quality.valid,
                }
                for r in new
            ],
            "clock_s": w.clock_s,
            "batteries": {d.id: d.battery.remaining_units for d in w.drones.values()},
            "stop": f"{w.outage_stop.reason}: {w.outage_stop.detail}",
            "belief": supported_text(w.outage_belief),
        }
        if not new:
            # nothing could be flown: give the policy a turn to ask for something else, but a
            # third idle round in a row is a stall and ends the run
            idle += 1
            sim.step(0)
            turn = co.run_turn_now(())
            row["turn"] = asdict(turn)
            trace.append(row)
            if idle >= 3:
                break
            continue
        idle = 0
        triggers = tuple(i for i, e in w.events if e.kind == "reading")
        turn = co.run_turn_now(triggers)
        row["turn"] = asdict(turn)
        row["requests_after"] = [
            {
                "id": a.id,
                "method": str(a.request.method),
                "status": str(a.status),
                "rationale": a.request.rationale[:120],
            }
            for a in sim.snapshot.requests.values()
        ]
        trace.append(row)
    return sim, trace


def lane10(out: Path, llm: bool, llm_rounds: int) -> dict:
    lane = Lane(10, out)

    def scripted(tb, trigger_ids=()):
        return agents.run_adaptive_scripted_turn(tb, trigger_ids, conclude=True)

    sim, trace = closed_loop(scripted, 7, 8, tiny_network())
    w = sim.snapshot
    truth = sim.evaluator_truth().manifest()["fault"]
    interval = fact(w.outage_belief, "fault_interval")
    lane.check(
        w.investigation.status.value == "CONCLUDED",
        f"scripted policy concluded ({w.investigation.status})",
    )
    lane.check(
        interval is not None
        and interval.supported
        and interval.interval.start_mm <= truth["offset_mm"] <= interval.interval.end_mm,
        "the conclusion covers the hidden fault (privileged check)",
    )
    linked = [r for t in trace for r in t.get("new_readings", [])]
    lane.check(
        bool(linked) and all(r["requests"] for r in linked), "every reading names its request"
    )
    lane.check(
        set(w.investigation.conclusion["causal_link_evidence_ids"]) <= set(w.readings),
        "the conclusion cites executed readings only",
    )
    lane.check(w.clock_s < 3600, "no recharge wait: the clock only advanced by flying")
    lane.facts["scripted_trace"] = trace
    lane.facts["policy_label"] = "scripted requests + optimizer (not LLM agency)"
    lane.private = {"truth": sim.evaluator_truth().manifest()}
    if llm:
        started = time.time()
        captured: list[dict] = []

        def language_model(tb, trigger_ids=()):
            result = agents.run_llm_turn(tb, trigger_ids=trigger_ids)
            captured.append(
                {
                    "snapshot_revision": tb.snapshot.revision,
                    "trigger_ids": list(trigger_ids),
                    "validation_errors": result.errors,
                    "output": result.output.model_dump(mode="json"),
                    "transcript": [
                        {
                            "role": m["role"],
                            "content": str(m["content"])[:1500],
                            "tool_calls": [c.get("name") for c in (m["tool_calls"] or [])],
                        }
                        for m in result.transcript
                    ],
                }
            )
            return result

        language_model.author = "manager-llm"
        sim2, trace2 = closed_loop(language_model, 7, llm_rounds, tiny_network(), wait_s=120)
        (lane.dir / "llm-turns.json").write_text(json.dumps(captured, indent=1, default=str))
        w2 = sim2.snapshot
        turns = [t["turn"] for t in trace2 if "turn" in t] + [trace2[0]["turn"]]
        authors = [p.author for p in w2.proposals]
        lane.facts["llm_proposal_authors"] = authors
        lane.facts["llm_model"] = agents.MODEL
        lane.facts["llm_wall_s"] = round(time.time() - started, 1)
        lane.facts["llm_trace"] = trace2
        lane.check(
            any(t["status"] in ("accepted", "proposed") for t in turns),
            "a real model turn produced a validated proposal",
        )
        methods = {m["method"] for t in trace2 for m in t.get("new_readings", [])}
        lane.facts["llm_methods_executed"] = sorted(methods)
        lane.check(bool(w2.requests), "the LLM's requests reached the optimizer")
        lane.check(
            bool(authors) and all(a == "manager-llm" for a in authors),
            "every recorded proposal is labelled as a language-model turn",
        )
        served = [r for t in trace2 for r in t.get("new_readings", []) if r["requests"]]
        lane.check(bool(served), "an executed reading names the LLM's accepted request")
        used_tools = {c for t in captured for m in t["transcript"] for c in m["tool_calls"]}
        lane.facts["llm_tools_called"] = sorted(used_tools)
        lane.check(
            "reconstruction" in used_tools, "the model consulted the system's reconstruction tool"
        )
        later = [t for t in captured if t["trigger_ids"]]
        lane.check(
            bool(later) and any(t["output"]["measurement_requests"] for t in later),
            "after a reading the model requested again (a changed request)",
        )
        lane.private["llm_truth"] = sim2.evaluator_truth().manifest()
    else:
        lane.facts["llm"] = (
            "not run (pass --llm); scripted and replayed runs stay separately labelled"
        )
    return lane.finish()


# -- performance ---------------------------------------------------------------------------------


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, math.ceil(q * len(ordered)) - 1))]


def perf_probe(out: Path, trunk: Path | None, repeats: int = 12) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    result: dict = {"caps": asdict(outage.OutageCaps())}
    # model construction and the pure belief / value work, tiny analytic and full dataset
    for label, network in (("tiny", tiny_network()), ("full", load_network())):
        registry = build_registry(network)
        geometry = Geometry(network)
        t0 = time.perf_counter()
        model = build_model(network, registry, geometry)
        build_s = time.perf_counter() - t0
        sim = Simulation(network, scenario=ScenarioConfig(seed=3))
        # readings from the live runtime so the likelihoods see real payloads
        co = Coordinator(
            sim,
            manager_fn=lambda tb, trigger_ids=(): agents.run_adaptive_scripted_turn(
                tb, trigger_ids, conclude=True
            ),
        )
        co.run_turn_now()
        sim.step(0)
        sim.submit_route_pass("perf")
        for _ in range(180):
            sim.step(1)
        readings = dict(sim.snapshot.readings)
        cold, warm = [], []
        for i in range(repeats):
            model._echo_cache.clear() if i == 0 else None
            t0 = time.perf_counter()
            belief = outage.reconstruct(model, readings, ())
            (cold if i == 0 else warm).append(time.perf_counter() - t0)
        t0 = time.perf_counter()
        props, values = outage.propose_experiments(model, belief, readings)
        propose_s = time.perf_counter() - t0
        warm_proposals = []
        for _ in range(repeats):
            t0 = time.perf_counter()
            outage.propose_experiments(model, belief, readings)
            warm_proposals.append(time.perf_counter() - t0)
        pool, _ = outage.experiment_pool(model, belief)
        tasks = [(f"T{i}", a) for i, (_, a) in enumerate(pool[: model.caps.max_alternatives])]
        samples = []
        for _ in range(repeats):
            t0 = time.perf_counter()
            outage.conditioned_values(model, belief, tasks, readings)
            samples.append(time.perf_counter() - t0)
        result[label] = {
            "candidates": len(model.candidates),
            "cells_per_branch": model.cells_per_branch,
            "build_model_s": round(build_s, 4),
            "readings": len(readings),
            "belief_cold_s": round(cold[0], 4),
            "belief_warm_p50_s": round(statistics.median(warm), 5),
            "belief_warm_max_s": round(max(warm), 5),
            "propose_experiments_cold_s": round(propose_s, 4),
            "propose_experiments_warm_p50_s": round(statistics.median(warm_proposals), 4),
            "value_20_tasks_p50_s": round(statistics.median(samples), 4),
            "value_20_tasks_p95_s": round(percentile(samples, 0.95), 4),
            "proposals": len(props),
            "bins_max": max((v.bins for v in values.values), default=0),
        }
    # end to end: ingestion -> published belief -> value -> validated route, head vs trunk
    result["route_response"] = {
        label: interleaved_route_response(trunk, repeats, label) for label in ("tiny", "full")
    }
    budget = {"belief_value_s": 0.25, "route_response_s": 2.0, "route_p95_s": 1.0}
    result["budgets"] = budget
    result["verdict"] = {
        "belief_warm_within_budget": all(
            result[k]["belief_warm_p50_s"] <= budget["belief_value_s"] for k in ("tiny", "full")
        ),
        "value_within_budget": all(
            result[k]["value_20_tasks_p50_s"] <= budget["belief_value_s"] for k in ("tiny", "full")
        ),
        "propose_warm_within_budget": all(
            result[k]["propose_experiments_warm_p50_s"] <= budget["belief_value_s"]
            for k in ("tiny", "full")
        ),
        "route_p95_within_1s": all(
            (result["route_response"][k][who]["p95_s"] or 0) <= budget["route_p95_s"]
            for k in ("tiny", "full")
            for who in ("head", "trunk")
            if who in result["route_response"][k]
        ),
    }
    (out / "perf.json").write_text(json.dumps(result, indent=2))
    return result


ROUTE_PROBE = r"""
import json, statistics, sys, time
sys.path.insert(0, "tests")
from fixtures_three_method import tiny_network
from grid_ops import agents
from grid_ops.coordinator import Coordinator
from grid_ops.sim import Simulation
from grid_ops.scenario import ScenarioConfig
from grid_ops.world import DEADLINE_S
adaptive = hasattr(agents, "run_adaptive_scripted_turn")
from grid_ops.grid import load_network
network = tiny_network() if sys.argv[2] == "tiny" else load_network()
sim = Simulation(network, scenario=ScenarioConfig(seed=7))
def adaptive_fn(tb, trigger_ids=()):
    return agents.run_adaptive_scripted_turn(tb, trigger_ids)
fn = adaptive_fn if adaptive else agents.run_scripted_turn
co = Coordinator(sim, manager_fn=fn)
co.run_turn_now()
sim.step(0)
sim.submit_route_pass("warm")
lat = []
for _ in range(int(sys.argv[1])):
    n = len(sim.snapshot.readings)
    for _ in range(240):
        sim.step(1)
        w = sim.snapshot
        if len(w.readings) > n and all(d.phase == "perched" for d in w.drones.values()):
            break
    co.run_turn_now(tuple(i for i, e in sim.snapshot.events if e.kind == "reading"))
    t0 = time.perf_counter()
    sim.step(0)
    sim.submit_route_pass("probe")
    lat.append(time.perf_counter() - t0)
    if sim.snapshot.clock_s >= DEADLINE_S:
        break
print(json.dumps({"adaptive": adaptive, "latencies": lat}))
"""


def interleaved_route_response(trunk: Path | None, repeats: int, network: str = "tiny") -> dict:
    import os

    heads = {"head": ROOT}
    if trunk is not None:
        heads["trunk"] = trunk
    samples: dict[str, list[float]] = {k: [] for k in heads}
    for _ in range(3):  # interleave in small rounds so drift hits both equally
        for name, root in heads.items():
            env = {**os.environ, "PYTHONPATH": str(root / "src")}
            proc = subprocess.run(
                [sys.executable, "-c", ROUTE_PROBE, str(max(4, repeats // 3)), network],
                capture_output=True,
                text=True,
                cwd=root,
                env=env,
            )
            try:
                samples[name] += json.loads(proc.stdout.strip().splitlines()[-1])["latencies"]
            except (json.JSONDecodeError, IndexError):
                samples[name].append(float("nan"))
    summary = {}
    for name, values in samples.items():
        good = [v for v in values if v == v]
        summary[name] = {
            "n": len(good),
            "p50_s": round(statistics.median(good), 4) if good else None,
            "p95_s": round(percentile(good, 0.95), 4) if good else None,
            "max_s": round(max(good), 4) if good else None,
        }
    summary["note"] = (
        f"{network} route pass after fresh evidence (step + plan + validate + apply), warmed, "
        "interleaved head/trunk; "
        "trunk and head run different workloads, so no speed ratio is claimed"
    )
    return summary


def run(out: Path, trunk: Path | None, llm: bool, llm_rounds: int, lanes: list[int]) -> list[dict]:
    results = []
    for n in lanes:
        if n == 1:
            results.append(lane1(out, trunk))
        elif n == 10:
            results.append(lane10(out, llm, llm_rounds))
        else:
            results.append(globals()[f"lane{n}"](out))
    return results
