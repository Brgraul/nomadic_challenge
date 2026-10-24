"""Seeded construction of the hidden AC network (`truth.build`)."""

from __future__ import annotations

import math
import random
from collections import Counter, defaultdict

import pandapower as pp

from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import Geometry
from grid_ops.grid import Network
from grid_ops.scenario import (
    MAX_OPERATING_POINTS,
    ScenarioConfig,
    ScenarioInfeasible,
    ScenarioMode,
)
from grid_ops.truth.episode import build_episode as build_episode
from grid_ops.truth.local import build_local_field
from grid_ops.truth.state import Limits, TruthState

TRAFO_STD_TYPE = "160 MVA 380/110 kV"
LINE_STD_TYPES = {110.0: "243-AL1/39-ST1A 110.0", 380.0: "490-AL1/64-ST1A 380.0"}
LOAD_MW = (10.0, 60.0)
LOAD_PF = 0.95
GEN_MW = (5.0, 40.0)
GEN_PROBABILITY = 0.4
BOUNDARY_P_MW = (-15.0, 15.0)
BOUNDARY_Q_MVAR = (-5.0, 5.0)
LINE_FACTOR = (0.9, 1.1)  # catalogue impedance times length varies by +-10%
# Declared fixtures: healthy is tight, incident declares its expected (wider) envelope, and
# switching transitions are checked against the incident envelope in both modes.
HEALTHY_LIMITS = Limits(0.95, 1.05, 60.0)
INCIDENT_LIMITS = Limits(0.90, 1.10, 100.0)
INCIDENT_SWITCHED = ("LINE_024",)
INITIAL_SOURCE_VM_PU = 1.01
# Incident fixture constraint on the operating point: the LINE_029 hot joint must be excitable,
# so its circuits must carry clearly more than the 20 A excitation threshold.
INCIDENT_MIN_CURRENT_A = {"LINE_029/C1": 25.0, "LINE_029/C2": 25.0}


def choose_sources(network: Network) -> dict[str, str]:
    """Highest-voltage boundary per component (ties by node id); else the first substation."""
    by_component: dict[str, list] = defaultdict(list)
    for node in network.nodes.values():
        by_component[node.component].append(node)
    voltage: dict[str, float] = defaultdict(float)
    for seg in network.segments.values():
        for n in (seg.from_node, seg.to_node):
            voltage[n] = max(voltage[n], seg.voltage_kv)
    sources, missing = {}, []
    for component, nodes in sorted(by_component.items()):
        boundaries = [n for n in nodes if n.type == "boundary_endpoint"]
        if boundaries:
            best = min(boundaries, key=lambda n: (-voltage[n.id], n.id))
            sources[component] = best.id
            continue
        subs = sorted(n.id for n in nodes if n.type == "abstract_substation")
        if subs:
            sources[component] = subs[0]
        else:
            missing.append(component)
    if missing:
        raise ScenarioInfeasible(
            "initial components without any source", {"source_less_components": missing}
        )
    return sources


def _make_net(
    network: Network, registry: CircuitRegistry, sources: dict[str, str], rng: random.Random
) -> tuple[pp.pandapowerNet, dict[str, int]]:
    net = pp.create_empty_network(name="bavaria-synthetic-truth", sn_mva=100.0)
    kvs: dict[str, set[float]] = defaultdict(set)
    for c in registry.circuits.values():
        kvs[c.from_node].add(c.nominal_kv)
        kvs[c.to_node].add(c.nominal_kv)
    bus: dict[tuple[str, float], int] = {}
    for node_id in sorted(kvs):
        for kv in sorted(kvs[node_id]):
            bus[(node_id, kv)] = pp.create_bus(net, vn_kv=kv, name=f"{node_id}@{kv:g}")
    line_index: dict[str, int] = {}
    for cid in sorted(registry.circuits):
        c = registry.circuits[cid]
        line_index[cid] = pp.create_line(
            net,
            bus[(c.from_node, c.nominal_kv)],
            bus[(c.to_node, c.nominal_kv)],
            length_km=c.length_km * rng.uniform(*LINE_FACTOR),
            std_type=LINE_STD_TYPES[c.nominal_kv],
            name=cid,
        )
    for node_id, volts in sorted(kvs.items()):
        if len(volts) > 1:
            pp.create_transformer(
                net,
                bus[(node_id, max(volts))],
                bus[(node_id, min(volts))],
                std_type=TRAFO_STD_TYPE,
                name=f"TR_{node_id}",
            )

    def main_bus(node_id: str) -> int:
        return bus[(node_id, min(kvs[node_id]))]

    source_nodes = set(sources.values())
    for component, node_id in sorted(sources.items()):
        pp.create_ext_grid(
            net, bus[(node_id, max(kvs[node_id]))], vm_pu=INITIAL_SOURCE_VM_PU, name=component
        )
    for node in sorted(network.nodes.values(), key=lambda n: n.id):
        if node.id not in kvs:
            continue
        if node.type in ("abstract_substation", "terminal"):
            p = rng.uniform(*LOAD_MW)
            pp.create_load(
                net,
                main_bus(node.id),
                p_mw=p,
                q_mvar=p * math.tan(math.acos(LOAD_PF)),
                name=f"LOAD_{node.id}",
            )
            if node.type == "abstract_substation" and rng.random() < GEN_PROBABILITY:
                pp.create_sgen(
                    net, main_bus(node.id), p_mw=rng.uniform(*GEN_MW), name=f"GEN_{node.id}"
                )
        elif node.type == "boundary_endpoint" and node.id not in source_nodes:
            pp.create_load(
                net,
                main_bus(node.id),
                p_mw=rng.uniform(*BOUNDARY_P_MW),
                q_mvar=rng.uniform(*BOUNDARY_Q_MVAR),
                name=f"EXCH_{node.id}",
            )
    return net, line_index


def build(
    network: Network,
    registry: CircuitRegistry,
    geometry: Geometry,
    config: ScenarioConfig,
    required_chargeable: tuple[str, ...] = (),
) -> TruthState:
    """Try at most MAX_OPERATING_POINTS seeded operating points; the first valid one wins."""
    sources = choose_sources(network)
    local = build_local_field(geometry, network, config)
    healthy = config.mode is ScenarioMode.HEALTHY_BENCHMARK
    limits = HEALTHY_LIMITS if healthy else INCIDENT_LIMITS
    failures: Counter[str] = Counter()
    worst: list[str] = []
    for k in range(MAX_OPERATING_POINTS):
        rng = random.Random(f"{config.seed}|operating-point|{k}")
        net, line_index = _make_net(network, registry, sources, rng)
        if not healthy:
            for seg in INCIDENT_SWITCHED:
                for c in registry.by_segment[seg]:
                    net.line.at[line_index[c.id], "in_service"] = False
        try:
            state = TruthState(
                net,
                network,
                registry,
                geometry,
                config,
                local,
                line_index,
                sources,
                limits,
                INCIDENT_LIMITS,
                k,
            )
        except Exception as e:  # non-convergence etc.
            failures["power flow failed"] += 1
            worst = [f"{type(e).__name__}: {e}"]
            continue
        problems = list(state.initial_problems)
        for cid in required_chargeable:
            site_state = state.circuit_states[cid]
            if not (site_state.energized and site_state.i_from_a >= 5.0):
                problems.append(f"starting circuit {cid} cannot charge a drone")
        if not healthy:
            for cid, minimum in INCIDENT_MIN_CURRENT_A.items():
                if state.circuit_states[cid].i_from_a < minimum:
                    problems.append(f"fixture: {cid} carries under {minimum:g} A")
        if not problems:
            return state
        for p in problems:
            failures[p.split(" ")[0] if p.startswith("starting") else p.split(" ", 1)[-1][:40]] += 1
        worst = sorted(problems, key=lambda p: not p.startswith(("starting", "fixture")))[:6]
    raise ScenarioInfeasible(
        f"no valid operating point in {MAX_OPERATING_POINTS} attempts",
        {
            "attempts": MAX_OPERATING_POINTS,
            "failure_kinds": dict(failures.most_common(8)),
            "last_problems": worst,
            "mode": str(config.mode),
            "limits": limits,
        },
    )
