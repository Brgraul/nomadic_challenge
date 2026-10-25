"""Seeded construction of the hidden AC network and the privileged episode builder."""

from __future__ import annotations

import math
import random
from collections import defaultdict

import pandapower as pp

from grid_ops.circuits import CircuitRegistry
from grid_ops.grid import Network
from grid_ops.scenario import ScenarioInfeasible
from grid_ops.truth.episode import build_episode as build_episode
from grid_ops.truth.local import build_local_field as build_local_field

TRAFO_STD_TYPE = "160 MVA 380/110 kV"
LINE_STD_TYPES = {110.0: "243-AL1/39-ST1A 110.0", 380.0: "490-AL1/64-ST1A 380.0"}
LOAD_MW = (10.0, 60.0)
LOAD_PF = 0.95
GEN_MW = (5.0, 40.0)
GEN_PROBABILITY = 0.4
BOUNDARY_P_MW = (-15.0, 15.0)
BOUNDARY_Q_MVAR = (-5.0, 5.0)
LINE_FACTOR = (0.9, 1.1)  # catalogue impedance times length varies by +-10%
INITIAL_SOURCE_VM_PU = 1.01


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
                bus[(node.id, min(kvs[node.id]))],
                p_mw=p,
                q_mvar=p * math.tan(math.acos(LOAD_PF)),
                name=f"LOAD_{node.id}",
            )
            if node.type == "abstract_substation" and rng.random() < GEN_PROBABILITY:
                pp.create_sgen(
                    net,
                    bus[(node.id, min(kvs[node.id]))],
                    p_mw=rng.uniform(*GEN_MW),
                    name=f"GEN_{node.id}",
                )
        elif node.type == "boundary_endpoint" and node.id not in source_nodes:
            pp.create_load(
                net,
                bus[(node.id, min(kvs[node.id]))],
                p_mw=rng.uniform(*BOUNDARY_P_MW),
                q_mvar=rng.uniform(*BOUNDARY_Q_MVAR),
                name=f"EXCH_{node.id}",
            )
    return net, line_index
