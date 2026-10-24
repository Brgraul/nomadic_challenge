"""Offline held-out calibration of the DC-versus-AC discrepancy envelope.

For each held-out seed, build the hidden AC operating point, balance the DC model with the
same injections (the declared source absorbs losses) and record the aggregate-P difference per
segment. Bounds are an empirical quantile over the calibration split; coverage is measured on a
separate validation split. Active scenario truth is never consulted.
"""

from __future__ import annotations

import json
from collections import defaultdict

import numpy as np

from grid_ops.circuits import build_registry
from grid_ops.envelope import ENVELOPE_PATH
from grid_ops.estimation import ElectricalModel
from grid_ops.geometry import Geometry
from grid_ops.grid import load_network
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.truth.build import build, choose_sources
from grid_ops.truth.state import TruthState

CALIBRATION_SEEDS = tuple(range(100, 112))
VALIDATION_SEEDS = tuple(range(112, 120))
QUANTILE = 0.95
START = tuple(f"{s}/C1" for s in ["LINE_040", "LINE_025", "LINE_026", "LINE_002", "LINE_056"])


def dc_vs_ac(truth: TruthState, model: ElectricalModel) -> dict[str, float]:
    """DC aggregate flow minus AC aggregate from-terminal P, per segment, in MW."""
    net = truth.net
    inj = np.zeros(len(model.nodes))
    bus_node = {int(i): name.split("@")[0] for i, name in net.bus.name.items()}
    for bus_index, node in bus_node.items():
        inj[model.node_index[node]] += -float(net.res_bus.at[bus_index, "p_mw"])
    # DC balance: each component's declared source absorbs the AC losses
    sources = choose_sources(model.network)
    for comp_nodes in model.components:
        gap = inj[list(comp_nodes)].sum()
        names = {model.nodes[i] for i in comp_nodes}
        src = next((s for s in sources.values() if s in names), None)
        if src is not None:
            inj[model.node_index[src]] -= gap
    theta, *_ = np.linalg.lstsq(model.B, inj, rcond=None)
    out = {}
    for seg in model.active_segments():
        ac = sum(truth.circuit_states[c.id].p_from_mw for c in model.registry.by_segment[seg])
        out[seg] = float(model.flow_rows[seg] @ theta) - ac
    return out


def calibrate(
    calibration_seeds=CALIBRATION_SEEDS, validation_seeds=VALIDATION_SEEDS, quantile=QUANTILE
) -> dict:
    network = load_network()
    registry = build_registry(network)
    geometry = Geometry(network)
    model = ElectricalModel(network, registry)

    def samples(seeds):
        acc = defaultdict(list)
        for seed in seeds:
            cfg = ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, seed)
            truth = build(network, registry, geometry, cfg, START)
            for seg, d in dc_vs_ac(truth, model).items():
                acc[seg].append(abs(d))
        return acc

    cal, val = samples(calibration_seeds), samples(validation_seeds)
    bounds = {}
    for seg, values in cal.items():
        bound = float(np.quantile(values, quantile))
        inside = sum(v <= bound for v in val[seg]) / max(len(val[seg]), 1)
        bounds[seg] = {
            "bound_mw": round(bound, 4),
            "sample_count": len(values),
            "empirical_coverage": round(inside, 4),
        }
    return {
        "version": "discrepancy-v1",
        "domain": "healthy_benchmark operating family: all circuits in service, epoch E0, "
        "seeded loads 10-60 MW, generation 5-40 MW, boundary exchanges +-15 MW",
        "calibration_seeds": list(calibration_seeds),
        "validation_seeds": list(validation_seeds),
        "quantile": quantile,
        "bounds": bounds,
    }


def main() -> None:
    data = calibrate()
    ENVELOPE_PATH.parent.mkdir(parents=True, exist_ok=True)
    ENVELOPE_PATH.write_text(json.dumps(data, indent=1, sort_keys=True) + "\n")
    worst = max(b["bound_mw"] for b in data["bounds"].values())
    low = min(b["empirical_coverage"] for b in data["bounds"].values())
    print(
        f"wrote {ENVELOPE_PATH}: {len(data['bounds'])} segments, max bound {worst:.1f} MW, "
        f"min validation coverage {low:.2f}"
    )


if __name__ == "__main__":
    main()
