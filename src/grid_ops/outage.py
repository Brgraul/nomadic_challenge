"""Public outage reconstruction: finite candidate model, likelihoods and experiment value.

Everything here is computed from public inputs only: the nominal topology, the declared circuit
registry and sensor capabilities, public priors, and immutable readings. It never receives a
hidden episode, a private seed or a solved operating point, and the candidate pool is built the
same way for every episode with the same public inputs.

The declared model family (published in `PublicOutageModel.limits`) is one primary open-conductor
fault on a radial terminal branch, followed by a supply-side protective trip, plus a live-unloaded
branch alternative and an explicit broad residual. Posterior probabilities are conditional on that
family and on the declared nuisance regimes; they are not calibrated frequencies.
"""

from __future__ import annotations

import hashlib
import math
from collections import defaultdict
from collections.abc import Collection, Iterable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from enum import StrEnum
from itertools import pairwise

import numpy as np

from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import Geometry, Interval
from grid_ops.grid import Network
from grid_ops.sensing import (
    ContactCurrent,
    CurrentPayload,
    EchoEdge,
    LidarPass,
    LidarPayload,
    Reading,
    SensorCapabilities,
    TdrPayload,
    TdrTest,
    trace_echoes,
)

OUTAGE_MODEL_VERSION = "outage-v1"

# -- declared model constants (benchmark settings, not operational standards) --------------------
PROTECTOR_FRACTION = 0.1  # supply-side protective device, as a fraction of the branch length
FAULT_RANGE = (0.2, 0.8)  # one primary fault inside this span of the branch
FAULT_MASS, UNLOADED_MASS, RESIDUAL_MASS = 0.80, 0.15, 0.05
REPORT_AREA_BOOST = 3.0  # public storm reports raise the prior of branches they overlap
REGIMES = ((0.5, 0.25), (1.0, 0.50), (2.0, 0.25))  # nuisance load scale and prior
LOAD_MEDIAN_A, LOAD_SIGMA_LN = 80.0, 1.3
ZERO_SIGMA_A = 2.0  # model spread of a live-unloaded or disconnected conductor's current
ENERGIZED_MIN_A = 5.0  # an RMS reading above this is direct evidence of an energized conductor
HAZARD_P = 0.9  # a primary fault leaves a visible low-clearance condition
HAZARD_RATE_PER_KM = 0.05  # unrelated visible hazards
HAZARD_CLEARANCE_M = 1.0
HAZARD_WINDOW_MM = 10_000
LIDAR_VALID_P = 0.85  # prior chance a planned scan sees the conductor at all
TDR_DETECT_P = 0.9
TDR_CLUTTER = 0.05
TDR_RESIDUAL = 0.3
LIDAR_RESIDUAL = 0.5
LIKELIHOOD_FLOOR = 1e-6
CURRENT_BIN_EDGES = (0.0, 3.0, 10.0, 30.0, 60.0, 120.0, 250.0, 500.0, math.inf)
SIGNATURE_ECHOES = 4
SIGNATURE_LOG_BIN = math.log(1.1)
MIN_USEFUL_BITS = 0.05  # below this an experiment is not worth flying


@dataclass(frozen=True)
class OutageCaps:
    max_candidates: int = 256
    cells_per_branch: int = 6
    max_alternatives: int = 20
    max_bins: int = 32


@dataclass(frozen=True)
class EvidenceGoal:
    """Frozen before policies are compared. The tolerance is a benchmark setting."""

    threshold: float = 0.95
    max_interval_cells: int = 2


@dataclass(frozen=True)
class Candidate:
    id: str
    kind: str  # "open_conductor" | "live_unloaded"
    segment_id: str
    circuit_id: str | None
    start_mm: int
    end_mm: int
    protector_mm: int
    prior: float

    @property
    def interval(self) -> Interval:
        return Interval(self.segment_id, self.start_mm, self.end_mm)


@dataclass(frozen=True)
class Branch:
    segment_id: str
    leaf_node: str
    supply_node: str
    length_mm: int
    protector_mm: int
    weight: float


def supply_nodes(network: Network) -> frozenset[str]:
    """Public supply rule: highest-voltage boundary per component, else the first substation."""
    voltage: dict[str, float] = defaultdict(float)
    for seg in network.segments.values():
        for n in (seg.from_node, seg.to_node):
            voltage[n] = max(voltage[n], seg.voltage_kv)
    by_component: dict[str, list] = defaultdict(list)
    for node in network.nodes.values():
        by_component[node.component].append(node)
    out = set()
    for _, nodes in sorted(by_component.items()):
        boundaries = [n for n in nodes if n.type == "boundary_endpoint"]
        if boundaries:
            out.add(min(boundaries, key=lambda n: (-voltage[n.id], n.id)).id)
            continue
        subs = sorted(n.id for n in nodes if n.type == "abstract_substation")
        if subs:
            out.add(subs[0])
    return frozenset(out)


@dataclass
class PublicOutageModel:
    """The candidate catalogue and the pure forward model. Contains no episode information."""

    network: Network
    registry: CircuitRegistry
    geometry: Geometry
    capabilities: SensorCapabilities
    caps: OutageCaps
    goal: EvidenceGoal
    candidates: tuple[Candidate, ...]
    branches: dict[str, Branch]
    residual_prior: float
    omitted_segments: tuple[str, ...]
    cells_per_branch: int
    limits: tuple[str, ...]
    version: str = OUTAGE_MODEL_VERSION
    _edges: dict = field(default_factory=dict, repr=False, compare=False)
    _echo_cache: dict = field(default_factory=dict, repr=False, compare=False)

    @property
    def truncated(self) -> bool:
        return bool(self.omitted_segments)

    @property
    def index(self) -> dict[str, int]:
        return {c.id: i for i, c in enumerate(self.candidates)}


def build_model(
    network: Network,
    registry: CircuitRegistry,
    geometry: Geometry,
    capabilities: SensorCapabilities | None = None,
    *,
    caps: OutageCaps | None = None,
    goal: EvidenceGoal | None = None,
) -> PublicOutageModel:
    """Enumerate the finite candidate family from public topology and public priors."""
    caps = caps or OutageCaps()
    goal = goal or EvidenceGoal()
    capabilities = capabilities or SensorCapabilities()
    sources = supply_nodes(network)
    degree: dict[str, int] = defaultdict(int)
    for seg in network.segments.values():
        degree[seg.from_node] += 1
        degree[seg.to_node] += 1
    report_segments = {
        seg_id
        for seg_id in network.segments
        if any(
            iv.end_mm > iv.start_mm
            for area in network.areas.values()
            for iv in geometry.clip_intervals(seg_id, area)
        )
    }
    branches: dict[str, Branch] = {}
    for seg in sorted(network.segments.values(), key=lambda s: s.id):
        leaf = next(
            (n for n in (seg.from_node, seg.to_node) if degree[n] == 1 and n not in sources), None
        )
        if leaf is None:
            continue
        supply = seg.to_node if leaf == seg.from_node else seg.from_node
        n = geometry.length_mm(seg.id)
        protector = round(
            n * (PROTECTOR_FRACTION if leaf == seg.to_node else 1 - PROTECTOR_FRACTION)
        )
        weight = REPORT_AREA_BOOST if seg.id in report_segments else 1.0
        branches[seg.id] = Branch(seg.id, leaf, supply, n, protector, weight)

    cells = caps.cells_per_branch
    kept = sorted(branches)
    omitted: list[str] = []

    def circuits_of(seg_id: str) -> int:
        return len(registry.by_segment[seg_id])

    def total(ids, k) -> int:
        return sum(circuits_of(s) * k + 1 for s in ids)

    while total(kept, cells) > caps.max_candidates and cells > 1:
        cells -= 1
    if total(kept, cells) > caps.max_candidates:
        # still too many: drop the lowest-prior branches and publish exactly which ones
        ranked = sorted(kept, key=lambda s: (-branches[s].weight, s))
        kept = []
        for seg_id in ranked:
            if total([*kept, seg_id], cells) <= caps.max_candidates:
                kept.append(seg_id)
        omitted = sorted(set(branches) - set(kept))
        kept = sorted(kept)
    weight_sum = sum(branches[s].weight for s in kept) or 1.0
    candidates: list[Candidate] = []
    for seg_id in kept:
        b = branches[seg_id]
        share = branches[seg_id].weight / weight_sum
        circuits = registry.by_segment[seg_id]
        lo, hi = round(b.length_mm * FAULT_RANGE[0]), round(b.length_mm * FAULT_RANGE[1])
        edges = [lo + round((hi - lo) * i / cells) for i in range(cells + 1)]
        per_fault = FAULT_MASS * share / (len(circuits) * cells)
        for circuit in circuits:
            for i, (a, z) in enumerate(pairwise(edges)):
                candidates.append(
                    Candidate(
                        f"F:{circuit.id}:{i}",
                        "open_conductor",
                        seg_id,
                        circuit.id,
                        a,
                        z,
                        b.protector_mm,
                        per_fault,
                    )
                )
        candidates.append(
            Candidate(
                f"U:{seg_id}",
                "live_unloaded",
                seg_id,
                None,
                0,
                b.length_mm,
                b.protector_mm,
                UNLOADED_MASS * share,
            )
        )
    limits = (
        "Family: one primary open-conductor fault on a radial terminal branch with a "
        f"supply-side protective device at {PROTECTOR_FRACTION:.0%} of the branch; faults on "
        "trunk or looped segments, several simultaneous faults and unmodelled causes are outside "
        "the family and can only appear as residual mass or model mismatch.",
        "Balanced-phase post-trip approximation; load regimes and charging current are nuisance "
        "parameters, not measured quantities.",
        f"Fault position is resolved to {cells} cells per branch, not continuously.",
        "Probabilities are conditional on this model and are not calibrated frequencies.",
    )
    if omitted:
        limits += (f"Candidate pool truncated: branches {omitted} are not represented.",)
    return PublicOutageModel(
        network,
        registry,
        geometry,
        capabilities,
        caps,
        goal,
        tuple(candidates),
        branches,
        RESIDUAL_MASS,
        tuple(omitted),
        cells,
        limits,
    )


# -- forward model: echoes ----------------------------------------------------------------------


def _base_edges(model: PublicOutageModel) -> dict[str, tuple[EchoEdge, ...]]:
    if "base" not in model._edges:
        model._edges["base"] = {
            cid: (
                EchoEdge(
                    (cid, 0),
                    (c.from_node, c.nominal_kv),
                    (c.to_node, c.nominal_kv),
                    model.geometry.length_mm(c.segment_id),
                ),
            )
            for cid, c in model.registry.circuits.items()
        }
    return model._edges["base"]


def _tripped_edges(
    model: PublicOutageModel, cand: Candidate, circuit_id: str, fault_mm: int | None
) -> tuple[EchoEdge, ...]:
    circuit = model.registry.circuits[circuit_id]
    n = model.geometry.length_mm(circuit.segment_id)
    a, b = (circuit.from_node, circuit.nominal_kv), (circuit.to_node, circuit.nominal_kv)
    fault_here = fault_mm is not None and circuit_id == cand.circuit_id
    cuts = sorted({cand.protector_mm, fault_mm} if fault_mm is not None else {cand.protector_mm})
    open_cuts = {cand.protector_mm} | ({fault_mm} if fault_here else set())
    edges: list[EchoEdge] = []
    prev, prev_pos = a, 0
    for i, cut in enumerate(cuts):
        if cut in open_cuts:
            left, right = (circuit_id, i, "L"), (circuit_id, i, "R")
        else:
            left = right = (circuit_id, i, "M")
        edges.append(EchoEdge((circuit_id, len(edges)), prev, left, cut - prev_pos))
        prev, prev_pos = right, cut
    edges.append(EchoEdge((circuit_id, len(edges)), prev, b, n - prev_pos))
    return tuple(edges)


def _adjacency(
    model: PublicOutageModel, overlay: Mapping[str, tuple[EchoEdge, ...]], kv: float
) -> dict:
    adjacency: dict = defaultdict(list)
    edges = dict(_base_edges(model))
    edges.update(overlay)
    for cid, es in edges.items():
        if model.registry.circuits[cid].nominal_kv != kv:
            continue
        for e in es:
            adjacency[e.a].append(e)
            adjacency[e.b].append(e)
    return adjacency


def _trace(
    model: PublicOutageModel,
    cand: Candidate | None,
    circuit_id: str,
    offset_mm: int,
    fault_mm: int | None,
) -> tuple[tuple[tuple[float, float], ...], bool]:
    """Echoes with the fault at `fault_mm`; None means only the protective device is open."""
    circuit = model.registry.circuits[circuit_id]
    if cand is None or cand.kind != "open_conductor":
        key: tuple = ("base", circuit_id, offset_mm)
    elif cand.segment_id == circuit.segment_id:
        key = (cand.id, fault_mm, circuit_id, offset_mm)
    else:
        key = ("outside", cand.segment_id, circuit_id, offset_mm)
    hit = model._echo_cache.get(key)
    if hit is not None:
        return hit
    overlay: dict[str, tuple[EchoEdge, ...]] = {}
    if key[0] != "base":
        here = key[0] != "outside"
        for c in model.registry.by_segment[cand.segment_id]:
            overlay[c.id] = _tripped_edges(model, cand, c.id, fault_mm if here else None)
    adjacency = _adjacency(model, overlay, circuit.nominal_kv)
    edges = overlay.get(circuit_id) or _base_edges(model)[circuit_id]
    start = next((e for e in edges if offset_mm < _edge_end(edges, e)), edges[-1])
    cap = model.capabilities
    result = trace_echoes(
        adjacency,
        start,
        offset_mm - _edge_start(edges, start),
        range_m=cap.tdr_range_m,
        max_paths=cap.tdr_max_paths,
    )
    model._echo_cache[key] = result
    return result


def predicted_echoes(
    model: PublicOutageModel, cand: Candidate | None, circuit_id: str, offset_mm: int
) -> tuple[tuple[tuple[float, float, float], ...], bool]:
    """Echo (distance m, amplitude, position spread m) a TDR test would see under `cand`.

    A tripped branch reflects at its open protective device, which masks everything beyond it;
    the fault itself is visible only from the de-energized side. `cand` None is the unaffected
    network. A candidate on another segment only changes the reflection through its protective
    device, so those share one cached trace per segment. The fault is somewhere in its cell, so
    echoes that move with the fault carry the cell's standard deviation as extra spread.
    """
    circuit = model.registry.circuits[circuit_id]
    if cand is None or cand.kind != "open_conductor" or cand.segment_id != circuit.segment_id:
        echoes, truncated = _trace(model, cand, circuit_id, offset_mm, None)
        return tuple((d, a, 0.0) for d, a in echoes), truncated
    mid = (cand.start_mm + cand.end_mm) // 2
    echoes, truncated = _trace(model, cand, circuit_id, offset_mm, mid)
    ends = [_trace(model, cand, circuit_id, offset_mm, f)[0] for f in (cand.start_mm, cand.end_mm)]
    spread = (cand.end_mm - cand.start_mm) / 1000 / math.sqrt(12)
    fixed = [
        any(abs(d - d2) < 0.5 and a == a2 for other in ends for d2, a2 in other) for d, a in echoes
    ]
    return tuple(
        (d, a, 0.0 if all_ends else spread) for (d, a), all_ends in zip(echoes, fixed, strict=True)
    ), truncated


def predicted_variants(
    model: PublicOutageModel, cand: Candidate | None, circuit_id: str, offset_mm: int
) -> list[tuple[float, tuple[tuple[float, float, float], ...], bool]]:
    """(weight, echoes, truncated) alternatives for one candidate at one coupling point.

    When the coupling point lies inside the candidate's own fault cell the fault may be on either
    side of it, and the two sides reflect differently (the far side is masked by the open
    fault). The cell is split at the point and each part keeps its share of the cell.
    """
    circuit = model.registry.circuits[circuit_id]
    inside = (
        cand is not None
        and cand.kind == "open_conductor"
        and cand.segment_id == circuit.segment_id
        and cand.start_mm < offset_mm < cand.end_mm
    )
    if not inside:
        echoes, truncated = predicted_echoes(model, cand, circuit_id, offset_mm)
        return [(1.0, echoes, truncated)]
    width = cand.end_mm - cand.start_mm
    out = []
    for lo, hi in ((cand.start_mm, offset_mm), (offset_mm, cand.end_mm)):
        echoes, truncated = predicted_echoes(
            model, replace(cand, start_mm=lo, end_mm=hi), circuit_id, offset_mm
        )
        out.append(((hi - lo) / width, echoes, truncated))
    return out


def _edge_start(edges: Sequence[EchoEdge], edge: EchoEdge) -> int:
    return sum(e.length_mm for e in edges[: edges.index(edge)])


def _edge_end(edges: Sequence[EchoEdge], edge: EchoEdge) -> int:
    return _edge_start(edges, edge) + edge.length_mm


# -- evidence ------------------------------------------------------------------------------------


@dataclass(frozen=True)
class OperationalEvidence:
    """A public operational event: a switching report or a charging handshake."""

    id: str
    kind: str  # "switching_report" | "handshake_ok" | "handshake_failed"
    segment_id: str
    circuit_id: str = ""
    clock_s: int = 0


@dataclass(frozen=True)
class CandidateWeight:
    candidate_id: str
    weight: float


@dataclass(frozen=True)
class SupportedFact:
    # fault_segment | fault_interval | faulted_circuit | service_state | observed_current |
    # charge_availability
    kind: str
    subject: str
    statement: str
    probability: float
    supported: bool
    interval: Interval | None = None
    width_mm: int | None = None
    conditional_on: tuple[str, ...] = ()
    evidence_ids: tuple[str, ...] = ()


@dataclass(frozen=True)
class EquivalentAlternatives:
    label: str
    candidate_ids: tuple[str, ...]
    mass: float
    distinguishing_methods: tuple[str, ...] = ()


@dataclass(frozen=True)
class ModelChecks:
    residual_mass: float
    min_predictive_probability: float
    mismatch: bool
    notes: tuple[str, ...] = ()


@dataclass(frozen=True)
class RegionSummary:
    segment_id: str
    fault_mass: float
    unloaded_mass: float
    observed_energized: bool
    lidar_covered_fraction: float


@dataclass(frozen=True)
class OutageBelief:
    revision: int
    model_version: str
    digest: str
    candidate_weights: tuple[CandidateWeight, ...]
    residual_weight: float
    regime_weights: tuple[float, ...]
    supported_facts: tuple[SupportedFact, ...]
    unresolved_classes: tuple[EquivalentAlternatives, ...]
    unknown_regions: tuple[Interval, ...]
    regions: tuple[RegionSummary, ...]
    model_checks: ModelChecks
    evidence_ids: tuple[str, ...]
    truncated: bool
    limits: tuple[str, ...]

    def weights(self, model: PublicOutageModel) -> np.ndarray:
        by_id = {w.candidate_id: w.weight for w in self.candidate_weights}
        return np.array([by_id[c.id] for c in model.candidates])


def _normal_cdf(z: float) -> float:
    return 0.5 * math.erfc(-z / math.sqrt(2))


def _halfnormal_pdf(y: float, sigma: float) -> float:
    return 2 / (sigma * math.sqrt(2 * math.pi)) * math.exp(-0.5 * (y / sigma) ** 2)


def _halfnormal_cdf(y: float, sigma: float) -> float:
    return math.erf(y / (sigma * math.sqrt(2)))


def _lognormal_pdf(y: float, median: float, sigma: float) -> float:
    y = max(y, 0.01)
    return math.exp(-0.5 * ((math.log(y) - math.log(median)) / sigma) ** 2) / (
        y * sigma * math.sqrt(2 * math.pi)
    )


def _lognormal_cdf(y: float, median: float, sigma: float) -> float:
    if y <= 0:
        return 0.0
    if math.isinf(y):
        return 1.0
    return _normal_cdf((math.log(y) - math.log(median)) / sigma)


def _residual_current_pdf(y: float) -> float:
    broad = 1 / (y * math.log(1000 / 3)) if 3 <= y <= 1000 else LIKELIHOOD_FLOOR
    return 0.1 * _halfnormal_pdf(y, 2.5) + 0.9 * broad


def _union_length(intervals: Iterable[Interval]) -> int:
    total, end = 0, None
    for iv in sorted(intervals, key=lambda i: (i.start_mm, i.end_mm)):
        if end is None or iv.start_mm > end:
            total += iv.end_mm - iv.start_mm
            end = iv.end_mm
        elif iv.end_mm > end:
            total += iv.end_mm - end
            end = iv.end_mm
    return total


def _merge(intervals: Iterable[Interval]) -> list[Interval]:
    out: list[Interval] = []
    for iv in sorted(intervals, key=lambda i: (i.segment_id, i.start_mm, i.end_mm)):
        if out and out[-1].segment_id == iv.segment_id and iv.start_mm <= out[-1].end_mm:
            if iv.end_mm > out[-1].end_mm:
                out[-1] = Interval(iv.segment_id, out[-1].start_mm, iv.end_mm)
        else:
            out.append(iv)
    return out


def _subtract(span: Interval, covered: Sequence[Interval]) -> list[Interval]:
    pieces = [span]
    for c in covered:
        if c.segment_id != span.segment_id:
            continue
        nxt = []
        for p in pieces:
            if c.end_mm <= p.start_mm or c.start_mm >= p.end_mm:
                nxt.append(p)
                continue
            if c.start_mm > p.start_mm:
                nxt.append(Interval(p.segment_id, p.start_mm, c.start_mm))
            if c.end_mm < p.end_mm:
                nxt.append(Interval(p.segment_id, c.end_mm, p.end_mm))
        pieces = nxt
    return pieces


def _overlap_mm(region: Sequence[Interval], segment_id: str, lo: int, hi: int) -> int:
    return _union_length(
        Interval(segment_id, max(i.start_mm, lo), min(i.end_mm, hi))
        for i in region
        if i.segment_id == segment_id and min(i.end_mm, hi) > max(i.start_mm, lo)
    )


@dataclass
class _Observations:
    """Valid readings reduced to what the likelihood uses, with repeats merged."""

    currents: list[dict] = field(default_factory=list)
    lidar: list[dict] = field(default_factory=list)
    tdr: list[dict] = field(default_factory=list)
    ids: list[str] = field(default_factory=list)
    capped: dict = field(default_factory=dict)  # (circuit, offset) -> reading id of a path-cap test


def _group_obs(model: PublicOutageModel, readings: Mapping[str, Reading]) -> _Observations:
    rho = model.capabilities.systematic_fraction
    obs = _Observations()
    ordered = sorted(readings.values(), key=lambda r: (r.acquired_end_s, r.id))
    current_groups: dict[tuple, list[Reading]] = defaultdict(list)
    tdr_groups: dict[tuple, list[Reading]] = defaultdict(list)
    covered: list[Interval] = []
    positives: list[Interval] = []
    for r in ordered:
        if not r.quality.valid:
            p = r.payload
            if (
                isinstance(p, TdrPayload)
                and isinstance(r.acquisition, TdrTest)
                and p.path_truncated
                and r.quality.contact_ok
            ):
                c = r.acquisition.conductor
                key = (c.circuit_id, c.point.offset_mm)
                if key not in obs.capped:
                    obs.capped[key] = r.id
                    obs.ids.append(r.id)
            continue
        p = r.payload
        if isinstance(p, CurrentPayload) and isinstance(r.acquisition, ContactCurrent):
            c = p.conductor
            current_groups[(c.circuit_id, c.phase, c.point.offset_mm)].append(r)
        elif isinstance(p, TdrPayload) and isinstance(r.acquisition, TdrTest):
            c = r.acquisition.conductor
            tdr_groups[(c.circuit_id, c.point.offset_mm)].append(r)
        elif isinstance(p, LidarPayload) and isinstance(r.acquisition, LidarPass):
            inspected = list(p.inspected)
            if not inspected:
                continue
            hazard = p.clearance_m is not None and p.clearance_m < HAZARD_CLEARANCE_M
            if hazard:
                region = _merge(inspected)
                if all(_subtract(iv, positives) == [] for iv in region):
                    covered = _merge([*covered, *inspected])
                    continue  # a repeat of an already-seen hazard adds no independent evidence
                positives = _merge([*positives, *inspected])
            else:
                region = [piece for iv in inspected for piece in _subtract(iv, covered)]
                if not region:
                    continue
            covered = _merge([*covered, *inspected])
            obs.lidar.append({"region": region, "hazard": hazard, "id": r.id})
            obs.ids.append(r.id)
    for (circuit_id, _phase, offset), group in sorted(current_groups.items()):
        n = len(group)
        ys = [g.payload.rms_a for g in group]
        sigma = group[0].payload.uncertainty_a
        eff = sigma * math.sqrt(rho + (1 - rho) / n)
        obs.currents.append(
            {
                "circuit": circuit_id,
                "segment": circuit_id.split("/")[0],
                "offset": offset,
                "y": float(np.mean(ys)),
                "sigma": eff,
                "saturated": any(g.quality.saturated for g in group),
                "ids": tuple(g.id for g in group),
            }
        )
        obs.ids.extend(g.id for g in group)
    for (circuit_id, offset), group in sorted(tdr_groups.items()):
        n = len(group)
        first = group[0].payload
        counts = {len(g.payload.echoes) for g in group}
        echoes = []
        for i, e in enumerate(first.echoes):
            delay = (
                float(np.mean([g.payload.echoes[i].delay_s for g in group]))
                if len(counts) == 1
                else e.delay_s
            )
            sigma_t = e.delay_uncertainty_s * math.sqrt(rho + (1 - rho) / n)
            echoes.append((delay * first.propagation_mps / 2, e.polarity, sigma_t))
        obs.tdr.append(
            {
                "circuit": circuit_id,
                "offset": offset,
                "echoes": echoes,
                "rel": first.propagation_uncertainty_mps / first.propagation_mps,
                "speed": first.propagation_mps,
                "range": first.usable_range_m,
                "ids": tuple(g.id for g in group),
            }
        )
        obs.ids.extend(g.id for g in group)
    return obs


def _zero_class(cand: Candidate, segment_id: str) -> bool:
    return cand.segment_id == segment_id


def _current_loglik(model: PublicOutageModel, o: dict) -> tuple[np.ndarray, float]:
    """Log density of a current observation per candidate and regime, and under the residual."""
    y, sigma = o["y"], o["sigma"]
    s_zero = math.sqrt(ZERO_SIGMA_A**2 + sigma**2)
    ll = np.zeros((len(model.candidates), len(REGIMES)))
    sat = o["saturated"]
    for k, cand in enumerate(model.candidates):
        if _zero_class(cand, o["segment"]):
            ll[k, :] = math.log(
                max(
                    LIKELIHOOD_FLOOR,
                    (1 - _halfnormal_cdf(y, s_zero)) if sat else _halfnormal_pdf(y, s_zero),
                )
            )
        else:
            for r, (scale, _) in enumerate(REGIMES):
                median = LOAD_MEDIAN_A * scale
                dens = (
                    1 - _lognormal_cdf(y, median, LOAD_SIGMA_LN)
                    if sat
                    else _lognormal_pdf(y, median, LOAD_SIGMA_LN)
                )
                ll[k, r] = math.log(max(LIKELIHOOD_FLOOR, dens))
    res = math.log(
        max(LIKELIHOOD_FLOOR, 1 - _halfnormal_cdf(y, 2.5) if sat else _residual_current_pdf(y))
    )
    return ll, res


def _cell_cover(cand: Candidate, region: Sequence[Interval]) -> float:
    if cand.kind != "open_conductor":
        return 0.0
    lo, hi = cand.start_mm - HAZARD_WINDOW_MM, cand.end_mm + HAZARD_WINDOW_MM
    return _overlap_mm(region, cand.segment_id, lo, hi) / (hi - lo)


def _lidar_loglik(model: PublicOutageModel, o: dict) -> tuple[np.ndarray, float]:
    region = o["region"]
    q = 1 - math.exp(-HAZARD_RATE_PER_KM * _union_length(region) / 1e6)
    out = np.zeros(len(model.candidates))
    for k, cand in enumerate(model.candidates):
        seen = HAZARD_P * _cell_cover(cand, region)
        p_hazard = 1 - (1 - seen) * (1 - q)
        out[k] = math.log(max(LIKELIHOOD_FLOOR, p_hazard if o["hazard"] else 1 - p_hazard))
    return out, math.log(LIDAR_RESIDUAL)


def _echo_match(predicted: Sequence[tuple[float, float, float]], o: dict) -> float:
    """Log likelihood of an observed echo list given one predicted list (soft assignment)."""
    observed = [e for e in o["echoes"] if e[0] <= o["range"]]
    c_half = o["speed"] / 2
    log_l = 0.0
    claimed = [False] * len(observed)
    for dist, amp, spread in predicted:
        if dist > o["range"]:
            continue
        polarity = "positive" if amp >= 0 else "negative"
        sigma = max(1.0, math.hypot(o["rel"] * dist, spread))
        best = 0.0
        for i, (d_obs, pol, sigma_t) in enumerate(observed):
            if pol != polarity:
                continue
            s = math.sqrt(sigma**2 + (c_half * sigma_t) ** 2)
            z = (d_obs - dist) / s
            m = math.exp(-0.5 * z * z)
            if m > best:
                best = m
            if abs(z) <= 3:
                claimed[i] = True
        log_l += math.log((1 - TDR_DETECT_P) + TDR_DETECT_P * best)
    log_l += sum(math.log(TDR_CLUTTER) for hit in claimed if not hit)
    return log_l


def _tdr_loglik(model: PublicOutageModel, o: dict) -> tuple[np.ndarray, float]:
    index = {c.id: c for c in model.candidates}
    out = np.zeros(len(model.candidates))
    for k, cand in enumerate(model.candidates):
        total = 0.0
        for weight, predicted, truncated in predicted_variants(
            model, index[cand.id], o["circuit"], o["offset"]
        ):
            total += weight * (
                LIKELIHOOD_FLOOR if truncated else math.exp(_echo_match(predicted, o))
            )
        out[k] = math.log(max(total, LIKELIHOOD_FLOOR))
    res = len([e for e in o["echoes"] if e[0] <= o["range"]]) * math.log(TDR_RESIDUAL)
    return out, res


def _operational_loglik(
    model: PublicOutageModel, e: OperationalEvidence
) -> tuple[np.ndarray, float]:
    out = np.zeros(len(model.candidates))
    if e.kind == "switching_report":
        for k, cand in enumerate(model.candidates):
            hit = cand.kind == "open_conductor" and cand.segment_id == e.segment_id
            out[k] = math.log(0.9 if hit else 0.1)
        return out, math.log(0.3)
    return out, 0.0


def _logsumexp(a: np.ndarray, axis=None):
    m = np.max(a, axis=axis, keepdims=True)
    s = np.log(np.sum(np.exp(a - m), axis=axis, keepdims=True)) + m
    return np.squeeze(s, axis=axis) if axis is not None else float(np.squeeze(s))


def reconstruct(
    model: PublicOutageModel,
    readings: Mapping[str, Reading],
    operational: Sequence[OperationalEvidence] = (),
) -> OutageBelief:
    """Posterior over the candidate family from immutable readings and operational evidence.

    A pure function of the evidence set: duplicates and ordering cannot change the result.
    """
    obs = _group_obs(model, readings)
    k_n, r_n = len(model.candidates), len(REGIMES)
    ll = np.zeros((k_n, r_n))
    ll_res = 0.0
    for o in obs.currents:
        a, b = _current_loglik(model, o)
        ll += a
        ll_res += b
    for o in obs.lidar:
        a, b = _lidar_loglik(model, o)
        ll += a[:, None]
        ll_res += b
    for o in obs.tdr:
        a, b = _tdr_loglik(model, o)
        ll += a[:, None]
        ll_res += b
    for circuit_id, offset in sorted(obs.capped):
        # a deterministic physical fact: the walk hit the path cap, so no echo list exists
        a = np.array(
            [
                math.log(0.95 if predicted_echoes(model, c, circuit_id, offset)[1] else 0.02)
                for c in model.candidates
            ]
        )
        ll += a[:, None]
        ll_res += math.log(0.5)
    op_ids: list[str] = []
    for e in sorted({e.id: e for e in operational}.values(), key=lambda e: e.id):
        a, b = _operational_loglik(model, e)
        ll += a[:, None]
        ll_res += b
        op_ids.append(e.id)
    prior = np.array([c.prior for c in model.candidates])
    regime = np.array([p for _, p in REGIMES])
    log_joint = np.log(prior)[:, None] + np.log(regime)[None, :] + ll
    log_res = math.log(model.residual_prior) + ll_res
    norm = float(np.logaddexp(_logsumexp(log_joint), log_res))
    joint = np.exp(log_joint - norm)
    residual = math.exp(log_res - norm)
    weights = joint.sum(axis=1)
    regime_post = joint.sum(axis=0)
    evidence_ids = tuple(sorted({*obs.ids, *op_ids}))
    digest = hashlib.blake2b("|".join(evidence_ids).encode(), digest_size=6).hexdigest()
    belief = OutageBelief(
        revision=len(evidence_ids),
        model_version=model.version,
        digest=digest,
        candidate_weights=tuple(
            CandidateWeight(c.id, float(w)) for c, w in zip(model.candidates, weights, strict=True)
        ),
        residual_weight=float(residual),
        regime_weights=tuple(float(x) for x in regime_post),
        supported_facts=(),
        unresolved_classes=(),
        unknown_regions=(),
        regions=(),
        model_checks=ModelChecks(float(residual), 1.0, False),
        evidence_ids=evidence_ids,
        truncated=model.truncated,
        limits=model.limits,
    )
    return _summarize(model, belief, obs, ll, readings, operational)


# -- posterior summaries -------------------------------------------------------------------------


def _predictive_check(model, belief, obs, weights) -> float:
    """Smallest posterior-predictive probability of any observed outcome class."""
    smallest = 1.0
    regime = np.array(belief.regime_weights) / max(sum(belief.regime_weights), 1e-12)
    for o in obs.currents:
        y = o["y"]
        edges = CURRENT_BIN_EDGES
        b = next(i for i in range(len(edges) - 1) if edges[i] <= y < edges[i + 1])
        p = 0.0
        for k, cand in enumerate(model.candidates):
            if _zero_class(cand, o["segment"]):
                s = math.sqrt(ZERO_SIGMA_A**2 + o["sigma"] ** 2)
                pb = _halfnormal_cdf(edges[b + 1], s) - _halfnormal_cdf(edges[b], s)
                p += weights[k] * pb
            else:
                for r, (scale, _) in enumerate(REGIMES):
                    med = LOAD_MEDIAN_A * scale
                    pb = _lognormal_cdf(edges[b + 1], med, LOAD_SIGMA_LN) - _lognormal_cdf(
                        edges[b], med, LOAD_SIGMA_LN
                    )
                    p += weights[k] * regime[r] * pb
        smallest = min(smallest, p)
    return smallest


def _summarize(model, belief, obs, ll, readings, operational) -> OutageBelief:
    weights = belief.weights(model)
    total = float(weights.sum())
    facts: list[SupportedFact] = []
    goal = model.goal
    pp = _predictive_check(model, belief, obs, weights / total if total > 0 else weights)
    mismatch = belief.residual_weight > 0.5 or (obs.currents and pp < 1e-3)
    notes = []
    if belief.residual_weight > 0.5:
        notes.append("the broad residual explains the readings better than any modelled candidate")
    if obs.currents and pp < 1e-3:
        notes.append(f"a current reading has posterior-predictive probability {pp:.1e}")
    notes.append(
        "limited power: the residual and predictive checks do not detect every unmodelled cause"
    )
    checks = ModelChecks(belief.residual_weight, pp, bool(mismatch), tuple(notes))

    seg_mass: dict[str, float] = defaultdict(float)
    unloaded: dict[str, float] = defaultdict(float)
    for c, w in zip(model.candidates, weights, strict=True):
        (seg_mass if c.kind == "open_conductor" else unloaded)[c.segment_id] += float(w)
    conditions = model.limits[:1]
    ids = belief.evidence_ids
    best_seg = max(seg_mass, key=seg_mass.get, default=None)
    if best_seg is not None:
        p_seg = seg_mass[best_seg]
        facts.append(
            SupportedFact(
                "fault_segment",
                best_seg,
                f"Primary open-conductor fault lies on {best_seg} (conditional on the declared "
                "model family).",
                p_seg,
                p_seg >= goal.threshold and not mismatch,
                None,
                None,
                conditions,
                ids,
            )
        )
        cells = [
            (c, float(w))
            for c, w in zip(model.candidates, weights, strict=True)
            if c.kind == "open_conductor" and c.segment_id == best_seg
        ]
        by_cell: dict[tuple[int, int], float] = defaultdict(float)
        for c, w in cells:
            by_cell[(c.start_mm, c.end_mm)] += w
        ordered = sorted(by_cell.items())
        best = None
        for i in range(len(ordered)):
            mass = 0.0
            for j in range(i, len(ordered)):
                mass += ordered[j][1]
                if mass >= goal.threshold:
                    width = ordered[j][0][1] - ordered[i][0][0]
                    if best is None or width < best[0]:
                        best = (width, ordered[i][0][0], ordered[j][0][1], mass, j - i + 1)
                    break
        if best is not None:
            width, lo, hi, mass, n_cells = best
            ok = n_cells <= goal.max_interval_cells and not mismatch
            facts.append(
                SupportedFact(
                    "fault_interval",
                    best_seg,
                    f"Fault within {best_seg} {lo}-{hi} mm (95% region spans {n_cells} cell(s)).",
                    mass,
                    ok,
                    Interval(best_seg, lo, hi),
                    width,
                    conditions,
                    ids,
                )
            )
        circuits: dict[str, float] = defaultdict(float)
        for c, w in cells:
            circuits[c.circuit_id] += w
        top_c = max(circuits, key=circuits.get, default=None)
        if top_c is not None and len(model.registry.by_segment[best_seg]) > 1:
            facts.append(
                SupportedFact(
                    "faulted_circuit",
                    top_c,
                    f"The faulted conductor circuit is {top_c}.",
                    circuits[top_c],
                    circuits[top_c] >= goal.threshold and not mismatch,
                    None,
                    None,
                    conditions,
                    ids,
                )
            )
    for o in obs.currents:
        if o["y"] > ENERGIZED_MIN_A:
            facts.append(
                SupportedFact(
                    "observed_current",
                    o["circuit"],
                    f"{o['circuit']} carried {o['y']:.1f} A RMS at {o['offset']} mm: energized "
                    "there when sampled.",
                    1.0,
                    True,
                    None,
                    None,
                    (),
                    o["ids"],
                )
            )
    zero_obs = {o["segment"]: o for o in obs.currents if o["y"] <= ENERGIZED_MIN_A}
    for seg_id, o in sorted(zero_obs.items()):
        p_trip, p_unl = seg_mass.get(seg_id, 0.0), unloaded.get(seg_id, 0.0)
        has_other = any(t["circuit"].split("/")[0] == seg_id for t in obs.tdr) or any(
            e.segment_id == seg_id and e.kind == "switching_report" for e in operational
        )
        certified = p_trip >= goal.threshold and has_other and not mismatch
        facts.append(
            SupportedFact(
                "service_state",
                seg_id,
                f"Zero current on {seg_id}: disconnected after a trip ({p_trip:.2f}) or "
                f"live-unloaded ({p_unl:.2f}); zero current alone never certifies de-energization"
                + ("; here echo or switching evidence separates them." if certified else "."),
                p_trip,
                certified,
                None,
                None,
                conditions,
                o["ids"],
            )
        )
    for e in sorted({e.id: e for e in operational}.values(), key=lambda e: e.id):
        if e.kind.startswith("handshake"):
            ok = e.kind == "handshake_ok"
            facts.append(
                SupportedFact(
                    "charge_availability",
                    e.circuit_id,
                    f"Charging handshake on {e.circuit_id} {'succeeded' if ok else 'failed'}; "
                    "recharge availability is independent of fault location and service state.",
                    1.0,
                    True,
                    None,
                    None,
                    (),
                    (e.id,),
                )
            )
    classes = _equivalence_classes(model, weights, ll)
    inspected = [
        iv
        for r in readings.values()
        if isinstance(r.payload, LidarPayload) and r.quality.valid
        for iv in r.payload.inspected
    ]
    merged = _merge(inspected)
    unknown = _unknown_regions(model, weights, merged)
    regions = []
    energized_segments = {o["segment"] for o in obs.currents if o["y"] > ENERGIZED_MIN_A}
    for seg_id in sorted(model.branches):
        n = model.branches[seg_id].length_mm
        regions.append(
            RegionSummary(
                seg_id,
                seg_mass.get(seg_id, 0.0),
                unloaded.get(seg_id, 0.0),
                seg_id in energized_segments,
                _overlap_mm(merged, seg_id, 0, n) / max(1, n),
            )
        )
    return OutageBelief(
        belief.revision,
        belief.model_version,
        belief.digest,
        belief.candidate_weights,
        belief.residual_weight,
        belief.regime_weights,
        tuple(facts),
        classes,
        unknown,
        tuple(regions),
        checks,
        belief.evidence_ids,
        belief.truncated,
        belief.limits,
    )


def _equivalence_classes(model, weights, ll) -> tuple[EquivalentAlternatives, ...]:
    """Candidates with identical likelihood on every acquired observation cannot be told apart."""
    groups: dict[tuple, list[int]] = defaultdict(list)
    for k in range(len(model.candidates)):
        groups[
            tuple([*np.round(ll[k] - ll[k].max(), 3).tolist(), round(float(ll[k].max()), 3)])
        ].append(k)
    out = []
    for members in groups.values():
        mass = float(sum(weights[k] for k in members))
        if mass < 0.02:
            continue
        cands = [model.candidates[k] for k in members]
        segs = sorted({c.segment_id for c in cands})
        kinds = sorted({c.kind for c in cands})
        label = f"{'/'.join(kinds)} on {', '.join(segs[:3])}" + ("..." if len(segs) > 3 else "")
        methods = (
            ("tdr",) if len(cands) > 1 and len(segs) == 1 else ("tdr", "lidar", "contact_current")
        )
        out.append(EquivalentAlternatives(label, tuple(c.id for c in cands), mass, methods))
    return tuple(sorted(out, key=lambda a: -a.mass)[:12])


def _unknown_regions(model, weights, inspected) -> tuple[Interval, ...]:
    """Fault cells still carrying material mass that no LiDAR pass has inspected."""
    out: list[Interval] = []
    for c, w in zip(model.candidates, weights, strict=True):
        if c.kind != "open_conductor" or w < 0.01:
            continue
        out.extend(_subtract(c.interval, inspected))
    return tuple(_merge(out))


# -- experiment value ----------------------------------------------------------------------------


@dataclass(frozen=True)
class ExperimentValue:
    key: str
    method: str
    eig_bits: float
    bins: int
    note: str = ""


@dataclass(frozen=True)
class ExperimentValues:
    values: tuple[ExperimentValue, ...]
    omitted: tuple[str, ...] = ()

    def by_key(self) -> dict[str, ExperimentValue]:
        return {v.key: v for v in self.values}


def acquisition_key(acquisition) -> str:
    if isinstance(acquisition, LidarPass):
        t = acquisition.target
        return f"lidar:{t.segment_id}:{t.start_mm}-{t.end_mm}"
    c = acquisition.conductor
    return f"{acquisition.method.value}:{c.circuit_id}:{c.phase}:{c.point.offset_mm}"


def _entropy(p: np.ndarray, axis=-1) -> np.ndarray:
    with np.errstate(divide="ignore", invalid="ignore"):
        return -np.sum(np.where(p > 0, p * np.log2(p), 0.0), axis=axis)


def _outcome_matrix(model, belief, acquisition, readings_covered) -> tuple[np.ndarray, str]:
    """P(outcome | candidate) with regimes marginalised by their posterior; residual excluded."""
    k_n = len(model.candidates)
    regime = np.array(belief.regime_weights)
    regime = regime / max(regime.sum(), 1e-12)
    if isinstance(acquisition, ContactCurrent):
        seg = acquisition.conductor.circuit_id.split("/")[0]
        edges = CURRENT_BIN_EDGES
        p = np.zeros((k_n, len(edges) - 1))
        s_zero = math.sqrt(ZERO_SIGMA_A**2 + model.capabilities.current_sigma_a**2)
        for k, cand in enumerate(model.candidates):
            if _zero_class(cand, seg):
                cdf = [_halfnormal_cdf(e, s_zero) if not math.isinf(e) else 1.0 for e in edges]
                p[k] = np.diff(cdf)
            else:
                for r, (scale, _) in enumerate(REGIMES):
                    med = LOAD_MEDIAN_A * scale
                    cdf = [_lognormal_cdf(e, med, LOAD_SIGMA_LN) for e in edges]
                    p[k] += regime[r] * np.diff(cdf)
        return p, "current magnitude class"
    if isinstance(acquisition, LidarPass):
        t = acquisition.target
        region = _subtract(Interval(t.segment_id, t.start_mm, t.end_mm), readings_covered)
        q = 1 - math.exp(-HAZARD_RATE_PER_KM * _union_length(region) / 1e6)
        p = np.zeros((k_n, 3))  # invalid, hazard, clear
        for k, cand in enumerate(model.candidates):
            seen = HAZARD_P * _cell_cover(cand, region)
            p_h = 1 - (1 - seen) * (1 - q)
            p[k] = [1 - LIDAR_VALID_P, LIDAR_VALID_P * p_h, LIDAR_VALID_P * (1 - p_h)]
        return p, "hazard seen / clear / no view"
    c = acquisition.conductor
    # bin 0 is an invalid test: where the electrical path cap leaves only partial echoes under a
    # candidate, the acquisition is rejected rather than informative
    signatures: dict[tuple | None, int] = {None: 0}
    rows: list[list[tuple[int, float]]] = []
    for cand in model.candidates:
        row = []
        for weight, echoes, truncated in predicted_variants(
            model, cand, c.circuit_id, c.point.offset_mm
        ):
            sig = (
                None
                if truncated
                else tuple(
                    (1 if a >= 0 else -1, int(math.log(max(d, 1.0)) / SIGNATURE_LOG_BIN))
                    for d, a, _ in echoes[:SIGNATURE_ECHOES]
                )
            )
            row.append((signatures.setdefault(sig, len(signatures)), weight))
        rows.append(row)
    bins = min(len(signatures), model.caps.max_bins - 1) + 1
    p = np.zeros((k_n, bins))
    for k, row in enumerate(rows):
        for b, weight in row:
            p[k, min(b, bins - 1)] += weight
    return p, "echo signature"


def _eig(p: np.ndarray, weights: np.ndarray) -> float:
    w = weights / max(weights.sum(), 1e-12)
    marginal = w @ p
    return float(max(0.0, _entropy(marginal) - w @ _entropy(p, axis=1)))


def value_acquisitions(
    model: PublicOutageModel,
    belief: OutageBelief,
    acquisitions: Sequence[tuple[str, object]],
    readings: Mapping[str, Reading] | None = None,
) -> ExperimentValues:
    """Expected discrimination (bits about the candidate) per acquisition, public data only.

    Repeats of an already-acquired conductor point are discounted by the shared-error fraction;
    a repeated LiDAR span counts only its uninspected part, and a repeated echo test adds nothing.
    """
    readings = readings or {}
    weights = belief.weights(model)
    covered = _merge(
        iv
        for r in readings.values()
        if isinstance(r.payload, LidarPayload) and r.quality.valid
        for iv in r.payload.inspected
    )
    taken = {
        acquisition_key(r.acquisition): r
        for r in readings.values()
        if r.quality.valid
        or (isinstance(r.payload, TdrPayload) and r.payload.path_truncated and r.quality.contact_ok)
    }
    rho = model.capabilities.systematic_fraction
    by_key: dict[str, ExperimentValue] = {}
    values: list[ExperimentValue] = []
    omitted: list[str] = []
    for label, acq in acquisitions:
        key = acquisition_key(acq)
        if key not in by_key:
            if len(by_key) >= model.caps.max_alternatives:
                omitted.append(label)
                continue
            p, note = _outcome_matrix(model, belief, acq, covered)
            eig = _eig(p, weights)
            if key in taken:
                eig = 0.0 if not isinstance(acq, ContactCurrent) else eig * (1 - rho)
                note += "; repeat of an acquired point"
            by_key[key] = ExperimentValue(label, str(acq.method.value), eig, p.shape[1], note)
        values.append(replace(by_key[key], key=label))
    return ExperimentValues(tuple(values), tuple(omitted))


def joint_value(
    model: PublicOutageModel,
    belief: OutageBelief,
    acquisitions: Sequence[object],
    readings: Mapping[str, Reading] | None = None,
) -> float:
    """Expected discrimination of a small batch, outcomes taken jointly (at most three)."""
    if len(acquisitions) > 3:
        raise ValueError("joint value is enumerated for at most three acquisitions")
    readings = readings or {}
    covered = _merge(
        iv
        for r in readings.values()
        if isinstance(r.payload, LidarPayload) and r.quality.valid
        for iv in r.payload.inspected
    )
    weights = belief.weights(model)
    joint = None
    seen: set[str] = set()
    for acq in acquisitions:
        key = acquisition_key(acq)
        if key in seen:
            continue  # an identical acquisition in one batch carries no new outcome
        seen.add(key)
        p, _ = _outcome_matrix(model, belief, acq, covered)
        joint = p if joint is None else np.einsum("ka,kb->kab", joint, p).reshape(len(p), -1)
    return 0.0 if joint is None else _eig(joint, weights)


def rank_batch(
    model: PublicOutageModel,
    belief: OutageBelief,
    acquisitions: Sequence[object],
    size: int,
    readings: Mapping[str, Reading] | None = None,
) -> list[object]:
    """Greedy conditioned reranking: each pick maximises the joint value given earlier picks."""
    chosen: list[object] = []
    pool = list(acquisitions)
    current = 0.0
    while pool and len(chosen) < min(size, 3):
        best, best_gain = None, 1e-9
        for acq in pool:
            gain = joint_value(model, belief, [*chosen, acq], readings) - current
            if gain > best_gain:
                best, best_gain = acq, gain
        if best is None:
            break
        chosen.append(best)
        pool.remove(best)
        current += best_gain
    return chosen


def conditioned_values(
    model: PublicOutageModel,
    belief: OutageBelief,
    acquisitions: Sequence[tuple[str, object]],
    readings: Mapping[str, Reading] | None = None,
) -> ExperimentValues:
    """Standalone values re-expressed as conditioned gains for the best three.

    The best acquisition keeps its value, the second and third are valued by what they add to the
    earlier picks, and every other acquisition by what it adds to the best pick. Gains are
    therefore not summed across a plan beyond that bounded approximation.
    """
    alone = value_acquisitions(model, belief, acquisitions, readings)
    by_label = dict(acquisitions)
    ranked = [v for v in sorted(alone.values, key=lambda v: -v.eig_bits) if v.eig_bits > 0]
    chosen: list[object] = []
    gains: dict[str, float] = {}
    current = 0.0
    seen_keys: set[str] = set()
    for v in ranked:
        acq = by_label[v.key]
        key = acquisition_key(acq)
        if key in seen_keys or len(chosen) >= 3:
            continue
        seen_keys.add(key)
        total = joint_value(model, belief, [*chosen, acq], readings)
        gains[key] = max(0.0, total - current)
        current = max(current, total)
        chosen.append(acq)
    anchor = chosen[:1]
    base = joint_value(model, belief, anchor, readings) if anchor else 0.0
    out = []
    for v in alone.values:
        acq = by_label[v.key]
        key = acquisition_key(acq)
        if key in gains:
            bits = gains[key]
        elif anchor and v.eig_bits > 0:
            bits = max(0.0, joint_value(model, belief, [*anchor, acq], readings) - base)
        else:
            bits = 0.0
        out.append(replace(v, eig_bits=min(bits, v.eig_bits)))
    return ExperimentValues(tuple(out), alone.omitted)


@dataclass(frozen=True)
class ExperimentProposal:
    method: str
    segment_id: str
    circuit_id: str | None
    start_mm: int
    end_mm: int
    eig_bits: float
    note: str
    batch_order: int = 0  # 1..3: jointly conditioned pick for the next round; 0 otherwise


MAX_PROPOSAL_SEGMENTS = 6
NUDGE_MM = 20_000
BATCH_SIZE = 3


def experiment_pool(
    model: PublicOutageModel,
    belief: OutageBelief,
    occupied: Collection[tuple[str, int]] = (),
) -> tuple[list[tuple[str, object]], dict[str, ExperimentProposal]]:
    """The bounded candidate acquisitions around the leading segments, and their descriptions."""
    from grid_ops.geometry import LinePoint
    from grid_ops.sensing import ConductorPoint, ScanPose, ScanTrajectory, tdr_access

    weights = belief.weights(model)
    seg_mass: dict[str, float] = defaultdict(float)
    for c, w in zip(model.candidates, weights, strict=True):
        seg_mass[c.segment_id] += float(w)
    ordered = sorted(seg_mass.items(), key=lambda kv: (-kv[1], kv[0]))
    top = [s for s, m in ordered if m >= 0.01][:MAX_PROPOSAL_SEGMENTS]
    pool: list[tuple[str, object]] = []
    meta: dict[str, ExperimentProposal] = {}

    def add(acq, method, seg_id, circuit_id, start, end) -> None:
        label = acquisition_key(acq)
        if label not in meta:
            pool.append((label, acq))
            meta[label] = ExperimentProposal(method, seg_id, circuit_id, start, end, 0.0, "")

    def free(circuit_id: str, x: int, n: int) -> int:
        while (circuit_id, x) in occupied and x + NUDGE_MM < n:
            x += NUDGE_MM
        return x

    for seg_id in top:
        n = model.branches[seg_id].length_mm
        circuits = model.registry.by_segment[seg_id]
        first = ConductorPoint(
            LinePoint(seg_id, free(circuits[0].id, round(n * 0.5), n)), circuits[0].id
        )
        add(
            ContactCurrent(first),
            "contact_current",
            seg_id,
            circuits[0].id,
            first.point.offset_mm,
            first.point.offset_mm,
        )
        for circuit in circuits[:3]:
            for frac in (0.05, 0.3, 0.5, 0.7, 0.95):
                x = free(circuit.id, round(n * frac), n)
                cp = ConductorPoint(LinePoint(seg_id, x), circuit.id)
                access = tdr_access(cp)
                add(
                    TdrTest(access.id, cp, access.return_path_ids[0]),
                    "tdr",
                    seg_id,
                    circuit.id,
                    x,
                    x,
                )
        lo, hi = round(n * FAULT_RANGE[0]), round(n * FAULT_RANGE[1])
        mid = (lo + hi) // 2
        pos = model.geometry.position_of
        for a, z in ((lo, hi), (lo, mid), (mid, hi)):
            poses = (
                ScanPose(pos(LinePoint(seg_id, a)), 50.0, 0.0),
                ScanPose(pos(LinePoint(seg_id, z)), 50.0, 0.0),
            )
            add(
                LidarPass(Interval(seg_id, a, z), ScanTrajectory(poses, 30)),
                "lidar",
                seg_id,
                None,
                a,
                z,
            )
    return pool, meta


def propose_experiments(
    model: PublicOutageModel,
    belief: OutageBelief,
    readings: Mapping[str, Reading] | None = None,
    *,
    limit: int | None = None,
    occupied: Collection[tuple[str, int]] = (),
) -> tuple[tuple[ExperimentProposal, ...], ExperimentValues]:
    """Bounded, ranked candidate experiments for the manager to turn into requests.

    `occupied` holds the (circuit, offset) sites drones already perch on; coupling points are
    nudged along the line so a proposal never needs a site another drone is holding.

    The first `BATCH_SIZE` picks are chosen jointly: each is the best addition given the earlier
    ones, so two probes that would separate the same alternatives are not both proposed.
    """
    limit = limit or model.caps.max_alternatives
    pool, meta = experiment_pool(model, belief, occupied)
    values = value_acquisitions(model, belief, pool, readings)
    ranked = sorted(values.values, key=lambda v: (-v.eig_bits, v.key))
    by_label = dict(pool)
    useful = [by_label[v.key] for v in ranked if v.eig_bits >= MIN_USEFUL_BITS][:12]
    batch = rank_batch(model, belief, useful, BATCH_SIZE, readings)
    order = {acquisition_key(acq): i + 1 for i, acq in enumerate(batch)}
    proposals = tuple(
        ExperimentProposal(
            meta[v.key].method,
            meta[v.key].segment_id,
            meta[v.key].circuit_id,
            meta[v.key].start_mm,
            meta[v.key].end_mm,
            round(v.eig_bits, 4),
            v.note,
            order.get(v.key, 0),
        )
        for v in ranked[:limit]
    )
    return proposals, values


# -- stopping ------------------------------------------------------------------------------------


class StopReason(StrEnum):
    CONTINUE = "continue"
    SUPPORTED = "supported"
    AMBIGUOUS = "ambiguous"
    MODEL_MISMATCHED = "model_mismatched"
    NO_SAFE_EXPERIMENT = "no_safe_experiment"
    BUDGET_EXHAUSTED = "budget_exhausted"
    MODEL_LIMITED = "model_limited"


@dataclass(frozen=True)
class StopAssessment:
    reason: StopReason
    detail: str
    unresolved: tuple[str, ...] = ()


def assess_stop(
    model: PublicOutageModel,
    belief: OutageBelief,
    *,
    best_eig_bits: float,
    experiments_available: bool = True,
    budget_exhausted: bool = False,
) -> StopAssessment:
    """Why the investigation should continue or stop, never an unconditional all-clear."""
    unresolved = tuple(f"{a.label} ({a.mass:.2f})" for a in belief.unresolved_classes[:4])
    if belief.model_checks.mismatch:
        return StopAssessment(
            StopReason.MODEL_MISMATCHED,
            "The readings are not explained by the declared model family; no supported claim.",
            unresolved,
        )
    interval = next(
        (f for f in belief.supported_facts if f.kind == "fault_interval" and f.supported), None
    )
    if interval is not None:
        if belief.truncated:
            return StopAssessment(
                StopReason.MODEL_LIMITED,
                "Supported only inside a truncated candidate pool; not an unconditional claim.",
                unresolved,
            )
        return StopAssessment(
            StopReason.SUPPORTED,
            f"{interval.statement} Conditional on the declared model.",
            unresolved,
        )
    if budget_exhausted:
        return StopAssessment(
            StopReason.BUDGET_EXHAUSTED, "Time or energy budget exhausted.", unresolved
        )
    if not experiments_available:
        return StopAssessment(
            StopReason.NO_SAFE_EXPERIMENT, "No safe experiment can currently be flown.", unresolved
        )
    if best_eig_bits < MIN_USEFUL_BITS:
        return StopAssessment(
            StopReason.AMBIGUOUS,
            "Remaining alternatives cannot be separated by any available method.",
            unresolved,
        )
    return StopAssessment(
        StopReason.CONTINUE, f"Best available experiment adds {best_eig_bits:.2f} bits.", unresolved
    )
