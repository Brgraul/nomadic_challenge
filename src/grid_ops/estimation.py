"""Public reduced DC model: estimates from accepted real measurements, observability.

Unknowns are one angle per mapped node, minus one reference per currently connected reduced
component. Real measurements are aggregate from-terminal P per segment (summed over the segment's
circuits, with the public calibration covariance including correlations). Junctions and
intermediate nodes carry exact zero-injection constraints; substations, boundaries and the
terminal carry wide Gaussian injection priors (generation positive). The solve eliminates
references and exact constraints with a rank-revealing factorisation and never inverts a normal
matrix. This model is a deliberate simplification of the hidden AC network; its discrepancy is
quantified separately (ticket 08) and never read from hidden truth.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
import scipy.linalg as sla

from grid_ops.circuits import BASE_MVA, CircuitRegistry
from grid_ops.envelope import DiscrepancyEnvelope
from grid_ops.grid import Network

if TYPE_CHECKING:
    from grid_ops.world import Reading, World

RANK_TOL = 1e-9
PRIOR_SIGMA_MW = 30.0
PRIOR_MEAN_MW = {"abstract_substation": -30.0, "terminal": -30.0, "boundary_endpoint": 0.0}
ZERO_INJECTION_TYPES = ("junction", "intermediate")
MODEL_REVISION = "reduced-dc-v1"
BASE = BASE_MVA


@dataclass(frozen=True)
class EstimateUnavailable:
    reason: str
    diagnostics: tuple[str, ...] = ()


@dataclass(frozen=True)
class FlowEstimate:
    p_mw: float
    conditional_std_mw: float
    discrepancy_bound_mw: float | None
    uncertainty_radius_95_mw: float | None
    observable: bool


@dataclass(frozen=True)
class GridEstimate:
    clock_s: int
    model_revision: str
    operating_epoch: str
    prior_only: bool
    flows: dict[str, FlowEstimate]
    injections_mw: dict[str, FlowEstimate]
    accepted_reading_ids: tuple[str, ...]
    zero_injection_rank: int
    total_rank: int
    n_unknowns: int
    observable_segments: tuple[str, ...]
    assumptions: tuple[str, ...]
    diagnostics: tuple[str, ...] = ()
    prior_conflicts: tuple[str, ...] = ()  # nodes where the prior disagrees with the data
    envelope_version: str | None = None
    radius_unavailable: dict[str, str] | None = None  # segment -> why no 95% radius


@dataclass(frozen=True)
class Observability:
    n_unknowns: int  # angles after eliminating references
    n_zero_injection: int
    zero_injection_rank: int
    total_rank: int
    independent_readings: int  # rank added by real aggregate rows over exact constraints
    observable_segments: tuple[str, ...]
    components: int
    per_component_minimum: dict[str, int]
    selected_segments: tuple[str, ...]  # one independent set reaching total_rank


class ElectricalModel:
    """The reduced DC model for one assumed topology. Pure and immutable once built."""

    def __init__(
        self,
        network: Network,
        registry: CircuitRegistry,
        out_of_service: frozenset[str] = frozenset(),
        epoch: str = "E0",
        revision: str = MODEL_REVISION,
    ) -> None:
        self.network, self.registry = network, registry
        self.epoch, self.revision = epoch, revision
        self.out_of_service = out_of_service  # known out-of-service segments (whole segment)
        self.nodes = tuple(sorted(network.nodes))
        self.node_index = {n: i for i, n in enumerate(self.nodes)}
        self.segments = tuple(sorted(network.segments))
        self.b_circuit = {c.id: c.b_mw_per_rad for c in registry.circuits.values()}
        self.b_seg = {
            s: 0.0 if s in out_of_service else sum(c.b_mw_per_rad for c in registry.by_segment[s])
            for s in self.segments
        }
        self._build()

    @property
    def topology_key(self) -> tuple:
        return (self.revision, self.epoch, tuple(sorted(self.out_of_service)))

    def _build(self) -> None:
        net = self.network
        n = len(self.nodes)
        parent = list(range(n))

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for s in self.segments:
            if self.b_seg[s] > 0:
                a, c = self._ends(s)
                parent[find(a)] = find(c)
        groups: dict[int, list[int]] = {}
        for i in range(n):
            groups.setdefault(find(i), []).append(i)
        self.components = tuple(tuple(sorted(g)) for g in sorted(groups.values()))
        self.references = tuple(min(g) for g in self.components)
        self.columns = tuple(i for i in range(n) if i not in set(self.references))
        self._col = {i: k for k, i in enumerate(self.columns)}
        self.comp_of = {i: ci for ci, g in enumerate(self.components) for i in g}
        self.component_names = {
            ci: net.nodes[self.nodes[g[0]]].component for ci, g in enumerate(self.components)
        }
        # nodal injection matrix (reduced columns): inj = B theta
        self.B = np.zeros((n, len(self.columns)))
        self.flow_rows = {}
        for s in self.segments:
            a, c = self._ends(s)
            row = np.zeros(len(self.columns))
            if a in self._col:
                row[self._col[a]] += self.b_seg[s]
            if c in self._col:
                row[self._col[c]] -= self.b_seg[s]
            self.flow_rows[s] = row
            for node, sign in ((a, 1.0), (c, -1.0)):
                self.B[node] += sign * row
        self.zero_nodes = tuple(
            i for i, name in enumerate(self.nodes) if net.nodes[name].type in ZERO_INJECTION_TYPES
        )
        self.prior_nodes = tuple(
            i for i, name in enumerate(self.nodes) if net.nodes[name].type in PRIOR_MEAN_MW
        )
        self.prior_mean = np.array(
            [PRIOR_MEAN_MW[net.nodes[self.nodes[i]].type] for i in self.prior_nodes]
        )
        self.Z = self.B[list(self.zero_nodes)]

    def _ends(self, segment_id: str) -> tuple[int, int]:
        seg = self.network.segments[segment_id]
        return self.node_index[seg.from_node], self.node_index[seg.to_node]

    def active_segments(self) -> tuple[str, ...]:
        return tuple(s for s in self.segments if self.b_seg[s] > 0)


# -- rank utilities ---------------------------------------------------------------


def _normalise(m: np.ndarray) -> np.ndarray:
    if m.size == 0:
        return m
    norms = np.linalg.norm(m, axis=1, keepdims=True)
    return m / np.maximum(norms, 1e-300)


def _rank(m: np.ndarray) -> int:
    if m.size == 0:
        return 0
    return int(np.linalg.matrix_rank(_normalise(m), tol=RANK_TOL))


def observability(
    model: ElectricalModel, accepted_readings: list[Reading] | tuple[Reading, ...] = ()
) -> Observability:
    """Structural observability from real aggregate rows and exact zero-injection constraints.

    Pseudo-measurements (priors) are excluded. Repeated rows add no structural rank. With no
    readings, `independent_readings` is the minimum independent set size for the whole model."""
    zr = _rank(model.Z)
    active = model.active_segments()
    all_flow = (
        np.array([model.flow_rows[s] for s in active])
        if active
        else np.zeros((0, len(model.columns)))
    )
    total = _rank(np.vstack([model.Z, all_flow]))
    segments_read = sorted({r.point.segment_id for r in accepted_readings} & set(active))
    rows = (
        np.array([model.flow_rows[s] for s in segments_read])
        if segments_read
        else np.zeros((0, len(model.columns)))
    )
    observable = _observable_segments(model, np.vstack([model.Z, rows]), active)

    # greedy independent selection over exact constraints (QR with pivoting on projected rows)
    selected = _independent_rows(model, active)
    per_comp: dict[str, int] = {}
    for ci, nodes in enumerate(model.components):
        cols = [model._col[i] for i in nodes if i in model._col]
        if not cols:
            continue
        mask = np.zeros(len(model.columns), dtype=bool)
        mask[cols] = True
        z_rows = model.Z[:, mask]
        z_rows = z_rows[np.abs(z_rows).sum(axis=1) > 0]
        f_rows = np.array(
            [model.flow_rows[s][mask] for s in active if model.comp_of[model._ends(s)[0]] == ci]
        )
        zr_c = _rank(z_rows)
        tot_c = _rank(np.vstack([z_rows, f_rows])) if f_rows.size else zr_c
        name = model.component_names[ci]
        per_comp[name] = per_comp.get(name, 0) + (tot_c - zr_c)
    return Observability(
        n_unknowns=len(model.columns),
        n_zero_injection=len(model.zero_nodes),
        zero_injection_rank=zr,
        total_rank=total,
        independent_readings=total - zr,
        observable_segments=tuple(observable),
        components=len(model.components),
        per_component_minimum=per_comp,
        selected_segments=tuple(selected),
    )


def _observable_segments(
    model: ElectricalModel, rows: np.ndarray, active: tuple[str, ...]
) -> list[str]:
    """Flows determined by the given rows alone (projection onto their row space)."""
    if rows.size == 0:
        return []
    _, s, vt = np.linalg.svd(_normalise(rows), full_matrices=False)
    r = int((s > RANK_TOL * s[0]).sum()) if s.size else 0
    basis = vt[:r]
    out = []
    for seg in active:
        f = model.flow_rows[seg]
        norm = np.linalg.norm(f)
        if norm == 0:
            continue
        resid = f - basis.T @ (basis @ f) if r else f
        if np.linalg.norm(resid) <= 1e-7 * norm:
            out.append(seg)
    return out


def _independent_rows(model: ElectricalModel, active: tuple[str, ...]) -> list[str]:
    """One minimum independent set of aggregate rows over the exact constraints."""
    chosen: list[str] = []
    basis_rows = [r for r in _normalise(model.Z)] if model.Z.size else []
    current = np.array(basis_rows) if basis_rows else np.zeros((0, len(model.columns)))
    rank = _rank(current)
    for s in active:
        cand = np.vstack([current, model.flow_rows[s][None, :]])
        r = _rank(cand)
        if r > rank:
            chosen.append(s)
            current, rank = cand, r
    return chosen


# -- readings to measurement rows ---------------------------------------------------------------


@dataclass(frozen=True)
class _Row:
    reading_id: str
    segment_id: str
    z: float
    members: tuple[str, ...]  # p_mw payload names summed into the row


def _measurement_rows(model: ElectricalModel, readings) -> tuple[list[_Row], list[str]]:
    rows, problems = [], []
    for r in readings:
        if r.covariance is None or r.kind != "current":
            continue
        seg = r.point.segment_id
        if model.b_seg.get(seg, 0.0) <= 0.0:
            problems.append(f"{r.id}: {seg} is out of service in the assumed model")
            continue
        payload = dict(r.payload)
        names = tuple(
            f"{c.id}.p_mw" for c in model.registry.by_segment[seg] if f"{c.id}.p_mw" in payload
        )
        values = [payload[n] for n in names]
        if not names or not all(np.isfinite(values)):
            problems.append(f"{r.id}: missing or non-finite P")
            continue
        rows.append(_Row(r.id, seg, float(sum(values)), names))
    return rows, problems


def _p_covariance(
    rows: list[_Row], readings_by_id: dict[str, Reading], shared_rho: float
) -> np.ndarray:
    """Covariance of aggregate P rows, including circuit correlations and the shared calibration
    component that repeats of the same segment keep (repeating cannot average it away)."""
    n = len(rows)
    cov = np.zeros((n, n))
    for a in range(n):
        ra = readings_by_id[rows[a].reading_id]
        for b in range(a, n):
            rb = readings_by_id[rows[b].reading_id]
            if rows[a].segment_id != rows[b].segment_id:
                continue
            total = 0.0
            if a == b:
                ca = ra.covariance
                idx = [ca.names.index(m) for m in rows[a].members]
                total = float(ca.as_array()[np.ix_(idx, idx)].sum())
            else:
                for m in rows[a].members:
                    sa = ra.covariance.std(m)
                    for m2 in rows[b].members:
                        sb = rb.covariance.std(m2)
                        total += shared_rho * sa * sb
            cov[a, b] = cov[b, a] = total
    return cov


# -- the estimate ---------------------------------------------------------------


ASSUMPTIONS = (
    "reduced DC model: lossless, 100 MVA base, circuit reactances from the public catalogue",
    "voltage-specific buses collapsed at each mapped node; ideal transformers, zero phase shift",
    "junction and intermediate nodes: exact zero injection",
    "substation/terminal injection prior -30 MW and boundary prior 0 MW, sigma 30 MW "
    "(pseudo-measurements, not evidence of observability)",
    "AC/DC, transformer and parameter mismatch are not included in conditional uncertainty",
)


def estimate(
    model: ElectricalModel,
    accepted_readings: list[Reading] | tuple[Reading, ...],
    *,
    clock_s: int,
    shared_rho: float = 0.0,
    prior_means: np.ndarray | None = None,
    envelope: DiscrepancyEnvelope | None = None,
    active_seed: int | None = None,
) -> GridEstimate | EstimateUnavailable:
    readings = [r for r in accepted_readings if r.operating_epoch == model.epoch]
    rows, problems = _measurement_rows(model, readings)
    by_id = {r.id: r for r in readings}
    n_x = len(model.columns)
    # exact constraints: eliminate with an SVD null space
    if model.Z.size:
        _, sz, vt = np.linalg.svd(model.Z, full_matrices=True)
        rz = int((sz > RANK_TOL * max(sz[0], 1.0)).sum()) if sz.size else 0
        nullspace = vt[rz:].T
    else:
        rz, nullspace = 0, np.eye(n_x)
    n_y = nullspace.shape[1]
    parts_a, parts_b = [], []
    if rows:
        h = np.array([model.flow_rows[r.segment_id] for r in rows])
        cov = _p_covariance(rows, by_id, shared_rho)
        try:
            chol = np.linalg.cholesky(cov)
        except np.linalg.LinAlgError:
            return EstimateUnavailable(
                "measurement covariance is not positive definite", tuple(problems)
            )
        z = np.array([r.z for r in rows])
        parts_a.append(sla.solve_triangular(chol, h @ nullspace, lower=True))
        parts_b.append(sla.solve_triangular(chol, z, lower=True))
    mu = model.prior_mean if prior_means is None else prior_means
    pri = model.B[list(model.prior_nodes)]
    parts_a.append(pri @ nullspace / PRIOR_SIGMA_MW)
    parts_b.append(mu / PRIOR_SIGMA_MW)
    a, b = np.vstack(parts_a), np.concatenate(parts_b)
    if not np.all(np.isfinite(a)) or not np.all(np.isfinite(b)):
        return EstimateUnavailable("non-finite inputs", tuple(problems))
    u, s, vt = np.linalg.svd(a, full_matrices=False)
    rank = int((s > RANK_TOL * s[0]).sum()) if s.size else 0
    if rank < n_y:
        return EstimateUnavailable(
            f"information matrix is singular (rank {rank} of {n_y})", tuple(problems)
        )
    y = vt.T @ ((u.T @ b) / s)
    x = nullspace @ y
    t = nullspace @ vt.T / s  # theta = t @ (U^T b), cov(theta) = t t^T
    obs = observability(model, readings)
    observable = set(obs.observable_segments)
    flows, injections = {}, {}
    unavailable: dict[str, str] = {}
    for seg in model.segments:
        row = model.flow_rows[seg]
        std = float(np.linalg.norm(row @ t))
        bound = radius = None
        if envelope is None:
            unavailable[seg] = "no discrepancy envelope"
        else:
            ok, why = envelope.supports(seg, model.out_of_service, model.epoch, active_seed)
            if ok:
                bound = envelope.bounds[seg].bound_mw
                radius = 1.96 * std + bound
            else:
                unavailable[seg] = why
        flows[seg] = FlowEstimate(float(row @ x), std, bound, radius, seg in observable)
    prior_res = (pri @ nullspace @ y - mu) / PRIOR_SIGMA_MW
    conflicts = tuple(
        model.nodes[model.prior_nodes[i]] for i in np.flatnonzero(np.abs(prior_res) > 3.0)
    )
    inj = model.B @ x
    for i, name in enumerate(model.nodes):
        std = float(np.linalg.norm(model.B[i] @ t))
        injections[name] = FlowEstimate(float(inj[i]), std, None, None, False)
    return GridEstimate(
        clock_s,
        model.revision,
        model.epoch,
        not rows,
        flows,
        injections,
        tuple(r.reading_id for r in rows),
        obs.zero_injection_rank,
        obs.total_rank,
        n_x,
        obs.observable_segments,
        ASSUMPTIONS,
        tuple(problems),
        conflicts,
        envelope.version if envelope else None,
        unavailable,
    )


# -- world integration ---------------------------------------------------------------


@dataclass
class EstimationState:
    """Cache so unchanged electrical inputs never trigger another solve."""

    model: ElectricalModel | None = None
    key: tuple | None = None
    estimate: GridEstimate | EstimateUnavailable | None = None
    solves: int = 0


def effective_readings(world: World) -> tuple[Reading, ...]:
    """Accepted CURRENT readings with dispositions applied: excluded ones are dropped and epoch
    reassignments take effect, while the stored readings themselves stay immutable."""
    from dataclasses import replace

    out = []
    for r in sorted(world.readings.values(), key=lambda r: r.id):
        if r.kind != "current" or r.id in world.excluded_readings:
            continue
        epoch = world.epoch_overrides.get(r.id, r.operating_epoch)
        out.append(r if epoch == r.operating_epoch else replace(r, operating_epoch=epoch))
    return tuple(out)


def envelope_for(world: World) -> DiscrepancyEnvelope | None:
    return world.envelope


def refresh(st: EstimationState, world: World, network: Network, registry: CircuitRegistry) -> bool:
    """Recompute the public estimate if (and only if) its electrical inputs changed.

    `st` is owned by the Simulation (the model holds the network); only the resulting estimate
    and a short log are published in the public World."""
    out_of_service = frozenset(world.known_switching) | world.accepted_out_of_service
    if st.model is None or st.model.out_of_service != out_of_service:
        st.model = ElectricalModel(network, registry, out_of_service)
    readings = effective_readings(world)
    key = (
        st.model.topology_key,
        tuple((r.id, r.operating_epoch) for r in readings),
        world.sensor_model.revision,
        world.sensor_model.shared_rho,
        world.scenario_seed,
    )
    if key == st.key:
        return False
    st.estimate = estimate(
        st.model,
        readings,
        clock_s=world.clock_s,
        shared_rho=world.sensor_model.shared_rho,
        envelope=envelope_for(world),
        active_seed=world.scenario_seed,
    )
    st.key = key
    st.solves += 1
    world.estimate = st.estimate
    world.estimation_solves = st.solves
    world.estimation_log.append(
        f"T+{world.clock_s // 60} min: estimate rebuilt from {len(readings)} reading(s), "
        f"{len(out_of_service)} known out-of-service segment(s)"
    )
    return True


def posterior_flow_covariance(
    model: ElectricalModel,
    readings: list[Reading] | tuple[Reading, ...],
    *,
    shared_rho: float = 0.0,
) -> np.ndarray | EstimateUnavailable:
    """Conditional covariance of every flow (segments x segments) given the readings' covariance
    only: variance does not depend on the measured values, so hypothetical readings can be scored
    before they exist. Full measurement correlations and priors are included."""
    rows, _ = _measurement_rows(model, list(readings))
    by_id = {r.id: r for r in readings}
    n_x = len(model.columns)
    if model.Z.size:
        _, sz, vt = np.linalg.svd(model.Z, full_matrices=True)
        rz = int((sz > RANK_TOL * max(sz[0], 1.0)).sum()) if sz.size else 0
        nullspace = vt[rz:].T
    else:
        nullspace = np.eye(n_x)
    parts = []
    if rows:
        h = np.array([model.flow_rows[r.segment_id] for r in rows])
        cov = _p_covariance(rows, by_id, shared_rho)
        try:
            chol = np.linalg.cholesky(cov)
        except np.linalg.LinAlgError:
            return EstimateUnavailable("measurement covariance is not positive definite")
        parts.append(sla.solve_triangular(chol, h @ nullspace, lower=True))
    parts.append(model.B[list(model.prior_nodes)] @ nullspace / PRIOR_SIGMA_MW)
    a = np.vstack(parts)
    _, s, vt2 = np.linalg.svd(a, full_matrices=False)
    rank = int((s > RANK_TOL * s[0]).sum()) if s.size else 0
    if rank < nullspace.shape[1]:
        return EstimateUnavailable("information matrix is singular")
    t = nullspace @ vt2.T / s
    flows = np.array([model.flow_rows[seg] for seg in model.segments])
    ft = flows @ t
    return ft @ ft.T
