"""Hidden synthetic physics. Only `sim`, `evaluation` and this package may import it.

The AC network is an answer key for the exercise, not a reconstruction of Bavaria's real
connectivity. Nothing here is copied into public state: callers get answers to physical
questions through the `grid_ops.world.Physics` protocol, and every observation carries noise
keyed by scenario seed, canonical observation identity, kind, physical epoch and repeat index.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass

import numpy as np
import pandapower as pp
import pandapower.topology as top

from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import MM_PER_KM, Geometry, Interval, LandingSite, LinePoint
from grid_ops.grid import Network
from grid_ops.scenario import (
    CLEARANCE_SIGMA_M,
    ENV_TEMP_C,
    HEATING_C_PER_A2,
    I_CHARGE_MIN_A,
    MOTION_SIGMA_M,
    TEMP_SIGMA_C,
    CircuitSensorConfig,
    ScenarioConfig,
    ScenarioInfeasible,
)
from grid_ops.truth.local import (
    BASE_CLEARANCE_M,
    BASE_MOTION_CALM_M,
    BASE_MOTION_GUST_M,
    LocalField,
)
from grid_ops.world import Measurement, SensorSample


@dataclass(frozen=True)
class Limits:
    v_min_pu: float
    v_max_pu: float
    max_loading_pct: float


@dataclass(frozen=True)
class CircuitState:
    """Hidden per-circuit AC result. Terminal quantities use the from-bus-into-line convention."""

    energized: bool
    p_from_mw: float = 0.0
    q_from_mvar: float = 0.0
    p_to_mw: float = 0.0
    q_to_mvar: float = 0.0
    i_from_a: float = 0.0
    i_to_a: float = 0.0
    loss_mw: float = 0.0
    v_from_kv: float = 0.0
    v_to_kv: float = 0.0
    loading_pct: float = 0.0


def _digest(*parts: object) -> int:
    h = hashlib.blake2b("|".join(map(str, parts)).encode(), digest_size=8)
    return int.from_bytes(h.digest(), "big")


def _normals(n: int, *key: object) -> np.ndarray:
    return np.random.default_rng(_digest(*key)).standard_normal(n)


class TruthState:
    def __init__(
        self,
        net: pp.pandapowerNet,
        network: Network,
        registry: CircuitRegistry,
        geometry: Geometry,
        config: ScenarioConfig,
        local: LocalField,
        line_index: dict[str, int],
        sources: dict[str, str],
        limits: Limits,
        transition_limits: Limits,
        operating_point: int,
    ) -> None:
        self.net = net
        self.network = network
        self.registry = registry
        self.geometry = geometry
        self.config = config
        self.local = local
        self.sources = sources  # component -> source node (declared, public modelling choice)
        self.limits = limits
        self.transition_limits = transition_limits
        self.operating_point = operating_point
        self.epoch = 0
        self._line = line_index
        self._sensors = CircuitSensorConfig()
        self.circuit_states: dict[str, CircuitState] = {}
        self.unsupplied_buses: frozenset[int] = frozenset()
        self.initial_problems = self.solve()

    # -- AC solution -----------------------------------------------------------------------

    def solve(self) -> list[str]:
        """Run the AC power flow. Returns limit violations (empty when the state is valid)."""
        net = self.net
        self.unsupplied_buses = frozenset(top.unsupplied_buses(net))
        pp.runpp(net, init="dc", calculate_voltage_angles=True, max_iteration=60)
        problems: list[str] = []
        states: dict[str, CircuitState] = {}
        for cid, idx in self._line.items():
            c = self.registry.circuits[cid]
            line = net.line.loc[idx]
            live = bool(line.in_service) and int(line.from_bus) not in self.unsupplied_buses
            if not live:
                states[cid] = CircuitState(False)
                continue
            r = net.res_line.loc[idx]
            values = [
                r.p_from_mw,
                r.q_from_mvar,
                r.p_to_mw,
                r.q_to_mvar,
                r.i_from_ka,
                r.i_to_ka,
                r.pl_mw,
                r.vm_from_pu,
                r.vm_to_pu,
                r.loading_percent,
            ]
            if not all(math.isfinite(float(v)) for v in values):
                problems.append(f"non-finite result on {cid}")
                states[cid] = CircuitState(False)
                continue
            states[cid] = CircuitState(
                True,
                float(r.p_from_mw),
                float(r.q_from_mvar),
                float(r.p_to_mw),
                float(r.q_to_mvar),
                float(r.i_from_ka) * 1000,
                float(r.i_to_ka) * 1000,
                float(r.pl_mw),
                float(r.vm_from_pu) * c.nominal_kv,
                float(r.vm_to_pu) * c.nominal_kv,
                float(r.loading_percent),
            )
            vmin, vmax = min(r.vm_from_pu, r.vm_to_pu), max(r.vm_from_pu, r.vm_to_pu)
            lim = self.limits
            if vmin < lim.v_min_pu or vmax > lim.v_max_pu:
                problems.append(f"{cid} voltage {vmin:.3f}..{vmax:.3f} pu")
            if r.loading_percent > lim.max_loading_pct:
                problems.append(f"{cid} loading {r.loading_percent:.0f}%")
        for idx in net.trafo.index:
            hv, lv = int(net.trafo.at[idx, "hv_bus"]), int(net.trafo.at[idx, "lv_bus"])
            if hv in self.unsupplied_buses or lv in self.unsupplied_buses:
                continue
            loading = float(net.res_trafo.at[idx, "loading_percent"])
            if not math.isfinite(loading):
                problems.append(f"non-finite trafo {idx}")
            elif loading > self.limits.max_loading_pct:
                problems.append(f"trafo {net.trafo.at[idx, 'name']} loading {loading:.0f}%")
        balance = (
            float(net.res_ext_grid.p_mw.fillna(0).sum())
            + float(net.res_sgen.p_mw.fillna(0).sum())
            - float(net.res_load.p_mw.fillna(0).sum())
            - float(net.res_line.pl_mw.fillna(0).sum())
            - float(net.res_trafo.pl_mw.fillna(0).sum())
        )
        if abs(balance) > 0.05:
            problems.append(f"power balance off by {balance:.3f} MW")
        self.circuit_states = states
        return problems

    def set_in_service(self, segment_id: str, in_service: bool) -> None:
        """Switch every circuit of a segment. Rolls back and raises if the new state is invalid."""
        old = {cid: bool(self.net.line.at[self._line[cid], "in_service"]) for cid in self._line}
        old_states, old_unsupplied = self.circuit_states, self.unsupplied_buses
        for c in self.registry.by_segment[segment_id]:
            self.net.line.at[self._line[c.id], "in_service"] = in_service
        try:
            saved = self.limits
            self.limits = self.transition_limits
            try:
                problems = self.solve()
            finally:
                self.limits = saved
        except Exception as e:
            problems = [f"power flow failed: {e}"]
        if problems:
            for cid, flag in old.items():
                self.net.line.at[self._line[cid], "in_service"] = flag
            self.circuit_states, self.unsupplied_buses = old_states, old_unsupplied
            pp.runpp(self.net, init="dc", calculate_voltage_angles=True, max_iteration=60)
            raise ScenarioInfeasible(
                f"switching {segment_id} violates declared limits", {"problems": problems[:10]}
            )
        self.epoch += 1

    # -- Physics protocol ------------------------------------------------------------------

    def is_live(self, circuit_id: str) -> bool:
        return self.circuit_states[circuit_id].energized

    def is_chargeable(self, site: LandingSite) -> bool:
        """Declared capability rule: a live circuit with from-terminal current >= 5 A."""
        state = self.circuit_states.get(site.circuit_id)
        return state is not None and state.energized and state.i_from_a >= I_CHARGE_MIN_A

    def energized(self) -> frozenset[str]:
        """Evaluator view: segments with at least one energized circuit."""
        return frozenset(
            seg
            for seg, cs in self.registry.by_segment.items()
            if any(self.circuit_states[c.id].energized for c in cs)
        )

    def sense(
        self, point: LinePoint, kind: Measurement, clock_s: int, identity: str, repeat: int
    ) -> SensorSample:
        if kind in (Measurement.VOLTAGE_PRESENCE, Measurement.CURRENT):
            return self._sense_electrical(point, kind, identity, repeat)
        return self._sense_local(point, kind, clock_s, identity, repeat)

    # -- electrical ------------------------------------------------------------------------

    def _noise(
        self, identity: str, kind: Measurement, repeat: int, n: int, group_keys
    ) -> np.ndarray:
        """Unit-variance noise: independent part keyed by observation, shared part by group."""
        rho = self.config.shared_rho
        indep = _normals(n, self.config.seed, identity, kind, self.epoch, repeat)
        if rho == 0:
            return indep
        shared = np.array([_normals(1, self.config.seed, "shared", *g)[0] for g in group_keys])
        return math.sqrt(1 - rho) * indep + math.sqrt(rho) * shared

    def _sense_electrical(
        self, point: LinePoint, kind: Measurement, identity: str, repeat: int
    ) -> SensorSample:
        circuits = self.registry.by_segment[point.segment_id]
        quantities = (
            ("voltage_kv",)
            if kind is Measurement.VOLTAGE_PRESENCE
            else (
                "current_a",
                "p_mw",
                "q_mvar",
                "voltage_kv",
            )
        )
        names, means, sigmas, groups = [], [], [], []
        for c in circuits:
            s = self.circuit_states[c.id]
            truth = {
                "voltage_kv": s.v_from_kv,
                "current_a": s.i_from_a,
                "p_mw": s.p_from_mw,
                "q_mvar": s.q_from_mvar,
            }
            for q in quantities:
                names.append(f"{c.id}.{q}")
                means.append(truth[q])
                sigmas.append(self._sensors.sigma(q, c.nominal_kv))
                groups.append((c.segment_id, q, kind if kind is Measurement.CURRENT else "any"))
        z = self._noise(identity, kind, repeat, len(names), groups)
        values = tuple(
            (n, float(m + s * e)) for n, m, s, e in zip(names, means, sigmas, z, strict=True)
        )
        return SensorSample(kind, values)

    # -- local -----------------------------------------------------------------------------

    def _sense_local(
        self, point: LinePoint, kind: Measurement, clock_s: int, identity: str, repeat: int
    ) -> SensorSample:
        fp = self.geometry.footprint(point)
        occluded = [o for o in self.local.occluded if o.segment_id == point.segment_id]
        inspected = _subtract(fp, occluded)
        span = sum(b - a for a, b in inspected)
        visible = span / max(fp.end_mm - fp.start_mm, 1)
        noise = float(_normals(1, self.config.seed, identity, kind, self.epoch, repeat)[0])
        conditions: list[tuple[str, float]] = [("visible_fraction", visible)]
        values: tuple[tuple[str, float], ...] = ()
        overlapping = [
            a
            for a in self.local.anomalies
            if a.interval.segment_id == point.segment_id
            and any(a.interval.start_mm < e and s < a.interval.end_mm for s, e in inspected)
        ]
        if span > 0:
            match kind:
                case Measurement.CLEARANCE:
                    worst = min(
                        [BASE_CLEARANCE_M]
                        + [a.value for a in overlapping if a.kind == "vegetation"]
                    )
                    values = (("clearance_m", worst + CLEARANCE_SIGMA_M * noise),)
                case Measurement.CONDUCTOR_TEMP:
                    current = self._segment_current(point.segment_id) * self.local.load_factor(
                        point.segment_id, clock_s
                    )
                    excited = current > 20.0
                    excess = max(
                        [0.0] + [a.value for a in overlapping if a.kind == "hot_joint" and excited]
                    )
                    temp = ENV_TEMP_C + HEATING_C_PER_A2 * current**2 + excess
                    values = (("temp_c", temp + TEMP_SIGMA_C * noise),)
                    conditions += [
                        ("excitation_current_a", current),
                        ("excited", 1.0 if excited else 0.0),
                    ]
                case Measurement.MOTION:
                    gust = self.local.gust_at(clock_s)
                    amp = BASE_MOTION_GUST_M if gust else BASE_MOTION_CALM_M
                    if gust:
                        amp = max([amp] + [a.value for a in overlapping if a.kind == "motion"])
                    values = (("amplitude_m", amp + MOTION_SIGMA_M * noise),)
                    conditions.append(("qualifying_gust", 1.0 if gust else 0.0))
        return SensorSample(
            kind,
            values,
            tuple(conditions),
            tuple((a, b) for a, b in inspected),
        )

    def _segment_current(self, segment_id: str) -> float:
        states = [self.circuit_states[c.id] for c in self.registry.by_segment[segment_id]]
        live = [s.i_from_a for s in states if s.energized]
        return max(live, default=0.0)


def _subtract(fp: Interval, occluded: list[Interval]) -> list[tuple[int, int]]:
    parts = [(fp.start_mm, fp.end_mm)]
    for o in sorted(occluded, key=lambda i: i.start_mm):
        nxt = []
        for a, b in parts:
            if o.end_mm <= a or o.start_mm >= b:
                nxt.append((a, b))
                continue
            if a < o.start_mm:
                nxt.append((a, o.start_mm))
            if o.end_mm < b:
                nxt.append((o.end_mm, b))
        parts = nxt
    return [(a, b) for a, b in parts if b > a]


__all__ = ["MM_PER_KM", "CircuitState", "Limits", "TruthState"]
