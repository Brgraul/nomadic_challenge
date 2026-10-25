"""Privileged causal episodes and acquisition physics. Never pass an Episode to a policy."""

from __future__ import annotations

import cmath
import hashlib
import math
import random
import uuid
from dataclasses import dataclass
from itertools import pairwise

import numpy as np
import pandapower as pp
import pandapower.topology as top

from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import Geometry, Interval, LinePoint
from grid_ops.grid import Network
from grid_ops.sensing import (
    AcquisitionQuality,
    AcquisitionSample,
    ContactCurrent,
    CurrentPayload,
    EchoEdge,
    ExecutedAcquisition,
    LidarPass,
    LidarPayload,
    PublicEpisodeView,
    SensorCapabilities,
    TdrEcho,
    TdrPayload,
    TdrTest,
    tdr_access,
    trace_echoes,
)
from grid_ops.truth.local import LidarScene, SceneHazard


@dataclass(frozen=True)
class EpisodeSpec:
    episode_seed: int
    noise_seed: int
    fault_segment_id: str | None = None
    fault_fraction: float | None = None
    run_id: str | None = None


@dataclass(frozen=True)
class FaultEvent:
    circuit_id: str
    offset_mm: int
    kind: str = "open-conductor"
    occurred_s: int = -2


@dataclass(frozen=True)
class ProtectionAction:
    circuit_ids: tuple[str, ...]
    offset_mm: int
    opened_s: int = -1
    reason: str = "trip following primary conductor fault"


@dataclass(frozen=True)
class ConductorSpan:
    circuit_id: str
    start_mm: int
    end_mm: int
    from_bus: int
    to_bus: int
    line_index: int


class Episode:
    def __init__(
        self,
        *,
        spec: EpisodeSpec,
        public: PublicEpisodeView,
        net,
        network: Network,
        registry: CircuitRegistry,
        geometry: Geometry,
        spans: tuple[ConductorSpan, ...],
        fault: FaultEvent,
        protection: ProtectionAction,
        scene: LidarScene,
        unsupplied_before: frozenset[int],
    ) -> None:
        self.spec = spec
        self.public = public
        self.net = net
        self.network = network
        self.registry = registry
        self.geometry = geometry
        self.spans = spans
        self.fault = fault
        self.protection = protection
        self.scene = scene
        self.unsupplied = frozenset(top.unsupplied_buses(net))
        self.newly_unsupplied = self.unsupplied - unsupplied_before
        self._spans_by_circuit = {
            cid: tuple(s for s in spans if s.circuit_id == cid) for cid in registry.circuits
        }

    def manifest(self) -> dict:
        from dataclasses import asdict

        return {
            "episode_spec": asdict(self.spec),
            "fault": asdict(self.fault),
            "protection": asdict(self.protection),
            "newly_unsupplied_buses": sorted(int(b) for b in self.newly_unsupplied),
            "model": "post-trip balanced AC; no transient short-circuit model",
        }

    def _noise(self, group: str, acquisition_id: str) -> float:
        rho = self.public.capabilities.systematic_fraction

        def normal(key: str) -> float:
            raw = hashlib.blake2b(f"{self.spec.noise_seed}|{key}".encode(), digest_size=8).digest()
            return float(np.random.default_rng(int.from_bytes(raw, "big")).normal())

        return math.sqrt(rho) * normal(group) + math.sqrt(1 - rho) * normal(
            f"{group}|{acquisition_id}"
        )

    def current_at(self, conductor) -> float:
        point = conductor.point
        self.geometry.position_of(point)
        spans = self._spans_by_circuit[conductor.circuit_id]
        span = next((s for s in spans if s.start_mm <= point.offset_mm < s.end_mm), spans[-1])
        if point.offset_mm > span.end_mm:
            raise ValueError("conductor offset outside span")
        if span.from_bus in self.unsupplied or span.to_bus in self.unsupplied:
            return 0.0
        r = self.net.res_line.loc[span.line_index]
        a = self.net.res_bus.loc[span.from_bus]
        b = self.net.res_bus.loc[span.to_bus]
        kv = float(self.net.bus.at[span.from_bus, "vn_kv"])
        va = cmath.rect(float(a.vm_pu) * kv, math.radians(float(a.va_degree)))
        vb = cmath.rect(float(b.vm_pu) * kv, math.radians(float(b.va_degree)))
        ia = (complex(r.p_from_mw, r.q_from_mvar) / (math.sqrt(3) * va)).conjugate()
        ib = -(complex(r.p_to_mw, r.q_to_mvar) / (math.sqrt(3) * vb)).conjugate()
        fraction = (point.offset_mm - span.start_mm) / max(1, span.end_mm - span.start_mm)
        return abs(ia + fraction * (ib - ia)) * 1000

    def acquire(self, executed: ExecutedAcquisition) -> AcquisitionSample:
        a = executed.acquisition
        if isinstance(a, LidarPass):
            return self._lidar(executed)
        conductor = a.conductor
        self.geometry.position_of(conductor.point)
        if conductor.circuit_id not in self.registry.circuits:
            raise ValueError("unknown conductor")
        enough_time = executed.ended_s - executed.started_s >= (
            a.integration_s if isinstance(a, ContactCurrent) else a.duration_s
        )
        contact = (
            executed.landed_conductor == conductor
            and enough_time
            and executed.completed_fraction == 1.0
        )
        if isinstance(a, ContactCurrent):
            group = f"current:{conductor.circuit_id}:{conductor.phase}"
            cap = self.public.capabilities
            current = (
                max(
                    0.0,
                    self.current_at(conductor)
                    + cap.current_sigma_a * self._noise(group, executed.id),
                )
                if contact
                else 0.0
            )
            saturated = current >= cap.current_saturation_a
            payload = CurrentPayload(
                conductor,
                min(current, cap.current_saturation_a),
                cap.current_sigma_a,
                a.integration_s,
            )
            return AcquisitionSample(
                executed.id,
                payload,
                AcquisitionQuality(
                    contact,
                    "" if contact else "landed contact and integration interval required",
                    saturated,
                    contact,
                    group,
                ),
            )
        return self._tdr(executed, contact)

    def _tdr(self, executed: ExecutedAcquisition, contact: bool) -> AcquisitionSample:
        a = executed.acquisition
        assert isinstance(a, TdrTest)
        cap = self.public.capabilities
        access = tdr_access(a.conductor)
        reason = ""
        if not contact:
            reason = "landed contact and full acquisition interval required"
        elif a.access_site_id != access.id:
            reason = "unknown coupling access site"
        elif a.return_path_id not in access.return_path_ids:
            reason = "valid return path required"
        elif a.coupling_profile_id not in access.coupling_profiles:
            reason = "unsupported coupling profile"
        elif a.coupling_profile_id == "isolated-only-v1" and not (
            set(a.isolation_evidence_ids) & set(executed.valid_isolation_evidence_ids)
        ):
            reason = "validated isolation evidence required; zero current is insufficient"
        group = f"tdr:{a.conductor.circuit_id}:{a.conductor.point.offset_mm}"
        echoes = ()
        truncated = False
        if not reason:
            distances, truncated = self._echo_paths(
                a.conductor.circuit_id, a.conductor.point.offset_mm
            )
            bias = self._noise(group, executed.id) * cap.tdr_delay_sigma_s
            echoes = tuple(
                TdrEcho(
                    max(0.0, 2 * distance / cap.propagation_mps + bias),
                    amplitude,
                    "positive" if amplitude >= 0 else "negative",
                    cap.tdr_delay_sigma_s,
                )
                for distance, amplitude in distances
            )
        if truncated:
            reason = "electrical path cap reached; echoes are partial, no clean negative"
        payload = TdrPayload(
            echoes,
            cap.propagation_mps,
            cap.propagation_uncertainty_mps,
            cap.tdr_range_m,
            "path-limited" if truncated else ("invalid" if reason else "coupled"),
            truncated,
        )
        return AcquisitionSample(
            executed.id, payload, AcquisitionQuality(not reason, reason, False, contact, group)
        )

    def _echo_paths(
        self, circuit_id: str, offset_mm: int
    ) -> tuple[tuple[tuple[float, float], ...], bool]:
        spans = self._spans_by_circuit[circuit_id]
        current = next((s for s in spans if s.start_mm <= offset_mm < s.end_mm), spans[-1])
        parents = {int(b): int(b) for b in self.net.bus.index}

        def root(bus):
            while parents[bus] != bus:
                bus = parents[bus]
            return bus

        for switch in self.net.switch.itertuples():
            if switch.et == "b" and switch.closed:
                parents[root(int(switch.element))] = root(int(switch.bus))
        kv = self.registry.circuits[circuit_id].nominal_kv
        adjacency: dict[int, list[EchoEdge]] = {}
        edges: dict[int, EchoEdge] = {}
        for span in self.spans:
            if self.registry.circuits[span.circuit_id].nominal_kv != kv:
                continue
            edge = EchoEdge(
                span.line_index,
                root(span.from_bus),
                root(span.to_bus),
                span.end_mm - span.start_mm,
            )
            edges[span.line_index] = edge
            adjacency.setdefault(edge.a, []).append(edge)
            adjacency.setdefault(edge.b, []).append(edge)
        cap = self.public.capabilities
        return trace_echoes(
            adjacency,
            edges[current.line_index],
            offset_mm - current.start_mm,
            range_m=cap.tdr_range_m,
            max_paths=cap.tdr_max_paths,
        )

    def _lidar(self, executed: ExecutedAcquisition) -> AcquisitionSample:
        a = executed.acquisition
        assert isinstance(a, LidarPass)
        cap = self.public.capabilities
        target = a.target
        if not 0 <= target.start_mm < target.end_mm <= self.geometry.length_mm(target.segment_id):
            raise ValueError("invalid scan interval")
        resolution = max(
            cap.lidar_resolution_mm,
            math.ceil((target.end_mm - target.start_mm) / cap.lidar_max_cells),
        )
        bounds = [*range(target.start_mm, target.end_mm, resolution), target.end_mm]
        centres = [(x + y) // 2 for x, y in pairwise(bounds)]
        origin = a.trajectory.poses[0].position
        xyz = np.array(
            [
                (
                    *_xy(self.geometry.position_of(LinePoint(target.segment_id, mm)), origin),
                    self.scene.conductor_height_m,
                )
                for mm in centres
            ]
        )
        visible = np.zeros(len(centres), dtype=bool)
        fraction = min(
            executed.completed_fraction,
            (executed.ended_s - executed.started_s) / a.trajectory.duration_s,
        )
        count = min(128, max(2, math.ceil(a.trajectory.duration_s * fraction) + 1))
        if fraction > 0:
            for f in np.linspace(0, fraction, count):
                pose = _interpolate_pose(a.trajectory.poses, float(f))
                pos = np.array((*_xy(pose[0], origin), pose[1]))
                az, el = math.radians(pose[2]), math.radians(pose[3])
                look = np.array(
                    [math.sin(az) * math.cos(el), math.cos(az) * math.cos(el), math.sin(el)]
                )
                rays = xyz - pos
                ranges = np.linalg.norm(rays, axis=1)
                in_view = (
                    (ranges <= cap.lidar_range_m)
                    & (ranges > 0)
                    & ((rays @ look) >= ranges * math.cos(math.radians(cap.lidar_fov_deg / 2)))
                )
                for o in self.scene.occluders:
                    lo = np.array((*_xy(o.lower_lonlat, origin), o.bottom_m))
                    hi = np.array((*_xy(o.upper_lonlat, origin), o.top_m))
                    with np.errstate(divide="ignore", invalid="ignore"):
                        t0, t1 = (lo - pos) / rays, (hi - pos) / rays
                    near = np.max(np.minimum(t0, t1), axis=1)
                    far = np.min(np.maximum(t0, t1), axis=1)
                    in_view &= ~((far >= np.maximum(near, 0)) & (near < 1) & (far > 0))
                visible |= in_view
        inspected = _merge_cells(target.segment_id, bounds, visible)
        missing = _merge_cells(target.segment_id, bounds, ~visible)
        group = f"lidar:{target.segment_id}"
        clearance = None
        if inspected:
            value = min(
                [2.0]
                + [
                    h.clearance_m
                    for h in self.scene.hazards
                    if any(
                        h.interval.segment_id == i.segment_id
                        and h.interval.start_mm < i.end_mm
                        and h.interval.end_mm > i.start_mm
                        for i in inspected
                    )
                ]
            )
            clearance = max(0.0, value + 0.05 * self._noise(group, executed.id))
        valid = bool(inspected)
        return AcquisitionSample(
            executed.id,
            LidarPayload(inspected, missing, clearance, 0.05, resolution),
            AcquisitionQuality(
                valid,
                "" if valid else "no geometric visibility; no negative evidence",
                False,
                True,
                group,
            ),
        )


def _xy(position, origin) -> tuple[float, float]:
    return (
        (position[0] - origin[0]) * 111_195 * math.cos(math.radians(origin[1])),
        (position[1] - origin[1]) * 111_195,
    )


def _interpolate_pose(poses, fraction):
    x = fraction * (len(poses) - 1)
    i = min(int(x), len(poses) - 2)
    f = x - i
    a, b = poses[i], poses[i + 1]
    da = (b.look_azimuth_deg - a.look_azimuth_deg + 180) % 360 - 180
    return (
        (
            a.position[0] + f * (b.position[0] - a.position[0]),
            a.position[1] + f * (b.position[1] - a.position[1]),
        ),
        a.height_m + f * (b.height_m - a.height_m),
        a.look_azimuth_deg + f * da,
        a.look_elevation_deg + f * (b.look_elevation_deg - a.look_elevation_deg),
    )


def _merge_cells(segment_id, bounds, selected) -> tuple[Interval, ...]:
    result: list[Interval] = []
    for i, present in enumerate(selected):
        if present:
            if result and result[-1].end_mm == bounds[i]:
                old = result.pop()
                result.append(Interval(segment_id, old.start_mm, bounds[i + 1]))
            else:
                result.append(Interval(segment_id, bounds[i], bounds[i + 1]))
    return tuple(result)


def build_episode(
    network: Network,
    registry: CircuitRegistry,
    geometry: Geometry,
    spec: EpisodeSpec,
    *,
    fixture_id: str = "bavaria-public",
    fixture_version: str = "1",
    policy_seed: int = 0,
    capabilities: SensorCapabilities | None = None,
) -> Episode:
    from grid_ops.truth.build import _make_net, choose_sources

    capabilities = capabilities or SensorCapabilities()
    sources = choose_sources(network)
    rng = random.Random(spec.episode_seed)
    candidates = []
    for seg in sorted(network.segments.values(), key=lambda x: x.id):
        ends = [
            n
            for n in (seg.from_node, seg.to_node)
            if len(network.segments_at(n)) == 1 and n not in sources.values()
        ]
        if ends:
            candidates.append((seg.id, ends[0]))
    if spec.fault_segment_id:
        candidates = [x for x in candidates if x[0] == spec.fault_segment_id]
    if not candidates:
        raise ValueError("episode requires a radial non-source terminal branch")
    sid, downstream = rng.choice(candidates)
    n = geometry.length_mm(sid)
    fraction = spec.fault_fraction if spec.fault_fraction is not None else rng.uniform(0.4, 0.7)
    if not 0.2 < fraction < 0.8:
        raise ValueError("fault fraction must be inside .2.. .8")
    fault_mm = round(n * fraction)
    seg = network.segments[sid]
    protector_mm = round(n * (0.1 if downstream == seg.to_node else 0.9))
    for attempt in range(100):
        net, indices = _make_net(
            network, registry, sources, random.Random(f"{spec.episode_seed}|ac|{attempt}")
        )
        try:
            pp.runpp(net, init="dc", max_iteration=60, numba=False)
        except pp.LoadflowNotConverged:
            continue
        before = frozenset(top.unsupplied_buses(net))
        spans = []
        for cid, idx in indices.items():
            row = net.line.loc[idx].copy()
            length = geometry.length_mm(registry.circuits[cid].segment_id)
            if registry.circuits[cid].segment_id != sid:
                spans.append(ConductorSpan(cid, 0, length, int(row.from_bus), int(row.to_bus), idx))
                continue
            net.line.drop(idx, inplace=True)
            cuts = sorted((0, protector_mm, fault_mm, n))
            prev_bus = int(row.from_bus)
            for left, right in pairwise(cuts):
                bus = (
                    int(row.to_bus)
                    if right == n
                    else pp.create_bus(
                        net, vn_kv=registry.circuits[cid].nominal_kv, name=f"{cid}@{right}:left"
                    )
                )
                new_idx = pp.create_line(
                    net,
                    prev_bus,
                    bus,
                    length_km=float(row.length_km) * (right - left) / n,
                    std_type=row.std_type,
                    name=f"{cid}:{left}..{right}",
                )
                spans.append(ConductorSpan(cid, left, right, prev_bus, bus, new_idx))
                if right != n:
                    next_bus = pp.create_bus(
                        net, vn_kv=registry.circuits[cid].nominal_kv, name=f"{cid}@{right}:right"
                    )
                    opened = right == protector_mm or (
                        right == fault_mm and cid == registry.by_segment[sid][0].id
                    )
                    pp.create_switch(net, bus, next_bus, et="b", closed=not opened)
                    prev_bus = next_bus
        try:
            pp.runpp(net, init="dc", max_iteration=60, numba=False)
        except pp.LoadflowNotConverged:
            continue
        live = net.res_bus.vm_pu.dropna()
        if len(live) and (live.min() < 0.85 or live.max() > 1.15):
            continue
        public = PublicEpisodeView(
            spec.run_id or uuid.uuid4().hex, fixture_id, fixture_version, policy_seed, capabilities
        )
        fault = FaultEvent(registry.by_segment[sid][0].id, fault_mm)
        protection = ProtectionAction(tuple(c.id for c in registry.by_segment[sid]), protector_mm)
        scene = LidarScene(
            (SceneHazard(Interval(sid, max(0, fault_mm - 10_000), min(n, fault_mm + 10_000)), 0.3),)
        )
        episode = Episode(
            spec=spec,
            public=public,
            net=net,
            network=network,
            registry=registry,
            geometry=geometry,
            spans=tuple(spans),
            fault=fault,
            protection=protection,
            scene=scene,
            unsupplied_before=before,
        )
        if episode.newly_unsupplied:
            return episode
    raise ValueError("no converged post-trip operating point in 100 attempts")
