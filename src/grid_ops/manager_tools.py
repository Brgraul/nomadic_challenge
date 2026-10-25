"""Read-only manager tools pinned to one immutable snapshot.

A `Toolbox` is built from a deep copy of the public World and nothing else: it holds no
reference to the simulation, hidden physics, command queue or hold. Every result is labelled
with the snapshot revision. The three-method runtime runs no MW inference, so there is no
computing tool: everything here reads accepted requests, compiled tasks, sensing payloads and
derived knowledge.
"""

from __future__ import annotations

import copy
import json
from dataclasses import asdict, dataclass, field

from grid_ops import assessment
from grid_ops.circuits import CircuitRegistry
from grid_ops.geometry import Geometry
from grid_ops.grid import Network
from grid_ops.sensing import (
    METHOD_CATALOG,
    ContactCurrent,
    LidarPass,
    RequestStatus,
    TdrTest,
)
from grid_ops.world import World, format_clock


@dataclass(frozen=True)
class DiagnosticRecord:
    id: str
    snapshot_revision: int
    candidate_out_of_service: tuple[str, ...]
    summary: dict


@dataclass
class PinnedSnapshot:
    revision: int
    world: World

    @classmethod
    def pin(cls, world: World) -> PinnedSnapshot:
        return cls(world.revision, copy.deepcopy(world))

    @property
    def label(self) -> str:
        return f"[snapshot r{self.revision} @ {format_clock(self.world.clock)}]"


@dataclass
class Toolbox:
    snapshot: PinnedSnapshot
    network: Network
    registry: CircuitRegistry
    diagnostics: dict[str, DiagnosticRecord] = field(default_factory=dict)

    # -- helpers ------------------------------------------------------------------------------

    def _out(self, payload) -> str:
        return f"{self.snapshot.label} " + json.dumps(payload, default=str, indent=None)

    @property
    def world(self) -> World:
        return self.snapshot.world

    def _reading_row(self, r, *, detailed: bool) -> dict:
        acq = r.acquisition
        if isinstance(acq, LidarPass):
            location = {
                "segment_id": acq.target.segment_id,
                "start_mm": acq.target.start_mm,
                "end_mm": acq.target.end_mm,
            }
        elif isinstance(acq, ContactCurrent | TdrTest):
            location = {
                "circuit_id": acq.conductor.circuit_id,
                "phase": acq.conductor.phase,
                "segment_id": acq.conductor.point.segment_id,
                "offset_mm": acq.conductor.point.offset_mm,
            }
        else:
            location = {}
        row = {
            "id": r.id,
            "method": str(acq.method),
            "drone": r.drone_id,
            "task": r.task_id,
            "requests": list(r.request_ids),
            "time": format_clock(r.acquired_end_s / 60),
            "location": location,
            "values": asdict(r.payload),
            "quality": asdict(r.quality),
            "epoch": r.evidence_epoch,
            "sensor_model_revision": r.sensor_model_revision,
        }
        if detailed:
            interp = self.world.interpretations.get(r.id)
            row["interpretation"] = None if interp is None else str(interp.outcome)
        return row

    # -- tools --------------------------------------------------------------------------------

    def overview(self) -> str:
        w = self.world
        return self._out(
            {
                "scenario": str(w.scenario_mode),
                "clock": format_clock(w.clock),
                "readings": len(w.readings),
                "requests": sum(
                    1 for a in w.requests.values() if a.status is RequestStatus.ACTIVE
                ),
                "drones": {d.id: d.phase for d in w.drones.values()},
                "known_switching": w.known_switching,
                "hazards": [h.id for h in w.hazards],
                "accepted_hypotheses": sorted(w.hypotheses),
                "note": "no drone measurements exist" if not w.readings else "measurements exist",
            }
        )

    def list_reports(self) -> str:
        return self._out(
            [
                {"id": r.id, "time": r.timestamp, "confidence": r.confidence, "text": r.description}
                for r in self.network.reports.values()
            ]
        )

    def segments_in_report(self, report_id: str) -> str:
        """Exact spans (segment and km range) of mapped lines inside a report area."""
        area = self.network.areas.get(report_id)
        if area is None:
            return self._out({"error": f"unknown report area {report_id}"})
        geo = Geometry(self.network)
        spans = []
        for seg in sorted(self.network.segments):
            for iv in geo.clip_intervals(seg, area):
                if iv.end_mm > iv.start_mm:
                    spans.append(
                        {
                            "segment_id": seg,
                            "start_km": round(iv.start_mm / 1e6, 3),
                            "end_km": round(iv.end_mm / 1e6, 3),
                        }
                    )
        return self._out(spans)

    def list_readings(self, method: str | None = None) -> str:
        rows = []
        for r in self.world.readings.values():
            if method and str(r.acquisition.method) != method:
                continue
            rows.append(self._reading_row(r, detailed=True))
        return self._out(rows)

    def get_reading(self, reading_id: str) -> str:
        r = self.world.readings.get(reading_id)
        if r is None:
            return self._out({"error": f"unknown reading {reading_id}"})
        return self._out(self._reading_row(r, detailed=True))

    def scope_assessment(
        self, segment_id: str, start_km: float, end_km: float, condition: str
    ) -> str:
        span = assessment.SpanRef(segment_id, round(start_km * 1e6), round(end_km * 1e6))
        a = assessment.assess_scope(self.world, span, condition)
        return self._out(
            {
                "scope": f"{segment_id} {start_km}-{end_km} km",
                "condition": condition,
                "verdict": str(a.verdict),
                "evidence": a.evidence_ids,
                "explanation": a.explanation,
                "untested_mm": sum(i.end_mm - i.start_mm for i in a.untested),
            }
        )

    def list_requests(self) -> str:
        w = self.world
        rejected = {u.request_id: u.reason for u in w.unavailable_work}
        rows = []
        for a in sorted(w.requests.values(), key=lambda a: a.id):
            served = sorted(
                (t for t in w.tasks.values() if t.request_id == a.id), key=lambda t: t.id
            )
            rows.append(
                {
                    "id": a.id,
                    "method": str(a.request.method),
                    "target": self._target_summary(a.request),
                    "status": str(a.status),
                    "urgency": a.request.urgency,
                    "served_by": [{"task": t.id, "status": str(t.status)} for t in served],
                    "rejected_reason": rejected.get(a.id),
                    "retired_reason": a.retired_reason or None,
                }
            )
        return self._out(rows)

    @staticmethod
    def _target_segment(request) -> str:
        target = request.target
        if hasattr(target, "segment_id"):
            return target.segment_id
        return target.circuit_id.split("/")[0]

    @staticmethod
    def _target_summary(request) -> str:
        target = request.target
        if hasattr(target, "segment_id"):
            return f"{target.segment_id} {target.start_mm}-{target.end_mm} mm"
        return f"{target.circuit_id} phase {target.phase} {target.start_mm}-{target.end_mm} mm"

    def scan_coverage(self, segment_id: str) -> str:
        """Union of lidar-inspected and missing intervals for one segment, from the payloads."""
        inspected, missing = [], []
        for r in self.world.readings.values():
            payload = r.payload
            if not getattr(payload, "inspected", ()):
                continue
            inspected += [iv for iv in payload.inspected if iv.segment_id == segment_id]
            missing += [iv for iv in payload.missing if iv.segment_id == segment_id]
        ins = assessment.merge(inspected)
        mis = [x for part in assessment.merge(missing) for x in assessment._minus(part, ins)]
        return self._out(
            {
                "segment_id": segment_id,
                "inspected_mm": sum(i.end_mm - i.start_mm for i in ins),
                "inspected": [{"start_mm": i.start_mm, "end_mm": i.end_mm} for i in ins],
                "missing_mm": sum(i.end_mm - i.start_mm for i in mis),
                "missing": [{"start_mm": i.start_mm, "end_mm": i.end_mm} for i in mis],
            }
        )

    def charge_sites(self) -> str:
        w = self.world
        wanted: set[str] = set()
        for a in w.requests.values():
            if a.status is not RequestStatus.ACTIVE:
                continue
            seg = self._target_segment(a.request)
            wanted.update(c.id for c in w.circuits_by_segment.get(seg, ()))
        geo = Geometry(self.network)
        for area in self.network.areas.values():
            for seg in sorted(self.network.segments):
                if any(iv.end_mm > iv.start_mm for iv in geo.clip_intervals(seg, area)):
                    wanted.update(c.id for c in w.circuits_by_segment.get(seg, ()))
        rows = []
        for cid in sorted(wanted):
            ch = w.knowledge.chargeability.get(cid)
            rows.append(
                {
                    "circuit": cid,
                    "state": str(ch.state) if ch else "unknown",
                    "reason": ch.reason if ch else "no record",
                    "expires_s": ch.expires_s if ch else None,
                }
            )
        return self._out(rows)

    def capabilities(self) -> str:
        cap = self.world.capabilities
        return self._out(
            {
                "methods": [
                    {"method": m.method.value, "airborne": m.airborne, "description": m.description}
                    for m in METHOD_CATALOG
                ],
                "sensor": asdict(cap),
                "assumptions": list(self.world.brief),
            }
        )

    def reconstruction(self) -> str:
        """The deterministic public posterior: supported facts, alternatives and what is unknown."""
        w = self.world
        b = w.outage_belief
        if b is None:
            return self._out({"error": "no reconstruction is available in this snapshot"})
        stop = w.outage_stop
        return self._out(
            {
                "model_version": b.model_version,
                "evidence_ids": list(b.evidence_ids),
                "supported_facts": [
                    {
                        "kind": f.kind,
                        "subject": f.subject,
                        "statement": f.statement,
                        "probability": round(f.probability, 3),
                        "supported": f.supported,
                        "interval": None
                        if f.interval is None
                        else {
                            "segment_id": f.interval.segment_id,
                            "start_mm": f.interval.start_mm,
                            "end_mm": f.interval.end_mm,
                        },
                        "conditional_on": list(f.conditional_on),
                        "evidence_ids": list(f.evidence_ids),
                    }
                    for f in b.supported_facts
                ],
                "alternatives_not_yet_separated": [
                    {
                        "label": a.label,
                        "mass": round(a.mass, 3),
                        "candidates": len(a.candidate_ids),
                        "distinguishing_methods": list(a.distinguishing_methods),
                    }
                    for a in b.unresolved_classes
                ],
                "unknown_regions": [
                    {"segment_id": i.segment_id, "start_mm": i.start_mm, "end_mm": i.end_mm}
                    for i in b.unknown_regions[:20]
                ],
                "regions": [
                    {
                        "segment_id": r.segment_id,
                        "fault_mass": round(r.fault_mass, 3),
                        "unloaded_mass": round(r.unloaded_mass, 3),
                        "observed_current_live": r.observed_energized,
                        "lidar_covered_fraction": round(r.lidar_covered_fraction, 3),
                    }
                    for r in sorted(b.regions, key=lambda r: -r.fault_mass)[:12]
                ],
                "model_checks": {
                    "residual_mass": round(b.model_checks.residual_mass, 3),
                    "mismatch": b.model_checks.mismatch,
                    "notes": list(b.model_checks.notes),
                },
                "stop_assessment": None
                if stop is None
                else {"state": str(stop.reason), "detail": stop.detail},
                "candidate_pool_truncated": b.truncated,
                "limits": list(b.limits),
                "note": "Probabilities are conditional on the declared model and are computed "
                "by the system; do not restate or invent them.",
            }
        )

    def suggest_experiments(self) -> str:
        """Ranked candidate measurements by public expected discrimination, for you to request."""
        w = self.world
        rows = [
            {
                "method": p.method,
                "segment_id": p.segment_id,
                "circuit_id": p.circuit_id,
                "start_mm": p.start_mm,
                "end_mm": p.end_mm,
                "expected_discrimination_bits": p.eig_bits,
                "note": p.note,
            }
            for p in w.experiment_proposals
        ]
        return self._out(
            {
                "experiments": rows,
                "omitted_open_tasks": list(w.experiment_omitted),
                "note": "Bits are the system's expected discrimination between remaining "
                "alternatives. You choose method and scope; the optimizer chooses drones, "
                "routes and charging.",
            }
        )

    def list_events(self, limit: int = 30) -> str:
        return self._out(
            [
                {
                    "id": eid,
                    "time": format_clock(e.clock_s / 60),
                    "kind": e.kind,
                    "detail": e.detail,
                }
                for eid, e in self.world.events[-limit:]
            ]
        )

    def list_hypotheses(self) -> str:
        return self._out({h: v for h, v in self.world.hypotheses.items()})


def build_toolbox(world: World, network: Network, registry: CircuitRegistry) -> Toolbox:
    return Toolbox(PinnedSnapshot.pin(world), network, registry)
