"""Read-only manager tools pinned to one immutable snapshot.

A `Toolbox` is built from a deep copy of the public World and nothing else: it holds no
reference to the simulation, hidden physics, command queue or hold. Every result is labelled
with the snapshot revision. `test_electrical_candidate` is the only computing tool; it re-solves
the reduced model on the pinned readings under a candidate topology and records a diagnostic id
that a final proposal must cite.
"""

from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field

from grid_ops import assessment, estimation
from grid_ops.circuits import CircuitRegistry
from grid_ops.consistency import check_consistency
from grid_ops.grid import Network
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

    # -- tools --------------------------------------------------------------------------------

    def overview(self) -> str:
        w = self.world
        return self._out(
            {
                "scenario": str(w.scenario_mode),
                "clock": format_clock(w.clock),
                "readings": len(w.readings),
                "drones": {d.id: d.phase for d in w.drones.values()},
                "known_switching": w.known_switching,
                "hazards": [h.id for h in w.hazards],
                "accepted_hypotheses": sorted(w.hypotheses),
                "estimate": "prior-only"
                if getattr(w.estimate, "prior_only", False)
                else "has data",
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
        from grid_ops.geometry import Geometry

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

    def list_readings(self, kind: str | None = None) -> str:
        rows = []
        for r in self.world.readings.values():
            if kind and r.kind != kind:
                continue
            rows.append(
                {
                    "id": r.id,
                    "kind": str(r.kind),
                    "site": r.site_id,
                    "time": format_clock(r.clock_s / 60),
                    "values": dict(r.payload),
                    "conditions": dict(r.conditions),
                    "interpretation": str(
                        getattr(self.world.interpretations.get(r.id), "outcome", "electrical")
                    ),
                }
            )
        return self._out(rows)

    def get_reading(self, reading_id: str) -> str:
        r = self.world.readings.get(reading_id)
        if r is None:
            return self._out({"error": f"unknown reading {reading_id}"})
        interp = self.world.interpretations.get(reading_id)
        return self._out(
            {
                "id": r.id,
                "kind": str(r.kind),
                "point": f"{r.point.segment_id} @ {r.point.offset_km:.3f} km",
                "values": dict(r.payload),
                "conditions": dict(r.conditions),
                "inspected_mm": r.inspected,
                "epoch": r.operating_epoch,
                "interpretation": None
                if interp is None
                else {
                    "outcome": str(interp.outcome),
                    "reasons": interp.reasons,
                    "detection_probability": interp.detection_probability,
                },
            }
        )

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

    def estimate_summary(self) -> str:
        est = self.world.estimate
        if est is None or not hasattr(est, "flows"):
            return self._out({"estimate": "unavailable", "detail": getattr(est, "reason", None)})
        return self._out(
            {
                "prior_only": est.prior_only,
                "accepted_readings": est.accepted_reading_ids,
                "observable_segments": est.observable_segments,
                "assumptions": est.assumptions,
                "flows": {
                    s: {
                        "p_mw": round(f.p_mw, 2),
                        "std": round(f.conditional_std_mw, 2),
                        "radius95": f.uncertainty_radius_95_mw,
                    }
                    for s, f in est.flows.items()
                    if f.observable
                },
            }
        )

    def consistency_summary(self) -> str:
        rep = self.world.consistency
        if rep is None:
            return self._out({"consistency": "none"})
        return self._out(
            {
                "status": str(rep.global_status),
                "dof": rep.degrees_of_freedom,
                "suspects": rep.suspect_reading_ids,
                "meaning": rep.suspect_meaning,
                "critical_sets": rep.critical_sets,
                "explanations": [e.label for e in rep.explanations],
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

    def test_electrical_candidate(self, out_of_service_segments: list[str]) -> str:
        """Re-solve the reduced model on the pinned readings under a candidate topology. Isolated:
        nothing outside this toolbox changes."""
        known = {s for s in out_of_service_segments if s in self.network.segments}
        bad = sorted(set(out_of_service_segments) - known)
        model = estimation.ElectricalModel(self.network, self.registry, frozenset(known))
        base = estimation.ElectricalModel(
            self.network, self.registry, frozenset(self.world.known_switching)
        )
        readings = estimation.effective_readings(self.world)
        rho = self.world.sensor_model.shared_rho
        out = {}
        for name, m in (("candidate", model), ("baseline", base)):
            est = estimation.estimate(m, readings, clock_s=self.world.clock_s, shared_rho=rho)
            cons = check_consistency(
                m,
                readings,
                shared_rho=rho,
                envelope=self.world.envelope,
                known_switching_clock=dict(self.world.known_switching),
                active_seed=self.world.scenario_seed,
            )
            out[name] = {
                "estimate": "unavailable" if not hasattr(est, "flows") else "ok",
                "dof": cons.degrees_of_freedom,
                "chi2": cons.global_chi2,
                "status": str(cons.global_status),
                "components": len(m.components),
            }
        n = len(self.diagnostics) + 1
        diag = DiagnosticRecord(
            f"DX-r{self.snapshot.revision}-{n:02d}",
            self.snapshot.revision,
            tuple(sorted(known)),
            out,
        )
        self.diagnostics[diag.id] = diag
        verdict = (
            "indistinguishable: both topologies are untestable with these readings"
            if out["candidate"]["status"] == "NOT_TESTABLE"
            and out["baseline"]["status"] == "NOT_TESTABLE"
            else "see chi2/status; model-limited statistics justify no exclusion"
        )
        return self._out(
            {"diagnostic_id": diag.id, "unknown_segments": bad, "results": out, "verdict": verdict}
        )


def build_toolbox(world: World, network: Network, registry: CircuitRegistry) -> Toolbox:
    return Toolbox(PinnedSnapshot.pin(world), network, registry)
