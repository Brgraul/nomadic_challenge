"""Shared tiny-network fixtures for the three-method runtime tests."""

from __future__ import annotations

from dataclasses import dataclass, field

from grid_ops.circuits import Circuit
from grid_ops.energy import BatteryState
from grid_ops.geometry import Geometry, LandingSite, LinePoint
from grid_ops.grid import Area, Network, Node, Report, Segment
from grid_ops.sensing import (
    AcquisitionQuality,
    AcquisitionSample,
    ConductorPoint,
    ContactCurrent,
    CurrentPayload,
)
from grid_ops.world import Drone, Perched, World

NODE_A = (11.0, 48.0)
NODE_B = (11.01, 48.0)
NODE_C = (11.02, 48.0)


def tiny_network() -> Network:
    nodes = {
        "N1": Node("N1", "abstract_substation", None, "K1", NODE_A),
        "N2": Node("N2", "pole", None, "K1", NODE_B),
        "N3": Node("N3", "pole", None, "K1", NODE_C),
        "N4": Node("N4", "terminal", None, "K1", (11.03, 48.0)),
        "N5": Node("N5", "terminal", None, "K1", (11.04, 48.0)),
        "N6": Node("N6", "terminal", None, "K1", (11.05, 48.0)),
    }
    segments = {
        "LINE_040": Segment("LINE_040", "K1", "N1", "N2", 110.0, 1, 1.11, (NODE_A, NODE_B)),
        "LINE_025": Segment("LINE_025", "K1", "N2", "N3", 110.0, 1, 1.11, (NODE_B, NODE_C)),
        "LINE_026": Segment("LINE_026", "K1", "N3", "N4", 110.0, 1, 1.11, (NODE_C, (11.03, 48.0))),
        "LINE_002": Segment("LINE_002", "K1", "N3", "N5", 110.0, 1, 1.11, (NODE_C, (11.04, 48.0))),
        "LINE_056": Segment("LINE_056", "K1", "N3", "N6", 110.0, 1, 1.11, (NODE_C, (11.05, 48.0))),
    }
    areas = {
        "SYN_REPORT_02": Area(
            "SYN_REPORT_02",
            "report_area",
            "report 02 area",
            ((10.99, 47.99), (11.03, 47.99), (11.03, 48.01), (10.99, 48.01)),
        ),
    }
    reports = {
        "SYN_REPORT_02": Report(
            "SYN_REPORT_02", "vegetation", "Vegetation encroachment possible.", "T-2h", "low",
            "SYN_REPORT_02",
        ),
        "SYN_REPORT_03": Report(
            "SYN_REPORT_03", "switching", "LINE_040 was precautionarily switched off.", "T-1h",
            "high", "SYN_REPORT_03",
        ),
    }
    return Network(nodes=nodes, segments=segments, areas=areas, reports=reports)


def fake_circuit(sid: str, index: int = 1) -> Circuit:
    return Circuit(f"{sid}/C{index}", sid, index, 110.0, "N1", "N2", "fake", 1.0, 0.1, 1000.0)


@dataclass
class StubPhysics:
    chargeable: frozenset[str] = frozenset()
    acquired: list = field(default_factory=list)

    def is_chargeable(self, site: LandingSite) -> bool:
        return site.circuit_id in self.chargeable

    def acquire(self, executed):
        self.acquired.append(executed)
        conductor = executed.landed_conductor or ConductorPoint(
            LinePoint("LINE_040", 0), "LINE_040/C1"
        )
        payload = CurrentPayload(conductor, 12.0, 1.0, 10)
        return AcquisitionSample(
            executed.id, payload, AcquisitionQuality(True, "", False, True, "current:LINE_040/C1")
        )


def two_drone_world() -> tuple[World, Geometry]:
    net = tiny_network()
    geo = Geometry(net)
    drones = {}
    perches = {"DRONE_1": ("LINE_040", 0), "DRONE_2": ("LINE_025", 0)}
    for drone_id, (sid, mm) in perches.items():
        point = LinePoint(sid, mm)
        perch = LandingSite(point, f"{sid}/C1")
        drones[drone_id] = Drone(
            drone_id, geo.position_of(point), BatteryState(), Perched(perch)
        )
    w = World(clock_s=0, drones=drones, belief=dict.fromkeys(net.segments, "unknown"))
    w.segment_lengths = {sid: geo.length_mm(sid) for sid in net.segments}
    w.circuits_by_segment = {
        sid: (fake_circuit(sid),) for sid in net.segments
    }
    return w, geo


def site_at(w: World, seg: str = "LINE_040", offset_mm: int = 0) -> LandingSite:
    return LandingSite(LinePoint(seg, offset_mm), f"{seg}/C1")


def contact(acq_site: LandingSite, integration_s: int = 10) -> ContactCurrent:
    return ContactCurrent(ConductorPoint(acq_site.point, acq_site.circuit_id), integration_s)


# -- appended for tests/test_planning_v2.py ------------------------------------------------------


def seed_handshakes(w: World, circuits: tuple[str, ...] = ("LINE_040/C1", "LINE_025/C1")) -> None:
    """Successful landing handshakes: both fixture circuits become chargeable and eligible."""
    from grid_ops import knowledge

    for circuit_id in circuits:
        knowledge.ingest_handshake(w, circuit_id, True, "setup", charging=False)


def contact_task(
    w: World, task_id: str, site: LandingSite, request_id: str, urgency: int = 5
) -> MeasurementTaskV2:
    """Register an active contact-current request and its PENDING task at `site`."""
    from grid_ops.sensing import (
        AcceptedRequest,
        ConductorScope,
        CurrentRequest,
        MeasurementTaskV2,
        OutcomeSpec,
        RequestStatus,
        TaskStatusV2,
    )

    request = CurrentRequest(
        id=request_id,
        hypothesis_ids=(),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=urgency,
        source_revision="rev0",
        replaces_request_ids=(),
        target=ConductorScope(circuit_id=site.circuit_id, phase="A", start_mm=0, end_mm=1),
    )
    w.requests[request_id] = AcceptedRequest(
        request_id, request, RequestStatus.ACTIVE, w.clock_s, "rev0"
    )
    task = MeasurementTaskV2(
        task_id, request_id, contact(site), request_id, TaskStatusV2.PENDING, urgency
    )
    w.set_task(task)
    return task


from typing import TYPE_CHECKING  # noqa: E402

if TYPE_CHECKING:
    from grid_ops.sensing import MeasurementTaskV2
