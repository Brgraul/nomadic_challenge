"""Behavior tests for itinerary execution: energy continuity, charging, readings, deadlines."""

from __future__ import annotations

from dataclasses import dataclass, field

import pytest

from grid_ops.energy import DEFAULT_ENERGY_MODEL, BatteryState
from grid_ops.geometry import Geometry, Interval, LandingSite, LinePoint
from grid_ops.grid import Network, Node, Segment
from grid_ops.sensing import (
    AcquisitionQuality,
    AcquisitionSample,
    ConductorPoint,
    ContactCurrent,
    CurrentPayload,
    LidarPass,
    MeasurementTaskV2,
    ScanPose,
    ScanTrajectory,
    TaskStatusV2,
)
from grid_ops.world import (
    DEADLINE_S,
    LAND_S,
    TAKEOFF_S,
    Charge,
    Drone,
    GroundSample,
    Itinerary,
    Land,
    LineStatus,
    Perched,
    Scan,
    ScheduledStep,
    Takeoff,
    Transit,
    World,
    advance_to,
    start_itinerary,
)

NODE_A = (11.0, 48.0)
NODE_B = (11.01, 48.0)
NODE_C = (11.02, 48.0)


def tiny_network() -> Network:
    nodes = {
        "N1": Node("N1", "substation", None, "K1", NODE_A),
        "N2": Node("N2", "pole", None, "K1", NODE_B),
        "N3": Node("N3", "pole", None, "K1", NODE_C),
    }
    segments = {
        "LINE_040": Segment("LINE_040", "K1", "N1", "N2", 110.0, 1, 1.11, (NODE_A, NODE_B)),
        "LINE_025": Segment("LINE_025", "K1", "N2", "N3", 110.0, 1, 1.11, (NODE_B, NODE_C)),
    }
    return Network(nodes=nodes, segments=segments, areas={}, reports={})


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


@pytest.fixture()
def world():
    net = tiny_network()
    geo = Geometry(net)
    drones = {}
    for drone_id in ("DRONE_1", "DRONE_2"):
        point = LinePoint("LINE_040", 0)
        perch = LandingSite(point, "LINE_040/C1")
        drones[drone_id] = Drone(drone_id, geo.position_of(point), BatteryState(), Perched(perch))
    w = World(clock_s=0, drones=drones, belief=dict.fromkeys(net.segments, LineStatus.UNKNOWN))
    w.segment_lengths = {sid: geo.length_mm(sid) for sid in net.segments}
    from grid_ops.circuits import Circuit

    w.circuits_by_segment = {
        sid: (
            Circuit(f"{sid}/C1", sid, 1, 110.0, "N1", "N2", "fake", 1.0, 0.1, 1000.0),
        )
        for sid in net.segments
    }
    return w


def site_at(w: World, seg: str = "LINE_040", offset_mm: int = 0) -> LandingSite:
    return LandingSite(LinePoint(seg, offset_mm), f"{seg}/C1")


def ready_drone(w: World, site: LandingSite, geo: Geometry) -> None:
    d = w.drones["DRONE_1"]
    d.position = geo.position_of(site.point)
    d.state = Perched(site)


def test_hop_drains_energy_and_never_resets(world):
    geo = Geometry(tiny_network())
    a, b = site_at(world), site_at(world, "LINE_025", 500_000)
    pos_a, pos_b = geo.position_of(a.point), geo.position_of(b.point)
    physics = StubPhysics()
    d = world.drones["DRONE_1"]
    ready_drone(world, a, geo)
    conductor = ConductorPoint(b.point, b.circuit_id)
    it = Itinerary(
        (
            ScheduledStep(0, 0, TAKEOFF_S, Takeoff(a, pos_a)),
            ScheduledStep(1, TAKEOFF_S, TAKEOFF_S + 60, Transit(pos_a, pos_b)),
            ScheduledStep(2, TAKEOFF_S + 60, TAKEOFF_S + 60 + LAND_S, Land(b, pos_b)),
            ScheduledStep(
                3,
                TAKEOFF_S + 60 + LAND_S,
                TAKEOFF_S + 60 + LAND_S + 10,
                GroundSample((ContactCurrent(conductor, 10),), b, ("T1",), pos_b),
            ),
        ),
        b,
    )
    start_units = d.battery.remaining_units
    start_itinerary(world, "DRONE_1", it)
    events = advance_to(world, physics, it.end_s)
    assert d.state == Perched(b)
    assert d.battery.remaining_units < start_units
    assert d.battery.observed_s == it.end_s
    assert d.history == ("T1",)
    assert world.readings["R0001"].task_id == "T1"
    assert any(e.kind == "reading" for e in events)


def test_two_contacts_no_recharge_then_depart(world):
    geo = Geometry(tiny_network())
    a = site_at(world)
    pos = geo.position_of(a.point)
    physics = StubPhysics()
    d = world.drones["DRONE_1"]
    ready_drone(world, a, geo)
    conductor = ConductorPoint(a.point, a.circuit_id)
    members = (ContactCurrent(conductor, 10), ContactCurrent(conductor, 10))
    it = Itinerary(
        (
            ScheduledStep(0, 0, 10, GroundSample(members, a, ("T1", "T2"), pos)),
            ScheduledStep(1, 10, 10 + TAKEOFF_S, Takeoff(a, pos)),
        ),
        a,
    )
    before = d.battery.remaining_units
    start_itinerary(world, "DRONE_1", it)
    advance_to(world, physics, it.end_s)
    assert len(world.readings) == 2
    expected = before - 10 * DEFAULT_ENERGY_MODEL.ground_units_per_s - (
        TAKEOFF_S * DEFAULT_ENERGY_MODEL.units_per_airborne_s
        + DEFAULT_ENERGY_MODEL.takeoff_units
    )
    assert d.battery.remaining_units == expected
    assert d.state == Perched(a)


def test_observed_partial_charge_adds_exact_gain(world):
    geo = Geometry(tiny_network())
    a = site_at(world)
    pos = geo.position_of(a.point)
    physics = StubPhysics(chargeable=frozenset({a.circuit_id}))
    d = world.drones["DRONE_1"]
    ready_drone(world, a, geo)
    it = Itinerary(
        (
            ScheduledStep(0, 0, LAND_S, Land(a, pos)),
            ScheduledStep(1, LAND_S, LAND_S + 10, Charge(a, 10, pos)),
        ),
        a,
    )
    start_itinerary(world, "DRONE_1", it)
    advance_to(world, physics, LAND_S)
    before_charge = d.battery.remaining_units
    events = advance_to(world, physics, it.end_s)
    gain = DEFAULT_ENERGY_MODEL.net_charge_units_per_s * 10
    assert d.battery.remaining_units == min(
        DEFAULT_ENERGY_MODEL.capacity_units, before_charge + gain
    )
    assert d.state == Perched(a)
    assert any(e.kind == "charged" for e in events)


def test_failed_handshake_leaves_perched_without_gain(world):
    geo = Geometry(tiny_network())
    a = site_at(world)
    pos = geo.position_of(a.point)
    physics = StubPhysics(chargeable=frozenset())
    d = world.drones["DRONE_1"]
    ready_drone(world, a, geo)
    it = Itinerary(
        (
            ScheduledStep(0, 0, LAND_S, Land(a, pos)),
            ScheduledStep(1, LAND_S, LAND_S + 10, Charge(a, 10, pos)),
        ),
        a,
    )
    start_itinerary(world, "DRONE_1", it)
    events = advance_to(world, physics, it.end_s)
    assert d.state == Perched(a)
    assert any(e.kind == "charge_failed" for e in events)
    assert d.battery.remaining_units < DEFAULT_ENERGY_MODEL.capacity_units


def test_shortened_scan_publishes_partial_fraction(world):
    geo = Geometry(tiny_network())
    a = site_at(world)
    pos = geo.position_of(a.point)
    physics = StubPhysics()
    ready_drone(world, a, geo)
    end = LinePoint("LINE_040", geo.length_mm("LINE_040"))
    poses = (ScanPose(pos, 50.0, 0.0, -60.0), ScanPose(geo.position_of(end), 50.0, 0.0, -60.0))
    scan = LidarPass(
        Interval("LINE_040", 0, geo.length_mm("LINE_040")), ScanTrajectory(poses, 100), 10.0
    )
    it = Itinerary((ScheduledStep(0, 0, 40, Scan(scan, "T1")),), a)
    start_itinerary(world, "DRONE_1", it)
    advance_to(world, physics, it.end_s)
    assert physics.acquired[0].completed_fraction == pytest.approx(0.4)
    assert world.readings["R0001"].acquired_end_s == 40


def test_energy_exhaustion_raises(world):
    geo = Geometry(tiny_network())
    a = site_at(world)
    pos = geo.position_of(a.point)
    far_end = LinePoint("LINE_025", geo.length_mm("LINE_025"))
    far = geo.position_of(far_end)
    physics = StubPhysics()
    d = world.drones["DRONE_1"]
    ready_drone(world, a, geo)
    d.battery = BatteryState(remaining_units=1000)
    it = Itinerary(
        (ScheduledStep(0, 0, 60, Transit(pos, far)),), site_at(world, "LINE_025", far_end.offset_mm)
    )
    start_itinerary(world, "DRONE_1", it)
    with pytest.raises(RuntimeError, match="energy exhausted"):
        advance_to(world, physics, it.end_s)


def test_advance_caps_at_deadline(world):
    physics = StubPhysics()
    advance_to(world, physics, DEADLINE_S + 9999)
    assert world.clock_s == DEADLINE_S


def test_task_status_lifecycle_through_steps(world):
    geo = Geometry(tiny_network())
    a = site_at(world)
    pos = geo.position_of(a.point)
    physics = StubPhysics()
    ready_drone(world, a, geo)
    conductor = ConductorPoint(a.point, a.circuit_id)
    world.set_task(
        MeasurementTaskV2(
            "T1", "REQ1", ContactCurrent(conductor, 10), "REQ1", TaskStatusV2.RESERVED, 3
        )
    )
    it = Itinerary(
        (ScheduledStep(0, 0, 10, GroundSample((ContactCurrent(conductor, 10),), a, ("T1",), pos)),),
        a,
    )
    start_itinerary(world, "DRONE_1", it)
    assert world.tasks["T1"].status is TaskStatusV2.MEASURING
    advance_to(world, physics, it.end_s)
    assert world.tasks["T1"].status is TaskStatusV2.COMPLETED
    assert world.tasks["T1"].reading_id == "R0001"
    assert world.readings["R0001"].request_ids == ("REQ1",)


def test_land_without_charge_next_keeps_energy(world):
    geo = Geometry(tiny_network())
    a = site_at(world)
    pos = geo.position_of(a.point)
    physics = StubPhysics(chargeable=frozenset({a.circuit_id}))
    d = world.drones["DRONE_1"]
    ready_drone(world, a, geo)
    it = Itinerary((ScheduledStep(0, 0, LAND_S, Land(a, pos)),), a)
    before = d.battery.remaining_units
    start_itinerary(world, "DRONE_1", it)
    advance_to(world, physics, it.end_s)
    assert d.battery.remaining_units == before - (
        LAND_S * DEFAULT_ENERGY_MODEL.units_per_airborne_s + DEFAULT_ENERGY_MODEL.landing_units
    )
    assert d.state == Perched(a)
