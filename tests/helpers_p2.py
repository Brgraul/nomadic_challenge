"""Shared helpers for the migrated three-method tests (not collected by pytest).

Kept tests build deterministic evidence by injecting LiDAR readings into a live Simulation's
world and republishing the snapshot: the runtime treats them exactly like readings a drone
published (interpretation, hazards, coverage, events, tied-readings semantics).
"""

from __future__ import annotations

import copy

from fixtures_three_method import tiny_network

from grid_ops.assessment import ingest_lidar
from grid_ops.grid import load_network
from grid_ops.sensing import (
    AcquisitionQuality,
    Interval,
    LidarPass,
    LidarPayload,
    LinePoint,
    Reading,
    ScanPose,
    ScanTrajectory,
)
from grid_ops.sim import Simulation
from grid_ops.world import Event, World


def real_sim(**kw) -> Simulation:
    """The public bavaria fixture through the real Simulation (drones, circuits, episode)."""
    return Simulation(load_network(), **kw)


def tiny_sim(**kw) -> Simulation:
    """The tiny fixture network through the real Simulation (fast, deterministic)."""
    return Simulation(tiny_network(), **kw)


def lidar_reading(
    geo,
    rid: str,
    segment: str,
    start_mm: int,
    end_mm: int,
    clearance_m: float | None,
    *,
    t0: int = 0,
    drone_id: str = "DRONE_1",
    task_id: str = "T0001",
    missing: tuple[Interval, ...] = (),
    inspected: tuple[Interval, ...] | None = None,
) -> Reading:
    """A public LiDAR reading over `segment`, as the episode would publish it."""
    iv = Interval(segment, start_mm, end_mm)
    lo = LinePoint(segment, start_mm)
    hi = LinePoint(segment, end_mm if end_mm > start_mm else start_mm + 1)
    poses = (
        ScanPose(geo.position_of(lo), 50.0, 0.0),
        ScanPose(geo.position_of(hi), 50.0, 0.0),
    )
    acq = LidarPass(iv, ScanTrajectory(poses, max(1, (end_mm - start_mm) // 10 or 1)))
    payload = LidarPayload(
        inspected=inspected if inspected is not None else (iv,),
        missing=missing,
        clearance_m=clearance_m,
        uncertainty_m=0.05,
        resolution_mm=10_000,
    )
    return Reading(
        id=rid,
        task_id=task_id,
        request_ids=(),
        drone_id=drone_id,
        acquisition=acq,
        acquired_start_s=t0,
        acquired_end_s=t0 + acq.trajectory.duration_s,
        payload=payload,
        quality=AcquisitionQuality(True),
    )


def inject_lidar(
    sim: Simulation,
    rid: str,
    segment: str,
    start_mm: int,
    end_mm: int,
    clearance_m: float | None,
    *,
    world: World | None = None,
) -> Reading:
    """Publish one LiDAR reading into the live world: evidence, event and snapshot together."""
    w = world if world is not None else sim._world
    reading = lidar_reading(
        sim.geometry, rid, segment, start_mm, end_mm, clearance_m, t0=w.clock_s
    )
    w.readings[rid] = reading
    ingest_lidar(w, reading)
    w.events.append((f"EVT{len(w.events) + 1:04d}", Event(w.clock_s, "reading", reading.drone_id, rid)))
    w.revision += 1
    if world is None:
        sim.snapshot = copy.deepcopy(w)
    return reading


def declare_stop(sim: Simulation, reason=None, detail: str = "fixture-declared stop state") -> None:
    """Fix the reconstruction's stop state, for tests of rules that sit downstream of it.

    The injected readings bypass the sim thread's belief refresh, so the state is set directly
    on the live world and republished.
    """
    from grid_ops.outage import StopAssessment, StopReason

    sim._world.outage_stop = StopAssessment(reason or StopReason.SUPPORTED, detail)
    sim.snapshot = copy.deepcopy(sim._world)


def publish(w: World, sim: Simulation) -> None:
    sim.snapshot = copy.deepcopy(w)