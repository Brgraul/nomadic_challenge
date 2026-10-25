"""The request-to-task lifecycle.

A task exists only because an *accepted* measurement request needs it. `refresh` compiles every
active request through `sensing.compile_requests`, keeps locked attempts (reserved or in
progress), cancels unstarted attempts whose request retired or whose alternative group is already
answered, and reports every rejected request as explicitly unavailable work. It never invents a
method: the request owns it.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from grid_ops.geometry import Geometry, Interval
from grid_ops.sensing import (
    LidarPass,
    MeasurementTaskV2,
    RequestStatus,
    TaskStatusV2,
    compile_requests,
)
from grid_ops.world import World

MAX_OPEN_TASKS = 40


@dataclass(frozen=True)
class Unavailable:
    request_id: str
    reason: str


def _inspected_intervals(world: World) -> tuple[Interval, ...]:
    out = []
    for reading in world.readings.values():
        payload = reading.payload
        if hasattr(payload, "inspected"):
            out.extend(payload.inspected)
    return tuple(out)


def _answered_groups(world: World) -> set[str]:
    """Alternative groups with a completed valid attempt earn nothing further."""
    answered = set()
    for task in world.tasks.values():
        if task.status is TaskStatusV2.COMPLETED and task.reading_id in world.readings:
            answered.add(task.alternative_group_id)
    return answered


def refresh(world: World, network, geometry: Geometry) -> tuple[list[str], list[Unavailable]]:
    """Recompute open tasks from accepted requests. Returns (change notes, unavailable work)."""
    notes: list[str] = []
    active = [a for a in world.requests.values() if a.status is RequestStatus.ACTIVE]
    compiled = compile_requests(
        network,
        geometry,
        world.capabilities,
        [a.request for a in active],
        covered_intervals=_inspected_intervals(world),
        clock_s=world.clock_s,
    )
    unavailable = [Unavailable(r.request_id, r.reason) for r in compiled.rejections]
    answered = _answered_groups(world)
    locked = {
        task.alternative_group_id
        for task in world.tasks.values()
        if task.status in (TaskStatusV2.RESERVED, TaskStatusV2.MEASURING)
    }
    wanted: dict[tuple[str, str], MeasurementTaskV2] = {}
    for task in compiled.tasks:
        if task.alternative_group_id in answered:
            continue
        if task.alternative_group_id in locked:
            continue
        key = (task.request_id, _acquisition_key(task.acquisition))
        wanted.setdefault(key, task)
    existing = {
        (t.request_id, _acquisition_key(t.acquisition)): t
        for t in world.tasks.values()
        if t.status in (TaskStatusV2.PENDING,)
    }
    open_count = sum(
        1
        for t in world.tasks.values()
        if t.status in (TaskStatusV2.PENDING, TaskStatusV2.RESERVED, TaskStatusV2.MEASURING)
    )
    for key, task in wanted.items():
        if key in existing:
            continue
        if open_count >= MAX_OPEN_TASKS:
            unavailable.append(
                Unavailable(task.request_id, f"open-task cap {MAX_OPEN_TASKS} reached")
            )
            continue
        tid = f"T{world.next_task_number:04d}"
        world.next_task_number += 1
        world.set_task(
            replace(
                task,
                id=tid,
                status=TaskStatusV2.PENDING,
                note=f"compiled from request {task.request_id}",
            )
        )
        open_count += 1
        notes.append(f"{tid} compiled for request {task.request_id}")
    active_request_ids = {a.id for a in active}
    for task in list(world.tasks.values()):
        if task.status is not TaskStatusV2.PENDING:
            continue
        if task.request_id not in active_request_ids:
            reason = "its request was retired or rejected"
        elif task.alternative_group_id in answered:
            reason = "its alternative group is already answered by valid evidence"
        elif (task.request_id, _acquisition_key(task.acquisition)) not in wanted:
            reason = "its request no longer compiles this acquisition"
        else:
            continue
        world.set_task(
            replace(task, status=TaskStatusV2.CANCELLED, assigned_drone_id=None, note=reason)
        )
        notes.append(f"{task.id} cancelled: {reason}")
    world.unavailable_work = unavailable
    return notes, unavailable


def _acquisition_key(acquisition) -> tuple:
    if isinstance(acquisition, LidarPass):
        t = acquisition.target
        return ("lidar", t.segment_id, t.start_mm, t.end_mm,
                acquisition.trajectory.poses[0].position, acquisition.trajectory.duration_s)
    a = acquisition
    return (
        str(a.method),
        a.conductor.point.segment_id,
        a.conductor.point.offset_mm,
        a.conductor.circuit_id,
        a.conductor.phase,
    )
