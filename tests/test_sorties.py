"""Itinerary sorties: atomic validation, exact rejection reasons and dispatch through the validator."""

from __future__ import annotations

import copy
from dataclasses import replace

import pytest
from fixtures_three_method import StubPhysics, site_at, two_drone_world
from helpers_p2 import lidar_reading, real_sim

from grid_ops.geometry import LandingSite, LinePoint
from grid_ops.routes import SortieRequest, apply_sorties, validate_sorties
from grid_ops.sim import CommitSorties, Dispatch
from grid_ops.world import (
    DEADLINE_S,
    LAND_S,
    TAKEOFF_S,
    CommandRejected,
    Executing,
    Land,
    ScheduledStep,
    Takeoff,
    Transit,
    leg_seconds,
)


def world():
    return two_drone_world()


def request_for(w, drone_id, steps, final):
    return SortieRequest(
        drone_id, steps, final, w.revision, w.energy_model.revision, w.capabilities.revision
    )


def fly_steps(w, geo, drone_id, site):
    """Takeoff, transit, land: a minimal validated itinerary ending perched on `site`."""
    pos = geo.position_of(site.point)
    state = w.drones[drone_id].state
    start = state.perch if hasattr(state, "perch") else state.itinerary.final_perch
    start_pos = geo.position_of(start.point)
    leg = max(leg_seconds(start_pos, pos, w.drones[drone_id].speed_km_per_min), 1)
    return (
        ScheduledStep(0, 0, TAKEOFF_S, Takeoff(start, start_pos)),
        ScheduledStep(1, TAKEOFF_S, TAKEOFF_S + leg, Transit(start_pos, pos)),
        ScheduledStep(2, TAKEOFF_S + leg, TAKEOFF_S + leg + LAND_S, Land(site, pos)),
    )


# -- validation and atomic acceptance ----------------------------------------------------------------


def test_a_valid_itinerary_validates_and_ends_perched_on_its_final_perch():
    w, geo = world()
    target = site_at(w, "LINE_025", 50_000)
    req = request_for(w, "DRONE_1", fly_steps(w, geo, "DRONE_1", target), target)
    result = validate_sorties(w, geo, None, (req,))
    assert result.ok, [str(r) for r in result.rejections]
    assert result.plans[0].itinerary.final_perch == target
    assert result.plans[0].itinerary.end_s == req.steps[-1].end_s


def test_invalid_later_itineraries_leave_every_drone_unchanged():
    sim = real_sim()
    w, geo = sim._world, sim.geometry
    good = request_for(w, "DRONE_1", fly_steps(w, geo, "DRONE_1", site_at(w, "LINE_025", 50_000)),
                       site_at(w, "LINE_025", 50_000))
    far = LandingSite(LinePoint("LINE_024", 500_000), "LINE_024/C1")
    bad = request_for(w, "DRONE_9", fly_steps(w, geo, "DRONE_1", far), far)
    before = copy.deepcopy((w.drones, w.tasks, w.next_task_number))
    with pytest.raises(CommandRejected) as err:
        sim.submit(CommitSorties((good, bad), "operator")).result()
    assert "DRONE_9" in str(err.value)
    assert (w.drones, w.tasks, w.next_task_number) == before
    assert w.rejections[-1].reasons
    sim.submit(CommitSorties((good,), "operator")).result()  # the good itinerary alone is fine
    assert isinstance(w.drones["DRONE_1"].state, Executing)


def test_every_rejection_reason_is_reported():
    w, geo = world()
    reqs = (
        request_for(w, "DRONE_9", fly_steps(w, geo, "DRONE_1", site_at(w)), site_at(w)),
        request_for(w, "DRONE_1", (), site_at(w)),
    )
    result = validate_sorties(w, geo, None, reqs)
    assert {"unknown_drone", "empty_itinerary"} <= {r.code for r in result.rejections}


def test_a_second_itinerary_for_an_executing_drone_is_rejected():
    w, geo = world()
    target = site_at(w, "LINE_025", 50_000)
    first = request_for(w, "DRONE_1", fly_steps(w, geo, "DRONE_1", target), target)
    apply_sorties(w, StubPhysics(), validate_sorties(w, geo, None, (first,)).plans)
    second = request_for(w, "DRONE_1", fly_steps(w, geo, "DRONE_1", site_at(w)), site_at(w))
    result = validate_sorties(w, geo, None, (second,))
    assert not result.ok and result.rejections[0].code == "bad_start", (
        "a fresh itinerary for an executing drone must start at its current step boundary"
    )


def test_itineraries_past_the_deadline_are_rejected():
    w, geo = world()
    target = site_at(w, "LINE_025", 50_000)
    late = tuple(
        replace(s, start_s=s.start_s + DEADLINE_S, end_s=s.end_s + DEADLINE_S)
        for s in fly_steps(w, geo, "DRONE_1", target)
    )
    result = validate_sorties(w, geo, None, (request_for(w, "DRONE_1", late, target),))
    assert not result.ok and any(r.code == "deadline" for r in result.rejections)


def test_energy_is_recomputed_from_the_drone_battery_not_the_caller():
    w, geo = world()
    target = site_at(w, "LINE_025", 50_000)
    steps = fly_steps(w, geo, "DRONE_1", target)
    w.drones["DRONE_1"].battery = replace(w.drones["DRONE_1"].battery, remaining_units=1000)
    result = validate_sorties(w, geo, None, (request_for(w, "DRONE_1", steps, target),))
    assert not result.ok and any(r.code == "energy" for r in result.rejections)


# -- readings and dispatch ----------------------------------------------------------------------------


def test_readings_are_immutable():
    w, geo = world()
    w.readings["R0001"] = lidar_reading(geo, "R0001", "LINE_040", 0, 110_000, 2.0)
    with pytest.raises(Exception):
        w.readings["R0001"].payload = None  # type: ignore[misc]


def test_dispatch_builds_a_validated_itinerary_and_unknown_segments_are_rejected():
    sim = real_sim()
    msg = sim._apply(Dispatch("DRONE_1", "LINE_001", "operator"))
    assert "DRONE_1 departed for LINE_001" in msg
    assert isinstance(sim._world.drones["DRONE_1"].state, Executing)
    with pytest.raises(CommandRejected, match="Unknown segment"):
        sim._apply(Dispatch("DRONE_2", "NOPE", "operator"))