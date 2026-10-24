"""Ticket 03: validated multi-stop sorties, exact transitions and atomic acceptance."""

from __future__ import annotations

import copy
import dataclasses
import math

import pytest
from conftest import permissive

from grid_ops.geometry import MM_PER_KM, LandingSite, LinePoint
from grid_ops.grid import load_network
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.sim import CommitSorties, Dispatch, SetInService, Simulation
from grid_ops.world import (
    CHARGE_S,
    DEADLINE_S,
    DWELL_S,
    FLIGHT_S,
    Charging,
    CommandRejected,
    Flying,
    Measurement,
    Measuring,
    Ready,
    Stranded,
    TaskStatus,
    distance_km,
    leg_seconds,
)


@pytest.fixture(scope="module")
def network():
    return load_network()


@pytest.fixture
def sim(network):
    return permissive(Simulation(network))


def pt(segment: str, km: float) -> LinePoint:
    return LinePoint(segment, round(km * MM_PER_KM))


def land(segment: str, km: float) -> LandingSite:
    return LandingSite(pt(segment, km), f"{segment}/C1")


def sortie(drone, stops, landing):
    return SortieRequest(drone, tuple(stops), landing)


def run(sim, *sorties, author="operator"):
    return sim._apply(CommitSorties(tuple(sorties), author))


def two_stop_sortie():
    return sortie(
        "DRONE_3",
        [
            StopRequest(pt("LINE_026", 3.0), Measurement.VOLTAGE_PRESENCE),
            StopRequest(pt("LINE_026", 8.0), Measurement.CLEARANCE),
        ],
        land("LINE_026", 9.5),
    )


def expected_times(sim, drone_id, stops, landing):
    """Independent recomputation of the timeline from geometry, using the shared rounding rule."""
    drone = sim.snapshot.drones[drone_id]
    t, here, out = 0, drone.position, []
    for point, kind in stops:
        pos = sim.geometry.position_of(point)
        t += leg_seconds(here, pos, 1.0)
        out.append(t)
        t += DWELL_S[kind]
        out.append(t)
        here = pos
    t += leg_seconds(here, sim.geometry.position_of(landing.point), 1.0)
    return out, t


# -- execution -------------------------------------------------------------------------------


def test_multi_stop_sortie_executes_in_order_and_lands(sim):
    stops = [
        (pt("LINE_026", 3.0), Measurement.VOLTAGE_PRESENCE),
        (pt("LINE_026", 8.0), Measurement.CLEARANCE),
    ]
    landing = land("LINE_026", 9.5)
    times, land_t = expected_times(sim, "DRONE_3", stops, landing)
    run(sim, two_stop_sortie())
    w = sim._world
    assert isinstance(w.drones["DRONE_3"].state, Flying)
    assert {t.status for t in w.tasks.values()} == {TaskStatus.RESERVED}

    sim.step(times[0] / 60)
    d = sim.snapshot.drones["DRONE_3"]
    assert isinstance(d.state, Measuring) and d.state.stop.point == stops[0][0]
    assert d.position == sim.geometry.position_of(stops[0][0])

    sim.step((times[1] - times[0]) / 60)
    assert len(sim.snapshot.readings) == 1
    r = sim.snapshot.readings["R0001"]
    assert r.clock_s == times[1] and r.kind is Measurement.VOLTAGE_PRESENCE
    assert 95.0 < dict(r.payload)["LINE_026/C1.voltage_kv"] < 120.0
    sim.step((land_t - times[1]) / 60 + 0.001)
    snap = sim.snapshot
    assert [x.clock_s for x in snap.readings.values()] == [times[1], times[3]]
    assert snap.drones["DRONE_3"].history == ("T0001", "T0002")
    assert isinstance(snap.drones["DRONE_3"].state, Charging)
    assert snap.drones["DRONE_3"].state.until_s == land_t + CHARGE_S
    assert all(t.status is TaskStatus.COMPLETED for t in snap.tasks.values())


def test_readings_are_immutable(sim):
    run(sim, two_stop_sortie())
    sim.step(60)
    reading = next(iter(sim.snapshot.readings.values()))
    with pytest.raises(dataclasses.FrozenInstanceError):
        reading.payload = ()  # type: ignore[misc]


def test_position_and_battery_are_exact_mid_leg(sim):
    start = sim.snapshot.drones["DRONE_3"].position
    run(sim, two_stop_sortie())
    target = sim.geometry.position_of(pt("LINE_026", 3.0))
    leg = leg_seconds(start, target, 1.0)
    sim.step(leg / 2 / 60)
    d = sim.snapshot.drones["DRONE_3"]
    assert d.battery_s == FLIGHT_S - leg // 2
    f = (leg // 2) / leg
    assert d.position == pytest.approx(
        (start[0] + (target[0] - start[0]) * f, start[1] + (target[1] - start[1]) * f)
    )
    assert sim.snapshot.clock_s == leg // 2


def test_charging_battery_only_restored_when_charge_completes(sim):
    run(
        sim,
        sortie(
            "DRONE_3",
            [StopRequest(pt("LINE_026", 6.0), Measurement.CURRENT)],
            land("LINE_026", 6.2),
        ),
    )
    sim.step(5)
    d = sim.snapshot.drones["DRONE_3"]
    assert isinstance(d.state, Charging)
    low = d.battery_s
    assert 0 < low < FLIGHT_S
    # interrupt the charge: hidden switching, observable only as charging stopping
    sim._apply(SetInService("LINE_026", False, "scenario"))
    sim.step(1)
    assert isinstance(sim.snapshot.drones["DRONE_3"].state, Stranded)
    assert sim.snapshot.drones["DRONE_3"].battery_s == low
    sim.step(2)
    assert sim.snapshot.drones["DRONE_3"].battery_s == low
    # restore: the charge starts over, and only its completion refills the battery
    sim._apply(SetInService("LINE_026", True, "scenario"))
    resumed_at = sim._world.clock_s
    sim.step(1)
    st = sim.snapshot.drones["DRONE_3"].state
    assert isinstance(st, Charging)
    assert st.until_s == resumed_at + CHARGE_S
    sim.step(CHARGE_S / 60 - 2)
    assert sim.snapshot.drones["DRONE_3"].battery_s == low
    sim.step(2)
    d = sim.snapshot.drones["DRONE_3"]
    assert isinstance(d.state, Ready) and d.battery_s == FLIGHT_S
    assert d.position == sim.geometry.position_of(pt("LINE_026", 6.2))


def test_zero_distance_arrival_completes_without_division_or_drain(sim):
    here = sim.snapshot.drones["DRONE_3"].state.perch.point
    run(sim, sortie("DRONE_3", [StopRequest(here, Measurement.CURRENT)], land("LINE_026", 9.5)))
    d = sim._world.drones["DRONE_3"]
    assert isinstance(d.state, Measuring)  # arrived immediately, at commit time
    assert d.battery_s == FLIGHT_S and sim._world.clock_s == 0
    sim.step(DWELL_S[Measurement.CURRENT] / 60)
    assert len(sim.snapshot.readings) == 1
    assert sim.snapshot.readings["R0001"].clock_s == DWELL_S[Measurement.CURRENT]


def test_colocated_stops_chain_to_a_fixed_point_at_exact_times(sim):
    p = pt("LINE_026", 4.0)
    run(
        sim,
        sortie(
            "DRONE_3",
            [StopRequest(p, Measurement.VOLTAGE_PRESENCE), StopRequest(p, Measurement.CURRENT)],
            land("LINE_026", 4.0),
        ),
    )
    leg = leg_seconds(sim.snapshot.drones["DRONE_3"].position, sim.geometry.position_of(p), 1.0)
    sim.step(leg / 60 + (DWELL_S[Measurement.VOLTAGE_PRESENCE] + DWELL_S[Measurement.CURRENT]) / 60)
    times = [r.clock_s for r in sim.snapshot.readings.values()]
    assert times == [leg + 60, leg + 60 + 120]
    # zero-distance landing at the same exact point completes at the same instant as the reading
    assert isinstance(sim.snapshot.drones["DRONE_3"].state, Charging)
    assert sim.snapshot.drones["DRONE_3"].state.until_s == times[-1] + CHARGE_S


def test_hidden_only_changes_produce_no_public_events(sim):
    before = copy.deepcopy(sim.snapshot.log)
    sim._apply(SetInService("LINE_001", False, "scenario"))
    events = sim.step(10)
    assert events == []
    assert sim.snapshot.log == before


def test_attempt_ownership_and_repeat_link(sim):
    run(sim, two_stop_sortie())
    sim.step(120)
    first = sim.snapshot.readings["R0001"]
    assert sim.snapshot.tasks[first.task_id].reading_id == "R0001"
    # second sortie for the same drone is impossible while it is charging
    with pytest.raises(CommandRejected, match="drone_unavailable"):
        run(sim, two_stop_sortie())


def test_repeat_creates_new_attempt_linked_to_evidence(sim):
    run(
        sim,
        sortie(
            "DRONE_3",
            [StopRequest(pt("LINE_026", 3.0), Measurement.CURRENT)],
            land("LINE_026", 5.0),
        ),
    )
    sim.step(10)
    reading = next(iter(sim.snapshot.readings.values()))
    sim._world.drones["DRONE_3"].state = Ready(land("LINE_026", 5.0))
    sim._world.drones["DRONE_3"].battery_s = FLIGHT_S
    with pytest.raises(CommandRejected, match="repeat must state its purpose"):
        run(
            sim,
            sortie(
                "DRONE_3",
                [StopRequest(reading.point, Measurement.CURRENT, "", reading.id)],
                land("LINE_026", 4.0),
            ),
        )
    run(
        sim,
        sortie(
            "DRONE_3",
            [StopRequest(reading.point, Measurement.CURRENT, "confirm current", reading.id)],
            land("LINE_026", 4.0),
        ),
    )
    tasks = sim._world.tasks
    assert tasks["T0001"].status is TaskStatus.COMPLETED  # history preserved
    assert tasks["T0002"].repeat_of_reading_id == reading.id and tasks["T0002"].id != "T0001"


# -- locked dwell ----------------------------------------------------------------------------


def test_in_progress_dwell_is_locked_and_counted_once(sim):
    run(sim, two_stop_sortie())
    p1 = sim.geometry.position_of(pt("LINE_026", 3.0))
    leg = leg_seconds(sim.snapshot.drones["DRONE_3"].position, p1, 1.0)
    sim.step((leg + 20) / 60)  # 20 s into the 60 s voltage dwell
    d = sim._world.drones["DRONE_3"]
    assert isinstance(d.state, Measuring)
    battery_now = d.battery_s
    new_landing = land("LINE_026", 6.0)
    run(sim, sortie("DRONE_3", [], new_landing))  # replace the rest of the route
    state = sim._world.drones["DRONE_3"].state
    assert isinstance(state, Measuring) and state.stop.task_id == "T0001"
    assert state.route.stops == () and state.route.perch == new_landing
    sim.step(40 / 60)
    assert len(sim.snapshot.readings) == 1  # locked stop finished at its original time
    assert sim.snapshot.readings["R0001"].clock_s == leg + 60
    # unstarted second stop released, not cancelled and not duplicated
    assert sim.snapshot.tasks["T0002"].status is TaskStatus.PENDING
    assert sim.snapshot.tasks["T0002"].assigned_drone_id is None
    leg2 = leg_seconds(p1, sim.geometry.position_of(new_landing.point), 1.0)
    expected_battery_at_landing = battery_now - 40 - leg2
    sim.step(leg2 / 60 + 0.001)
    st = sim.snapshot.drones["DRONE_3"]
    assert isinstance(st.state, Charging)
    assert st.battery_s == expected_battery_at_landing  # remaining dwell charged exactly once


def test_mid_leg_replan_diverts_from_interpolated_position(sim):
    run(sim, two_stop_sortie())
    sim.step(60 / 60)
    pos = sim.snapshot.drones["DRONE_3"].position
    battery = sim.snapshot.drones["DRONE_3"].battery_s
    run(
        sim,
        sortie(
            "DRONE_3", [StopRequest(pt("LINE_026", 8.0), Measurement.MOTION)], land("LINE_026", 9.5)
        ),
    )
    d = sim._world.drones["DRONE_3"]
    assert isinstance(d.state, Flying) and d.state.origin == pos and d.battery_s == battery
    assert d.state.departed_s == 60
    assert sim._world.tasks["T0001"].status is TaskStatus.PENDING


# -- validation ------------------------------------------------------------------------------


def snapshot_of(sim):
    return copy.deepcopy(sim._world)


def test_invalid_later_sortie_leaves_every_drone_unchanged(sim):
    good = two_stop_sortie()
    bad = sortie(
        "DRONE_1",
        [
            StopRequest(pt("LINE_001", 0.05), Measurement.CURRENT),
            StopRequest(pt("LINE_040", 0.05), Measurement.CURRENT),
        ],
        land("LINE_040", 0.09),
    )
    before = snapshot_of(sim)
    with pytest.raises(CommandRejected, match="DRONE_1: battery"):
        run(sim, good, bad)
    assert sim._world.drones == before.drones
    assert sim._world.tasks == before.tasks and sim._world.next_task_number == 1
    assert sim._world.rejections[-1].reasons[0].startswith("DRONE_1: battery")
    # the good sortie alone is fine afterwards, with ids starting from T0001
    run(sim, good)
    assert "T0001" in sim._world.tasks


def test_every_rejection_reason_is_reported(sim):
    reqs = (
        sortie(
            "DRONE_1",
            [StopRequest(pt("LINE_040", 99.0), Measurement.CURRENT)],
            land("LINE_040", 0.05),
        ),
        sortie("DRONE_9", [], land("LINE_040", 0.05)),
        sortie("DRONE_2", [], LandingSite(pt("LINE_025", 0.02), "LINE_026/C1")),
    )
    with pytest.raises(CommandRejected) as err:
        run(sim, *reqs)
    msg = str(err.value)
    assert "bad_stop" in msg and "unknown_drone" in msg and "bad_landing" in msg


def test_sensor_capability_checked(sim):
    sim._world.drones["DRONE_3"].sensors = frozenset({Measurement.CURRENT})
    with pytest.raises(CommandRejected, match="sensor_capability"):
        run(sim, two_stop_sortie())


def test_battery_limit_from_authoritative_geometry(sim):
    # add dwell that busts the budget exactly: legal + dwell > battery
    stop = StopRequest(pt("LINE_040", 0.05), Measurement.MOTION)
    sim2 = permissive(Simulation(load_network()))
    pos = sim2.snapshot.drones["DRONE_1"].position
    first = leg_seconds(pos, sim2.geometry.position_of(stop.point), 1.0)
    second = leg_seconds(
        sim2.geometry.position_of(stop.point),
        sim2.geometry.position_of(land("LINE_026", 5.0).point),
        1.0,
    )
    need = first + DWELL_S[Measurement.MOTION] + second
    if need > FLIGHT_S:
        with pytest.raises(CommandRejected, match="battery"):
            run(sim2, sortie("DRONE_1", [stop], land("LINE_026", 5.0)))
    else:
        run(sim2, sortie("DRONE_1", [stop], land("LINE_026", 5.0)))


def test_deadline_rejects_stop_or_landing_beyond_t_plus_4(sim):
    w = sim._world
    w.clock_s = DEADLINE_S - 300
    with pytest.raises(CommandRejected, match="deadline"):
        run(
            sim,
            sortie(
                "DRONE_3",
                [StopRequest(pt("LINE_026", 9.0), Measurement.MOTION)],
                land("LINE_026", 9.5),
            ),
        )
    # a landing alone that arrives after the deadline is also refused
    far_leg = leg_seconds(
        w.drones["DRONE_3"].position, sim.geometry.position_of(pt("LINE_026", 10.8)), 1.0
    )
    assert far_leg > 0
    w.clock_s = DEADLINE_S - 10
    with pytest.raises(CommandRejected, match="deadline"):
        run(sim, sortie("DRONE_3", [], land("LINE_026", 10.8)))
    # exactly on time is allowed
    t_exact = leg_seconds(
        w.drones["DRONE_3"].position, sim.geometry.position_of(pt("LINE_026", 5.5)), 1.0
    )
    w.clock_s = DEADLINE_S - t_exact
    run(sim, sortie("DRONE_3", [], land("LINE_026", 5.5)))


def test_landing_occupancy_and_circuit_identity(sim):
    with pytest.raises(CommandRejected, match="bad_landing"):
        run(sim, sortie("DRONE_3", [], LandingSite(pt("LINE_026", 5.0), "LINE_025/C1")))
    # a perched drone's exact site cannot be taken by another drone
    perch = sim.snapshot.drones["DRONE_3"].state.perch
    with pytest.raises(CommandRejected, match="landing_occupied"):
        run(sim, sortie("DRONE_2", [], perch))
    # a Ready drone cannot be sent nowhere
    with pytest.raises(CommandRejected, match="no_flight"):
        run(sim, sortie("DRONE_3", [], perch))
    # two drones cannot plan the same exact site
    with pytest.raises(CommandRejected, match="landing_occupied"):
        run(
            sim,
            sortie("DRONE_3", [], land("LINE_026", 5.0)),
            sortie("DRONE_2", [], land("LINE_026", 5.0)),
        )


def test_dispatch_still_works_through_the_validator(sim):
    msg = sim._apply(Dispatch("DRONE_1", "LINE_001", "operator"))
    assert "DRONE_1 departed for LINE_001" in msg
    d = sim._world.drones["DRONE_1"]
    assert isinstance(d.state, Flying)
    dist = distance_km(sim.snapshot.drones["DRONE_1"].position, d.state.route.perch_position)
    assert d.state.arrives_s == math.ceil(60 * dist)
    with pytest.raises(CommandRejected, match="Unknown segment"):
        sim._apply(Dispatch("DRONE_2", "NOPE", "operator"))
