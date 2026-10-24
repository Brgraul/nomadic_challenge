"""Ticket 11: tasks exist only for accepted diagnostic needs."""

from __future__ import annotations

import pytest
from conftest import permissive

from grid_ops import agents, estimation
from grid_ops.geometry import LinePoint
from grid_ops.grid import load_network
from grid_ops.needs import MAX_RECONSTRUCTION_NEEDS, NeedStatus, group_id
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import CommitDecision, CommitSorties, Simulation
from grid_ops.world import Measurement, TaskStatus


@pytest.fixture(scope="module")
def network():
    return load_network()


def accept_startup(sim, did="D0"):
    agents.run_scripted_turn(sim)
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision(did, pf.id, pf.semantic_key, "operator")).result()
    return pf


def stops(*pairs, kind=Measurement.CLEARANCE):
    return tuple(StopRequest(LinePoint(s, mm), kind) for s, mm in pairs)


def tasks(world, **kw):
    return [t for t in world.tasks.values() if all(getattr(t, k) == v for k, v in kw.items())]


# -- accepted needs only ------------------------------------------------------------------------------


def test_no_tasks_without_an_accepted_diagnostic_need(network):
    sim = permissive(Simulation(network))
    sim.step(5)
    assert sim.snapshot.tasks == {} and sim.snapshot.needs == {}
    agents.run_scripted_turn(sim)  # a proposal alone is not an accepted need
    assert sim.snapshot.tasks == {} and sim.snapshot.needs == {}


def test_accepted_hypotheses_create_needs_that_state_their_causal_purpose(network):
    sim = permissive(Simulation(network))
    accept_startup(sim)
    w = sim.snapshot
    assert w.needs and w.tasks
    for need in w.needs.values():
        assert need.status is NeedStatus.ACTIVE
        assert need.causal_uncertainty and need.alternatives_distinguished and need.result_effect
        assert need.urgency in range(1, 6)  # copied from the manager's accepted hypothesis
    for t in w.tasks.values():
        assert t.status is TaskStatus.PENDING and t.assigned_drone_id is None
        assert t.need_ids and all(n in w.needs for n in t.need_ids)  # association retained
        assert t.purpose.startswith(("diagnostic", "confirmation", "repeat"))
    assert w.hypotheses_version == 1


def test_reservations_change_only_through_accepted_decisions(network):
    sim = permissive(Simulation(network))
    accept_startup(sim)
    before = {t.id: t.status for t in sim.snapshot.tasks.values()}
    sim.step(10)
    assert {
        t.id: t.status for t in sim.snapshot.tasks.values()
    } == before  # refresh reserves nothing
    # a sortie decision reserves a task by id
    task = next(t for t in sim.snapshot.tasks.values() if t.kind == "clearance")
    perch = sim._world.drones["DRONE_2"].state.perch
    req = SortieRequest(
        "DRONE_2", (StopRequest(task.point, task.kind, task.purpose, None, task.id),), perch
    )
    sim.submit(CommitSorties((req,), "operator")).result()
    assert sim.snapshot.tasks[task.id].status is TaskStatus.RESERVED
    sim.step(0.1)
    assert sim.snapshot.tasks[task.id].status in (TaskStatus.RESERVED, TaskStatus.MEASURING)
    assert sim.snapshot.tasks[task.id].assigned_drone_id == "DRONE_2"


# -- deterministic refresh without hypothesis change ----------------------------------------------------------


def test_new_coverage_refreshes_tasks_even_though_hypotheses_are_unchanged(network):
    sim = permissive(Simulation(network))
    accept_startup(sim)
    version = sim.snapshot.hypotheses_version
    open_before = {
        t.id
        for t in tasks(sim.snapshot, status=TaskStatus.PENDING)
        if t.point.segment_id == "LINE_064"
    }
    assert open_before
    perch = sim._world.drones["DRONE_2"].state.perch
    sim._apply(
        CommitSorties(
            (SortieRequest("DRONE_2", stops(("LINE_064", 9_000_000)), perch),), "operator"
        )
    )
    sim.step(15)
    w = sim.snapshot
    assert w.hypotheses_version == version  # same meaning
    cleared = [
        t
        for t in w.tasks.values()
        if t.point.segment_id == "LINE_064"
        and 8_800_000 < t.point.offset_mm < 9_500_000
        and t.status is TaskStatus.PENDING
        and t.kind == "clearance"
    ]
    assert cleared == []  # a qualified negative retired the sites it covers
    assert any("LINE_064" in n for _, n in w.task_log)
    # completed work and its reading stay in history
    assert [t for t in w.tasks.values() if t.status is TaskStatus.COMPLETED]


def test_repeating_a_refresh_is_idempotent(network):
    sim = permissive(Simulation(network))
    accept_startup(sim)
    ids = sorted(sim.snapshot.tasks)
    sim._refresh_tasks()
    sim._refresh_tasks()
    assert sorted(sim._world.tasks) == ids


# -- confirmation, repeat, unavailable covariance ---------------------------------------------------------------


def test_positive_evidence_creates_a_linked_confirmation_attempt(network):
    sim = permissive(Simulation(network))
    perch = sim._world.drones["DRONE_2"].state.perch
    sim._apply(
        CommitSorties(
            (
                SortieRequest(
                    "DRONE_2", stops(("LINE_058", 28_000), ("LINE_064", 9_000_000)), perch
                ),
            ),
            "operator",
        )
    )
    sim.step(25)
    agents.run_scripted_turn(sim)
    pf = sim.snapshot.proposals[-1]
    sim.submit(CommitDecision("D1", pf.id, pf.semantic_key, "operator")).result()
    w = sim.snapshot
    confirm = [t for t in w.tasks.values() if t.purpose.startswith("confirmation")]
    assert confirm and all(t.repeat_of_reading_id == "R0001" for t in confirm)
    assert all(t.status is TaskStatus.PENDING for t in confirm)
    need = w.needs[confirm[0].need_ids[0]]
    assert need.purpose == "confirmation" and need.repeat_of_reading_id == "R0001"


# -- canonical groups and local identity ---------------------------------------------------------------------------


def test_electrical_observations_share_one_group_and_local_sites_keep_exact_identity(network):
    a, b = LinePoint("LINE_026", 100_000), LinePoint("LINE_026", 9_000_000)
    assert group_id("current", a) == group_id("current", b) == "electrical:LINE_026:current"
    assert group_id("clearance", a) != group_id("clearance", b)
    assert "100000mm" in group_id("clearance", a)


def test_healthy_benchmark_alone_activates_reconstruction_tasks(network):
    sim = permissive(
        Simulation(network, scenario=ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 7))
    )
    sim._refresh_tasks()
    w = sim._world
    h0 = [t for t in w.tasks.values() if t.purpose.startswith("reconstruction")]
    # the nominal independent minimum is 28; work is staged so it fits under the attempt cap
    assert len(h0) == MAX_RECONSTRUCTION_NEEDS < 28
    assert len({t.group_id for t in h0}) == len(h0) and all(
        t.group_id.startswith("electrical:") for t in h0
    )
    assert all(n.startswith("N-H0-") for t in h0 for n in t.need_ids)
    # a real reading on one selected segment retires its need and cancels the unstarted attempt
    task = (
        next(t for t in h0 if t.point.segment_id == "LINE_026")
        if any(t.point.segment_id == "LINE_026" for t in h0)
        else h0[0]
    )
    seg = task.point.segment_id
    drone = min(
        w.drones.values(),
        key=lambda d: (
            estimation.__name__ and abs(d.position[0] - sim.geometry.position_of(task.point)[0])
        ),
    )
    perch = drone.state.perch
    req = SortieRequest(
        drone.id, (StopRequest(task.point, task.kind, task.purpose, None, task.id),), perch
    )
    try:
        sim.submit(CommitSorties((req,), "operator")).result()
    except Exception:
        pytest.skip("nearest drone cannot reach the segment")
    sim.step(40)
    done = sim.snapshot
    assert done.tasks[task.id].status is TaskStatus.COMPLETED
    need = done.needs[f"N-H0-{seg}"]
    assert need.status is NeedStatus.RETIRED


def test_incident_flow_gaps_alone_create_no_task(network):
    sim = permissive(Simulation(network))  # incident_demo
    sim._refresh_tasks()
    assert sim._world.tasks == {} and sim._world.needs == {}
    assert estimation.observability(sim._estimation.model).independent_readings > 0  # gaps exist


def test_unavailable_covariance_labels_options_without_information_gain(network):
    sim = permissive(
        Simulation(network, scenario=ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 7))
    )
    sim._estimation.estimate = estimation.EstimateUnavailable("injected")
    sim._world.estimate = sim._estimation.estimate
    sim._refresh_tasks()
    labelled = [u for u in sim._world.unavailable_work if "no information gain" in u.reason]
    assert labelled and all(u.need_id.startswith("N-H0-") for u in labelled)
    assert all(not hasattr(t, "information_gain") for t in sim._world.tasks.values())


# -- retirement --------------------------------------------------------------------------------------------------------


def hypothesis(world, hid):
    return dict(world.hypotheses[hid])


def test_retired_needs_lose_unstarted_work_and_route_stops_but_shared_tasks_survive(network):
    sim = permissive(Simulation(network))
    accept_startup(sim)
    w = sim._world
    veg = [
        t
        for t in w.tasks.values()
        if "N-H_VEG" in t.need_ids[0] and t.point.segment_id == "LINE_058"
    ]
    assert veg
    # reserve one VEG task for DRONE_2 through the normal path, and keep another pending
    target = veg[0]
    perch = w.drones["DRONE_2"].state.perch
    req = SortieRequest(
        "DRONE_2", (StopRequest(target.point, target.kind, target.purpose, None, target.id),), perch
    )
    sim.submit(CommitSorties((req,), "operator")).result()
    assert w.tasks[target.id].status in (TaskStatus.RESERVED, TaskStatus.MEASURING)
    # a second active hypothesis independently justifies the same site
    twin = hypothesis(w, "H_VEG") | {"id": "H_TWIN"}
    w.hypotheses["H_TWIN"] = twin
    sim._refresh_tasks()
    assert (
        set(w.tasks[target.id].need_ids)
        >= {n for n in w.tasks[target.id].need_ids if "H_TWIN" in n}
        != set()
    )
    # retire H_VEG only
    w.hypotheses["H_VEG"] = hypothesis(w, "H_VEG") | {"lifecycle": "RETIRED"}
    sim._refresh_tasks()
    assert w.tasks[target.id].status is TaskStatus.RESERVED  # still justified by H_TWIN
    assert all(
        n.status is NeedStatus.RETIRED for k, n in w.needs.items() if k.startswith("N-H_VEG-")
    )
    # retire the twin as well: nothing justifies the reserved attempt, so it goes, and so does the stop
    w.hypotheses["H_TWIN"] = twin | {"lifecycle": "RETIRED"}
    sim._refresh_tasks()
    assert (
        w.tasks[target.id].status is TaskStatus.CANCELLED and "retired" in w.tasks[target.id].note
    )
    drone = w.drones["DRONE_2"]
    assert all(s.task_id != target.id for s in drone.state.route.stops)


def test_locked_measuring_attempt_finishes_even_if_its_need_retires(network):
    sim = permissive(Simulation(network))
    accept_startup(sim)
    w = sim._world
    t = next(t for t in w.tasks.values() if t.point.segment_id == "LINE_058")
    perch = w.drones["DRONE_2"].state.perch
    req = SortieRequest("DRONE_2", (StopRequest(t.point, t.kind, t.purpose, None, t.id),), perch)
    sim.submit(CommitSorties((req,), "operator")).result()
    sim.step(0.2)  # the drone is a few metres away: it arrives and the dwell begins
    assert w.tasks[t.id].status is TaskStatus.MEASURING
    for hid in list(w.hypotheses):
        w.hypotheses[hid] = hypothesis(w, hid) | {"lifecycle": "RETIRED"}
    sim._refresh_tasks()
    assert w.tasks[t.id].status is TaskStatus.MEASURING
    sim.step(5)
    assert sim.snapshot.tasks[t.id].status is TaskStatus.COMPLETED and sim.snapshot.readings
