"""End-to-end runtime tests: request -> accept -> compile -> plan -> execute -> revise."""

from __future__ import annotations

import pytest
from fixtures_three_method import tiny_network

from grid_ops import agents
from grid_ops.coordinator import Coordinator
from grid_ops.sensing import ContactCurrent
from grid_ops.sim import Simulation
from grid_ops.world import DEADLINE_S, Perched


class TinySim(Simulation):
    """The real Simulation over the tiny fixture network."""

    def __init__(self, **kw):
        net = tiny_network()
        super().__init__(net, **kw)


@pytest.fixture()
def sim(monkeypatch):
    s = TinySim()
    return Coordinator(s), s


def tb_of(sim):
    coord, _ = sim
    return coord._toolbox()


def test_startup_turn_validates_and_requests_measurements(sim):
    tb = tb_of(sim)
    result = agents.run_scripted_turn(tb)
    assert result.errors == [], result.errors
    output = result.output
    methods = {r.method for r in output.measurement_requests}
    assert methods == {"lidar", "contact_current"}
    assert all(r.hypothesis_ids for r in output.measurement_requests)


def test_full_turnover_executes_a_request(sim):
    coord, sim = sim
    turn = coord.run_turn_now()
    assert turn.status == "accepted", turn.note
    w = sim.snapshot
    assert w.requests, "the accepted proposal must carry measurement requests"
    tasks = [t for t in w.tasks.values() if t.status in ("PENDING", "RESERVED", "MEASURING")]
    assert tasks, "compiled tasks must exist after acceptance"
    rp_text = sim.submit_route_pass("test turnover")
    assert "HELD" not in rp_text, rp_text
    w = sim.snapshot
    assert any(
        hasattr(d.state, "itinerary") for d in w.drones.values()
    ) or w.route_passes[-1].outcome != "accepted-plan"


def run_to_first_reading(sim, deadline_s: int = DEADLINE_S):
    """Advance the live simulation in steps until a reading publishes or time runs out."""
    for _ in range(200):
        if sim.snapshot.readings:
            return True
        if sim.snapshot.clock_s >= deadline_s:
            return False
        sim.step(min(90, (deadline_s - sim.snapshot.clock_s) / 60))
    return False


def test_contact_execution_publishes_reading_and_keeps_battery(sim):
    coord, sim = sim
    turn = coord.run_turn_now()
    assert turn.status == "accepted", turn.note
    rp_text = sim.submit_route_pass("test")
    assert "accepted-plan" in rp_text, rp_text
    w = sim.snapshot
    contact_tasks = [
        t
        for t in w.tasks.values()
        if t.status == "RESERVED" and isinstance(t.acquisition, ContactCurrent)
    ]
    if not contact_tasks:
        pytest.skip("planner chose no contact task on this episode")
    task = contact_tasks[0]
    drone_id = task.assigned_drone_id
    assert drone_id, "the planner must have assigned the task"
    drone = w.drones[drone_id]
    start_units = drone.battery.remaining_units
    assert run_to_first_reading(sim), "the accepted plan must execute a contact task"
    w = sim.snapshot
    drone = w.drones[drone_id]
    assert drone.battery.remaining_units <= start_units, "a landing must never add energy"
    assert w.readings and any(r.task_id == task.id for r in w.readings.values())
    assert w.tasks[task.id].status == "COMPLETED"
    assert isinstance(w.drones[drone_id].state, Perched)


def test_measurement_turn_revises_from_readings(sim):
    coord, sim = sim
    turn = coord.run_turn_now()
    assert turn.status == "accepted", turn.note
    sim.submit_route_pass("turnover")
    if not run_to_first_reading(sim):
        pytest.skip("no task completed before the deadline on this episode")
    w = sim.snapshot
    executed = [
        t for t in w.tasks.values() if t.status == "COMPLETED" and t.reading_id in w.readings
    ]
    assert executed
    tb = coord._toolbox()
    triggers = tuple(i for i, e in w.events if e.kind == "reading")
    result = agents.run_scripted_turn(tb, trigger_ids=triggers)
    assert result.errors == [], result.errors
    revised = {r.hypothesis_id for r in result.output.revisions}
    assert revised & {"H_SUPPLY", "H_VEG"}, "the executed readings must revise a claim"
    for rev in result.output.revisions:
        if rev.hypothesis_id == "H_SUPPLY":
            text = rev.finding_summary + rev.remaining_uncertainty
            assert "compatible with both" in text or "contradicted" in text


def test_scripted_mode_labelled_without_credentials(monkeypatch):
    monkeypatch.delenv("OPENROUTER_API_KEY", raising=False)
    assert not agents.llm_available()


def test_battery_continuity_across_two_turnovers(sim):
    coord, sim = sim
    coord.run_turn_now()
    sim.submit_route_pass("first")
    first_units = next(iter(sim.snapshot.drones.values())).battery.remaining_units
    for _ in range(60):
        if sim.snapshot.clock_s >= DEADLINE_S:
            break
        sim.step(min(90, (DEADLINE_S - sim.snapshot.clock_s) / 60))
    second_units = next(iter(sim.snapshot.drones.values())).battery.remaining_units
    assert second_units <= first_units, "a landing must never add energy"
