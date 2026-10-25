"""The console's pure views must render every drone phase and every three-method payload.

Direct state construction: no simulation, no hidden physics. Electrical quantities (voltage,
P/Q, MW), power direction and optical-imaging claims (RGB/IR/temperature) must never appear in
rendered rows: the three methods do not produce them.
"""

from __future__ import annotations

import re
from dataclasses import replace

from fixtures_three_method import tiny_network

from grid_ops.assessment import LidarCoverage
from grid_ops.energy import BatteryState
from grid_ops.geometry import Geometry, Interval, LandingSite, LinePoint
from grid_ops.map_view import build_deck
from grid_ops.sensing import (
    AcceptedRequest,
    AcquisitionQuality,
    ConductorPoint,
    ConductorScope,
    ContactCurrent,
    CorridorSpan,
    CurrentPayload,
    CurrentRequest,
    LidarPass,
    LidarPayload,
    LidarRequest,
    MeasurementTaskV2,
    Reading,
    RequestStatus,
    ScanPose,
    ScanTrajectory,
    TaskStatusV2,
    TdrEcho,
    TdrPayload,
    TdrTest,
)
from grid_ops.views import (
    coverage_rows,
    drone_rows,
    itinerary_rows,
    reading_rows,
    request_rows,
    route_rows,
    score_rows,
    task_rows,
)
from grid_ops.world import (
    Drone,
    Executing,
    GroundSample,
    Itinerary,
    Land,
    LineStatus,
    Perched,
    Scan,
    ScheduledStep,
    Stranded,
    Transit,
    World,
)

FORBIDDEN = re.compile(
    r"\b(voltage|mw|megawatt|rgb|infrared|ir|temperature|conductor_temp)\b|p_mw|q_mw|"
    r"power direction",
    re.IGNORECASE,
)

A, B = (11.0, 48.0), (11.01, 48.0)
SITE_B = LandingSite(LinePoint("LINE_025", 55_000), "LINE_025/C1")
LIDAR_ACQ = LidarPass(
    Interval("LINE_040", 0, 600_000),
    ScanTrajectory((ScanPose(A, 50.0, 0.0), ScanPose(B, 50.0, 0.0)), 60),
)
CONTACT_ACQ = ContactCurrent(ConductorPoint(LinePoint("LINE_025", 55_000), "LINE_025/C1"))


def _request(rid: str, method: str, status=RequestStatus.ACTIVE) -> AcceptedRequest:
    common = {
        "id": rid,
        "hypothesis_ids": ("H_VEG",),
        "rationale": "distinguish vegetation contact from a switching error before repair",
        "alternatives_distinguished": ("vegetation contact", "switching error"),
        "expected_outcomes": (),
        "evidence_ids": ("SYN_REPORT_02",),
        "urgency": 3,
        "source_revision": "r0",
        "replaces_request_ids": (),
    }
    if method == "lidar":
        req = LidarRequest(
            target=CorridorSpan(segment_id="LINE_040", start_mm=0, end_mm=600_000), **common
        )
    else:
        req = CurrentRequest(
            target=ConductorScope(circuit_id="LINE_025/C1", phase="A", start_mm=0, end_mm=600_000),
            **common,
        )
    return AcceptedRequest(rid, req, status, 0, "r0")


def _itinerary(t: int = 0) -> Itinerary:
    steps = (
        ScheduledStep(0, t, t + 60, Transit(A, B)),
        ScheduledStep(1, t + 60, t + 120, Scan(LIDAR_ACQ, "T0001")),
        ScheduledStep(2, t + 120, t + 150, GroundSample((CONTACT_ACQ,), SITE_B, ("T0002",), B)),
        ScheduledStep(3, t + 150, t + 180, Land(SITE_B, B)),
    )
    return Itinerary(steps, SITE_B)


def _world(**kw) -> World:
    network = tiny_network()
    batteries = kw.pop("batteries", {})
    drones = {
        "DRONE_1": Drone(
            "DRONE_1",
            (11.005, 48.0),
            BatteryState(remaining_units=batteries.get("DRONE_1", 900_000)),
            Executing(_itinerary(), 1),
            history=("T0000",),
        ),
        "DRONE_2": Drone(
            "DRONE_2",
            A,
            BatteryState(remaining_units=batteries.get("DRONE_2", 1_080_000)),
            Perched(SITE_B),
        ),
        "DRONE_3": Drone(
            "DRONE_3",
            A,
            BatteryState(remaining_units=batteries.get("DRONE_3", 300_000)),
            Stranded(SITE_B),
        ),
    }
    return World(
        clock_s=0, drones=drones, belief=dict.fromkeys(network.segments, LineStatus.UNKNOWN), **kw
    )


def _reading(rid: str, acquisition, payload, task_id="T0001", drone="DRONE_1") -> Reading:
    return Reading(
        id=rid,
        task_id=task_id,
        request_ids=("REQ-1",),
        drone_id=drone,
        acquisition=acquisition,
        acquired_start_s=0,
        acquired_end_s=60,
        payload=payload,
        quality=AcquisitionQuality(True),
    )


def _row_text(rows) -> str:
    return " ".join(str(v) for row in rows for v in row.values())


# -- drones ---------------------------------------------------------------------------------------


def test_drone_rows_render_all_three_phases():
    rows = drone_rows(_world())
    assert {r["phase"] for r in rows} == {"executing", "perched", "stranded"}
    by_id = {r["drone"]: r for r in rows}
    assert "charging" not in _row_text(rows).lower()
    executing = by_id["DRONE_1"]
    assert "lidar scan T0001" in executing["operation"]
    assert executing["itinerary_left"] == 3
    assert executing["step_end"] == "T+00:02"
    assert executing["tasks_done"] == 1
    assert by_id["DRONE_3"]["operation"] == f"stranded {SITE_B.circuit_id}"
    assert by_id["DRONE_2"]["operation"] == f"perched {SITE_B.circuit_id}"


def test_drone_rows_show_battery_units_and_percent_of_capacity():
    rows = {r["drone"]: r for r in drone_rows(_world())}
    assert rows["DRONE_1"]["battery_units"] == 900_000
    assert rows["DRONE_1"]["battery_pct"] == round(100 * 900_000 / 1_080_000, 1)
    assert rows["DRONE_3"]["battery_pct"] == round(100 * 300_000 / 1_080_000, 1)
    # battery never resets on landing: a drained drone keeps its drained level
    later = {r["drone"]: r for r in drone_rows(_world(batteries={"DRONE_2": 700_000}))}
    assert later["DRONE_2"]["battery_units"] == 700_000


# -- routes ---------------------------------------------------------------------------------------


def test_route_rows_render_itinerary_steps_and_final_perch():
    rows = route_rows(_world())
    assert len(rows) == 1
    row = rows[0]
    assert row["drone"] == "DRONE_1"
    assert "lidar scan T0001" in row["steps"]
    assert "ground sample T0002" in row["steps"]
    assert SITE_B.circuit_id in row["final_perch"]
    assert row["locked"].startswith("step 2/4")
    assert itinerary_rows(_world(), "DRONE_1")[1]["operation"] == "lidar scan T0001 @ LINE_040"
    assert itinerary_rows(_world(), "DRONE_2") == []


# -- tasks and requests ---------------------------------------------------------------------------


def _world_with_tasks() -> World:
    w = _world()
    w.tasks = {
        "T0001": MeasurementTaskV2(
            "T0001",
            "REQ-1",
            LIDAR_ACQ,
            "REQ-1",
            TaskStatusV2.MEASURING,
            3,
            assigned_drone_id="DRONE_1",
            note="compiled from request REQ-1",
        ),
        "T0002": MeasurementTaskV2(
            "T0002",
            "REQ-2",
            CONTACT_ACQ,
            "REQ-2",
            TaskStatusV2.PENDING,
            4,
            note="compiled from request REQ-2",
        ),
    }
    w.requests = {"REQ-1": _request("REQ-1", "lidar"), "REQ-2": _request("REQ-2", "current")}
    return w


def test_task_rows_render_method_request_status_and_owner():
    by_id = {r["task"]: r for r in task_rows(_world_with_tasks())}
    assert by_id["T0001"]["method"] == "lidar"
    assert by_id["T0002"]["method"] == "contact_current"
    assert by_id["T0001"]["request"] == "REQ-1"
    assert by_id["T0001"]["status"] == "MEASURING"
    assert by_id["T0001"]["drone"] == "DRONE_1"
    assert by_id["T0002"]["urgency"] == 4


def test_request_rows_flag_active_requests_without_a_completed_valid_attempt():
    rows = {r["request"]: r for r in request_rows(_world_with_tasks())}
    assert rows["REQ-1"]["method"] == "lidar"
    assert "span 0.000-0.600 km" in rows["REQ-1"]["target"]
    assert "phase A" in rows["REQ-2"]["target"]
    assert rows["REQ-2"]["unresolved"] == "EXPLICITLY UNRESOLVED"
    assert rows["REQ-2"]["tasks"] == "T0002"
    assert rows["REQ-1"]["rationale"].startswith("distinguish vegetation")

    # a completed valid attempt in the request's alternative group resolves it
    w = _world_with_tasks()
    w.set_task(replace(w.tasks["T0001"], status=TaskStatusV2.COMPLETED, reading_id="R0001"))
    w.readings["R0001"] = _reading("R0001", LIDAR_ACQ, _lidar_payload())
    rows = {r["request"]: r for r in request_rows(w)}
    assert rows["REQ-1"]["unresolved"] == ""

    # a retired request keeps its status and never claims to be unresolved
    w2 = _world_with_tasks()
    w2.requests["REQ-2"] = _request("REQ-2", "current", RequestStatus.RETIRED)
    rows = {r["request"]: r for r in request_rows(w2)}
    assert rows["REQ-2"]["status"] == "RETIRED"
    assert rows["REQ-2"]["unresolved"] == ""


# -- readings -------------------------------------------------------------------------------------


def _lidar_payload() -> LidarPayload:
    return LidarPayload(
        inspected=(Interval("LINE_040", 0, 500_000),),
        missing=(Interval("LINE_040", 500_000, 600_000),),
        clearance_m=0.42,
        uncertainty_m=0.05,
        resolution_mm=10_000,
    )


def _readings_world() -> World:
    w = _world()
    tdr = TdrPayload(
        echoes=(TdrEcho(1e-6, 0.8, "negative", 5e-8),),
        propagation_mps=2e8,
        propagation_uncertainty_mps=1e7,
        usable_range_m=50_000.0,
        coupling_quality="good",
    )
    current = CurrentPayload(
        conductor=ConductorPoint(LinePoint("LINE_025", 55_000), "LINE_025/C1"),
        rms_a=80.948,
        uncertainty_a=1.0,
        integration_s=10,
    )
    w.readings = {
        "R0001": _reading("R0001", LIDAR_ACQ, _lidar_payload()),
        "R0002": _reading(
            "R0002", TdrTest("access:x", CONTACT_ACQ.conductor, "return:x"), tdr, "T0003"
        ),
        "R0003": _reading("R0003", CONTACT_ACQ, current, "T0002"),
    }
    return w


def test_reading_rows_render_payload_specific_columns():
    rows = {r["reading"]: r for r in reading_rows(_readings_world())}
    lidar, tdr, current = rows["R0001"], rows["R0002"], rows["R0003"]
    assert lidar["method"] == "lidar"
    assert lidar["inspected_mm"] == 500_000
    assert lidar["missing_mm"] == 100_000
    assert lidar["clearance_m"] == 0.42
    assert lidar["clearance_uncertainty_m"] == 0.05
    assert tdr["method"] == "tdr"
    assert tdr["echoes"] == 1
    assert tdr["coupling"] == "good"
    assert tdr["usable_range_m"] == 50_000.0
    assert current["method"] == "contact_current"
    assert current["rms_a"] == 80.948
    assert current["uncertainty_a"] == 1.0
    assert current["conductor"] == "LINE_025/C1/A@0.055km"
    assert current["integration_s"] == 10


def test_reading_rows_never_render_electrical_or_imaging_claims():
    text = _row_text(reading_rows(_readings_world()))
    hit = FORBIDDEN.search(text)
    assert hit is None, hit.group() if hit else ""


# -- other panels ---------------------------------------------------------------------------------


def test_empty_and_partial_panels_render_without_raising():
    w = _world_with_tasks()
    assert score_rows(w) == []  # no preview published on a bare world
    assert coverage_rows(w) == []
    w.score_preview = [
        {
            "task": "T0001",
            "tier_points_5_to_1": (0, 0, 500, 0, 0),
            "owner_urgency": 3,
            "notes": "req:REQ-1",
        }
    ]
    w.coverage = [
        LidarCoverage(
            "R0001",
            "LINE_040",
            Interval("LINE_040", 0, 600_000),
            (Interval("LINE_040", 0, 500_000),),
            (Interval("LINE_040", 500_000, 600_000),),
            "epoch",
        )
    ]
    score = score_rows(w)[0]
    assert score["owner_urgency"] == 3 and "500" in score["points_by_urgency_5_to_1"]
    cov = coverage_rows(w)[0]
    assert cov["inspected_km"] == 0.5 and cov["missing_km"] == 0.1


def test_map_renders_every_phase_without_raising():
    network = tiny_network()
    build_deck(network, _world(), Geometry(network), None)
    build_deck(network, _world(), None, None)


# -- the reconstruction views (P3) -----------------------------------------------------------------------------


def _reconstructed_world(zero_on="LINE_026"):
    """A real simulation's public world plus a published zero-current reading on one branch."""
    import copy

    from grid_ops.outage import assess_stop, reconstruct
    from grid_ops.sim import Simulation

    sim = Simulation(tiny_network())
    w = copy.deepcopy(sim.snapshot)
    point = ConductorPoint(LinePoint(zero_on, 400_000), f"{zero_on}/C1")
    w.readings["R0001"] = Reading(
        "R0001",
        "T0001",
        (),
        "DRONE_1",
        ContactCurrent(point),
        0,
        10,
        CurrentPayload(point, 0.0, 1.0, 10),
        AcquisitionQuality(True),
    )
    w.outage_belief = reconstruct(sim.outage_model, w.readings, ())
    w.outage_stop = assess_stop(sim.outage_model, w.outage_belief, best_eig_bits=1.0)
    return w, sim


def test_reconstruction_rows_keep_current_service_and_fault_records_separate():
    from grid_ops.views import alternative_rows, fact_rows, reconstruction_lines, region_rows

    w, _ = _reconstructed_world()
    kinds = {r["kind"] for r in fact_rows(w)}
    assert {"fault_segment", "service_state"} <= kinds
    service = next(r for r in fact_rows(w) if r["kind"] == "service_state")
    assert service["status"] == "not supported" and "never certifies" in service["statement"]
    assert region_rows(w)[0]["segment"] == "LINE_026"
    assert alternative_rows(w)
    text = " ".join(reconstruction_lines(w))
    assert "CONTINUE" in text and "Model limit" in text


def test_zero_current_never_paints_a_line_as_de_energized_on_the_map():
    from grid_ops.views import reconstruction_status
    from grid_ops.world import LineStatus

    w, _ = _reconstructed_world()
    status = reconstruction_status(w)
    assert status.get("LINE_026") is LineStatus.SUSPECTED
    assert LineStatus.DE_ENERGIZED not in status.values()
    deck = build_deck(tiny_network(), w, Geometry(tiny_network()), None)
    assert "(reconstruction)" in str(deck.to_json())


def test_reconstruction_views_carry_no_voltage_power_or_optical_claims():
    from grid_ops.views import alternative_rows, experiment_rows, fact_rows, region_rows

    w, _ = _reconstructed_world()
    blob = " ".join(
        str(v)
        for rows in (fact_rows(w), alternative_rows(w), region_rows(w), experiment_rows(w))
        for row in rows
        for v in (*row.keys(), *row.values())
    ).lower()
    for banned in ("voltage", " mw", "mvar", "rgb", "infrared", "temperature"):
        assert banned not in blob
