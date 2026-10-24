"""Ticket 04: seeded incident and healthy scenarios, hidden AC physics and observations."""

from __future__ import annotations

import dataclasses
import math

import numpy as np
import pytest

from grid_ops.circuits import build_registry
from grid_ops.geometry import Geometry, LandingSite, LinePoint
from grid_ops.grid import load_network
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import (
    I_CHARGE_MIN_A,
    CircuitSensorConfig,
    ScenarioConfig,
    ScenarioInfeasible,
    ScenarioMode,
    SensorModel,
    public_brief,
)
from grid_ops.sim import CONTRADICTED_SEGMENT, CommitSorties, Simulation
from grid_ops.truth import build
from grid_ops.truth.build import HEALTHY_LIMITS, INCIDENT_LIMITS, choose_sources
from grid_ops.world import Measurement, observation_identity

START = tuple(f"{s}/C1" for s in ["LINE_040", "LINE_025", "LINE_026", "LINE_002", "LINE_056"])


@pytest.fixture(scope="module")
def network():
    return load_network()


@pytest.fixture(scope="module")
def registry(network):
    return build_registry(network)


@pytest.fixture(scope="module")
def geometry(network):
    return Geometry(network)


@pytest.fixture(scope="module")
def incident(network, registry, geometry):
    return build(network, registry, geometry, ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 7), START)


@pytest.fixture(scope="module")
def healthy(network, registry, geometry):
    return build(
        network, registry, geometry, ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 7), START
    )


# -- briefs ----------------------------------------------------------------------------------


def test_incident_brief_keeps_reports_and_precautionary_switching(network):
    text = "\n".join(public_brief(network, ScenarioConfig(ScenarioMode.INCIDENT_DEMO)))
    for i in range(1, 7):
        assert f"SYN_REPORT_0{i}" in text
    assert "precautionarily switched" in text and "LINE_024" in text
    assert "not a confirmed trip" in text
    assert "reconstruct circuit flows" not in text


def test_healthy_brief_is_counterfactual_and_selects_reconstruction(network):
    text = "\n".join(public_brief(network, ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK)))
    assert "every circuit is initially in service" in text
    assert "unused synthetic background" in text
    assert "reconstruct circuit flows" in text
    assert "precautionarily switched" not in text


# -- AC truth --------------------------------------------------------------------------------


def test_sources_follow_declared_selection(network):
    sources = choose_sources(network)
    assert sources["COMPONENT_07"] == "SUBSTATION_03"
    assert len(sources) == 7
    for comp, node_id in sources.items():
        node = network.nodes[node_id]
        assert node.component == comp
        if comp != "COMPONENT_07":
            assert node.type == "boundary_endpoint"


def test_ac_buses_transformers_and_circuits_are_explicit(incident, registry):
    net = incident.net
    assert len(net.line) == len(registry.circuits) == 105
    assert len(net.trafo) == 6  # six nodes mix 380 and 110 kV
    assert (net.trafo.shift_degree == 0).all()
    assert len(net.ext_grid) == 7  # one declared source per initial component, no hidden slack
    names = set(net.line.name)
    assert names == set(registry.circuits)


def test_incident_state_islands_are_explicit_not_hidden_slacks(incident):
    assert not incident.circuit_states["LINE_024/C1"].energized
    assert incident.unsupplied_buses == frozenset() or all(
        not incident.circuit_states[c].energized
        for c in incident.circuit_states
        if int(incident.net.line.at[incident._line[c], "from_bus"]) in incident.unsupplied_buses
    )
    assert len(incident.net.ext_grid) == 7


def test_islanding_switch_deenergizes_without_replacement_source(network, registry, geometry):
    t = build(network, registry, geometry, ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 7), START)
    ext_before = len(t.net.ext_grid)
    # LINE_033 is the only line of COMPONENT_07 (SUBSTATION_03 -> terminal): open it
    assert t.circuit_states["LINE_033/C1"].energized
    t.set_in_service("LINE_033", False)
    assert not t.circuit_states["LINE_033/C1"].energized
    assert len(t.net.ext_grid) == ext_before
    assert t.epoch == 1


def test_all_energized_results_finite_and_within_declared_limits(incident, healthy):
    for truth, lim in ((incident, INCIDENT_LIMITS), (healthy, HEALTHY_LIMITS)):
        for cid, s in truth.circuit_states.items():
            if not s.energized:
                continue
            vals = [s.p_from_mw, s.q_from_mvar, s.p_to_mw, s.q_to_mvar, s.i_from_a, s.i_to_a]
            assert all(math.isfinite(v) for v in vals), cid
            assert s.loading_pct <= lim.max_loading_pct, cid
            nominal = truth.registry.circuits[cid].nominal_kv
            for v in (s.v_from_kv, s.v_to_kv):
                assert lim.v_min_pu * nominal <= v <= lim.v_max_pu * nominal, cid
    assert all(s.energized for s in healthy.circuit_states.values())
    assert healthy.initial_problems == []


def test_terminal_quantities_use_from_bus_into_line_convention(incident):
    for s in incident.circuit_states.values():
        if s.energized and abs(s.p_from_mw) > 1:
            # lossless-ish line: what enters at one end leaves at the other (opposite signs)
            assert s.p_from_mw + s.p_to_mw == pytest.approx(s.loss_mw, abs=1e-6)
            assert s.loss_mw >= -1e-9
            assert s.p_from_mw * s.p_to_mw < 0 or abs(s.p_from_mw) < 1


def test_starting_circuits_can_charge(incident, healthy):
    for truth in (incident, healthy):
        for cid in START:
            site = LandingSite(LinePoint(cid.split("/")[0], 0), cid)
            assert truth.is_chargeable(site)
            assert truth.circuit_states[cid].i_from_a >= I_CHARGE_MIN_A


def test_deenergized_or_low_current_circuits_cannot_charge(incident):
    site = LandingSite(LinePoint("LINE_024", 0), "LINE_024/C1")
    assert not incident.is_chargeable(site)
    quiet = [
        c for c, s in incident.circuit_states.items() if s.energized and s.i_from_a < I_CHARGE_MIN_A
    ]
    for cid in quiet:
        assert not incident.is_chargeable(LandingSite(LinePoint(cid.split("/")[0], 0), cid))


def test_construction_is_bounded_and_reports_infeasibility(
    network, registry, geometry, monkeypatch
):
    import importlib

    tb = importlib.import_module("grid_ops.truth.build")

    monkeypatch.setattr(tb, "MAX_OPERATING_POINTS", 4)
    with pytest.raises(ScenarioInfeasible) as err:
        build(
            network,
            registry,
            geometry,
            ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 7),
            ("LINE_024/C1",),  # switched off in the incident: can never charge
        )
    d = err.value.diagnostics
    assert d["attempts"] == 4 and d["failure_kinds"] and d["last_problems"]
    assert "LINE_024/C1" in " ".join(d["last_problems"])


def test_deterministic_construction(network, registry, geometry, incident):
    again = build(network, registry, geometry, ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 7), START)
    assert again.operating_point == incident.operating_point
    assert again.circuit_states == incident.circuit_states
    other = build(network, registry, geometry, ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 8), START)
    assert other.circuit_states != incident.circuit_states


# -- sensing ---------------------------------------------------------------------------------


def test_electrical_payload_is_circuit_resolved(incident):
    sample = incident.sense(
        LinePoint("LINE_029", 500_000), Measurement.CURRENT, 0, "electrical:LINE_029", 0
    )
    names = [n for n, _ in sample.values]
    assert names == [
        f"LINE_029/C{k}.{q}" for k in (1, 2) for q in ("current_a", "p_mw", "q_mvar", "voltage_kv")
    ]
    assert dict(sample.values)["LINE_029/C1.voltage_kv"] > 80
    v = incident.sense(
        LinePoint("LINE_029", 0), Measurement.VOLTAGE_PRESENCE, 0, "electrical:LINE_029", 0
    )
    assert [n for n, _ in v.values] == ["LINE_029/C1.voltage_kv", "LINE_029/C2.voltage_kv"]


def test_public_covariance_follows_declared_sigmas(registry):
    model = SensorModel()
    cov = model.covariance("current", registry.by_segment["LINE_029"])
    assert cov.std("LINE_029/C1.current_a") == pytest.approx(1.0)
    assert cov.std("LINE_029/C1.p_mw") == pytest.approx(2.0)
    assert cov.std("LINE_029/C1.q_mvar") == pytest.approx(2.0)
    assert cov.std("LINE_029/C1.voltage_kv") == pytest.approx(1.1)  # 1% of 110 kV
    assert np.allclose(cov.as_array(), np.diag(np.diag(cov.as_array())))  # independent base case
    big = model.covariance("voltage_presence", registry.by_segment["LINE_001"])
    assert big.std("LINE_001/C1.voltage_kv") == pytest.approx(3.8)  # 1% of 380 kV
    refs = CircuitSensorConfig(i_ref_a=1000.0)
    assert refs.sigma("current_a", 110) == 10.0
    shared = SensorModel(shared_rho=0.5).covariance("current", registry.by_segment["LINE_029"])
    assert shared.matrix[0][4] == pytest.approx(0.5 * 1.0 * 1.0)  # C1/C2 current share bias
    assert shared.matrix[0][1] == 0.0  # different quantities stay independent


def test_noise_is_keyed_by_observation_not_by_order(incident):
    pt = LinePoint("LINE_029", 100)
    ident = "electrical:LINE_029"
    a0 = incident.sense(pt, Measurement.CURRENT, 5, ident, 0)
    a1 = incident.sense(pt, Measurement.CURRENT, 9, ident, 1)
    # same key, any time or call order: same realisation
    assert incident.sense(pt, Measurement.CURRENT, 999, ident, 1) == dataclasses.replace(a1)
    assert incident.sense(pt, Measurement.CURRENT, 1, ident, 0).values == a0.values
    assert a0.values != a1.values  # a repeat is a fresh draw
    # another offset on the same segment is the same canonical observation
    assert observation_identity(Measurement.CURRENT, pt) == observation_identity(
        Measurement.CURRENT, LinePoint("LINE_029", 3_000_000)
    )
    # a different seed changes the draw
    saved = incident.config
    incident.config = dataclasses.replace(saved, seed=99)
    try:
        assert incident.sense(pt, Measurement.CURRENT, 5, ident, 0).values != a0.values
    finally:
        incident.config = saved


def test_noise_matches_declared_sigma(incident):
    pt = LinePoint("LINE_029", 100)
    samples = np.array(
        [
            dict(incident.sense(pt, Measurement.CURRENT, 0, "electrical:LINE_029", r).values)[
                "LINE_029/C1.current_a"
            ]
            for r in range(2000)
        ]
    )
    truth_i = incident.circuit_states["LINE_029/C1"].i_from_a
    assert samples.mean() == pytest.approx(truth_i, abs=0.1)
    assert samples.std() == pytest.approx(1.0, abs=0.07)


def test_shared_calibration_correlation_survives_repeats(incident):
    pt = LinePoint("LINE_029", 100)
    saved = incident.config
    rho = 0.5
    incident.config = dataclasses.replace(saved, shared_rho=rho)
    truth_i = incident.circuit_states["LINE_029/C1"].i_from_a
    errs0, errs1 = [], []
    try:
        for seed in range(600):
            incident.config = dataclasses.replace(saved, shared_rho=rho, seed=seed)
            for r, bucket in ((0, errs0), (1, errs1)):
                v = dict(
                    incident.sense(pt, Measurement.CURRENT, 0, "electrical:LINE_029", r).values
                )
                bucket.append(v["LINE_029/C1.current_a"] - truth_i)
    finally:
        incident.config = saved
    corr = np.corrcoef(errs0, errs1)[0, 1]
    assert corr == pytest.approx(rho, abs=0.1)  # repeating does not remove the shared component
    assert np.std(errs0) == pytest.approx(1.0, abs=0.1)  # total std still the declared one


# -- local fixtures --------------------------------------------------------------------------


def sense(truth, kind, segment, offset_mm, t=0, repeat=0):
    pt = LinePoint(segment, offset_mm)
    return truth.sense(pt, kind, t, observation_identity(kind, pt), repeat)


def test_vegetation_fixture_on_line_058_vs_clear_interval(incident):
    veg = dict(sense(incident, Measurement.CLEARANCE, "LINE_058", 20_000).values)["clearance_m"]
    assert veg == pytest.approx(0.4, abs=0.2)
    clear = dict(sense(incident, Measurement.CLEARANCE, "LINE_064", 9_000_000).values)[
        "clearance_m"
    ]
    assert clear == pytest.approx(2.0, abs=0.25)  # qualified clean evidence


def test_thermal_fixture_needs_excitation(incident):
    joint = incident.local.anomalies[1]
    assert joint.kind == "hot_joint"
    mid = (joint.interval.start_mm + joint.interval.end_mm) // 2
    low = sense(incident, Measurement.CONDUCTOR_TEMP, "LINE_029", mid, t=5 * 60)
    assert dict(low.conditions)["excited"] == 0.0  # low-load window: inconclusive
    assert dict(low.values)["temp_c"] < 25
    loaded = sense(incident, Measurement.CONDUCTOR_TEMP, "LINE_029", mid, t=40 * 60)
    assert dict(loaded.conditions)["excited"] == 1.0
    assert dict(loaded.values)["temp_c"] > dict(low.values)["temp_c"] + 10
    # a clean spot under adequate load: qualified negative
    away = sense(incident, Measurement.CONDUCTOR_TEMP, "LINE_029", 200_000, t=40 * 60)
    assert dict(away.conditions)["excited"] == 1.0
    assert dict(away.values)["temp_c"] < dict(loaded.values)["temp_c"] - 10


def test_motion_fixture_only_during_gust_windows(incident):
    anomaly = incident.local.anomalies[2]
    mid = (anomaly.interval.start_mm + anomaly.interval.end_mm) // 2
    gust = sense(incident, Measurement.MOTION, "LINE_054", mid, t=30 * 60)
    assert dict(gust.conditions)["qualifying_gust"] == 1.0
    assert dict(gust.values)["amplitude_m"] == pytest.approx(0.30, abs=0.1)
    ref = sense(incident, Measurement.MOTION, "LINE_054", 100_000, t=30 * 60)
    assert dict(ref.values)["amplitude_m"] == pytest.approx(0.05, abs=0.1)
    calm = sense(incident, Measurement.MOTION, "LINE_054", mid, t=90 * 60)
    assert dict(calm.conditions)["qualifying_gust"] == 0.0
    assert dict(calm.values)["amplitude_m"] < 0.1  # no gust: nothing to see either way


def test_occluded_interval_is_not_inspected(incident):
    s = sense(incident, Measurement.CLEARANCE, "LINE_064", 8_300_000)
    assert dict(s.conditions)["visible_fraction"] == 0.0
    assert s.values == () and s.inspected == ()
    partial = sense(incident, Measurement.CLEARANCE, "LINE_064", 9_000_000)
    assert 0 < dict(partial.conditions)["visible_fraction"] < 1
    assert all(a >= 8_800_000 for a, _ in partial.inspected)


def test_healthy_mode_has_no_local_defects(healthy):
    assert healthy.local.anomalies == []
    c = dict(sense(healthy, Measurement.CLEARANCE, "LINE_058", 20_000).values)["clearance_m"]
    assert c > 1.5


# -- through the real simulation -------------------------------------------------------------


def test_sim_selects_scenario_and_publishes_only_public_observations(network):
    sim = Simulation(network, scenario=ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 3))
    snap = sim.snapshot
    assert snap.scenario_mode is ScenarioMode.HEALTHY_BENCHMARK and snap.scenario_seed == 3
    assert "counterfactual" in " ".join(snap.brief).lower()
    assert snap.known_switching == {}
    inc = Simulation(network)
    assert inc.snapshot.known_switching == {"LINE_024": 0}


def test_same_seed_reproduces_readings_and_other_seed_differs(network):
    def run(seed):
        sim = Simulation(network, scenario=ScenarioConfig(ScenarioMode.INCIDENT_DEMO, seed))
        req = SortieRequest(
            "DRONE_3",
            (StopRequest(LinePoint("LINE_026", 3_000_000), Measurement.CURRENT),),
            LandingSite(LinePoint("LINE_026", 6_000_000), "LINE_026/C1"),
        )
        sim._apply(CommitSorties((req,), "operator"))
        sim.step(30)
        return sim.snapshot.readings["R0001"]

    a, b, c = run(7), run(7), run(8)
    assert a.payload == b.payload and a.payload != c.payload
    assert a.covariance is not None and a.covariance.names[0] == "LINE_026/C1.current_a"
    assert a.sensor_model_revision == "synthetic-v0" or a.sensor_model_revision


def test_contradictory_switching_report_versus_live_voltage(network):
    sim = Simulation(
        network,
        scenario=ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 7, contradictory_switching=True),
    )
    assert CONTRADICTED_SEGMENT not in sim.snapshot.known_switching
    events = sim.step(21)
    assert [e.kind for e in events] == ["switching_report"]
    assert sim.snapshot.known_switching[CONTRADICTED_SEGMENT] == 20 * 60
    # hidden truth still has it live: a qualified voltage reading contradicts the public report
    s = sense(sim._truth, Measurement.VOLTAGE_PRESENCE, CONTRADICTED_SEGMENT, 100)
    assert all(v > 50 for _, v in s.values)
    assert sim._truth.circuit_states[f"{CONTRADICTED_SEGMENT}/C1"].energized


def test_hidden_values_stay_out_of_public_world(network):
    sim = Simulation(network)
    sim.step(5)
    blob = repr(sim.snapshot)
    for secret in (
        "operating_point",
        "hot_joint",
        "Anomaly",
        "LocalField",
        "physical_epoch",
        "net=",
    ):
        assert secret not in blob
    assert "LINE_058" not in " ".join(e.text for e in sim.snapshot.log)


def test_public_fixture_seed_is_policy_only():
    from dataclasses import asdict

    from grid_ops.scenario import PublicFixture

    fixture = asdict(PublicFixture(policy_seed=73))
    assert fixture == {"fixture_id": "bavaria-public", "version": "1", "policy_seed": 73}
