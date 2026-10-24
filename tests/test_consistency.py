"""Ticket 08: consistency diagnostics, discrepancy envelope and radius limits."""

from __future__ import annotations

import json

import numpy as np
import pytest
from conftest import permissive
from test_estimation import make_reading, synthetic_truth

from grid_ops import estimation
from grid_ops.assessment import Verdict
from grid_ops.circuits import build_registry
from grid_ops.consistency import (
    GlobalStatus,
    Testability,
    ac_check,
    check_consistency,
)
from grid_ops.envelope import ENVELOPE_PATH, ZERO_ENVELOPE, load_envelope
from grid_ops.estimation import ElectricalModel, estimate, observability
from grid_ops.geometry import LinePoint
from grid_ops.grid import load_network
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import (
    CommitSorties,
    ExcludeReading,
    ReassignEpoch,
    ReinstateReading,
    Simulation,
)
from grid_ops.world import CommandRejected, Measurement


@pytest.fixture(scope="module")
def network():
    return load_network()


@pytest.fixture(scope="module")
def registry(network):
    return build_registry(network)


@pytest.fixture(scope="module")
def nominal(network, registry):
    return ElectricalModel(network, registry)


def all_readings(model, truth, bias=None):
    bias = bias or {}
    return [
        make_reading(model, s, truth[s] + bias.get(s, 0.0), f"R{i:02d}")
        for i, s in enumerate(model.active_segments())
    ]


# -- what can be tested --------------------------------------------------------------------------------


def test_prior_only_has_nothing_to_test(nominal):
    rep = check_consistency(nominal, [], envelope=ZERO_ENVELOPE)
    assert rep.global_status is GlobalStatus.NOT_TESTABLE and rep.degrees_of_freedom == 0
    assert rep.suspect_reading_ids == () and "NOT TESTED" in rep.suspect_meaning


def test_minimally_observable_data_cannot_pass_a_bad_data_check(nominal):
    _, truth = synthetic_truth(nominal)
    chosen = observability(nominal).selected_segments
    readings = [make_reading(nominal, s, truth[s], f"R{i:02d}") for i, s in enumerate(chosen)]
    # one grossly wrong reading fits exactly when the data are minimally observable
    bad = make_reading(nominal, chosen[0], truth[chosen[0]] + 80.0, "R00")
    rep = check_consistency(nominal, [bad, *readings[1:]], envelope=ZERO_ENVELOPE)
    assert rep.degrees_of_freedom == 0
    assert rep.global_status is GlobalStatus.NOT_TESTABLE
    assert all(c.testability is Testability.NOT_TESTABLE for c in rep.checks)
    assert rep.suspect_reading_ids == () and "NOT TESTED" in rep.suspect_meaning  # not 'passed'


def test_redundancy_in_another_component_does_not_rescue_a_critical_reading(nominal):
    _, truth = synthetic_truth(nominal)
    chosen = observability(nominal).selected_segments
    comp01 = [
        s
        for s in chosen
        if nominal.component_names[nominal.comp_of[nominal._ends(s)[0]]] == "COMPONENT_01"
    ]
    other = next(
        s
        for s in nominal.active_segments()
        if nominal.component_names[nominal.comp_of[nominal._ends(s)[0]]] == "COMPONENT_02"
    )
    readings = [make_reading(nominal, s, truth[s], f"R{i:02d}") for i, s in enumerate(comp01)]
    bad_id = readings[0].id
    readings[0] = make_reading(nominal, comp01[0], truth[comp01[0]] + 60.0, bad_id)
    # redundancy elsewhere: the same COMPONENT_02 line measured twice
    readings += [
        make_reading(nominal, other, truth[other], "X1"),
        make_reading(nominal, other, truth[other], "X2"),
    ]
    rep = check_consistency(nominal, readings, envelope=ZERO_ENVELOPE)
    assert rep.degrees_of_freedom == 1
    assert rep.component_dof["COMPONENT_01"] == 0 and rep.component_dof["COMPONENT_02"] == 1
    bad = next(c for c in rep.checks if c.reading_id == bad_id)
    assert bad.testability is Testability.NOT_TESTABLE and not bad.candidate
    assert bad_id not in rep.suspect_reading_ids  # an empty list here is not a passed check
    assert ("X1", "X2") in rep.critical_sets  # the pair cannot localise an error between them


def test_redundant_data_flag_a_testable_outlier_without_excluding_it(nominal):
    _, truth = synthetic_truth(nominal)
    ok = all_readings(nominal, truth)
    rep0 = check_consistency(nominal, ok, envelope=ZERO_ENVELOPE)
    assert rep0.degrees_of_freedom == 36 and rep0.global_status is GlobalStatus.NOT_REFUTED
    assert rep0.suspect_reading_ids == ()
    testable = [c for c in rep0.checks if c.testability is Testability.TESTABLE]
    assert testable
    target = max(testable, key=lambda c: c.residual_variance_ratio)
    bad = all_readings(nominal, truth, bias={target.segment_id: 40.0})
    rep = check_consistency(nominal, bad, envelope=ZERO_ENVELOPE)
    assert rep.global_status is GlobalStatus.INCONSISTENT and rep.global_p_value < 0.001
    assert target.reading_id in rep.suspect_reading_ids
    flagged = next(c for c in rep.checks if c.reading_id == target.reading_id)
    assert abs(flagged.normalised_residual) > 3.0
    labels = {e.label for e in rep.explanations}
    assert {"wrong_topology", "sensor_bias", "parameter_mismatch"} <= labels  # none is chosen
    assert not hasattr(rep, "excluded")  # diagnostics never drop data by themselves


def test_shared_calibration_bias_is_invisible_to_the_residual_test(nominal):
    _, truth = synthetic_truth(nominal)
    seg = observability(nominal).selected_segments[0]
    rep = check_consistency(
        nominal,
        [
            make_reading(nominal, seg, truth[seg] + 15.0, "A", rho=0.5),
            make_reading(nominal, seg, truth[seg] + 15.0, "B", rho=0.5),
        ],
        shared_rho=0.5,
        envelope=ZERO_ENVELOPE,
    )
    # two repeats carrying the same bias agree with each other: nothing to detect
    assert not rep.suspect_reading_ids
    assert rep.global_status is not GlobalStatus.INCONSISTENT


# -- uncertainty -----------------------------------------------------------------------------------------


def test_envelope_records_domain_counts_coverage_and_is_held_out():
    env = load_envelope()
    assert env is not None and env.version == "discrepancy-v1"
    raw = json.loads(ENVELOPE_PATH.read_text())
    assert "all circuits in service" in raw["domain"]
    assert not set(env.calibration_seeds) & set(env.validation_seeds)
    assert min(env.calibration_seeds) > 50  # disjoint from scenario seeds used for operation
    b = env.bounds["LINE_026"]
    assert b.sample_count == len(env.calibration_seeds) and 0 <= b.empirical_coverage <= 1
    # an active seed that overlaps calibration data is not held out: the bound is not offered
    ok, why = env.supports("LINE_026", frozenset(), "E0", active_seed=env.calibration_seeds[0])
    assert not ok and "not held out" in why


def test_radius_is_conditional_plus_discrepancy_only_where_supported(network, registry):
    model = ElectricalModel(network, registry)
    env = load_envelope()
    _, truth = synthetic_truth(model)
    reading = make_reading(model, "LINE_026", truth["LINE_026"], "R1")
    est = estimate(model, [reading], clock_s=0, envelope=env, active_seed=7)
    f = est.flows["LINE_026"]
    assert f.discrepancy_bound_mw == env.bounds["LINE_026"].bound_mw
    assert f.uncertainty_radius_95_mw == pytest.approx(
        1.96 * f.conditional_std_mw + f.discrepancy_bound_mw
    )
    unsupported = [s for s, why in est.radius_unavailable.items()]
    assert unsupported and all(est.flows[s].uncertainty_radius_95_mw is None for s in unsupported)
    assert set(unsupported) == {s for s, b in env.bounds.items() if b.empirical_coverage < 0.8}
    # conditional std and the envelope are separate numbers: more data shrinks only the first
    many = [make_reading(model, "LINE_026", truth["LINE_026"], f"R{i}") for i in range(8)]
    est2 = estimate(model, many, clock_s=0, envelope=env, active_seed=7)
    g = est2.flows["LINE_026"]
    assert g.conditional_std_mw < f.conditional_std_mw
    assert g.discrepancy_bound_mw == f.discrepancy_bound_mw
    assert g.uncertainty_radius_95_mw >= g.discrepancy_bound_mw  # the floor stays


def test_repeated_correlated_readings_keep_the_systematic_floor(network, registry):
    model = ElectricalModel(network, registry)
    env = load_envelope()
    _, truth = synthetic_truth(model)
    reps = [make_reading(model, "LINE_026", truth["LINE_026"], f"R{i}", rho=0.5) for i in range(20)]
    est = estimate(model, reps, clock_s=0, shared_rho=0.5, envelope=env, active_seed=7)
    f = est.flows["LINE_026"]
    # 20 repeats cannot average below the shared calibration component sqrt(rho)*sigma_P
    assert f.conditional_std_mw > np.sqrt(0.5) * 2.0 * 0.95


def test_changed_topology_leaves_radius_unavailable(network, registry):
    model = ElectricalModel(network, registry, frozenset({"LINE_024"}))
    est = estimate(model, [], clock_s=0, envelope=load_envelope(), active_seed=7)
    assert all(f.uncertainty_radius_95_mw is None for f in est.flows.values())
    assert "topology" in next(iter(est.radius_unavailable.values()))


def test_unmeasured_ac_checks_stay_not_supported(network):
    sim = permissive(Simulation(network))
    for name in ("far_end_voltage_limit", "bus_voltage_band", "transformer_loading"):
        v, why = ac_check(sim.snapshot, name, "LINE_026/C1")
        assert v is Verdict.NOT_SUPPORTED and why
    v, _ = ac_check(sim.snapshot, "terminal_current_within_rating", "LINE_026/C1")
    assert v is Verdict.NOT_SUPPORTED  # no reading yet
    perch = sim._world.drones["DRONE_3"].state.perch
    stop = StopRequest(LinePoint("LINE_026", 3_000_000), Measurement.CURRENT)
    sim._apply(CommitSorties((SortieRequest("DRONE_3", (stop,), perch),), "operator"))
    sim.step(12)
    v, why = ac_check(sim.snapshot, "terminal_current_within_rating", "LINE_026/C1")
    assert v in (Verdict.PASS, Verdict.INDETERMINATE, Verdict.FAIL) and "R0001" in why


# -- live simulation: AC readings against the DC model ---------------------------------------------------


def two_current_readings(sim):
    for drone, seg in (("DRONE_3", "LINE_026"), ("DRONE_2", "LINE_064")):
        perch = sim._world.drones[drone].state.perch
        stop = StopRequest(
            LinePoint(seg, 4_000_000 if seg == "LINE_026" else 9_000_000), Measurement.CURRENT
        )
        sim._apply(CommitSorties((SortieRequest(drone, (stop,), perch),), "operator"))
    sim.step(15)


def test_incident_readings_are_model_limited_not_faulty(network):
    sim = permissive(Simulation(network))
    for drone, mm in (("DRONE_3", 4_000_000), ("DRONE_2", 5_000_000)):  # same line, two drones
        perch = sim._world.drones[drone].state.perch
        stop = StopRequest(LinePoint("LINE_026", mm), Measurement.CURRENT)
        sim._apply(CommitSorties((SortieRequest(drone, (stop,), perch),), "operator"))
    sim.step(20)
    rep = sim.snapshot.consistency
    assert len(rep.checks) == 2 and rep.degrees_of_freedom == 1
    assert all(c.testability is Testability.MODEL_LIMITED for c in rep.checks)
    assert "topology" in rep.checks[0].note  # LINE_024 switching leaves the calibration domain
    assert rep.global_status is GlobalStatus.MODEL_LIMITED
    assert not rep.suspect_reading_ids and not sim.snapshot.excluded_readings  # nothing dropped
    assert {e.label for e in rep.explanations} >= {"wrong_topology", "sensor_bias"}


def test_two_independent_readings_alone_cannot_be_tested(network):
    sim = permissive(Simulation(network))
    two_current_readings(sim)
    rep = sim.snapshot.consistency
    assert rep.global_status is GlobalStatus.NOT_TESTABLE and rep.degrees_of_freedom == 0
    assert all(c.testability is Testability.NOT_TESTABLE for c in rep.checks)


def test_healthy_mode_ac_readings_get_supported_limits(network):
    sim = permissive(
        Simulation(network, scenario=ScenarioConfig(ScenarioMode.HEALTHY_BENCHMARK, 7))
    )
    two_current_readings(sim)
    est = sim.snapshot.estimate
    assert est.envelope_version == "discrepancy-v1"
    f = est.flows["LINE_026"]
    supported = f.uncertainty_radius_95_mw is not None
    assert supported == ("LINE_026" not in est.radius_unavailable)
    rep = sim.snapshot.consistency
    assert rep.degrees_of_freedom == 0 and rep.global_status is GlobalStatus.NOT_TESTABLE


# -- dispositions -----------------------------------------------------------------------------------------


def test_exclusion_reinstatement_and_epoch_reassignment_are_recorded_and_recompute(network):
    sim = permissive(Simulation(network))
    two_current_readings(sim)
    w = sim._world
    ids = sorted(w.readings)
    before = w.estimation_solves
    original = {k: v for k, v in w.readings.items()}
    full = sim.snapshot.estimate.flows["LINE_026"].p_mw
    sim._apply(ExcludeReading(ids[0], "operator doubts the calibration", "operator"))
    assert w.estimation_solves == before + 1 and ids[0] in w.excluded_readings
    assert estimation.effective_readings(w)[0].id == ids[1] and w.estimate.accepted_reading_ids == (
        ids[1],
    )
    sim._apply(ReinstateReading(ids[0], "calibration confirmed", "operator"))
    assert w.estimation_solves == before + 2 and not w.excluded_readings
    assert w.estimate.flows["LINE_026"].p_mw == pytest.approx(full)
    sim._apply(ReassignEpoch(ids[1], "E1", "switching reported after this reading", "operator"))
    assert w.estimate.accepted_reading_ids == (ids[0],)  # not pooled into the E0 fit
    assert w.readings == original  # raw readings are untouched
    actions = [d.action for d in w.dispositions]
    assert actions == ["EXCLUDED", "REINSTATED", "EPOCH_REASSIGNED"]
    assert w.dispositions[2].from_epoch == "E0" and w.dispositions[2].to_epoch == "E1"
    assert all(d.reason for d in w.dispositions)


def test_dispositions_validate_inputs(network):
    sim = permissive(Simulation(network))
    two_current_readings(sim)
    rid = sorted(sim._world.readings)[0]
    with pytest.raises(CommandRejected, match="reason"):
        sim._apply(ExcludeReading(rid, " ", "operator"))
    with pytest.raises(CommandRejected, match="Unknown reading"):
        sim._apply(ExcludeReading("R9999", "x", "operator"))
    with pytest.raises(CommandRejected, match="not excluded"):
        sim._apply(ReinstateReading(rid, "x", "operator"))
    sim._apply(ExcludeReading(rid, "x", "operator"))
    with pytest.raises(CommandRejected, match="already excluded"):
        sim._apply(ExcludeReading(rid, "x", "operator"))
    with pytest.raises(CommandRejected, match="different epoch"):
        sim._apply(ReassignEpoch(rid, "E0", "x", "operator"))
