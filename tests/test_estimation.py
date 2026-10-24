"""Ticket 07: public reduced-DC estimates and observability."""

from __future__ import annotations

import numpy as np
import pytest
from conftest import permissive

from grid_ops import estimation
from grid_ops.circuits import build_registry
from grid_ops.estimation import (
    ElectricalModel,
    EstimateUnavailable,
    GridEstimate,
    _p_covariance,
    estimate,
    observability,
)
from grid_ops.geometry import LinePoint
from grid_ops.grid import load_network
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import SensorModel
from grid_ops.sim import CommitSorties, Simulation
from grid_ops.world import Measurement, Reading


@pytest.fixture(scope="module")
def network():
    return load_network()


@pytest.fixture(scope="module")
def registry(network):
    return build_registry(network)


@pytest.fixture(scope="module")
def nominal(network, registry):
    return ElectricalModel(network, registry)


# -- synthetic truth in the model's own world ----------------------------------------


def synthetic_truth(model, seed=1):
    """Random injections at prior nodes summing to zero per component, exact zero injection."""
    rng = np.random.default_rng(seed)
    inj = np.zeros(len(model.nodes))
    for nodes in model.components:
        free = [i for i in nodes if i in model.prior_nodes]
        if len(free) < 2:
            continue
        v = rng.normal(0, 25, len(free))
        v -= v.mean()
        inj[free] = v
    theta, *_ = np.linalg.lstsq(model.B, inj, rcond=None)
    assert np.allclose(model.B @ theta, inj, atol=1e-6)
    flows = {s: float(model.flow_rows[s] @ theta) for s in model.segments}
    return inj, flows


def make_reading(model, segment, flow_mw, rid, rho=0.0, noise=0.0, clock=0):
    circuits = model.registry.by_segment[segment]
    active = [c for c in circuits]
    total_b = sum(c.b_mw_per_rad for c in active)
    payload = tuple(
        (f"{c.id}.{q}", flow_mw * c.b_mw_per_rad / total_b + noise if q == "p_mw" else 50.0)
        for c in active
        for q in ("current_a", "p_mw", "q_mvar", "voltage_kv")
    )
    cov = SensorModel(shared_rho=rho).covariance("current", tuple(active))
    return Reading(
        rid,
        "T",
        "s",
        "D",
        LinePoint(segment, 0),
        Measurement.CURRENT,
        clock,
        "E0",
        payload,
        "v",
        "",
        cov,
    )


# -- the nominal fixture ----------------------------------------


def test_nominal_fixture_reproduces_declared_ranks(nominal):
    obs = observability(nominal)
    assert obs.n_unknowns == 52 and obs.components == 7
    assert obs.n_zero_injection == 24 and obs.zero_injection_rank == 24
    assert obs.total_rank == 52
    assert obs.independent_readings == 28  # the algebraic minimum, not a diagnosis requirement
    assert obs.per_component_minimum == {
        "COMPONENT_01": 20,
        "COMPONENT_02": 2,
        "COMPONENT_03": 2,
        "COMPONENT_04": 1,
        "COMPONENT_05": 1,
        "COMPONENT_06": 1,
        "COMPONENT_07": 1,
    }
    assert len(obs.selected_segments) == 28
    # no flow is pinned by the exact zero-injection rows alone
    assert obs.observable_segments == ()


def test_graph_counts_behind_the_fixture(network, nominal):
    assert len(network.nodes) == 59 and len(network.segments) == 64
    types = [n.type for n in network.nodes.values()]
    assert types.count("junction") == 20 and types.count("intermediate") == 4
    assert types.count("abstract_substation") == 15 and types.count("boundary_endpoint") == 19
    cycles = len(nominal.active_segments()) - len(nominal.nodes) + len(nominal.components)
    assert cycles == 12


def test_selected_independent_set_reaches_full_rank(nominal):
    obs = observability(nominal)
    fake = [make_reading(nominal, s, 0.0, f"R{i}") for i, s in enumerate(obs.selected_segments)]
    full = observability(nominal, fake)
    assert len(full.observable_segments) == len(nominal.active_segments())  # all flows pinned
    # repeated rows add no structural rank
    repeated = fake + [
        make_reading(nominal, s, 0.0, f"X{i}") for i, s in enumerate(obs.selected_segments)
    ]
    assert observability(nominal, repeated).observable_segments == full.observable_segments


def test_changed_topology_recalculates_references_and_observability(network, registry, nominal):
    cut = ElectricalModel(network, registry, frozenset({"LINE_024"}))
    n, c = observability(nominal), observability(cut)
    assert len(cut.active_segments()) == 63
    assert c.components + 0 == len(cut.components)
    assert c.n_unknowns == len(cut.nodes) - c.components  # references rebuilt per component
    assert c.independent_readings == c.total_rank - c.zero_injection_rank
    assert (c.components, c.n_unknowns) != (n.components, n.n_unknowns) or (
        c.selected_segments != n.selected_segments
    )
    assert "LINE_024" not in c.selected_segments
    assert cut.flow_rows["LINE_024"].any() == False  # noqa: E712 - zero through-flow


# -- estimates ----------------------------------------


def test_startup_estimate_is_prior_only_and_labelled(nominal):
    est = estimate(nominal, [], clock_s=0)
    assert isinstance(est, GridEstimate) and est.prior_only
    assert est.observable_segments == () and est.accepted_reading_ids == ()
    assert all(not f.observable for f in est.flows.values())
    assert all(f.conditional_std_mw > 0 for f in est.flows.values())
    assert all(f.uncertainty_radius_95_mw is None for f in est.flows.values())  # ticket 08
    assert any("pseudo-measurements" in a for a in est.assumptions)


def test_matching_model_noiseless_recovery_with_compatible_priors(nominal):
    inj, truth = synthetic_truth(nominal)
    obs = observability(nominal)
    readings = [
        make_reading(nominal, s, truth[s], f"R{i}") for i, s in enumerate(obs.selected_segments)
    ]
    # compatible priors (the true injections) => zero residual, exact recovery of every flow
    est = estimate(nominal, readings, clock_s=0, prior_means=inj[list(nominal.prior_nodes)])
    assert isinstance(est, GridEstimate) and not est.prior_only
    for s in nominal.segments:
        assert est.flows[s].p_mw == pytest.approx(truth[s], abs=1e-6), s
    for s in obs.selected_segments:
        assert est.flows[s].observable


def test_mismatched_priors_are_a_bias_case(nominal):
    _, truth = synthetic_truth(nominal)
    obs = observability(nominal)
    readings = [
        make_reading(nominal, s, truth[s], f"R{i}") for i, s in enumerate(obs.selected_segments)
    ]
    est = estimate(nominal, readings, clock_s=0)  # default -30/0 MW priors disagree with truth
    errors = [abs(est.flows[s].p_mw - truth[s]) for s in nominal.segments]
    assert max(errors) > 0.05  # finite-weight priors bias even a noiseless real-data fit
    assert any("pseudo-measurements" in a for a in est.assumptions)


def test_measured_flows_follow_the_readings_and_uncertainty_shrinks(nominal):
    _, truth = synthetic_truth(nominal)
    seg = "LINE_026"
    prior = estimate(nominal, [], clock_s=0).flows[seg].conditional_std_mw
    est = estimate(nominal, [make_reading(nominal, seg, truth[seg], "R1")], clock_s=0)
    assert est.flows[seg].p_mw == pytest.approx(truth[seg], abs=3 * 2.0)
    assert est.flows[seg].conditional_std_mw < prior
    assert est.accepted_reading_ids == ("R1",)
    assert est.flows[seg].conditional_std_mw <= 2.0  # aggregate sigma of one reading, 2 MW


def test_repeated_readings_cannot_remove_a_shared_calibration_component(nominal):
    seg = "LINE_026"
    two = [
        make_reading(nominal, seg, -50.0, "R1", rho=0.5),
        make_reading(nominal, seg, -50.0, "R2", rho=0.5),
    ]
    cov = _p_covariance(
        [
            estimation._Row("R1", seg, -50.0, ("LINE_026/C1.p_mw",)),
            estimation._Row("R2", seg, -50.0, ("LINE_026/C1.p_mw",)),
        ],
        {r.id: r for r in two},
        0.5,
    )
    assert cov[0, 1] == pytest.approx(0.5 * 2.0 * 2.0) and cov[0, 0] == pytest.approx(4.0)
    shared = estimate(nominal, two, clock_s=0, shared_rho=0.5).flows[seg].conditional_std_mw
    plain = [make_reading(nominal, seg, -50.0, "R1"), make_reading(nominal, seg, -50.0, "R2")]
    indep = estimate(nominal, plain, clock_s=0).flows[seg].conditional_std_mw
    assert shared > indep  # the shared component stays: averaging helps less


def test_unavailable_when_inputs_are_invalid_or_singular(nominal):
    bad_cov = make_reading(nominal, "LINE_026", -50.0, "R1")
    zero = Reading(
        **{
            **bad_cov.__dict__,
            "covariance": type(bad_cov.covariance)(
                bad_cov.covariance.names,
                tuple(
                    tuple(0.0 for _ in bad_cov.covariance.names) for _ in bad_cov.covariance.names
                ),
            ),
        }
    )
    out = estimate(nominal, [zero], clock_s=0)
    assert isinstance(out, EstimateUnavailable) and "positive definite" in out.reason
    nan = make_reading(nominal, "LINE_026", float("nan"), "R2")
    skipped = estimate(nominal, [nan], clock_s=0)
    assert isinstance(skipped, GridEstimate) and skipped.diagnostics and skipped.prior_only
    # no priors and no readings leaves a singular information matrix: say so, invent no bars
    nominal_no_priors = ElectricalModel(nominal.network, nominal.registry)
    nominal_no_priors.prior_nodes = ()
    nominal_no_priors.prior_mean = np.zeros(0)
    single = estimate(nominal_no_priors, [], clock_s=0)
    assert isinstance(single, EstimateUnavailable) and "singular" in single.reason


def test_readings_from_another_epoch_are_not_pooled(nominal):
    r = make_reading(nominal, "LINE_026", -40.0, "R1")
    other = Reading(**{**r.__dict__, "operating_epoch": "E1"})
    assert estimate(nominal, [other], clock_s=0).accepted_reading_ids == ()


# -- through the real simulation ----------------------------------------


def fly_current(sim, drone="DRONE_3", seg="LINE_026", km=3.0):
    perch = sim._world.drones[drone].state.perch
    stop = StopRequest(LinePoint(seg, round(km * 1e6)), Measurement.CURRENT)
    sim._apply(CommitSorties((SortieRequest(drone, (stop,), perch),), "operator"))
    sim.step(12)


def test_sim_publishes_prior_only_then_updates_with_the_first_current_reading(network):
    sim = permissive(Simulation(network))
    start = sim.snapshot
    assert isinstance(start.estimate, GridEstimate) and start.estimate.prior_only
    assert start.estimation_solves == 1
    # LINE_024 is a known switching report: its through-flow is zero by construction
    assert start.estimate.flows["LINE_024"].p_mw == 0.0
    assert not start.estimate.flows["LINE_024"].observable
    fly_current(sim)
    w = sim.snapshot
    assert not w.estimate.prior_only and w.estimation_solves == 2
    reading = next(iter(w.readings.values()))
    measured = sum(v for k, v in reading.payload if k.endswith("p_mw"))
    assert w.estimate.flows["LINE_026"].p_mw == pytest.approx(measured, abs=3.0)
    assert w.estimate.flows["LINE_026"].observable  # the measured row pins its own flow
    assert w.estimate.observable_segments == ("LINE_026",)  # and nothing else
    assert w.estimate.accepted_reading_ids == (reading.id,)


def test_unchanged_electrical_inputs_cause_no_solve(network):
    sim = permissive(Simulation(network))
    fly_current(sim)
    solves = sim.snapshot.estimation_solves
    sim.step(30)  # time passing alone changes no electrical input
    assert sim.snapshot.estimation_solves == solves
    # a local reading adds evidence of another kind, still no electrical solve
    perch = sim._world.drones["DRONE_2"].state.perch
    stop = StopRequest(LinePoint("LINE_058", 20_000), Measurement.CLEARANCE)
    sim._apply(CommitSorties((SortieRequest("DRONE_2", (stop,), perch),), "operator"))
    sim.step(8)
    assert len(sim.snapshot.readings) == 2
    assert sim.snapshot.estimation_solves == solves
    assert not estimation.refresh(
        sim._estimation, sim._world, sim.network, sim.circuits
    )  # explicit refresh with unchanged inputs does nothing


def test_deenergization_alone_does_not_locate_an_open_switch(network):
    from grid_ops import knowledge

    sim = permissive(Simulation(network))
    before = sim._estimation.model.topology_key
    # a qualified DEAD voltage reading (upper 95% bound below 2% of nominal) on LINE_041
    cov = sim._world.sensor_model.covariance(
        "voltage_presence", sim.circuits.by_segment["LINE_041"]
    )
    reading = Reading(
        "R0099",
        "T",
        "s",
        "D",
        LinePoint("LINE_041", 100_000),
        Measurement.VOLTAGE_PRESENCE,
        0,
        "E0",
        (("LINE_041/C1.voltage_kv", 0.0),),
        "v",
        "",
        cov,
    )
    sim._world.readings[reading.id] = reading
    knowledge.ingest_reading(sim._world, reading)
    assert sim._world.knowledge.circuits["LINE_041/C1"].energization == "de_energized"
    assert sim._world.knowledge.chargeability["LINE_041/C1"].state == "unavailable"
    sim._refresh_estimate()
    assert sim._estimation.model.topology_key == before  # no silent branch deletion
    assert sim._world.estimate.flows["LINE_041"].p_mw != 0.0  # still modelled as carrying flow


def test_known_switching_report_rebuilds_the_model(network):
    from grid_ops.scenario import ScenarioConfig, ScenarioMode

    cfg = ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 7, contradictory_switching=True)
    sim = Simulation(network, scenario=cfg)
    solves = sim.snapshot.estimation_solves
    sim.step(21)  # the scripted public switching report arrives at T+00:20
    w = sim.snapshot
    assert w.estimation_solves == solves + 1
    assert w.estimate.flows["LINE_009"].p_mw == 0.0
    assert any("2 known out-of-service" in line for line in w.estimation_log)
