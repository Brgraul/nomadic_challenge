"""P3: the public outage reconstruction, experiment value and stopping assessment.

Readings come from a privileged hidden episode (as in the live runtime), but everything under
test sees only public inputs and immutable readings. The checks are on observable behaviour: what
the posterior supports, what it refuses to certify, and what stays unresolved.
"""

from __future__ import annotations

import ast
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from fixtures_three_method import tiny_network

from grid_ops import outage
from grid_ops.circuits import build_registry
from grid_ops.geometry import Geometry, Interval, LinePoint
from grid_ops.outage import (
    OperationalEvidence,
    OutageCaps,
    StopReason,
    assess_stop,
    build_model,
    reconstruct,
)
from grid_ops.sensing import (
    AcquisitionQuality,
    ConductorPoint,
    ContactCurrent,
    CurrentPayload,
    ExecutedAcquisition,
    LidarPass,
    LidarPayload,
    Reading,
    ScanPose,
    ScanTrajectory,
    TdrTest,
    tdr_access,
)
from grid_ops.truth.episode import EpisodeSpec, build_episode


class Lab:
    """A tiny public model plus one hidden episode that answers acquisitions."""

    def __init__(
        self, segment="LINE_026", fraction=0.6, seed=3, noise_seed=5, double=(), **cap_changes
    ):
        self.network = tiny_network()
        for seg_id in double:  # parallel circuits on these segments
            seg = self.network.segments[seg_id]
            self.network.segments[seg_id] = replace(seg, circuits=2)
        self.registry = build_registry(self.network)
        self.geometry = Geometry(self.network)
        self.model = build_model(self.network, self.registry, self.geometry)
        from grid_ops.sensing import SensorCapabilities

        self.capabilities = replace(SensorCapabilities(), **cap_changes)
        self.model.capabilities = self.capabilities
        self.episode = build_episode(
            self.network,
            self.registry,
            self.geometry,
            EpisodeSpec(seed, noise_seed, segment, fraction, "run"),
            capabilities=self.capabilities,
        )
        self.readings: dict[str, Reading] = {}
        self.segment = segment
        self.length = self.geometry.length_mm(segment)
        self.n = 0

    def take(self, acquisition, *, landed=True) -> Reading:
        self.n += 1
        ident = f"R{self.n:04d}"
        duration = (
            acquisition.trajectory.duration_s
            if isinstance(acquisition, LidarPass)
            else getattr(acquisition, "integration_s", None) or acquisition.duration_s
        )
        landed_at = (
            None if isinstance(acquisition, LidarPass) or not landed else acquisition.conductor
        )
        executed = ExecutedAcquisition(ident, acquisition, 0, duration, "DRONE_1", landed_at)
        sample = self.episode.acquire(executed)
        reading = Reading(
            ident, "T", (), "DRONE_1", acquisition, 0, duration, sample.payload, sample.quality
        )
        self.readings[ident] = reading
        return reading

    def current(self, circuit_id, fraction, segment=None) -> Reading:
        seg = segment or circuit_id.split("/")[0]
        x = round(self.geometry.length_mm(seg) * fraction)
        return self.take(ContactCurrent(ConductorPoint(LinePoint(seg, x), circuit_id)))

    def tdr(self, circuit_id, fraction) -> Reading:
        seg = circuit_id.split("/")[0]
        x = round(self.geometry.length_mm(seg) * fraction)
        cp = ConductorPoint(LinePoint(seg, x), circuit_id)
        access = tdr_access(cp)
        return self.take(TdrTest(access.id, cp, access.return_path_ids[0]))

    def scan(self, segment, lo, hi) -> Reading:
        pos = self.geometry.position_of
        poses = (
            ScanPose(pos(LinePoint(segment, lo)), 50.0, 0.0, -60.0),
            ScanPose(pos(LinePoint(segment, hi)), 50.0, 0.0, -60.0),
        )
        return self.take(LidarPass(Interval(segment, lo, hi), ScanTrajectory(poses, 30)))

    def forge_current(self, circuit_id, fraction, rms_a) -> Reading:
        """A published current reading with an exact value, for checks that need one."""
        seg = circuit_id.split("/")[0]
        x = round(self.geometry.length_mm(seg) * fraction)
        point = ConductorPoint(LinePoint(seg, x), circuit_id)
        self.n += 1
        ident = f"R{self.n:04d}"
        payload = CurrentPayload(point, rms_a, self.capabilities.current_sigma_a, 10)
        reading = Reading(
            ident,
            "T",
            (),
            "DRONE_1",
            ContactCurrent(point),
            0,
            10,
            payload,
            AcquisitionQuality(True, correlation_group=f"current:{circuit_id}:A"),
        )
        self.readings[ident] = reading
        return reading

    def belief(self, operational=()):
        return reconstruct(self.model, self.readings, operational)


def weights_of(belief):
    return {w.candidate_id: w.weight for w in belief.candidate_weights}


def fact(belief, kind):
    return next((f for f in belief.supported_facts if f.kind == kind), None)


# -- the public catalogue ------------------------------------------------------------------------


def test_the_catalogue_is_built_from_public_inputs_only():
    tree = ast.parse(Path(outage.__file__).read_text())
    imported = {
        node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom) and node.module
    }
    forbidden = {"grid_ops.truth", "grid_ops.truth.episode", "grid_ops.sim", "grid_ops.world"}
    assert not (imported & forbidden), imported & forbidden


def test_the_same_public_inputs_give_the_same_candidates_whatever_the_hidden_episode():
    a, b = (
        Lab(seed=1, noise_seed=2, segment="LINE_026"),
        Lab(seed=9, noise_seed=8, segment="LINE_002"),
    )
    assert a.model.candidates == b.model.candidates
    wa, wb = weights_of(a.belief()), weights_of(b.belief())
    assert wa == wb  # identical public evidence (none) gives identical beliefs


def test_the_prior_covers_every_declared_branch_with_a_live_unloaded_alternative():
    lab = Lab()
    kinds = {c.kind for c in lab.model.candidates}
    assert kinds == {"open_conductor", "live_unloaded"}
    assert set(lab.model.branches) == {"LINE_026", "LINE_002", "LINE_056"}
    belief = lab.belief()
    assert math.isclose(sum(weights_of(belief).values()) + belief.residual_weight, 1.0)
    assert belief.residual_weight == pytest.approx(outage.RESIDUAL_MASS)
    assert not any(f.supported for f in belief.supported_facts)


# -- an identifiable fault -----------------------------------------------------------------------


def test_a_tiny_open_conductor_case_is_supported_within_the_declared_resolution():
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.current("LINE_026/C1", 0.5)
    lab.tdr("LINE_026/C1", 0.35)
    lab.tdr("LINE_026/C1", 0.75)
    belief = lab.belief()
    interval = fact(belief, "fault_interval")
    assert interval is not None and interval.supported, interval
    truth = lab.episode.fault.offset_mm
    assert interval.interval.start_mm <= truth <= interval.interval.end_mm
    assert interval.width_mm <= 2 * (lab.length * 0.6) / lab.model.cells_per_branch + 1
    assert interval.evidence_ids and set(interval.evidence_ids) <= set(lab.readings)
    stop = assess_stop(lab.model, belief, best_eig_bits=1.0)
    assert stop.reason is StopReason.SUPPORTED
    assert "declared model" in stop.detail


def test_supported_facts_cite_only_executed_readings():
    lab = Lab()
    lab.current("LINE_026/C1", 0.5)
    lab.tdr("LINE_026/C1", 0.35)
    belief = lab.belief()
    assert set(belief.evidence_ids) == set(lab.readings)
    for f in belief.supported_facts:
        assert set(f.evidence_ids) <= set(lab.readings)


# -- zero current ------------------------------------------------------------------------------


def test_zero_current_leaves_live_unloaded_and_disconnected_alternatives_open():
    lab = Lab(segment="LINE_026")
    lab.current("LINE_026/C1", 0.5)
    belief = lab.belief()
    weights = weights_of(belief)
    unloaded = weights["U:LINE_026"]
    tripped = sum(w for k, w in weights.items() if k.startswith("F:LINE_026"))
    assert unloaded > 0.02 and tripped > unloaded
    service = fact(belief, "service_state")
    assert service is not None and not service.supported
    assert "never certifies" in service.statement
    assert assess_stop(lab.model, belief, best_eig_bits=1.0).reason is StopReason.CONTINUE
    other = fact(belief, "fault_segment")
    assert other.subject == "LINE_026" and not other.supported


def test_disconnected_is_not_certified_from_current_alone_even_with_a_strong_prior():
    lab = Lab(segment="LINE_026")
    for fraction in (0.2, 0.5, 0.8, 0.9):
        lab.current("LINE_026/C1", fraction)
    service = fact(lab.belief(), "service_state")
    assert not service.supported


def test_a_nonzero_reading_is_direct_observation_of_a_live_conductor():
    lab = Lab(segment="LINE_026")
    reading = lab.current("LINE_002/C1", 0.5)
    assert reading.payload.rms_a > outage.ENERGIZED_MIN_A
    live = [f for f in lab.belief().supported_facts if f.kind == "observed_current"]
    assert live and live[0].probability == 1.0 and live[0].evidence_ids == (reading.id,)


# -- TDR branches ------------------------------------------------------------------------------


def test_tdr_from_the_disconnected_side_localizes_but_the_protective_device_masks_beyond_it():
    lab = Lab(segment="LINE_026", fraction=0.6)
    protector = lab.model.branches["LINE_026"].protector_mm
    upstream = lab.tdr("LINE_026/C1", 0.02)
    assert protector > round(lab.length * 0.02)
    # from the supply side the open protective device reflects first; the fault is not visible
    assert len(upstream.payload.echoes) >= 1
    beliefs = []
    for fraction in (0.4, 0.7):
        other = Lab(segment="LINE_026", fraction=fraction)
        other.tdr("LINE_026/C1", 0.02)
        beliefs.append(weights_of(other.belief()))
    cells = [k for k in beliefs[0] if k.startswith("F:LINE_026/C1")]
    # the supply-side test cannot tell the two fault positions apart
    assert all(
        math.isclose(beliefs[0][k], beliefs[1][k], rel_tol=1e-6, abs_tol=1e-9) for k in cells
    )


def test_a_downstream_test_separates_fault_cells_that_the_upstream_test_cannot():
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.tdr("LINE_026/C1", 0.02)
    before = weights_of(lab.belief())
    lab.tdr("LINE_026/C1", 0.3)
    after = weights_of(lab.belief())
    cells = sorted(k for k in after if k.startswith("F:LINE_026/C1"))
    assert max(after[k] for k in cells) > 2 * max(before[k] for k in cells)


def test_parallel_circuits_stay_compatible_until_a_test_tells_them_apart():
    lab = Lab(segment="LINE_002", fraction=0.6, double=("LINE_002",))
    circuits = [c.id for c in lab.registry.by_segment["LINE_002"]]
    assert len(circuits) == 2
    weights = weights_of(lab.belief())
    masses = {c: sum(w for k, w in weights.items() if k.startswith(f"F:{c}:")) for c in circuits}
    assert masses[circuits[0]] == pytest.approx(masses[circuits[1]])
    lab.current("LINE_002/C1", 0.5)
    lab.tdr(circuits[0], 0.35)
    lab.tdr(circuits[1], 0.35)
    after = weights_of(lab.belief())
    faulted = sum(w for k, w in after.items() if k.startswith(f"F:{circuits[0]}:"))
    healthy = sum(w for k, w in after.items() if k.startswith(f"F:{circuits[1]}:"))
    assert faulted > 10 * healthy


# -- LiDAR, hazards and confounders -------------------------------------------------------------


def test_a_hazard_alone_never_concludes_outage_causation():
    lab = Lab(segment="LINE_026", fraction=0.6)
    reading = lab.scan("LINE_026", round(lab.length * 0.55), round(lab.length * 0.65))
    assert reading.quality.valid and reading.payload.clearance_m < outage.HAZARD_CLEARANCE_M
    belief = lab.belief()
    interval = fact(belief, "fault_interval")
    assert interval is None or not interval.supported
    assert assess_stop(lab.model, belief, best_eig_bits=1.0).reason is not StopReason.SUPPORTED


def test_an_unrelated_hazard_shifts_belief_only_by_its_likelihood_ratio():
    lab = Lab(segment="LINE_026", fraction=0.6)
    prior = weights_of(lab.belief())
    lab.scan(
        "LINE_002",
        round(lab.geometry.length_mm("LINE_002") * 0.4),
        round(lab.geometry.length_mm("LINE_002") * 0.6),
    )
    after = weights_of(lab.belief())
    assert after["F:LINE_026/C1:0"] == pytest.approx(prior["F:LINE_026/C1:0"], rel=0.5)


def test_an_uninspected_interval_is_never_a_clean_negative():
    lab = Lab(segment="LINE_026", fraction=0.6)
    reading = lab.take(
        LidarPass(
            Interval("LINE_026", 0, lab.length),
            ScanTrajectory(
                (
                    ScanPose(lab.geometry.position_of(LinePoint("LINE_026", 0)), 50.0, 0.0),
                    ScanPose(
                        lab.geometry.position_of(LinePoint("LINE_026", lab.length)), 50.0, 0.0
                    ),
                ),
                30,
            ),
        )
    )
    blind = replace(reading, quality=AcquisitionQuality(False, "no view"))
    blind = replace(
        blind, payload=LidarPayload((), (Interval("LINE_026", 0, lab.length),), None, 0.05, 10_000)
    )
    lab.readings = {blind.id: blind}
    assert weights_of(lab.belief()) == weights_of(Lab().belief())


# -- repeats, ordering and idempotence ---------------------------------------------------------


def test_correlated_current_repeats_do_not_grow_confidence_like_independent_points():
    repeated, distinct = Lab(segment="LINE_026"), Lab(segment="LINE_026")
    for _ in range(6):
        repeated.forge_current("LINE_002/C1", 0.5, 90.0)
    for fraction in (0.1, 0.25, 0.4, 0.55, 0.7, 0.85):
        distinct.forge_current("LINE_002/C1", fraction, 90.0)

    def entropy(weights):
        total = sum(weights)
        return -sum(w / total * math.log(w / total) for w in weights if w > 0)

    # the load regime is shared: six points at one place carry one observation's worth of it
    assert entropy(repeated.belief().regime_weights) > entropy(distinct.belief().regime_weights)
    group = outage._group_obs(repeated.model, repeated.readings).currents
    rho = repeated.capabilities.systematic_fraction
    assert len(group) == 1
    assert group[0]["sigma"] >= math.sqrt(rho) * repeated.capabilities.current_sigma_a


def test_repeated_negative_scans_add_no_independent_evidence():
    lab = Lab(segment="LINE_026", fraction=0.3)
    lo, hi = round(lab.length * 0.55), round(lab.length * 0.75)
    first = lab.scan("LINE_026", lo, hi)
    assert (
        first.payload.clearance_m is not None
        and first.payload.clearance_m >= outage.HAZARD_CLEARANCE_M
    )
    once = weights_of(lab.belief())
    lab.scan("LINE_026", lo, hi)
    lab.scan("LINE_026", lo, hi)
    thrice = weights_of(lab.belief())
    assert once == pytest.approx(thrice)


def test_ingestion_is_idempotent_and_order_independent():
    lab = Lab()
    lab.current("LINE_026/C1", 0.5)
    lab.tdr("LINE_026/C1", 0.35)
    lab.scan("LINE_026", round(lab.length * 0.5), round(lab.length * 0.7))
    forward = lab.belief()
    shuffled = dict(reversed(list(lab.readings.items())))
    again = reconstruct(lab.model, shuffled, ())
    twice = reconstruct(lab.model, {**lab.readings, **lab.readings}, ())
    assert forward.digest == again.digest == twice.digest
    assert weights_of(forward) == weights_of(again) == weights_of(twice)


def test_the_posterior_is_a_pure_function_of_the_evidence_set():
    lab = Lab()
    lab.current("LINE_026/C1", 0.5)
    first = weights_of(lab.belief())
    lab.model._echo_cache.clear()
    assert weights_of(lab.belief()) == first


# -- operational evidence ---------------------------------------------------------------------


def test_a_switching_report_shifts_belief_to_that_branch_without_certifying_it():
    lab = Lab()
    base = weights_of(lab.belief())
    report = OperationalEvidence("SW:LINE_002", "switching_report", "LINE_002", clock_s=600)
    after = weights_of(lab.belief([report]))
    before_mass = sum(w for k, w in base.items() if k.startswith("F:LINE_002"))
    after_mass = sum(w for k, w in after.items() if k.startswith("F:LINE_002"))
    assert after_mass > 2 * before_mass
    assert not fact(lab.belief([report]), "fault_interval")


def test_charge_availability_is_reported_separately_from_the_fault():
    lab = Lab()
    ok = OperationalEvidence("EV1", "handshake_ok", "LINE_040", "LINE_040/C1", 0)
    failed = OperationalEvidence("EV2", "handshake_failed", "LINE_026", "LINE_026/C1", 10)
    belief = lab.belief([ok, failed])
    charge = [f for f in belief.supported_facts if f.kind == "charge_availability"]
    assert {f.subject for f in charge} == {"LINE_040/C1", "LINE_026/C1"}
    assert weights_of(belief) == weights_of(lab.belief())  # a handshake never moves fault belief


# -- model mismatch --------------------------------------------------------------------------


def test_two_faults_outside_the_family_refuse_a_supported_claim():
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.forge_current("LINE_026/C1", 0.5, 0.0)  # a trip on LINE_026
    lab.forge_current("LINE_002/C1", 0.5, 0.0)  # and a second, simultaneous trip
    lab.forge_current("LINE_056/C1", 0.5, 0.0)  # and a third
    belief = lab.belief()
    assert belief.model_checks.mismatch
    assert belief.residual_weight > 0.5
    assert not any(f.supported for f in belief.supported_facts if f.kind != "observed_current")
    stop = assess_stop(lab.model, belief, best_eig_bits=2.0)
    assert stop.reason is StopReason.MODEL_MISMATCHED


def test_the_residual_has_a_stated_limited_power():
    lab = Lab()
    lab.current("LINE_026/C1", 0.5)
    notes = " ".join(lab.belief().model_checks.notes)
    assert "limited power" in notes


# -- truncation and budgets ------------------------------------------------------------------


def test_a_truncated_pool_never_gives_an_unconditional_claim():
    lab = Lab(segment="LINE_026", fraction=0.6)
    lab.model = build_model(
        lab.network, lab.registry, lab.geometry, caps=OutageCaps(max_candidates=4)
    )
    assert lab.model.truncated and lab.model.omitted_segments
    lab.current("LINE_026/C1", 0.5)
    lab.tdr("LINE_026/C1", 0.3)
    lab.tdr("LINE_026/C1", 0.8)
    belief = lab.belief()
    assert belief.truncated
    assert any("truncated" in line for line in belief.limits)
    stop = assess_stop(lab.model, belief, best_eig_bits=0.0)
    assert stop.reason is not StopReason.SUPPORTED


def test_no_progress_states_are_reported_not_hidden():
    lab = Lab()
    belief = lab.belief()
    assert assess_stop(lab.model, belief, best_eig_bits=0.0).reason is StopReason.AMBIGUOUS
    assert (
        assess_stop(lab.model, belief, best_eig_bits=1.0, experiments_available=False).reason
        is StopReason.NO_SAFE_EXPERIMENT
    )
    assert (
        assess_stop(lab.model, belief, best_eig_bits=1.0, budget_exhausted=True).reason
        is StopReason.BUDGET_EXHAUSTED
    )


# -- experiment value ------------------------------------------------------------------------


def test_equivalent_repeats_have_no_new_value():
    lab = Lab(segment="LINE_026", fraction=0.6)
    reading = lab.tdr("LINE_026/C1", 0.3)
    again = TdrTest(
        reading.acquisition.access_site_id,
        reading.acquisition.conductor,
        reading.acquisition.return_path_id,
    )
    values = outage.value_acquisitions(lab.model, lab.belief(), [("again", again)], lab.readings)
    assert values.values[0].eig_bits == 0.0
    taken = lab.current("LINE_002/C1", 0.5)
    repeat = ContactCurrent(taken.acquisition.conductor)
    belief = lab.belief()
    value = outage.value_acquisitions(lab.model, belief, [("repeat", repeat)], lab.readings)
    undiscounted, _ = outage._outcome_matrix(lab.model, belief, repeat, [])
    full = outage._eig(undiscounted, belief.weights(lab.model))
    rho = lab.capabilities.systematic_fraction
    assert "repeat of an acquired point" in value.values[0].note
    assert value.values[0].eig_bits == pytest.approx(full * (1 - rho))


def test_a_scan_already_inspected_is_worth_nothing_more():
    lab = Lab(segment="LINE_026", fraction=0.3)
    lo, hi = round(lab.length * 0.5), round(lab.length * 0.7)
    pos = lab.geometry.position_of
    scan = LidarPass(
        Interval("LINE_026", lo, hi),
        ScanTrajectory(
            (
                ScanPose(pos(LinePoint("LINE_026", lo)), 50.0, 0.0),
                ScanPose(pos(LinePoint("LINE_026", hi)), 50.0, 0.0),
            ),
            30,
        ),
    )
    belief = lab.belief()
    before = outage.value_acquisitions(lab.model, belief, [("scan", scan)], lab.readings).values[0]
    lab.scan("LINE_026", lo, hi)
    after = outage.value_acquisitions(
        lab.model, lab.belief(), [("scan", scan)], lab.readings
    ).values[0]
    assert before.eig_bits > 0 and after.eig_bits < 0.05 * before.eig_bits


def test_values_are_public_only_and_do_not_depend_on_the_hidden_episode():
    a, b = Lab(seed=1, noise_seed=2), Lab(seed=7, noise_seed=8, segment="LINE_056")
    pa, va = outage.propose_experiments(a.model, a.belief())
    pb, vb = outage.propose_experiments(b.model, b.belief())
    assert pa == pb
    assert [v.eig_bits for v in va.values] == [v.eig_bits for v in vb.values]


def test_conditioned_batches_do_not_pay_twice_for_the_same_discrimination():
    lab = Lab()
    belief = lab.belief()
    pool, _ = outage.experiment_pool(lab.model, belief)
    useful = [acq for _, acq in pool][:12]
    single = max(outage.joint_value(lab.model, belief, [a]) for a in useful)
    pair = max(outage.joint_value(lab.model, belief, [a, b]) for a in useful for b in useful)
    assert pair >= single - 1e-12
    same = useful[0]
    assert outage.joint_value(lab.model, belief, [same, same]) == pytest.approx(
        outage.joint_value(lab.model, belief, [same])
    )
    batch = outage.rank_batch(lab.model, belief, useful, 3)
    assert len(batch) <= 3
    assert (
        outage.joint_value(lab.model, belief, batch) <= math.log2(len(lab.model.candidates)) + 1e-9
    )


def test_proposals_are_bounded_and_ranked_by_expected_discrimination():
    lab = Lab()
    proposals, values = outage.propose_experiments(lab.model, lab.belief())
    assert 0 < len(proposals) <= lab.model.caps.max_alternatives
    bits = [p.eig_bits for p in proposals]
    assert bits == sorted(bits, reverse=True)
    assert 1 <= sum(1 for p in proposals if p.batch_order) <= outage.BATCH_SIZE
    assert all(v.bins <= lab.model.caps.max_bins for v in values.values)


# -- the tiny adaptive oracle ----------------------------------------------------------------


def noiseless_outcomes(lab: Lab, belief):
    """Deterministic outcome class of every pooled experiment for every candidate."""
    pool, _ = outage.experiment_pool(lab.model, belief)
    outcomes = {}
    for label, acq in pool:
        p, _ = outage._outcome_matrix(lab.model, belief, acq, [])
        outcomes[label] = tuple(int(np.argmax(row)) for row in p)
    return outcomes


def optimal_worst_case_depth(outcomes, candidates):
    """Fewest experiments that always reduce a candidate set to an inseparable class."""
    from functools import cache

    labels = sorted(outcomes)

    @cache
    def depth(states: frozenset) -> int:
        best = None
        for label in labels:
            groups = {}
            for k in states:
                groups.setdefault(outcomes[label][k], []).append(k)
            if len(groups) == 1:
                continue
            worst = 1 + max(depth(frozenset(g)) for g in groups.values())
            if best is None or worst < best:
                best = worst
        return 0 if best is None else best

    return depth(frozenset(candidates))


def greedy_worst_case_depth(lab: Lab, belief, outcomes):
    weights = belief.weights(lab.model)
    pool, _ = outage.experiment_pool(lab.model, belief)
    acq = dict(pool)

    def depth(states: frozenset) -> int:
        w = np.where([k in states for k in range(len(weights))], weights, 0.0)
        best, best_gain = None, 1e-9
        for label in sorted(outcomes):
            p, _ = outage._outcome_matrix(lab.model, belief, acq[label], [])
            gain = outage._eig(p, w)
            groups = {outcomes[label][k] for k in states}
            if len(groups) > 1 and gain > best_gain:
                best, best_gain = label, gain
        if best is None:
            return 0
        groups = {}
        for k in states:
            groups.setdefault(outcomes[best][k], []).append(k)
        return 1 + max(depth(frozenset(g)) for g in groups.values())

    return depth(frozenset(range(len(weights))))


def test_the_greedy_value_policy_stays_close_to_the_adaptive_oracle_on_a_tiny_fixture():
    lab = Lab()
    belief = lab.belief()
    outcomes = noiseless_outcomes(lab, belief)
    candidates = range(len(lab.model.candidates))
    oracle = optimal_worst_case_depth(outcomes, candidates)
    greedy = greedy_worst_case_depth(lab, belief, outcomes)
    assert oracle >= math.ceil(math.log(len(lab.model.candidates), max(2, 8)))
    assert oracle <= greedy <= oracle + 2, (oracle, greedy)
