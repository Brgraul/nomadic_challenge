"""Ticket 06: coverage and scoped condition findings, driven through the real simulation."""

from __future__ import annotations

import pytest
from conftest import permissive

from grid_ops import assessment
from grid_ops.assessment import (
    Outcome,
    SpanRef,
    Verdict,
    assess_scope,
    coverage_report,
    detection_probability,
)
from grid_ops.geometry import MM_PER_KM, LandingSite, LinePoint, union_length_mm
from grid_ops.grid import load_network
from grid_ops.knowledge import LandingPolicy, assess_landing
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import DETECTION_REQUIREMENT
from grid_ops.sim import CommitSorties, Simulation
from grid_ops.world import Measurement, Ready


@pytest.fixture(scope="module")
def network():
    return load_network()


def fresh(network):
    return permissive(Simulation(network))


def pt(seg, km):
    return LinePoint(seg, round(km * MM_PER_KM))


def fly(sim, drone, stops, minutes=29):
    """Commit one sortie ending back on the drone's own perch, then run the clock."""
    perch = sim._world.drones[drone].state.perch
    reqs = tuple(StopRequest(p, k) for p, k in stops)
    sim._apply(CommitSorties((SortieRequest(drone, reqs, perch),), "operator"))
    sim.step(minutes)
    return sim.snapshot


def reading_at(w, segment, offset_mm=None, kind=None):
    for r in w.readings.values():
        same_kind = kind is None or r.kind == kind
        if r.point.segment_id == segment and same_kind and offset_mm in (None, r.point.offset_mm):
            return r
    raise AssertionError("no such reading")


def veg_sim(network):
    sim = fresh(network)
    w = fly(
        sim,
        "DRONE_2",
        [
            (pt("LINE_058", 0.03), Measurement.CLEARANCE),
            (pt("LINE_064", 9.0), Measurement.CLEARANCE),
            (pt("LINE_064", 9.4), Measurement.CLEARANCE),
            (pt("LINE_064", 8.3), Measurement.CLEARANCE),
        ],
    )
    return sim, w


# -- payloads and coverage -----------------------------------------------------------------------


def test_readings_retain_numbers_visibility_conditions_and_epoch(network):
    _, w = veg_sim(network)
    r = reading_at(w, "LINE_058")
    assert dict(r.payload)["clearance_m"] == pytest.approx(0.4, abs=0.2)
    assert r.operating_epoch == "E0"
    assert dict(r.conditions)["visible_fraction"] == 1.0
    assert r.inspected and r.id and r.sensor_model_revision


def test_vegetation_fixture_gives_a_detected_hazard_that_blocks_landing(network):
    sim, w = veg_sim(network)
    r = reading_at(w, "LINE_058")
    interp = w.interpretations[r.id]
    assert interp.outcome is Outcome.POSITIVE
    assert [h.segment_id for h in w.hazards] == ["LINE_058"]
    hz = w.hazards[0]
    assert hz.verdict == "FAIL" and hz.evidence_ids == (r.id,)
    # a known hazard forbids landing on its interval, whatever the policy
    site = LandingSite(pt("LINE_058", 0.03), "LINE_058/C1")
    sim._world.landing_policy = LandingPolicy(allow_uncertain=True)
    verdict = assess_landing(sim._world, site, sim._world.clock_s + 60)
    assert not verdict.eligible and "hazard" in " ".join(verdict.reasons)


def test_qualified_negative_clears_only_the_inspected_scope(network):
    _, w = veg_sim(network)
    r = reading_at(w, "LINE_064", 9_000_000)
    interp = w.interpretations[r.id]
    assert interp.outcome is Outcome.QUALIFIED_NEGATIVE
    assert interp.detection_probability >= DETECTION_REQUIREMENT
    # footprint 8.5-9.5 km minus the 7.8-8.8 km canopy leaves 8.8-9.5 km inspected
    assert [(i.start_mm, i.end_mm) for i in interp.inspected] == [(8_800_000, 9_500_000)]
    inside = assess_scope(w, SpanRef("LINE_064", 8_800_000, 9_500_000), "vegetation_clearance")
    assert inside.verdict is Verdict.PASS and inside.evidence_ids
    assert "do not prove mechanical integrity" in inside.explanation
    wider = assess_scope(w, SpanRef("LINE_064", 8_000_000, 9_500_000), "vegetation_clearance")
    assert wider.verdict is Verdict.INDETERMINATE  # does not clear what was not inspected
    assert wider.untested  # the occluded canopy interval is explicit
    whole = assess_scope(
        w, SpanRef("LINE_064", 0, w.segment_lengths["LINE_064"]), "vegetation_clearance"
    )
    assert (
        whole.verdict is Verdict.INDETERMINATE and whole.untested
    )  # regional alternative preserved


def test_overlapping_inspections_do_not_multiply_length(network):
    _, w = veg_sim(network)
    scope = SpanRef("LINE_064", 8_000_000, 10_000_000)
    rep = coverage_report(w, scope, "clearance")
    nominal_sum = sum(
        c.nominal.end_mm - c.nominal.start_mm for c in w.coverage if c.segment_id == "LINE_064"
    )
    inspected_sum = sum(
        i.end_mm - i.start_mm for c in w.coverage if c.segment_id == "LINE_064" for i in c.inspected
    )
    assert rep.inspected_mm == union_length_mm(list(rep.inspected))
    assert rep.inspected_mm < inspected_sum <= nominal_sum  # 9.0 and 9.4 km overlap
    assert rep.inspected_mm == 1_100_000  # 8.8-9.9 km, counted once
    assert rep.stop_count == 3


def test_occluded_and_uninspected_intervals_stay_visible(network):
    _, w = veg_sim(network)
    r = reading_at(w, "LINE_064", 8_300_000)
    interp = w.interpretations[r.id]
    assert interp.outcome is Outcome.INCONCLUSIVE and "not visible" in interp.reasons[0]
    rep = coverage_report(w, SpanRef("LINE_064", 7_000_000, 10_000_000), "clearance")
    assert [(i.start_mm, i.end_mm) for i in rep.occluded] == [(7_800_000, 8_800_000)]
    assert (7_000_000, 8_800_000) in {(i.start_mm, i.end_mm) for i in rep.uninspected}
    assert rep.scope_mm == 3_000_000


# -- thermal ---------------------------------------------------------------------------------------


def joint_mid(sim):
    a = next(a for a in sim._truth.local.anomalies if a.kind == "hot_joint")
    return a, (a.interval.start_mm + a.interval.end_mm) // 2


def test_low_load_thermal_negative_is_inconclusive(network):
    sim = fresh(network)
    _, mid = joint_mid(sim)
    w = fly(sim, "DRONE_1", [(LinePoint("LINE_029", mid), Measurement.CONDUCTOR_TEMP)])
    r = reading_at(w, "LINE_029")
    assert r.clock_s < 20 * 60  # inside the light-loading window
    interp = w.interpretations[r.id]
    assert interp.outcome is Outcome.INCONCLUSIVE and "excitation" in interp.reasons[0]
    assert interp.detection_probability == 0.0
    assert w.hazards == []
    a = assess_scope(w, SpanRef("LINE_029", 0, w.segment_lengths["LINE_029"]), "hot_joint")
    assert a.verdict is Verdict.INDETERMINATE and a.inconclusive


def test_loaded_thermal_positive_and_clean_negative(network):
    sim = fresh(network)
    anomaly, mid = joint_mid(sim)
    sim.step(21)  # past the light-loading window
    w = fly(
        sim,
        "DRONE_1",
        [
            (LinePoint("LINE_029", mid), Measurement.CONDUCTOR_TEMP),
            (LinePoint("LINE_029", 200_000), Measurement.CONDUCTOR_TEMP),
        ],
        minutes=15,
    )
    hot = reading_at(w, "LINE_029", mid)
    clean = reading_at(w, "LINE_029", 200_000)
    assert w.interpretations[hot.id].outcome is Outcome.POSITIVE
    assert w.interpretations[clean.id].outcome is Outcome.QUALIFIED_NEGATIVE
    assert assess_scope(w, SpanRef("LINE_029", 0, 700_000), "hot_joint").verdict is Verdict.PASS
    scope = SpanRef("LINE_029", 0, 1_200_000)
    a = assess_scope(w, scope, "hot_joint")
    assert a.verdict is Verdict.INDETERMINATE  # partly covered: the rest stays open
    joint_scope = SpanRef("LINE_029", anomaly.interval.start_mm, anomaly.interval.end_mm)
    assert assess_scope(w, joint_scope, "hot_joint").verdict is Verdict.FAIL
    # the hazard blocks landing exactly on the failed interval only
    assert any(h.segment_id == "LINE_029" for h in w.hazards)


# -- motion ---------------------------------------------------------------------------------------


def test_motion_outside_a_gust_is_inconclusive_and_inside_is_qualified(network):
    quiet = fresh(network)
    anomaly = next(a for a in quiet._truth.local.anomalies if a.kind == "motion")
    mid = (anomaly.interval.start_mm + anomaly.interval.end_mm) // 2
    w = fly(quiet, "DRONE_5", [(LinePoint("LINE_054", mid), Measurement.MOTION)])
    r = reading_at(w, "LINE_054")
    assert r.clock_s < 20 * 60
    interp = w.interpretations[r.id]
    assert interp.outcome is Outcome.INCONCLUSIVE and "gust" in interp.reasons[0]
    assert w.hazards == []

    gusty = fresh(network)
    gusty.step(25)  # inside the 20-50 min gust window
    w = fly(
        gusty,
        "DRONE_5",
        [
            (LinePoint("LINE_054", mid), Measurement.MOTION),
            (LinePoint("LINE_054", 3_000_000), Measurement.MOTION),
        ],
        minutes=20,
    )
    pos = reading_at(w, "LINE_054", mid)
    neg = reading_at(w, "LINE_054", 3_000_000)
    assert w.interpretations[pos.id].outcome is Outcome.POSITIVE
    assert w.interpretations[neg.id].outcome is Outcome.QUALIFIED_NEGATIVE
    span = SpanRef("LINE_054", 2_500_000, 3_500_000)
    assert assess_scope(w, span, "conductor_motion").verdict is Verdict.PASS


# -- scope and support ------------------------------------------------------------------------


def test_unsupported_conditions_are_not_cleared(network):
    _, w = veg_sim(network)
    a = assess_scope(w, SpanRef("LINE_064", 0, 1_000_000), "ice_accretion")
    assert a.verdict is Verdict.NOT_SUPPORTED and a.untested


def test_detection_requirement_gates_qualified_negatives():
    full = {"visible_fraction": 1.0}
    assert detection_probability("clearance", full) >= DETECTION_REQUIREMENT
    assert detection_probability("clearance", {"visible_fraction": 0.0}) == 0.0
    assert detection_probability("conductor_temp", {**full, "excitation_current_a": 10}) == 0.0
    assert detection_probability("conductor_temp", {**full, "excitation_current_a": 40}) >= 0.95
    assert detection_probability("motion", {**full, "qualifying_gust": 0.0}) == 0.0
    assert detection_probability("motion", {**full, "qualifying_gust": 1.0}) >= 0.95


def test_findings_never_claim_causation(network):
    _, w = veg_sim(network)
    for a in assessment.segment_assessments(w):
        assert "do not show this condition caused the outage" in a.explanation


def test_ready_state_unchanged_by_local_findings(network):
    _sim, w = veg_sim(network)
    assert isinstance(
        w.drones["DRONE_2"].state, __import__("grid_ops.world", fromlist=["Charging"]).Charging
    )
    assert isinstance(w.drones["DRONE_3"].state, Ready)
