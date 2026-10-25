"""Coverage and scoped findings from LiDAR readings (the only local method)."""

from __future__ import annotations

import pytest
from fixtures_three_method import two_drone_world
from helpers_p2 import inject_lidar, tiny_sim

from grid_ops import assessment
from grid_ops.assessment import (
    Outcome,
    SpanRef,
    Verdict,
    assess_scope,
    coverage_report,
    interpret_lidar,
)
from grid_ops.geometry import Interval
from grid_ops.knowledge import assess_landing, assess_perch
from grid_ops.manager_tools import build_toolbox


def world():
    return two_drone_world()


def reading(
    w, geo, rid, segment, clearance_m, start=0, end=110_000, missing=(), inspected=None
):
    from helpers_p2 import lidar_reading

    r = lidar_reading(
        geo, rid, segment, start, end, clearance_m, missing=missing, inspected=inspected
    )
    w.readings[rid] = r
    assessment.ingest_lidar(w, r)
    return r


def test_below_threshold_clearance_is_a_detected_hazard():
    w, geo = world()
    r = reading(w, geo, "R0001", "LINE_040", 0.4)
    interp = interpret_lidar(r)
    assert interp.outcome is Outcome.POSITIVE
    assert interp.value == pytest.approx(0.4)
    assert [h.segment_id for h in w.hazards] == ["LINE_040"]
    hz = w.hazards[0]
    assert hz.verdict == "FAIL" and hz.evidence_ids == (r.id,)
    assert "0.5" in " ".join(hz.assumptions)  # the declared threshold is on the record
    cov = w.coverage[0]
    assert cov.reading_id == r.id and cov.inspected == (Interval("LINE_040", 0, 110_000),)


def test_a_known_hazard_forbids_landing_on_its_interval_only():
    w, geo = world()
    reading(w, geo, "R0001", "LINE_040", 0.4)
    from grid_ops.geometry import LandingSite, LinePoint

    blocked = LandingSite(LinePoint("LINE_040", 50_000), "LINE_040/C1")
    verdict = assess_perch(w, blocked)
    assert not verdict.eligible and "hazard" in " ".join(verdict.reasons)
    assert not assess_landing(w, blocked, w.clock_s + 60).eligible
    free = LandingSite(LinePoint("LINE_025", 0), "LINE_025/C1")
    assert assess_perch(w, free).eligible  # hazards are interval-scoped, not line-scoped


def test_above_threshold_is_inconclusive_and_never_clears_the_scope():
    w, geo = world()
    r = reading(w, geo, "R0001", "LINE_040", 2.0)
    assert w.interpretations[r.id].outcome is Outcome.INCONCLUSIVE
    assert w.hazards == []
    a = assess_scope(w, SpanRef("LINE_040", 0, 110_000), "vegetation_clearance")
    assert a.verdict is Verdict.INDETERMINATE
    assert a.inconclusive == (Interval("LINE_040", 0, 110_000),)  # inspected but not conclusive
    assert a.untested == ()  # nothing was uninspected; nothing is cleared either
    assert "never a clear" in a.explanation


def test_unsupported_conditions_are_not_cleared():
    w, geo = world()
    reading(w, geo, "R0001", "LINE_040", 2.0)
    a = assess_scope(w, SpanRef("LINE_040", 0, 110_000), "ice_accretion")
    assert a.verdict is Verdict.NOT_SUPPORTED and a.untested
    assert "can test" in a.explanation


def test_findings_never_claim_causation():
    w, geo = world()
    reading(w, geo, "R0001", "LINE_040", 0.4)
    reading(w, geo, "R0002", "LINE_025", 2.0)
    for a in assessment.segment_assessments(w):
        assert "do not show this condition caused the outage" in a.explanation


def test_overlapping_inspections_count_their_union_once():
    w, geo = world()
    reading(w, geo, "R0001", "LINE_040", 2.0, start=0, end=110_000)
    reading(w, geo, "R0002", "LINE_040", 2.0, start=50_000, end=110_000)
    rep = coverage_report(w, SpanRef("LINE_040", 0, 110_000))
    assert rep.inspected_mm == 110_000  # the overlap is counted once
    assert rep.stop_count == 2
    assert rep.scope_mm == 110_000


def test_missing_extent_stays_explicitly_uninspected():
    w, geo = world()
    iv = Interval("LINE_040", 0, 110_000)
    r = reading(
        w, geo, "R0001", "LINE_040", 2.0,
        inspected=(Interval("LINE_040", 0, 40_000),),
        missing=(Interval("LINE_040", 40_000, 110_000),),
    )
    assert w.interpretations[r.id].inspected == (Interval("LINE_040", 0, 40_000),)
    rep = coverage_report(w, SpanRef("LINE_040", 0, 110_000))
    assert (40_000, 110_000) in {(i.start_mm, i.end_mm) for i in rep.uninspected}
    assert rep.inspected_mm == 40_000


def test_published_readings_reach_the_public_surfaces_with_no_truth_leak():
    sim = tiny_sim()
    inject_lidar(sim, "R0001", "LINE_040", 0, 110_000, 0.4)
    sim.step(0)
    w = sim.snapshot
    assert w.readings and w.hazards and w.interpretations
    tb = build_toolbox(w, sim.network, sim.circuits)
    text = tb.get_reading("R0001")
    assert "clearance_m" in text and "lidar" in text
    for secret in ("episode_seed", "noise_seed", "manifest", "energized"):
        assert secret not in text