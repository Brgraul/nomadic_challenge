"""Shared evidence is valued once, under manager-owned request urgency (three-method model)."""

from __future__ import annotations

from dataclasses import replace

import pytest
from fixtures_three_method import contact, site_at, tiny_network, two_drone_world
from helpers_p2 import lidar_reading

from grid_ops import scoring
from grid_ops.geometry import Geometry, Interval
from grid_ops.scoring import PlanScore, build_context, explain, score_plan
from grid_ops.sensing import (
    LidarPass,
    MeasurementTaskV2,
    RequestStatus,
    ScanPose,
    ScanTrajectory,
    TaskStatusV2,
)


def world():
    return two_drone_world()


def contact_request(w, rid="REQ-1", urgency=5, status=RequestStatus.ACTIVE):
    from grid_ops.sensing import AcceptedRequest, ConductorScope, CurrentRequest, OutcomeSpec

    request = CurrentRequest(
        id=rid,
        hypothesis_ids=(),
        rationale="r",
        alternatives_distinguished=("a", "b"),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=urgency,
        source_revision="0",
        replaces_request_ids=(),
        target=ConductorScope(circuit_id="LINE_040/C1", phase="A", start_mm=0, end_mm=110_000),
    )
    w.requests[rid] = AcceptedRequest(rid, request, status, w.clock_s, "0")
    return request


def lidar_request(w, rid="REQ-L", urgency=3):
    from grid_ops.sensing import AcceptedRequest, CorridorSpan, LidarRequest, OutcomeSpec

    request = LidarRequest(
        id=rid,
        hypothesis_ids=(),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=urgency,
        source_revision="0",
        replaces_request_ids=(),
        target=CorridorSpan(segment_id="LINE_040", start_mm=0, end_mm=110_000),
    )
    w.requests[rid] = AcceptedRequest(rid, request, RequestStatus.ACTIVE, w.clock_s, "0")
    return request


def lidar_task(w, task_id, segment, start_mm, end_mm, rid="REQ-L", urgency=3):
    iv = Interval(segment, start_mm, end_mm)
    poses = (ScanPose((0.0, 0.0), 50.0, 0.0), ScanPose((0.001, 0.0), 50.0, 0.0))
    acq = LidarPass(iv, ScanTrajectory(poses, 10))
    task = MeasurementTaskV2(task_id, rid, acq, rid, TaskStatusV2.PENDING, urgency)
    w.set_task(task)
    return task


def tier_at(tiers: tuple[int, ...], urgency: int) -> int:
    """Tiers are ordered 5..1; the urgency-u award sits at index 5-u."""
    return tiers[5 - urgency]


# -- one award per alternative group -------------------------------------------------------------


def test_contact_value_is_one_flat_award_per_group_at_the_request_urgency():
    w, _geo = world()
    contact_request(w, urgency=4)
    for tid, offset in (("T0001", 0), ("T0002", 50_000)):
        from grid_ops.sensing import MeasurementTaskV2

        w.set_task(
            MeasurementTaskV2(
                tid, "REQ-1", contact(site_at(w, "LINE_040", offset)), "REQ-1",
                TaskStatusV2.PENDING, 2,
            )
        )
    ctx = build_context(w)
    alone = score_plan(w, ctx, ["T0001"])
    both = score_plan(w, ctx, ["T0001", "T0002"])
    assert tier_at(alone.tiers, 4) == scoring.REQUEST_POINTS
    assert sum(alone.tiers) == scoring.REQUEST_POINTS  # nothing at any other urgency
    assert both.tiers == alone.tiers  # a second attempt of the same group earns nothing
    assert score_plan(w, ctx, ["T0002", "T0001", "T0001"]).tiers == alone.tiers


def test_lidar_value_is_the_length_of_the_requested_span_counted_once():
    w, _geo = world()
    lidar_request(w, urgency=3)
    lidar_task(w, "T0001", "LINE_040", 0, 110_000)
    ctx = build_context(w)
    one = score_plan(w, ctx, ["T0001"])
    assert one.tiers[2] == pytest.approx(110_000 / 1000, abs=1)  # 1 point per metre
    assert tier_at(one.tiers, 3) == pytest.approx(110_000 / 1000, abs=1)


def test_planned_lidar_value_counts_only_still_uninspected_extent():
    """Cells already covered by a completed valid reading earn nothing again."""
    w, geo = world()
    lidar_request(w)
    lidar_task(w, "T0001", "LINE_040", 0, 110_000)
    w.readings["R0001"] = lidar_reading(
        Geometry(tiny_network()), "R0001", "LINE_040", 50_000, 110_000, 2.0
    )
    ctx = build_context(w)
    s = score_plan(w, ctx, ["T0001"])
    assert s.tiers[2] == pytest.approx(50_000 / 1000, abs=1)  # only 0-50 m is still undecided


def test_request_urgency_owns_the_tier_not_a_planner_side_field():
    w, _geo = world()
    contact_request(w, urgency=4)
    from grid_ops.sensing import MeasurementTaskV2

    w.set_task(
        MeasurementTaskV2(
            "T0001", "REQ-1", contact(site_at(w, "LINE_040", 0)), "REQ-1",
            TaskStatusV2.PENDING, 2,
        )
    )
    before = score_plan(w, build_context(w), ["T0001"])
    w.set_task(replace(w.tasks["T0001"], urgency=5))  # planner-side rewrite
    after = score_plan(w, build_context(w), ["T0001"])
    assert after == before  # the award stays at the request's urgency tier, value unchanged
    assert tier_at(after.tiers, 4) == scoring.REQUEST_POINTS


def test_ordering_of_task_ids_never_changes_the_score():
    w, _geo = world()
    contact_request(w)
    from grid_ops.sensing import MeasurementTaskV2

    w.set_task(
        MeasurementTaskV2(
            "T0001", "REQ-1", contact(site_at(w, "LINE_040", 0)), "REQ-1",
            TaskStatusV2.PENDING, 3,
        )
    )
    ctx = build_context(w)
    assert score_plan(w, ctx, ["T0001"]) == score_plan(w, ctx, list(reversed(["T0001"])))


# -- the fixed comparison -----------------------------------------------------------------------------------------


def test_urgency_tiers_precede_everything():
    hi = PlanScore((10, 0, 0, 0, 0), risk=9, switching=9, airtime_s=9999, charge_s=9999)
    lo = PlanScore((0, 0, 0, 0, 100_000), risk=0, switching=0, airtime_s=1, charge_s=0)
    assert hi.better_than(lo) and not lo.better_than(hi)


def test_risk_then_switching_then_airtime_then_charge_break_ties():
    base = dict(tiers=(5, 4, 3, 2, 1))
    a = PlanScore(**base, risk=1, switching=5, airtime_s=900, charge_s=0)
    b = PlanScore(**base, risk=0, switching=9, airtime_s=1800, charge_s=0)
    c = PlanScore(**base, risk=0, switching=2, airtime_s=1800, charge_s=0)
    d = PlanScore(**base, risk=0, switching=2, airtime_s=900, charge_s=0)
    e = PlanScore(**base, risk=0, switching=2, airtime_s=900, charge_s=600)
    assert b.better_than(a) and c.better_than(b) and d.better_than(c) and d.better_than(e)
    # less charge waiting is better: e adds charge time and can never beat d


def test_scores_are_compared_within_a_fixed_integer_quantum():
    a = PlanScore((500, 0, 0, 0, 0), risk=0, switching=0, airtime_s=0, charge_s=0)
    b = PlanScore((501, 0, 0, 0, 0), risk=0, switching=0, airtime_s=0, charge_s=0)
    c = PlanScore((503, 0, 0, 0, 0), risk=0, switching=0, airtime_s=0, charge_s=0)
    assert a.equal_within_quantum(b) and not a.equal_within_quantum(c)


def test_explain_names_the_scale_and_the_tolerance():
    w, _geo = world()
    ctx = build_context(w)
    lines = explain(w, ctx, score_plan(w, ctx, []))
    assert any("not a calibrated causal entropy" in x for x in lines)
    assert any("quantisation tolerance: 1 unit" in x for x in lines)


# -- explicit unresolved work ----------------------------------------------------------------------


def test_unresolved_requests_are_named_until_a_valid_attempt_answers_the_group():
    w, geo = world()
    contact_request(w)
    from grid_ops.sensing import MeasurementTaskV2

    w.set_task(
        MeasurementTaskV2(
            "T0001", "REQ-1", contact(site_at(w, "LINE_040", 0)), "REQ-1",
            TaskStatusV2.PENDING, 3,
        )
    )
    assert scoring.unresolved_requests(w) == [("REQ-1", "no completed valid attempt yet")]
    w.readings["R0001"] = lidar_reading(Geometry(tiny_network()), "R0001", "LINE_040", 0, 110_000, 12.0)
    w.set_task(replace(w.tasks["T0001"], status=TaskStatusV2.COMPLETED, reading_id="R0001"))
    assert scoring.unresolved_requests(w) == []


def test_retired_requests_never_reach_the_scorer_as_open_work():
    """The lifecycle, not the scorer, retires a request's unstarted attempts (see test_needs)."""
    from grid_ops import needs

    w, geo = world()
    net = tiny_network()
    contact_request(w, status=RequestStatus.RETIRED)
    from grid_ops.sensing import MeasurementTaskV2

    w.set_task(
        MeasurementTaskV2(
            "T0001", "REQ-1", contact(site_at(w, "LINE_040", 0)), "REQ-1",
            TaskStatusV2.PENDING, 3,
        )
    )
    notes, _unavailable = needs.refresh(w, net, geo)
    assert w.tasks["T0001"].status is TaskStatusV2.CANCELLED
    assert any("retired" in n for n in notes)