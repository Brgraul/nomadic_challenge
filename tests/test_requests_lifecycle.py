"""Behavior tests for the request-lifecycle part of proposal validation."""

from __future__ import annotations

import pydantic
import pytest
from fixtures_three_method import tiny_network, two_drone_world

from grid_ops.circuits import CircuitRegistry
from grid_ops.hypotheses import ManagerOutput
from grid_ops.manager_tools import build_toolbox
from grid_ops.proposals import validate
from grid_ops.sensing import (
    AcceptedRequest,
    ConductorScope,
    CorridorSpan,
    CurrentRequest,
    LidarRequest,
    OutcomeSpec,
    RequestStatus,
)


def registry(net):
    return CircuitRegistry(net, {})


def make_tb():
    net = tiny_network()
    w, _ = two_drone_world()
    w.hypotheses["H1"] = {"id": "H1", "priority": 3, "lifecycle": "ACTIVE"}
    return build_toolbox(w, net, registry(net))


def request(tb, rid="REQ-1", replaces=(), **overrides) -> CurrentRequest:
    fields = dict(
        id=rid,
        hypothesis_ids=("H1",),
        rationale="distinguish H1",
        alternatives_distinguished=("a",),
        expected_outcomes=(OutcomeSpec(description="o"),),
        evidence_ids=(),
        urgency=3,
        source_revision=str(tb.snapshot.revision),
        replaces_request_ids=replaces,
        target=ConductorScope(circuit_id="LINE_040/C1", phase="A", start_mm=0, end_mm=1_000_000),
    )
    fields.update(overrides)
    return CurrentRequest(**fields)


def output(tb, requests=(), retire=()) -> ManagerOutput:
    return ManagerOutput(
        summary="s", measurement_requests=list(requests), retire_request_ids=list(retire)
    )


def add_request(w, rid, status=RequestStatus.ACTIVE):
    req = CurrentRequest(
        id=rid,
        hypothesis_ids=("H1",),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(),
        evidence_ids=(),
        urgency=2,
        source_revision="0",
        replaces_request_ids=(),
        target=ConductorScope(circuit_id="LINE_040/C1", phase="A", start_mm=0, end_mm=1),
    )
    w.requests[rid] = AcceptedRequest(rid, req, status, 0, "0")
    return req


def test_duplicate_request_ids_rejected():
    tb = make_tb()
    errs = validate(output(tb, [request(tb), request(tb)]), tb)
    assert any("duplicate id" in e for e in errs), errs


def test_retire_unknown_request_rejected():
    tb = make_tb()
    errs = validate(output(tb, retire=("REQ-GONE",)), tb)
    assert any("REQ-GONE does not exist" in e for e in errs), errs


def test_retire_inactive_request_rejected():
    tb = make_tb()
    add_request(tb.world, "REQ-OLD", RequestStatus.RETIRED)
    errs = validate(output(tb, retire=("REQ-OLD",)), tb)
    assert any("REQ-OLD is RequestStatus.RETIRED" in e or "not ACTIVE" in e for e in errs), errs


def test_replaced_request_must_exist_and_be_active():
    tb = make_tb()
    errs = validate(output(tb, [request(tb, replaces=("REQ-MISSING",))]), tb)
    assert any("REQ-MISSING does not exist" in e for e in errs), errs
    add_request(tb.world, "REQ-OLD", RequestStatus.RETIRED)
    errs = validate(output(tb, [request(tb, replaces=("REQ-OLD",))]), tb)
    assert any("not ACTIVE" in e for e in errs), errs


def test_replaced_request_may_not_also_be_retired():
    tb = make_tb()
    add_request(tb.world, "REQ-OLD")
    errs = validate(output(tb, [request(tb, replaces=("REQ-OLD",))], retire=("REQ-OLD",)), tb)
    assert any("both replaced and retired" in e for e in errs), errs


def test_unknown_hypothesis_in_request_rejected():
    tb = make_tb()
    errs = validate(output(tb, [request(tb, hypothesis_ids=("H-NOPE",))]), tb)
    assert any("unknown hypothesis H-NOPE" in e for e in errs), errs


def test_unknown_evidence_link_in_request_rejected():
    tb = make_tb()
    errs = validate(output(tb, [request(tb, evidence_ids=("R0001",))]), tb)
    assert any("evidence link R0001" in e for e in errs), errs


def test_source_revision_mismatch_rejected():
    tb = make_tb()
    errs = validate(output(tb, [request(tb, source_revision="999")]), tb)
    assert any("source_revision" in e for e in errs), errs


def test_valid_requests_pass():
    tb = make_tb()
    add_request(tb.world, "REQ-OLD")
    errs = validate(output(tb, [request(tb, replaces=("REQ-OLD",))]), tb)
    assert errs == []


def test_urgency_bounds_are_enforced_by_pydantic():
    with pytest.raises(pydantic.ValidationError):
        request(make_tb(), urgency=0)
    with pytest.raises(pydantic.ValidationError):
        request(make_tb(), urgency=6)


def test_union_parses_both_request_models():
    tb = make_tb()
    lidar = LidarRequest(
        id="REQ-L",
        hypothesis_ids=("H1",),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(),
        evidence_ids=(),
        urgency=1,
        source_revision=str(tb.snapshot.revision),
        replaces_request_ids=(),
        target=CorridorSpan(segment_id="LINE_040", start_mm=0, end_mm=500_000),
    )
    out = ManagerOutput(summary="s", measurement_requests=[lidar, request(tb)])
    assert out.measurement_requests[0].method == "lidar"
    assert out.measurement_requests[1].method == "contact_current"
