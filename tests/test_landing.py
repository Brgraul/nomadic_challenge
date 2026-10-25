"""Landing eligibility: mechanical perches, handshake-confirmed charging evidence and policy."""

from __future__ import annotations

from fixtures_three_method import contact, site_at, two_drone_world
from helpers_p2 import real_sim

from grid_ops import knowledge
from grid_ops.knowledge import (
    EVIDENCE_LIFETIME_S,
    ChargeState,
    Energization,
    FaultAssessment,
    LandingPolicy,
    assess_landing,
    assess_perch,
    ingest_handshake,
    ingest_report,
    invalid_endpoints,
    landing_candidates,
)
from grid_ops.routes import SortieRequest, apply_sorties, validate_sorties
from grid_ops.sensing import (
    AcceptedRequest,
    ConductorScope,
    CurrentRequest,
    MeasurementTaskV2,
    RequestStatus,
    TaskStatusV2,
)
from grid_ops.world import (
    LAND_S,
    TAKEOFF_S,
    Land,
    ScheduledStep,
    Takeoff,
    Transit,
    leg_seconds,
    seed_starting_handshakes,
)


def world():
    return two_drone_world()


def handshaked_world():
    w, geo = world()
    seed_starting_handshakes(w)
    return w, geo


# -- starting evidence --------------------------------------------------------------------------------


def test_starting_perches_have_explicit_successful_handshakes():
    sim = real_sim()
    w = sim._world
    starts = {d.state.perch.circuit_id for d in w.drones.values()}
    for cid, c in w.knowledge.chargeability.items():
        if cid in starts:
            assert c.state is ChargeState.CONFIRMED
            assert c.expires_s == EVIDENCE_LIFETIME_S
            assert any(w.knowledge.evidence[e].source == "handshake" for e in c.evidence_ids)
        elif cid != "LINE_024/C1":
            assert c.state is ChargeState.UNKNOWN
    ck = w.knowledge.circuits["LINE_024/C1"]
    assert ck.energization is Energization.DE_ENERGIZED
    assert w.knowledge.chargeability["LINE_024/C1"].state is ChargeState.UNAVAILABLE


def test_a_successful_handshake_confirms_only_its_own_circuit():
    w, _geo = world()
    ingest_handshake(w, "LINE_040/C1", True, "setup", charging=False)
    assert w.knowledge.chargeability["LINE_040/C1"].state is ChargeState.CONFIRMED
    assert w.knowledge.chargeability["LINE_025/C1"].state is ChargeState.UNKNOWN
    assert w.knowledge.circuits["LINE_040/C1"].energization is Energization.LIVE


def test_a_failed_handshake_makes_the_circuit_unavailable():
    w, _geo = world()
    ingest_handshake(w, "LINE_040/C1", False, "d1", charging=False)
    ch = w.knowledge.chargeability["LINE_040/C1"]
    assert ch.state is ChargeState.UNAVAILABLE and "failed" in ch.reason


# -- evidence lifetime --------------------------------------------------------------------------------


def test_evidence_expires_exactly_at_its_lifetime_and_the_expiry_is_public():
    sim = real_sim()
    perch = sim._world.drones["DRONE_1"].state.perch
    w = sim._world
    assert assess_landing(w, perch, EVIDENCE_LIFETIME_S - 1).eligible
    assert not assess_landing(w, perch, EVIDENCE_LIFETIME_S).eligible
    events = sim.step(30)  # T+00:30: the handshake evidence expires
    expired = [e for e in events if e.kind == "evidence_expired"]
    assert expired and all(e.clock_s == EVIDENCE_LIFETIME_S for e in expired)
    assert sim._world.knowledge.chargeability[perch.circuit_id].state is ChargeState.UNKNOWN


# -- switching reports --------------------------------------------------------------------------------


def test_a_switching_report_makes_charging_unavailable_and_blocks_endpoints():
    w, _geo = world()
    ingest_handshake(w, "LINE_040/C1", True, "d1", charging=False)
    w.clock_s += 1  # the report is newer than the stale handshake observation
    ingest_report(w, "LINE_040", w.clock_s, "Dispatch reports LINE_040 switched off.")
    ch = w.knowledge.chargeability["LINE_040/C1"]
    assert ch.state is ChargeState.UNAVAILABLE
    site = site_at(w, "LINE_040", 50_000)
    assert not assess_landing(w, site, w.clock_s + 60).eligible
    permissive = LandingPolicy(allow_uncertain=True)
    assert not assess_landing(w, site, w.clock_s, permissive).eligible


def test_a_landed_perch_stays_a_valid_perch_on_a_reported_dead_circuit():
    w, _geo = world()
    ingest_report(w, "LINE_025", w.clock_s, "Dispatch reports LINE_025 switched off.")
    assert assess_perch(w, site_at(w, "LINE_025", 0)).eligible
    # a dead line is a mechanically valid perch; only Charge steps need charging evidence


# -- policy --------------------------------------------------------------------------------------------


def test_uncertain_endpoints_are_an_explicit_opt_in_and_hazards_are_never_allowed():
    w, _geo = world()
    unknown = site_at(w, "LINE_025", 50_000)
    v = assess_landing(w, unknown, 600)
    assert not v.eligible and v.policy == "confirmed-only"
    w2, _ = world()
    w2.landing_policy = LandingPolicy(allow_uncertain=True)
    v2 = assess_landing(w2, unknown, 600)
    assert v2.eligible and "uncertain endpoint" in v2.reasons[0]
    assert v2.policy.startswith("uncertain-endpoints")
    w2.hazards.append(FaultAssessment("H1", "LINE_025", 0, 110_000, "FAIL", ("R1",)))
    assert not assess_landing(w2, unknown, 600).eligible
    assert "hazard" in " ".join(assess_perch(w2, unknown).reasons)


# -- candidates and occupancy --------------------------------------------------------------------------


def test_landing_candidates_include_perches_projections_and_explanations():
    sim = real_sim()
    w = sim._world
    drone = w.drones["DRONE_1"]
    cands = landing_candidates(w, sim.geometry, "DRONE_1", lambda s: w.clock_s + 600)
    assert {"perch", "projection", "sample"} <= {c.source for c in cands}
    assert drone.state.perch in {c.site for c in cands if c.source == "perch"}
    assert all(c.site.circuit_id.startswith(c.site.point.segment_id + "/") for c in cands)
    own = next(c for c in cands if c.site == drone.state.perch)
    assert own.eligibility.eligible
    dead = next(
        (c for c in cands if c.site.point.segment_id == "LINE_024"),
        None,
    )
    assert dead is not None and dead.eligibility.eligible, (
        "a reported-dead circuit is a mechanically valid perch; only Charge steps need "
        "charging evidence"
    )
    assert drone.state.perch in {c.site for c in cands if c.occupant}


def test_candidates_are_derived_from_compiled_ground_tasks():
    w, geo = world()
    site = site_at(w, "LINE_025", 50_000)
    request = CurrentRequest(
        id="REQ-1",
        hypothesis_ids=(),
        rationale="r",
        alternatives_distinguished=("a",),
        expected_outcomes=(),
        evidence_ids=(),
        urgency=3,
        source_revision="0",
        replaces_request_ids=(),
        target=ConductorScope(circuit_id="LINE_025/C1", phase="A", start_mm=0, end_mm=110_000),
    )
    w.requests["REQ-1"] = AcceptedRequest("REQ-1", request, RequestStatus.ACTIVE, 0, "0")
    w.set_task(MeasurementTaskV2("T0001", "REQ-1", contact(site), "REQ-1", TaskStatusV2.PENDING, 3))
    cands = landing_candidates(w, geo, "DRONE_1", lambda s: 900)
    assert any(c.source == "task" and c.site == site for c in cands)


def test_planned_landings_invalidated_by_new_evidence_are_reported():
    w, geo = world()
    site = site_at(w, "LINE_025", 50_000)
    pos = geo.position_of(site.point)
    start = w.drones["DRONE_1"].state.perch
    start_pos = geo.position_of(start.point)
    leg = max(leg_seconds(start_pos, pos, w.drones["DRONE_1"].speed_km_per_min), 1)
    steps = (
        ScheduledStep(0, 0, TAKEOFF_S, Takeoff(start, start_pos)),
        ScheduledStep(1, TAKEOFF_S, TAKEOFF_S + leg, Transit(start_pos, pos)),
        ScheduledStep(2, TAKEOFF_S + leg, TAKEOFF_S + leg + LAND_S, Land(site, pos)),
    )
    req = SortieRequest(
        "DRONE_1", steps, site, w.revision, w.energy_model.revision, w.capabilities.revision
    )
    result = validate_sorties(w, geo, None, (req,))
    assert result.ok, [str(r) for r in result.rejections]
    apply_sorties(w, _NoAcquisitions(), result.plans)
    assert invalid_endpoints(w) == []
    w.hazards.append(FaultAssessment("H1", "LINE_025", 0, 110_000, "FAIL", ("R1",)))
    bad = invalid_endpoints(w)
    assert [b[0] for b in bad] == ["DRONE_1"] and "hazard" in " ".join(bad[0][2].reasons)


class _NoAcquisitions:
    """A physics stub for applying itineraries without acquiring anything."""

    def is_chargeable(self, site):
        return False

    def acquire(self, executed):  # pragma: no cover - not reached in these tests
        raise AssertionError("no acquisition was scheduled")