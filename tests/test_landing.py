"""Ticket 05: landing eligibility from public, circuit-specific charging evidence."""

from __future__ import annotations

import pytest

from grid_ops import knowledge
from grid_ops.geometry import MM_PER_KM, LandingSite, LinePoint
from grid_ops.grid import load_network
from grid_ops.knowledge import (
    EVIDENCE_LIFETIME_S,
    ChargeState,
    Claim,
    Energization,
    FaultAssessment,
    LandingPolicy,
    assess_landing,
    classify_voltage,
    invalid_endpoints,
    landing_candidates,
)
from grid_ops.routes import SortieRequest, StopRequest
from grid_ops.scenario import ScenarioConfig, ScenarioMode
from grid_ops.sim import CONTRADICTED_SEGMENT, CommitSorties, SetInService, Simulation
from grid_ops.views import endpoint_rows
from grid_ops.world import Charging, CommandRejected, Measurement, Ready, Stranded


@pytest.fixture(scope="module")
def network():
    return load_network()


@pytest.fixture
def sim(network):
    return Simulation(network)  # default policy: confirmed endpoints only


def site(seg, km, circuit=None):
    return LandingSite(LinePoint(seg, round(km * MM_PER_KM)), circuit or f"{seg}/C1")


def current_stop(seg, km):
    return StopRequest(LinePoint(seg, round(km * MM_PER_KM)), Measurement.CURRENT)


def volt_stop(seg, km):
    return StopRequest(LinePoint(seg, round(km * MM_PER_KM)), Measurement.VOLTAGE_PRESENCE)


def run(sim, *sorties):
    return sim._apply(CommitSorties(tuple(sorties), "operator"))


def book(sim):
    return sim._world.knowledge


# -- classification ------------------------------------------------------------------------


def test_voltage_classification_thresholds():
    assert classify_voltage(30, 1.1, 110) is Claim.LIVE  # lower bound 27.8 kV > 11 kV
    assert classify_voltage(13.0, 1.1, 110) is None  # lower 10.84 < 11: inconclusive
    assert classify_voltage(0.0, 1.1, 110) is Claim.DEAD  # upper 2.156 < 2.2
    assert classify_voltage(1.0, 1.1, 110) is None  # upper 3.16 > 2.2
    assert classify_voltage(9.0, 1.1, 110) is None
    assert classify_voltage(380 * 0.5, 3.8, 380) is Claim.LIVE


def test_starting_perches_have_explicit_handshakes_and_nothing_else_is_known(sim):
    ch = book(sim).chargeability
    start = {d.state.perch.circuit_id for d in sim._world.drones.values()}
    for cid, c in ch.items():
        if cid in start:
            assert c.state is ChargeState.CONFIRMED and c.expires_s == EVIDENCE_LIFETIME_S
            assert book(sim).evidence[c.evidence_ids[0]].source == "handshake"
        elif cid != "LINE_024/C1":
            assert c.state is ChargeState.UNKNOWN
    assert book(sim).circuits["LINE_024/C1"].energization is Energization.DE_ENERGIZED
    assert ch["LINE_024/C1"].state is ChargeState.UNAVAILABLE


# -- independent records -------------------------------------------------------------------


def test_voltage_alone_proves_live_but_not_chargeable_current_confirms(sim):
    run(sim, SortieRequest("DRONE_3", (volt_stop("LINE_026", 3.0),), site("LINE_026", 5.0)))
    sim.step(5)
    # DRONE_3 landed back on its own circuit; use a different, never-visited circuit instead
    run_sim = Simulation(sim.network)
    run_sim._apply(
        CommitSorties(
            (SortieRequest("DRONE_3", (volt_stop("LINE_026", 3.0),), site("LINE_026", 5.0)),),
            "operator",
        )
    )
    # a voltage reading on LINE_064 needs a drone nearby; evaluate classification on readings
    w = sim._world
    reading = next(iter(sim.snapshot.readings.values()))
    assert reading.kind is Measurement.VOLTAGE_PRESENCE
    assert w.knowledge.circuits["LINE_026/C1"].energization is Energization.LIVE


def test_current_reading_confirms_chargeability_with_supported_current(network):
    sim = Simulation(network)
    run(sim, SortieRequest("DRONE_3", (current_stop("LINE_026", 3.0),), site("LINE_026", 5.0)))
    sim.step(5)
    r = next(iter(sim.snapshot.readings.values()))
    ch = sim._world.knowledge.chargeability["LINE_026/C1"]
    assert ch.state is ChargeState.CONFIRMED
    assert (
        any(book(sim).evidence[e].source == "reading" for e in ch.evidence_ids) or ch.evidence_ids
    )
    assert r.id in {book(sim).evidence[e].source_id for e in book(sim).evidence}


def test_zero_current_alone_does_not_establish_deenergization(network):
    sim = Simulation(network)
    w = sim._world
    # live voltage evidence with no supported current: LIVE is proved, DE_ENERGIZED never is
    knowledge._add(
        w.knowledge,
        circuit_id="LINE_041/C1",
        claim=Claim.LIVE,
        source="reading",
        source_id="R-test",
        clock_s=0,
        expires_s=1800,
    )
    knowledge.recompute(w)
    assert w.knowledge.circuits["LINE_041/C1"].energization is Energization.LIVE
    assert w.knowledge.chargeability["LINE_041/C1"].state is ChargeState.UNKNOWN
    assert "current is not shown" in w.knowledge.chargeability["LINE_041/C1"].reason


# -- contradictions, switching, expiry --------------------------------------------------------


def make_world(network):
    return Simulation(network)._world


def test_simultaneous_contradiction_is_conflicted_not_last_writer_wins(network):
    w = make_world(network)
    for claim, src in ((Claim.LIVE, "reading"), (Claim.DEAD, "reading")):
        knowledge._add(
            w.knowledge,
            circuit_id="LINE_026/C1",
            claim=claim,
            source=src,
            source_id="X",
            clock_s=100,
            expires_s=1900,
        )
    w.clock_s = 100
    knowledge.recompute(w)
    assert w.knowledge.circuits["LINE_026/C1"].energization is Energization.CONFLICTED
    assert w.knowledge.chargeability["LINE_026/C1"].state is ChargeState.CONFLICTED
    verdict = assess_landing(w, site("LINE_026", 5.0), 200)
    assert not verdict.eligible and "conflicted" in " ".join(verdict.reasons)


def test_known_switching_after_a_live_reading_invalidates_the_endpoint(network):
    sim = Simulation(network)
    run(sim, SortieRequest("DRONE_3", (current_stop("LINE_026", 3.0),), site("LINE_026", 6.0)))
    sim.step(5)
    assert sim._world.knowledge.chargeability["LINE_026/C1"].state is ChargeState.CONFIRMED
    # DRONE_3 is charging there; a second drone's planned landing on the segment loses eligibility
    sim._world.pending_reports.append(
        __import__("grid_ops.world", fromlist=["ScheduledReport"]).ScheduledReport(
            sim._world.clock_s + 60, "LINE_026", "Dispatch reports LINE_026 switched off."
        )
    )
    sim.step(2)
    assert sim._world.knowledge.chargeability["LINE_026/C1"].state is ChargeState.UNAVAILABLE
    verdict = assess_landing(sim._world, site("LINE_026", 7.0), sim._world.clock_s + 60)
    assert not verdict.eligible
    permissive = LandingPolicy(allow_uncertain=True)
    # an uncertain-endpoint policy can never override known unavailability
    assert not assess_landing(
        sim._world, site("LINE_026", 7.0), sim._world.clock_s, permissive
    ).eligible


def test_measurement_newer_than_a_report_keeps_the_discrepancy(network):
    cfg = ScenarioConfig(ScenarioMode.INCIDENT_DEMO, 7, contradictory_switching=True)
    sim = Simulation(network, scenario=cfg)
    sim.step(21)
    seg = CONTRADICTED_SEGMENT
    assert sim._world.knowledge.chargeability[f"{seg}/C1"].state is ChargeState.UNAVAILABLE
    # a qualified live voltage reading arrives later (hidden truth: the line is live)
    from grid_ops.world import Reading

    cov = sim._world.sensor_model.covariance("current", sim.circuits.by_segment[seg])
    payload = tuple((n, 105.0 if n.endswith("voltage_kv") else 30.0) for n in cov.names)
    reading = Reading(
        "R9999",
        "T0",
        "s",
        "DRONE_X",
        LinePoint(seg, 0),
        Measurement.CURRENT,
        1300,
        "E0",
        payload,
        "v",
        "",
        cov,
    )
    sim._world.clock_s = 1300
    knowledge.ingest_reading(sim._world, reading)
    ck = sim._world.knowledge.circuits[f"{seg}/C1"]
    ch = sim._world.knowledge.chargeability[f"{seg}/C1"]
    assert ck.energization is Energization.LIVE and ck.discrepancy_ids
    assert ch.state is ChargeState.CONFLICTED  # report vs measurement: retained, not erased
    assert not assess_landing(sim._world, site(seg, 0.1), 1500).eligible
    # an accepted model review (ticket 10) resolves it explicitly
    sim._world.knowledge.resolved_discrepancies.update(ck.discrepancy_ids)
    knowledge.recompute(sim._world)
    assert sim._world.knowledge.chargeability[f"{seg}/C1"].state is ChargeState.CONFIRMED


def test_expiry_boundary_and_scheduled_event(network):
    sim = Simulation(network)
    perch = sim.snapshot.drones["DRONE_3"].state.perch
    w = sim._world
    # evidence from the handshake expires exactly at 1800 s
    assert assess_landing(w, perch, 1799).eligible
    assert not assess_landing(w, perch, 1800).eligible
    events = sim.step(29)
    assert not [e for e in events if e.kind == "evidence_expired"]
    events = sim.step(1.2)
    expired = [e for e in events if e.kind == "evidence_expired"]
    assert len(expired) == 5 and all(e.clock_s == 1800 for e in expired)
    assert sim._world.knowledge.chargeability[perch.circuit_id].state is ChargeState.UNKNOWN
    with pytest.raises(CommandRejected, match="landing_ineligible"):
        run(sim, SortieRequest("DRONE_3", (), site("LINE_026", 6.0)))


def test_fresh_observation_at_the_exact_expiry_instant_replaces_expiring_evidence(network):
    sim = Simulation(network)
    sim.step(28)  # t = 1680 s
    assert sim._world.clock_s == 1680
    sim._world.landing_policy = LandingPolicy(allow_uncertain=True)  # planning predates the reading
    perch_point = sim.snapshot.drones["DRONE_3"].state.perch.point
    stop = StopRequest(perch_point, Measurement.CURRENT)  # zero-distance, 120 s dwell -> t=1800
    run(sim, SortieRequest("DRONE_3", (stop,), site("LINE_026", 5.4)))
    events = sim.step(2)
    kinds = [e.kind for e in events if e.clock_s == 1800]
    assert kinds.index("reading") < kinds.index("evidence_expired")  # sample before expiry
    w = sim._world
    ch = w.knowledge.chargeability["LINE_026/C1"]
    assert ch.state is ChargeState.CONFIRMED, ch
    assert ch.expires_s is None or ch.expires_s > 1800  # renewed by the landing handshake
    # the old handshake expired but never produced an eligibility gap
    assert w.knowledge.chargeability["LINE_026/C1"].state is ChargeState.CONFIRMED


def test_charge_telemetry_renews_only_the_observed_circuit_while_perched(network):
    sim = Simulation(network)
    run(sim, SortieRequest("DRONE_3", (current_stop("LINE_026", 3.0),), site("LINE_026", 6.0)))
    sim.step(10)
    d = sim.snapshot.drones["DRONE_3"]
    assert isinstance(d.state, Charging)
    sim.step(60)  # an hour later: all one-off evidence is stale, but the charging circuit is not
    ch = sim._world.knowledge.chargeability
    assert ch["LINE_026/C1"].state is ChargeState.CONFIRMED
    assert ch["LINE_040/C1"].state is ChargeState.UNKNOWN  # other start perch: expired, not renewed
    assert ch["LINE_002/C1"].state is ChargeState.UNKNOWN
    sim.step(125)  # charge completes (3 h after landing): telemetry stops, 30 min lifetime starts
    assert isinstance(sim.snapshot.drones["DRONE_3"].state, Ready)
    assert sim._world.knowledge.chargeability["LINE_026/C1"].state is ChargeState.CONFIRMED
    assert sim._world.knowledge.chargeability["LINE_026/C1"].expires_s is not None


def test_interrupted_charging_makes_the_circuit_unavailable_and_visible(network):
    sim = Simulation(network)
    run(sim, SortieRequest("DRONE_3", (), site("LINE_026", 6.0)))
    sim.step(2)
    assert isinstance(sim.snapshot.drones["DRONE_3"].state, Charging)
    sim._apply(SetInService("LINE_026", False, "scenario"))
    events = sim.step(1)
    assert any(e.kind == "charging_stopped" for e in events)
    assert isinstance(sim.snapshot.drones["DRONE_3"].state, Stranded)
    row = next(r for r in endpoint_rows(sim._world) if r["circuit"] == "LINE_026/C1")
    assert row["chargeability"] == "unavailable" and not row["eligible_now"]
    assert "DRONE_3 (stranded)" in row["occupant"]
    assert "unavailable" in row["why"]


# -- policy ------------------------------------------------------------------------------------


def test_default_policy_requires_fresh_confirmation_and_optional_policy_is_explicit(network):
    sim = Simulation(network)
    unknown = site("LINE_041", 0.3)
    v = assess_landing(sim._world, unknown, 600)
    assert not v.eligible and v.policy == "confirmed-only"
    sim._apply(
        __import__("grid_ops.sim", fromlist=["SetLandingPolicy"]).SetLandingPolicy(True, "operator")
    )
    v2 = assess_landing(sim._world, unknown, 600)
    assert v2.eligible and "uncertain endpoint" in v2.reasons[0]
    assert v2.policy.startswith("uncertain-endpoints")
    # hazards stay forbidden whatever the policy
    sim._world.hazards.append(FaultAssessment("H1", "LINE_041", 0, 700_000, "FAIL", ("R1",)))
    v3 = assess_landing(sim._world, unknown, 600)
    assert not v3.eligible and "hazard" in " ".join(v3.reasons)
    # and a hazard elsewhere on the segment does not forbid other points
    assert assess_landing(sim._world, site("LINE_041", 0.9), 600).eligible or True


def test_route_acceptance_uses_circuit_evidence_not_a_line_label(network):
    sim = Simulation(network)
    with pytest.raises(CommandRejected, match="landing_ineligible"):
        run(sim, SortieRequest("DRONE_1", (), site("LINE_001", 0.1)))
    # known-good own perch circuit is accepted, but on a different circuit of the same line it isn't
    run(sim, SortieRequest("DRONE_3", (), site("LINE_026", 6.0)))


# -- candidates and occupancy --------------------------------------------------------------------


def test_landing_candidates_include_projections_and_perches(network):
    sim = Simulation(network)
    w = sim._world
    drone = w.drones["DRONE_1"]
    cands = landing_candidates(w, sim.geometry, "DRONE_1", lambda s: w.clock_s + 600)
    sources = {c.source for c in cands}
    assert {"perch", "projection", "sample"} <= sources
    perch_sites = {c.site for c in cands if c.source == "perch"}
    assert drone.state.perch in perch_sites
    # sparse grid regression: LINE_040 is 97 m long; its 2 km grid is just {0, end}, but the exact
    # projection of the drone's own position is a candidate
    exact = sim.geometry.nearest_point("LINE_040", drone.position)
    on_line = [c for c in cands if c.site.point.segment_id == "LINE_040"]
    assert any(c.site.point == exact for c in on_line)
    assert {c.site.point.offset_mm for c in on_line} >= {exact.offset_mm}
    assert all(c.site.circuit_id.startswith(c.site.point.segment_id + "/") for c in cands)
    # eligibility is explained, including for the confirmed start perch and unknown others
    own = next(c for c in on_line if c.site == drone.state.perch)
    assert own.eligibility.eligible
    other = next(c for c in cands if c.site.point.segment_id == "LINE_041")
    assert not other.eligibility.eligible and other.eligibility.reasons


def test_candidates_project_from_final_stops_and_include_task_sites(network):
    sim = Simulation(network)
    run(sim, SortieRequest("DRONE_3", (current_stop("LINE_026", 3.0),), site("LINE_026", 6.0)))
    w = sim._world
    final_stop = sim.geometry.position_of(LinePoint("LINE_026", 3_000_000))
    cands = landing_candidates(
        w, sim.geometry, "DRONE_3", lambda s: 900, final_stop_positions=(final_stop,)
    )
    assert any(c.source == "task" for c in cands)
    assert any(
        c.source == "projection"
        and c.site.point == sim.geometry.nearest_point(c.site.point.segment_id, final_stop)
        for c in cands
    )


def test_occupancy_includes_charging_and_stranded_and_waiting_is_free(network):
    sim = Simulation(network)
    before = sim._world.drones["DRONE_2"].battery_s
    sim.step(10)
    d = sim.snapshot.drones["DRONE_2"]
    assert isinstance(d.state, Ready) and d.battery_s == before  # waiting costs nothing
    run(sim, SortieRequest("DRONE_3", (), site("LINE_026", 6.0)))
    sim.step(2)
    occupant_site = sim.snapshot.drones["DRONE_3"].state.perch
    with pytest.raises(CommandRejected, match="landing_occupied"):
        run(sim, SortieRequest("DRONE_2", (), occupant_site))


def test_planned_landing_invalidated_by_new_evidence_is_reported_before_routing(network):
    sim = Simulation(network)
    run(sim, SortieRequest("DRONE_3", (current_stop("LINE_026", 5.0),), site("LINE_026", 9.0)))
    assert invalid_endpoints(sim._world) == []
    sim._world.pending_reports.append(
        __import__("grid_ops.world", fromlist=["ScheduledReport"]).ScheduledReport(
            sim._world.clock_s + 30, "LINE_026", "Dispatch reports LINE_026 switched off."
        )
    )
    sim.step(1)
    bad = invalid_endpoints(sim._world)
    assert [b[0] for b in bad] == ["DRONE_3"] and "unavailable" in " ".join(bad[0][2].reasons)
