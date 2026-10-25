"""Public operational knowledge: what the operator can prove about each circuit.

Three independent records per circuit, each with evidence provenance: energization
(`CircuitKnowledge`), charging capability (`Chargeability`) and, from ticket 06, fault and
coverage records. Nothing here reads hidden physics; every claim comes from a public report, a
qualified drone reading or a charging handshake. A combined line status label is never used to
authorise a landing.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from enum import StrEnum
from typing import TYPE_CHECKING

from grid_ops.geometry import Geometry, LandingSite, LinePoint

if TYPE_CHECKING:
    from grid_ops.world import World

EVIDENCE_LIFETIME_S = 30 * 60
PUBLIC_EPOCH = "E0"


class Energization(StrEnum):
    UNKNOWN = "unknown"
    LIVE = "live"
    DE_ENERGIZED = "de_energized"
    CONFLICTED = "conflicted"


class ChargeState(StrEnum):
    CONFIRMED = "confirmed"
    UNKNOWN = "unknown"
    UNAVAILABLE = "unavailable"
    CONFLICTED = "conflicted"


class Claim(StrEnum):
    LIVE = "live"  # 95% voltage lower bound above 10% of nominal
    DEAD = "dead"  # 95% voltage upper bound below 2% of nominal, or a switching report
    CHARGE_OK = "charge_ok"  # live and supported current >= 5 A, or a successful handshake
    CHARGE_FAIL = "charge_fail"  # failed handshake or charging stopped


@dataclass(frozen=True)
class Evidence:
    id: str
    circuit_id: str
    claim: Claim
    source: str  # "reading" | "report" | "handshake" | "telemetry"
    source_id: str
    clock_s: int
    expires_s: int | None  # None: persists (a report) or renewed while charging
    detail: str = ""
    renewed_while_charging: bool = False


@dataclass(frozen=True)
class CircuitKnowledge:
    circuit_id: str
    energization: Energization = Energization.UNKNOWN
    evidence_ids: tuple[str, ...] = ()
    observed_s: int | None = None
    expires_s: int | None = None
    public_epoch: str = PUBLIC_EPOCH
    discrepancy_ids: tuple[str, ...] = ()  # older evidence a newer measurement contradicts


@dataclass(frozen=True)
class Chargeability:
    circuit_id: str
    state: ChargeState = ChargeState.UNKNOWN
    evidence_ids: tuple[str, ...] = ()
    observed_s: int | None = None
    expires_s: int | None = None
    reason: str = "no evidence"


@dataclass(frozen=True)
class FaultAssessment:
    """Local condition findings (ticket 06). Independent of energization and chargeability."""

    id: str
    segment_id: str
    start_mm: int
    end_mm: int
    verdict: str
    evidence_ids: tuple[str, ...]
    assumptions: tuple[str, ...] = ()


@dataclass(frozen=True)
class LandingPolicy:
    """Default: freshly confirmed, hazard-free, model-compatible. Uncertain endpoints are an
    explicit opt-in with a documented risk; known unavailability and hazards are never allowed."""

    allow_uncertain: bool = False
    revision: str = "landing-default-v1"

    @property
    def name(self) -> str:
        return "uncertain-endpoints (risk accepted)" if self.allow_uncertain else "confirmed-only"


@dataclass(frozen=True)
class Eligibility:
    eligible: bool
    reasons: tuple[str, ...]
    policy: str


@dataclass
class KnowledgeBook:
    """Evidence and the records derived from it. Lives in the public World."""

    evidence: dict[str, Evidence] = field(default_factory=dict)
    circuits: dict[str, CircuitKnowledge] = field(default_factory=dict)
    chargeability: dict[str, Chargeability] = field(default_factory=dict)
    resolved_discrepancies: set[str] = field(default_factory=set)
    expired_notified: set[str] = field(default_factory=set)
    next_number: int = 1


# -- evidence ingestion ------------------------------------------------------------------------


def _add(book: KnowledgeBook, **kw) -> Evidence:
    e = Evidence(id=f"EV{book.next_number:04d}", **kw)
    book.next_number += 1
    book.evidence[e.id] = e
    return e


def ingest_reading(world: World, reading) -> list[Evidence]:
    """Three-method readings carry no energization evidence: current magnitude alone never
    establishes live or de-energized, and zero current is never proof of isolation."""
    return []


def ingest_report(world: World, segment_id: str, clock_s: int, text: str) -> None:
    """A public switching report: every circuit of the segment is stated de-energized."""
    for c in world.circuits_by_segment[segment_id]:
        _add(
            world.knowledge,
            circuit_id=c.id,
            claim=Claim.DEAD,
            source="report",
            source_id=segment_id,
            clock_s=clock_s,
            expires_s=None,
            detail=text,
        )
    recompute(world)


def ingest_handshake(
    world: World, circuit_id: str, ok: bool, source_id: str, charging: bool
) -> None:
    """A charging handshake on landing. A success while charging is renewed by its telemetry."""
    t = world.clock_s
    _add(
        world.knowledge,
        circuit_id=circuit_id,
        claim=Claim.CHARGE_OK if ok else Claim.CHARGE_FAIL,
        source="handshake",
        source_id=source_id,
        clock_s=t,
        expires_s=None if (ok and charging) else t + EVIDENCE_LIFETIME_S,
        detail="charging handshake " + ("succeeded" if ok else "failed"),
        renewed_while_charging=ok and charging,
    )
    recompute(world)


def end_charge_telemetry(world: World, circuit_id: str) -> None:
    """Charging stopped being observed: telemetry no longer renews the circuit's evidence."""
    book = world.knowledge
    for e in list(book.evidence.values()):
        if e.circuit_id == circuit_id and e.renewed_while_charging and e.expires_s is None:
            book.evidence[e.id] = replace(
                e, renewed_while_charging=False, expires_s=world.clock_s + EVIDENCE_LIFETIME_S
            )
    recompute(world)


# -- derivation --------------------------------------------------------------------------------


def _charging_circuits(world: World) -> set[str]:
    from grid_ops.world import Charge, Executing

    out = set()
    for d in world.drones.values():
        st = d.state
        if isinstance(st, Executing):
            op = st.step.operation
            if isinstance(op, Charge):
                out.add(op.site.circuit_id)
    return out


def _valid(world: World, e: Evidence, charging: set[str]) -> bool:
    if e.expires_s is None:
        return True
    if e.renewed_while_charging and e.circuit_id in charging:
        return True
    return e.expires_s > world.clock_s


def next_expiry(world: World) -> int | None:
    charging = _charging_circuits(world)
    times = [
        e.expires_s
        for e in world.knowledge.evidence.values()
        if e.expires_s is not None
        and e.expires_s > world.clock_s
        and not (e.renewed_while_charging and e.circuit_id in charging)
    ]
    return min(times) if times else None


def recompute(world: World) -> None:
    """Rebuild every circuit's records from the evidence still valid at world.clock_s."""
    book = world.knowledge
    charging = _charging_circuits(world)
    by_circuit: dict[str, list[Evidence]] = {}
    for e in book.evidence.values():
        if _valid(world, e, charging):
            by_circuit.setdefault(e.circuit_id, []).append(e)
    for cs in world.circuits_by_segment.values():
        for c in cs:
            ev = by_circuit.get(c.id, [])
            book.circuits[c.id], book.chargeability[c.id] = _derive(book, c.id, ev)


def _latest(ev: list[Evidence], *claims: Claim) -> list[Evidence]:
    picked = [e for e in ev if e.claim in claims]
    if not picked:
        return []
    t = max(e.clock_s for e in picked)
    return [e for e in picked if e.clock_s == t]


def _derive(
    book: KnowledgeBook, cid: str, ev: list[Evidence]
) -> tuple[CircuitKnowledge, Chargeability]:
    live = _latest(ev, Claim.LIVE, Claim.CHARGE_OK)
    dead = _latest(ev, Claim.DEAD)
    fail = _latest(ev, Claim.CHARGE_FAIL)
    ok = _latest(ev, Claim.CHARGE_OK)
    discrepancy: tuple[str, ...] = ()

    def ids(*groups) -> tuple[str, ...]:
        return tuple(sorted({e.id for g in groups for e in g}))

    if live and dead:
        t_live, t_dead = live[0].clock_s, dead[0].clock_s
        if t_live == t_dead:
            state, used = Energization.CONFLICTED, ids(live, dead)
        elif t_live > t_dead:
            state, used = Energization.LIVE, ids(live)
            reports = [e for e in dead if e.source == "report"]
            if reports and not set(ids(reports)) <= book.resolved_discrepancies:
                discrepancy = ids(reports)
        else:
            state, used = Energization.DE_ENERGIZED, ids(dead)
    elif live:
        state, used = Energization.LIVE, ids(live)
    elif dead:
        state, used = Energization.DE_ENERGIZED, ids(dead)
    else:
        state, used = Energization.UNKNOWN, ()
    stamp = max((e.clock_s for e in ev), default=None)
    expiries = [e.expires_s for e in ev if e.id in used and e.expires_s is not None]
    knowledge = CircuitKnowledge(
        cid,
        state,
        used,
        stamp,
        min(expiries) if expiries else None,
        PUBLIC_EPOCH,
        discrepancy,
    )

    # chargeability
    if fail and (not ok or fail[0].clock_s >= ok[0].clock_s):
        charge = Chargeability(
            cid,
            ChargeState.UNAVAILABLE,
            ids(fail),
            fail[0].clock_s,
            None,
            "charging failed or stopped here",
        )
    elif state is Energization.DE_ENERGIZED:
        charge = Chargeability(
            cid,
            ChargeState.UNAVAILABLE,
            used,
            knowledge.observed_s,
            None,
            "circuit is de-energized",
        )
    elif state is Energization.CONFLICTED:
        charge = Chargeability(
            cid,
            ChargeState.CONFLICTED,
            used,
            knowledge.observed_s,
            None,
            "contradictory qualified evidence at the same time",
        )
    elif discrepancy:
        charge = Chargeability(
            cid,
            ChargeState.CONFLICTED,
            ids(live) + discrepancy,
            knowledge.observed_s,
            None,
            "measurement contradicts the switching report; model review pending",
        )
    elif ok:
        exp = [e.expires_s for e in ok if e.expires_s is not None]
        charge = Chargeability(
            cid,
            ChargeState.CONFIRMED,
            ids(ok),
            ok[0].clock_s,
            min(exp) if exp else None,
            "live with supported current or a successful handshake",
        )
    elif state is Energization.LIVE:
        charge = Chargeability(
            cid,
            ChargeState.UNKNOWN,
            used,
            knowledge.observed_s,
            None,
            "live, but current is not shown to support charging",
        )
    else:
        charge = Chargeability(cid, ChargeState.UNKNOWN, (), None, None, "no fresh evidence")
    return knowledge, charge


# -- eligibility -------------------------------------------------------------------------------


def assess_perch(world: World, site: LandingSite) -> Eligibility:
    """Mechanical landing safety only: known hazards over the point. Chargeability never gates a
    perch, because a dead line is a valid perch; only Charge steps need assess_landing."""
    for h in world.hazards:
        if h.segment_id == site.point.segment_id and h.start_mm <= site.point.offset_mm <= h.end_mm:
            return Eligibility(
                False, (f"known landing hazard {h.id} ({h.verdict})",), world.landing_policy.name
            )
    return Eligibility(True, ("mechanically safe perch",), world.landing_policy.name)


def assess_landing(
    world: World, site: LandingSite, arrival_s: int, policy: LandingPolicy | None = None
) -> Eligibility:
    """Is this exact circuit an executable recharge endpoint at the predicted arrival time?"""
    policy = policy or world.landing_policy
    ch = world.knowledge.chargeability.get(site.circuit_id)
    if ch is None:
        return Eligibility(False, (f"unknown circuit {site.circuit_id}",), policy.name)
    reasons: list[str] = []
    hard = False
    for h in world.hazards:
        if h.segment_id == site.point.segment_id and h.start_mm <= site.point.offset_mm <= h.end_mm:
            reasons.append(f"known landing hazard {h.id} ({h.verdict})")
            hard = True
    if ch.state is ChargeState.UNAVAILABLE:
        reasons.append(f"charging unavailable: {ch.reason}")
        hard = True
    if ch.state is ChargeState.CONFLICTED:
        reasons.append(f"conflicted evidence: {ch.reason}")
    if ch.state is ChargeState.UNKNOWN:
        reasons.append(f"not confirmed: {ch.reason}")
    if ch.state is ChargeState.CONFIRMED and ch.expires_s is not None and ch.expires_s <= arrival_s:
        reasons.append(
            f"evidence expires at {_hms(ch.expires_s)}, before predicted arrival {_hms(arrival_s)}"
        )
    if not reasons:
        return Eligibility(True, ("freshly confirmed through arrival",), policy.name)
    if policy.allow_uncertain and not hard:
        return Eligibility(
            True, ("uncertain endpoint accepted by policy: " + "; ".join(reasons),), policy.name
        )
    return Eligibility(False, tuple(reasons), policy.name)


def _hms(seconds: int) -> str:
    return f"T+{seconds // 3600:02d}:{seconds % 3600 // 60:02d}"


# -- landing candidates ------------------------------------------------------------------------


@dataclass(frozen=True)
class LandingCandidate:
    site: LandingSite
    source: str  # perch | task | sample | projection
    eligibility: Eligibility
    occupant: str | None = None


def landing_candidates(
    world: World,
    geometry: Geometry,
    drone_id: str,
    arrival_s_for: callable,
    final_stop_positions: tuple[tuple[float, float], ...] = (),
) -> list[LandingCandidate]:
    """Candidate recharge endpoints for a drone: current perches, task sites, a coarse sampled
    grid, and exact projections of the drone and of every candidate final stop onto each
    reachable segment. Every candidate names its circuit; ineligible ones are kept, explained."""
    from grid_ops.world import Executing, Perched, Stranded

    drone = world.drones[drone_id]
    reach_km = drone.battery.remaining_units / 600 * drone.speed_km_per_min
    seen: dict[LandingSite, str] = {}

    def add(point: LinePoint, source: str) -> None:
        for c in world.circuits_by_segment[point.segment_id]:
            seen.setdefault(LandingSite(point, c.id), source)

    for d in world.drones.values():
        st = d.state
        if isinstance(st, Perched | Stranded):
            seen.setdefault(st.perch, "perch")
        elif isinstance(st, Executing):
            seen.setdefault(st.itinerary.final_perch, "perch")
    for t in world.tasks.values():
        from grid_ops.sensing import ContactCurrent, LidarPass, TdrTest

        a = t.acquisition
        if isinstance(a, ContactCurrent | TdrTest):
            add(a.conductor.point, "task")
        elif isinstance(a, LidarPass):
            add(LinePoint(a.target.segment_id, a.target.start_mm), "task")
            add(LinePoint(a.target.segment_id, a.target.end_mm), "task")
    origins = [drone.position, *final_stop_positions]
    for seg in world.circuits_by_segment:
        for o in origins:
            p = geometry.nearest_point(seg, o)
            from grid_ops.geometry import haversine_km

            if haversine_km(o, geometry.position_of(p)) <= reach_km:
                add(p, "projection")
        for p in geometry.landing_points(seg):
            if haversine_km(drone.position, geometry.position_of(p)) <= reach_km:
                add(p, "sample")
    occupied = {}
    for d in world.drones.values():
        if isinstance(d.state, Perched | Stranded):
            occupied[d.state.perch] = d.id
    out = []
    for site, source in seen.items():
        out.append(LandingCandidate(site, source, assess_perch(world, site), occupied.get(site)))
    return sorted(out, key=lambda c: (c.site.circuit_id, c.site.point.offset_mm))


def invalid_endpoints(world: World) -> list[tuple[str, LandingSite, Eligibility]]:
    """Planned landings that are no longer eligible (route continuation must be replaced)."""
    from grid_ops.world import Executing

    out = []
    for d in world.drones.values():
        st = d.state
        if isinstance(st, Executing):
            e = assess_perch(world, st.itinerary.final_perch)
            if not e.eligible:
                out.append((d.id, st.itinerary.final_perch, e))
    return out
