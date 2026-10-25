"""Atomic acceptance of explained decisions.

A decision carries identity, the evidence it was based on, the before-values it revises, any
sorties it wants and the measurement-request lifecycle it proposes. Acceptance first checks,
without any hold, that the proposal still means what it said (identity, semantic dependencies,
before-values, request references). It then takes a hold cycle, builds a private candidate
(hypotheses, explanations, model, accepted/retired requests) and validates every sortie against
the live fleet. Only if every check passes is anything applied, in one block. A failure discards
the candidate, attempts the same validated route-only continuation, and releases the cycle when
that continuation is safe.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, replace
from enum import StrEnum

from grid_ops import knowledge
from grid_ops.geometry import Geometry, haversine_km
from grid_ops.hypotheses import ManagerOutput
from grid_ops.proposals import ProposedStatus, tied_readings
from grid_ops.routes import SortieRequest, apply_sorties, current_requests, validate_sorties
from grid_ops.sensing import AcceptedRequest, RequestStatus
from grid_ops.world import (
    LAND_S,
    TAKEOFF_S,
    Charge,
    Executing,
    GroundSample,
    Land,
    Perched,
    ScheduledStep,
    Stranded,
    Takeoff,
    Transit,
    Wait,
    World,
    leg_seconds,
    position_at,
)


class Outcome(StrEnum):
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    SUPERSEDED = "SUPERSEDED"


@dataclass(frozen=True)
class Hold:
    """A bounded acceptance hold. Operator pause is a separate flag and is never touched."""

    cycle_id: int
    decision_id: str | None
    reason: str
    since_s: int


@dataclass(frozen=True)
class DecisionOutcome:
    decision_id: str
    outcome: Outcome
    reasons: tuple[str, ...]
    cycle_id: int | None
    fallback: str  # what happened to the fleet: "none needed" | "route-only continuation" | "HELD"
    clock_s: int


@dataclass(frozen=True)
class HypothesisRevisionRecord:
    """An accepted, explained revision. Never rewritten; the accepted hypothesis is derived."""

    id: str
    decision_id: str
    finding_id: str
    source_revision: int
    hypothesis_id: str
    before: dict | None
    after: dict
    change_kinds: tuple[str, ...]
    finding_summary: str
    update_reason: str
    remaining_uncertainty: str
    trigger_kind: str
    trigger_event_ids: tuple[str, ...]
    contributing_evidence_ids: tuple[str, ...]
    latest_measurement_ids: tuple[str, ...]
    diagnostic_ids: tuple[str, ...]
    clock_s: int


@dataclass(frozen=True)
class ManagerTurnRecord:
    id: str
    finding_id: str
    decision_id: str
    clock_s: int


class DecisionRejected(Exception):
    def __init__(self, reasons: list[str], *, superseded: bool = False) -> None:
        super().__init__("; ".join(reasons))
        self.reasons = reasons
        self.superseded = superseded


def _digest(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]


def hypothesis_hash(value: dict | None) -> str | None:
    return None if value is None else _digest(value)


def _referenced_request_ids(output: ManagerOutput) -> set[str]:
    ids = {r.id for r in output.measurement_requests}
    ids |= {r for req in output.measurement_requests for r in req.replaces_request_ids}
    ids |= set(output.retire_request_ids)
    return ids


def semantic_key(world: World, output: ManagerOutput) -> str:
    """What the explanation's meaning depends on. Pure drone motion is deliberately absent, so a
    proposal survives the fleet moving; changed evidence, epochs, model, request lifecycle or
    before-values do not."""
    tied: set[str] = set()
    for rev in output.revisions:
        tied |= set(tied_readings(world, rev.after.scope))
    referenced = _referenced_request_ids(output)
    return _digest(
        {
            "tied_readings": sorted(tied),
            "tied_epochs": {r: world.epoch_overrides.get(r) for r in sorted(tied)},
            "excluded": sorted(world.excluded_readings & tied),
            "known_switching": sorted(world.known_switching),
            "accepted_model": sorted(world.accepted_out_of_service),
            "sensor": world.capabilities.revision,
            "energy_model": world.energy_model.revision,
            "before": {
                rev.hypothesis_id: hypothesis_hash(world.hypotheses.get(rev.hypothesis_id))
                for rev in output.revisions
            },
            "referenced_requests": {
                rid: str(world.requests[rid].status) if rid in world.requests else "missing"
                for rid in sorted(referenced)
            },
        }
    )


# -- route-only continuation ----------------------------------------------------------------------


def landing_only_requests(
    world: World, geometry: Geometry, circuits
) -> tuple[tuple[SortieRequest, ...], list[str]]:
    """A minimal land-and-stay itinerary for every executing drone that still has steps left.

    Perched and stranded drones stay put and are omitted. The landing site must be free of known
    hazards; chargeability deliberately does not gate it, because a dead line is a valid perch."""
    taken: set = set()
    out, problems = [], []
    for d in sorted(world.drones.values(), key=lambda d: d.id):
        st = d.state
        if not isinstance(st, Executing) or st.step_index + 1 >= len(st.itinerary.steps):
            continue
        op = st.step.operation
        if isinstance(op, Wait) and not op.airborne:
            problems.append(f"{d.id}: perched wait carries no site to continue from")
            continue
        found = None
        if isinstance(op, Land | GroundSample | Charge) and _stay_ok(world, d.id, op.site, taken):
            req = _stay_request(world, d.id, st, op.site)
            if validate_sorties(world, geometry, circuits, (req,)).ok:
                found = req
        if found is None:
            for site in _safe_sites(world, geometry, d.id, taken):
                req = _landing_request(world, geometry, d.id, st, site)
                if req is not None and validate_sorties(world, geometry, circuits, (req,)).ok:
                    found = req
                    break
        if found is None:
            problems.append(f"{d.id}: no hazard-free reachable landing")
        else:
            taken.add(found.final_perch)
            out.append(found)
    return tuple(out), problems


def _stay_ok(world: World, drone_id: str, site, taken: set) -> bool:
    if _hazardous(world, site) or site in taken:
        return False
    for d in world.drones.values():
        if d.id == drone_id or not isinstance(d.state, Perched | Stranded):
            continue
        if d.state.perch == site:
            return False
    return True


def _hazardous(world: World, site) -> bool:
    return any(
        h.segment_id == site.point.segment_id
        and h.start_mm <= site.point.offset_mm <= h.end_mm
        for h in world.hazards
    )


def _stay_request(world: World, drone_id: str, st: Executing, site) -> SortieRequest:
    here = position_at(st.step, 1.0)
    steps = (ScheduledStep(0, st.step.end_s, st.step.end_s + 60, Wait(False, here)),)
    return SortieRequest(
        drone_id, steps, site, world.revision, world.energy_model.revision,
        world.capabilities.revision,
    )


def _safe_sites(world: World, geometry: Geometry, drone_id: str, taken: set):
    drone = world.drones[drone_id]
    candidates = knowledge.landing_candidates(
        world, geometry, drone_id, lambda site: world.clock_s + 1
    )
    ordered = sorted(
        candidates,
        key=lambda c: haversine_km(drone.position, geometry.position_of(c.site.point)),
    )
    for c in ordered:
        if _hazardous(world, c.site) or c.occupant or c.site in taken:
            continue
        yield c.site


def _landing_request(world: World, geometry: Geometry, drone_id: str, st: Executing, site):
    op = st.step.operation
    end_s = st.step.end_s
    here = position_at(st.step, 1.0)
    target = geometry.position_of(site.point)
    speed = world.drones[drone_id].speed_km_per_min
    if isinstance(op, Land | GroundSample | Charge):
        t1 = end_s + TAKEOFF_S
        leg = leg_seconds(here, target, speed)
        steps = (
            ScheduledStep(0, end_s, t1, Takeoff(op.site, here)),
            ScheduledStep(1, t1, t1 + max(leg, 1), Transit(here, target)),
            ScheduledStep(2, t1 + max(leg, 1), t1 + max(leg, 1) + LAND_S, Land(site, target)),
        )
    else:
        leg = leg_seconds(here, target, speed)
        steps = (
            ScheduledStep(0, end_s, end_s + max(leg, 1), Transit(here, target)),
            ScheduledStep(1, end_s + max(leg, 1), end_s + max(leg, 1) + LAND_S, Land(site, target)),
        )
    return SortieRequest(
        drone_id,
        steps,
        site,
        world.revision,
        world.energy_model.revision,
        world.capabilities.revision,
    )


def route_only_continuation(
    world: World, geometry: Geometry, circuits, physics
) -> tuple[str, list[str]]:
    """Keep every executing drone's remaining itinerary if the whole set still validates, else
    land them all. The world changes only when a complete valid continuation exists."""
    requests = current_requests(world)
    if not requests:
        return "none needed", []
    if validate_sorties(world, geometry, circuits, requests).ok:
        return "route-only continuation (unchanged routes)", []
    landing, problems = landing_only_requests(world, geometry, circuits)
    if problems or not landing:
        return "HELD", problems or ["no airborne continuation"]
    result = validate_sorties(world, geometry, circuits, landing)
    if not result.ok:
        return "HELD", [str(r) for r in result.rejections]
    apply_sorties(world, physics, result.plans)
    return "route-only continuation (landing-only)", []


# -- the transaction -------------------------------------------------------------------------------


def stage_model_change(world: World, segments: tuple[str, ...], author: str) -> str:
    sid = f"MC{len(world.staged_models) + 1:03d}"
    world.staged_models[sid] = {
        "segments": tuple(sorted(segments)),
        "author": author,
        "requested_s": world.clock_s,
    }
    world.record(
        author,
        f"Model change {sid} staged (accepted model unchanged): "
        f"{', '.join(sorted(segments)) or 'none'} out of service.",
    )
    return sid


def _test_candidate_model(sim_ctx, world: World, segments: frozenset[str]) -> list[str]:
    """No-op since the estimation removal: the three-method runtime runs no MW inference, so a
    staged model is never numerically tested against readings."""
    return []


def evaluate_commit(world: World, sim_ctx, geometry, circuits, cmd) -> dict:
    """Phase A: checks that need no hold. Raises DecisionRejected."""
    pf = next((p for p in world.proposals if p.id == cmd.finding_id), None)
    if pf is None:
        raise DecisionRejected([f"unknown finding {cmd.finding_id}"])
    if pf.status is not ProposedStatus.PROPOSED_NOT_ACCEPTED:
        raise DecisionRejected([f"{pf.id} is {pf.status}, not pending"])
    out = pf.output
    live_key = semantic_key(world, out)
    if live_key != pf.semantic_key or cmd.semantic_key != pf.semantic_key:
        raise DecisionRejected(
            [
                "evidence, epoch, model, request lifecycle or accepted hypotheses changed since "
                "the explanation was written; a fresh current-evidence turn is needed"
            ],
            superseded=True,
        )
    for rev in out.revisions:
        accepted = world.hypotheses.get(rev.hypothesis_id)
        want = None if rev.before is None else rev.before.model_dump(mode="json")
        if hypothesis_hash(accepted) != hypothesis_hash(want):
            raise DecisionRejected(
                [f"{rev.hypothesis_id}: before-values changed since the proposal"], superseded=True
            )
    return {"pf": pf}


def _request_candidate(
    world: World, out: ManagerOutput, cmd
) -> tuple[tuple[AcceptedRequest, ...], dict[str, str]]:
    accepted: list[AcceptedRequest] = []
    for req in out.measurement_requests:
        existing = world.requests.get(req.id)
        if existing is not None and existing.status is RequestStatus.ACTIVE:
            raise DecisionRejected([f"request {req.id} is already active"])
        accepted.append(
            AcceptedRequest(
                req.id, req, RequestStatus.ACTIVE, world.clock_s, str(req.source_revision)
            )
        )
    if len({a.id for a in accepted}) != len(accepted):
        raise DecisionRejected(["duplicate measurement request id in one output"])
    retire: dict[str, str] = {}
    for a in accepted:
        for old in a.request.replaces_request_ids:
            retire[old] = f"replaced by {a.id}"
    for rid in out.retire_request_ids:
        retire[rid] = f"retired by {cmd.decision_id}: {out.summary}"
    return tuple(accepted), retire


def apply_candidate(world: World, ctx: dict, cmd, cycle: int, clock_s: int) -> dict:
    """Build the private candidate. Returns what to apply; touches nothing."""
    pf = ctx["pf"]
    out = pf.output
    hypotheses = dict(world.hypotheses)
    revisions: list[HypothesisRevisionRecord] = []
    semantic_changed = False
    n = len(world.hypothesis_revisions)
    accepted_model = set(world.accepted_out_of_service)
    for rev in out.revisions:
        after = rev.after.model_dump(mode="json")
        before = hypotheses.get(rev.hypothesis_id)
        if before != after:
            semantic_changed = True
        hypotheses[rev.hypothesis_id] = after
        n += 1
        revisions.append(
            HypothesisRevisionRecord(
                f"REV{n:04d}",
                cmd.decision_id,
                pf.id,
                pf.snapshot_revision,
                rev.hypothesis_id,
                before,
                after,
                tuple(rev.change_kinds),
                rev.finding_summary,
                rev.update_reason,
                rev.remaining_uncertainty,
                rev.trigger_kind,
                tuple(rev.trigger_event_ids),
                tuple(rev.contributing_evidence_ids),
                tuple(rev.latest_measurement_ids),
                tuple(rev.diagnostic_ids),
                clock_s,
            )
        )
        if rev.after.model_change is not None:
            accepted_model |= set(rev.after.model_change)
    accepted_requests, retire = _request_candidate(world, out, cmd)
    return {
        "hypotheses": hypotheses,
        "revisions": revisions,
        "semantic_changed": semantic_changed,
        "accepted_model": frozenset(accepted_model),
        "model_changes": [
            r.after.model_change for r in out.revisions if r.after.model_change is not None
        ],
        "accepted_requests": accepted_requests,
        "retire": retire,
    }


def recovery_requests(world: World, geometry, circuits, physics) -> tuple[str, list[str]]:
    """A concluded case stops collecting: every executing drone keeps its validated continuation,
    and if that no longer validates it lands. Perched drones are left alone."""
    return route_only_continuation(world, geometry, circuits, physics)


def commit_apply(world: World, cand: dict, ctx: dict, cmd, plans, physics=None) -> None:
    """The single block that mutates accepted state. No validation happens here."""
    pf = ctx["pf"]
    from grid_ops.conclusion import Investigation, InvestigationStatus

    out = pf.output

    inv = world.investigation
    if out.conclusion is not None and out.conclusion.kind == "cause_identified":
        world.investigation = Investigation(
            InvestigationStatus.CONCLUDED,
            out.conclusion.model_dump(mode="json"),
            cmd.decision_id,
            world.clock_s,
            (*inv.history, (world.clock_s, f"concluded by {cmd.decision_id}")),
        )
    elif out.conclusion is not None:
        rec = next(
            (
                r
                for r in out.recommendations
                if r.id == out.conclusion.discriminating_recommendation_id
            ),
            None,
        )
        world.investigation = Investigation(
            inv.status,
            inv.conclusion,
            inv.decision_id,
            inv.concluded_s,
            (*inv.history, (world.clock_s, f"ambiguous: {out.conclusion.causal_statement}")),
            inv.reopen_flags,
            f"{out.conclusion.remaining_uncertainty} Discriminating step: "
            f"{rec.text if rec else 'see recommendation'}",
        )
    if out.reopen is not None:
        world.investigation = Investigation(
            InvestigationStatus.OPEN,
            inv.conclusion,
            inv.decision_id,
            inv.concluded_s,
            (*inv.history, (world.clock_s, f"reopened by {cmd.decision_id}: {out.reopen.reason}")),
        )
    world.hypotheses = cand["hypotheses"]
    world.hypothesis_revisions.extend(cand["revisions"])
    if cand["semantic_changed"]:
        world.hypotheses_version += 1
    world.accepted_out_of_service = cand["accepted_model"]
    for segs in cand["model_changes"]:
        for seg in segs:
            for c in world.circuits_by_segment.get(seg, ()):
                ck = world.knowledge.circuits.get(c.id)
                if ck and ck.discrepancy_ids:
                    world.knowledge.resolved_discrepancies.update(ck.discrepancy_ids)
    for acc in cand["accepted_requests"]:
        world.requests[acc.id] = acc
    for rid, reason in cand["retire"].items():
        if rid in world.requests:
            world.requests[rid] = replace(
                world.requests[rid], status=RequestStatus.RETIRED, retired_reason=reason
            )
    if cand["accepted_requests"] or cand["retire"]:
        world.requests_version += 1
    apply_sorties(world, physics, plans)
    world.manager_turns.append(
        ManagerTurnRecord(
            f"MT{len(world.manager_turns) + 1:03d}", pf.id, cmd.decision_id, world.clock_s
        )
    )
    idx = world.proposals.index(pf)
    world.proposals[idx] = replace(
        pf,
        status=ProposedStatus.ACCEPTED,
        history=(*pf.history, (world.clock_s, f"accepted by {cmd.decision_id}")),
    )


def note_proposal(world: World, finding_id: str, status: ProposedStatus | None, note: str) -> None:
    for i, p in enumerate(world.proposals):
        if p.id == finding_id:
            world.proposals[i] = replace(
                p, status=status or p.status, history=(*p.history, (world.clock_s, note))
            )
            return
