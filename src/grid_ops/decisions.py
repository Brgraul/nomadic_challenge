"""Atomic acceptance of explained decisions.

A decision carries identity, the evidence it was based on, the before-values it revises and any
sorties it wants. Acceptance first checks, without any hold, that the proposal still means what it
said (identity, semantic dependencies, before-values). It then takes a hold cycle, builds a private
candidate (hypotheses, explanations, model) and validates the candidate model and every sortie
against the live fleet. Only if every check passes is anything applied, in one block. A failure
discards the candidate, attempts the same validated route-only continuation, and releases the
cycle when that continuation is safe.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, replace
from enum import StrEnum

from grid_ops import estimation, knowledge
from grid_ops.consistency import GlobalStatus, check_consistency
from grid_ops.hypotheses import ManagerOutput
from grid_ops.proposals import ProposedStatus, tied_readings
from grid_ops.routes import SortieRequest, StopRequest, apply_sorties, validate_sorties
from grid_ops.world import Flying, Measuring, World


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


def semantic_key(world: World, output: ManagerOutput) -> str:
    """What the explanation's meaning depends on. Pure drone motion is deliberately absent, so a
    proposal survives the fleet moving; changed evidence, epochs, model or before-values do not."""
    tied: set[str] = set()
    for rev in output.revisions:
        tied |= set(tied_readings(world, rev.after.scope))
    return _digest(
        {
            "tied_readings": sorted(tied),
            "tied_epochs": {r: world.epoch_overrides.get(r) for r in sorted(tied)},
            "excluded": sorted(world.excluded_readings & tied),
            "electrical_readings": sorted(
                r.id for r in world.readings.values() if r.kind == "current"
            ),
            "known_switching": sorted(world.known_switching),
            "accepted_model": sorted(world.accepted_out_of_service),
            "sensor": [world.sensor_model.revision, world.sensor_model.shared_rho],
            "before": {
                rev.hypothesis_id: hypothesis_hash(world.hypotheses.get(rev.hypothesis_id))
                for rev in output.revisions
            },
        }
    )


# -- route-only continuation ----------------------------------------------------------------------


def current_requests(world: World, only: set[str] | None = None) -> tuple[SortieRequest, ...]:
    """The current continuation of every airborne drone, expressed as validatable requests."""
    out = []
    for d in world.drones.values():
        st = d.state
        if not isinstance(st, Flying | Measuring) or (only is not None and d.id not in only):
            continue
        stops = tuple(
            StopRequest(s.point, s.kind, world.tasks[s.task_id].purpose, None, s.task_id)
            for s in st.route.stops
        )
        out.append(SortieRequest(d.id, stops, st.route.perch))
    return tuple(out)


def landing_only_requests(
    world: World, geometry, circuits
) -> tuple[tuple[SortieRequest, ...], list[str]]:
    """A landing-only plan for every airborne drone under the accepted landing policy."""
    taken: set = set()
    out, problems = [], []
    for d in world.drones.values():
        if not isinstance(d.state, Flying | Measuring):
            continue
        found = None
        cands = knowledge.landing_candidates(world, geometry, d.id, lambda site: world.clock_s + 1)
        for c in cands:
            if not c.eligibility.eligible or c.occupant or c.site in taken:
                continue
            req = SortieRequest(d.id, (), c.site)
            if validate_sorties(world, geometry, circuits, (req,)).ok:
                found = req
                break
        if found is None:
            problems.append(f"{d.id}: no eligible reachable landing")
        else:
            taken.add(found.landing)
            out.append(found)
    return tuple(out), problems


def route_only_continuation(world: World, geometry, circuits) -> tuple[str, list[str]]:
    """Keep every airborne drone's route if the whole set still validates, else land them all.

    Returns (what happened, problems). The world is changed only when a complete valid
    continuation exists; otherwise nothing changes and the caller stays held."""
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
    apply_sorties(world, result.plans)
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
    """Independently test a staged model against the live accepted readings."""
    network, registry = sim_ctx
    model = estimation.ElectricalModel(
        network, registry, frozenset(world.known_switching) | segments
    )
    readings = estimation.effective_readings(world)
    est = estimation.estimate(
        model, readings, clock_s=world.clock_s, shared_rho=world.sensor_model.shared_rho
    )
    if not hasattr(est, "flows"):
        return [f"candidate model: estimate unavailable ({est.reason})"]
    rep = check_consistency(
        model,
        readings,
        shared_rho=world.sensor_model.shared_rho,
        envelope=world.envelope,
        known_switching_clock=dict(world.known_switching),
        active_seed=world.scenario_seed,
    )
    if rep.global_status is GlobalStatus.INCONSISTENT:
        return ["candidate model is inconsistent with the accepted readings"]
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
                "evidence, epoch, model, sensor assumptions or accepted hypotheses changed since "
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
    return {
        "hypotheses": hypotheses,
        "revisions": revisions,
        "semantic_changed": semantic_changed,
        "accepted_model": frozenset(accepted_model),
        "model_changes": [
            r.after.model_change for r in out.revisions if r.after.model_change is not None
        ],
    }


def recovery_requests(world: World, geometry, circuits):
    """Concluded cases stop collecting: every airborne drone keeps only its locked attempt and
    lands. Returns (requests, problems); the plan must validate as a whole or the decision fails."""
    problems: list[str] = []
    airborne = [d for d in world.drones.values() if isinstance(d.state, Flying | Measuring)]
    if not airborne:
        return (), []
    current = {r.drone_id: r for r in current_requests(world)}
    keep, need_landing = [], []
    for d in airborne:
        req = current[d.id]
        stripped = SortieRequest(d.id, (), req.landing)
        if validate_sorties(world, geometry, circuits, (stripped,)).ok:
            keep.append(stripped)
        else:
            need_landing.append(d.id)
    if need_landing:
        landing, problems = landing_only_requests(world, geometry, circuits)
        by = {r.drone_id: r for r in landing}
        for did in need_landing:
            if did in by:
                keep.append(by[did])
            else:
                problems.append(f"{did}: no eligible landing for its recovery route")
    return tuple(keep), problems


def commit_apply(world: World, cand: dict, ctx: dict, cmd, plans) -> None:
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
    apply_sorties(world, plans)
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
