"""Plain-data views of a public World for the console and for tests.

Three-method rendering only: lidar / tdr / contact_current payloads, itinerary phases and
battery units. Electrical inference (estimates, consistency, envelopes, MW) and the legacy
dwell/charging model have no view here.
"""

from __future__ import annotations

from grid_ops.sensing import (
    CurrentPayload,
    LidarPayload,
    RequestStatus,
    TdrPayload,
)
from grid_ops.world import (
    Charge,
    Executing,
    GroundSample,
    Land,
    Perched,
    Scan,
    Stranded,
    Takeoff,
    Transit,
    Wait,
    World,
    format_clock,
    position_at,
)


def _t(seconds: int) -> str:
    return format_clock(seconds / 60)


def _op_label(op) -> str:
    """A short human label for one scheduled operation."""
    match op:
        case Transit(origin, destination):
            return (
                f"transit to {destination[0]:.4f},{destination[1]:.4f}"
                if origin != destination
                else "transit"
            )
        case Scan(acquisition=acq, task_id=task_id):
            return f"lidar scan {task_id} @ {acq.target.segment_id}"
        case Land(site):
            return f"land {site.circuit_id}"
        case Takeoff(site):
            return f"takeoff {site.circuit_id}"
        case GroundSample(task_ids=task_ids):
            return f"ground sample {', '.join(task_ids)}"
        case Charge(site):
            return f"charge {site.circuit_id}"
        case Wait(airborne=airborne):
            return "wait airborne" if airborne else "wait perched"
    return type(op).__name__.lower()


def drone_rows(world: World) -> list[dict]:
    """Per drone: phase, position, battery units and % of capacity, the current operation with
    its step bounds, the remaining itinerary length and the completed-task count. A landing
    never adds energy and no charging state exists."""
    rows = []
    for d in sorted(world.drones.values(), key=lambda d: d.id):
        row = {
            "drone": d.id,
            "phase": d.phase,
            "position": f"{d.position[0]:.4f},{d.position[1]:.4f}",
            "battery_units": d.battery.remaining_units,
            "battery_pct": round(100 * d.battery.remaining_units / d.battery.capacity_units, 1),
            "operation": "",
            "step_end": "",
            "itinerary_left": 0,
            "tasks_done": len(d.history),
        }
        match d.state:
            case Perched(perch):
                row["operation"] = f"perched {perch.circuit_id}"
            case Stranded(perch):
                row["operation"] = f"stranded {perch.circuit_id}"
            case Executing(itinerary=it, step_index=i):
                step = it.steps[i]
                row["operation"] = _op_label(step.operation)
                row["step_end"] = _t(step.end_s)
                row["itinerary_left"] = len(it.steps) - i
        rows.append(row)
    return rows


def route_rows(world: World) -> list[dict]:
    """The live itinerary of every executing drone: its remaining steps and final perch."""
    rows = []
    for d in sorted(world.drones.values(), key=lambda d: d.id):
        match d.state:
            case Executing(itinerary=it, step_index=i):
                remaining = it.steps[i:]
            case _:
                continue
        rows.append(
            {
                "drone": d.id,
                "locked": f"step {i + 1}/{len(it.steps)} ends {_t(it.steps[i].end_s)}",
                "steps": " > ".join(
                    f"{_t(s.start_s)}-{_t(s.end_s)} {_op_label(s.operation)}" for s in remaining
                ),
                "final_perch": f"{it.final_perch.circuit_id} "
                f"@ {it.final_perch.point.offset_km:.3f} km",
            }
        )
    return rows


def itinerary_rows(world: World, drone_id: str) -> list[dict]:
    """Every step of one drone's itinerary with its anchor position and task ids."""
    d = world.drones.get(drone_id)
    if d is None or not isinstance(d.state, Executing):
        return []
    rows = []
    for step in d.state.itinerary.steps:
        anchor = position_at(step, 1.0)
        rows.append(
            {
                "step": step.index,
                "window": f"{_t(step.start_s)}-{_t(step.end_s)}",
                "operation": _op_label(step.operation),
                "at": f"{anchor[0]:.4f},{anchor[1]:.4f}",
            }
        )
    return rows


def task_rows(world: World) -> list[dict]:
    return [
        {
            "task": t.id,
            "method": str(t.acquisition.method),
            "request": t.request_id,
            "status": t.status,
            "drone": t.assigned_drone_id or "",
            "urgency": t.urgency,
            "group": t.alternative_group_id,
            "repeat_of": t.repeat_of_task_id or "",
            "reading": t.reading_id or "",
            "note": t.note,
        }
        for t in sorted(world.tasks.values(), key=lambda t: t.id)
    ]


def _target_summary(request) -> str:
    if request.method == "lidar":
        target = request.target
        return f"{target.segment_id} span {target.start_mm / 1e6:.3f}-{target.end_mm / 1e6:.3f} km"
    target = request.target
    return (
        f"{target.circuit_id} phase {target.phase} "
        f"{target.start_mm / 1e6:.3f}-{target.end_mm / 1e6:.3f} km"
    )


def request_rows(world: World) -> list[dict]:
    """Accepted requests with their compiled tasks and the explicit unresolved flag."""
    from grid_ops import scoring

    unresolved = dict(scoring.unresolved_requests(world))
    tasks_by_request: dict[str, list[str]] = {}
    for t in world.tasks.values():
        tasks_by_request.setdefault(t.request_id, []).append(t.id)
    rows = []
    for a in sorted(world.requests.values(), key=lambda a: a.id):
        unresolved_now = a.status is RequestStatus.ACTIVE and a.id in unresolved
        rows.append(
            {
                "request": a.id,
                "method": str(a.request.method),
                "target": _target_summary(a.request),
                "status": a.status,
                "urgency": a.request.urgency,
                "rationale": a.request.rationale[:120],
                "tasks": ", ".join(sorted(tasks_by_request.get(a.id, []))),
                "unresolved": "EXPLICITLY UNRESOLVED" if unresolved_now else "",
                "retired_reason": a.retired_reason,
            }
        )
    return rows


def reading_rows(world: World) -> list[dict]:
    """Payload-type-specific rows. Voltage, P/Q, power direction, RGB/IR and temperature have
    no column: the three methods do not produce them."""
    rows = []
    for r in sorted(world.readings.values(), key=lambda r: r.id):
        quality = "valid" if r.quality.valid else f"invalid: {r.quality.reason}"
        row = {
            "reading": r.id,
            "time": _t(r.acquired_start_s),
            "drone": r.drone_id,
            "task": r.task_id,
            "requests": ", ".join(r.request_ids),
            "method": str(r.acquisition.method),
            "quality": quality,
            "epoch": r.evidence_epoch,
        }
        payload = r.payload
        match payload:
            case LidarPayload():
                row.update(
                    {
                        "inspected_mm": sum(i.end_mm - i.start_mm for i in payload.inspected),
                        "missing_mm": sum(i.end_mm - i.start_mm for i in payload.missing),
                        "clearance_m": None
                        if payload.clearance_m is None
                        else round(payload.clearance_m, 3),
                        "clearance_uncertainty_m": round(payload.uncertainty_m, 3),
                        "resolution_mm": payload.resolution_mm,
                    }
                )
            case TdrPayload():
                row.update(
                    {
                        "echoes": len(payload.echoes),
                        "coupling": payload.coupling_quality,
                        "usable_range_m": round(payload.usable_range_m, 1),
                        "path_truncated": payload.path_truncated,
                    }
                )
            case CurrentPayload():
                conductor = payload.conductor
                row.update(
                    {
                        "rms_a": round(payload.rms_a, 3),
                        "uncertainty_a": round(payload.uncertainty_a, 3),
                        "conductor": f"{conductor.circuit_id}/{conductor.phase}"
                        f"@{conductor.point.offset_mm / 1e6:.3f}km",
                        "integration_s": payload.integration_s,
                    }
                )
        rows.append(row)
    return rows


def rejection_rows(world: World) -> list[dict]:
    return [
        {"time": _t(r.clock_s), "by": r.author, "reason": reason}
        for r in reversed(world.rejections)
        for reason in r.reasons
    ]


def known_switching_rows(world: World) -> list[dict]:
    return [
        {"segment": seg, "reported_at": _t(t)} for seg, t in sorted(world.known_switching.items())
    ]


def endpoint_rows(world: World) -> list[dict]:
    """Per-circuit landing eligibility with its reasons, expiry and occupant."""
    from grid_ops import knowledge
    from grid_ops.geometry import LandingSite, LinePoint

    occupant = {}
    for d in world.drones.values():
        match d.state:
            case Perched(perch) | Stranded(perch):
                occupant[perch.circuit_id] = f"{d.id} ({d.phase})"
    rows = []
    for cid, ch in world.knowledge.chargeability.items():
        ck = world.knowledge.circuits[cid]
        if ch.state == "unknown" and ck.energization == "unknown" and cid not in occupant:
            continue
        site = LandingSite(LinePoint(cid.split("/")[0], 0), cid)
        verdict = knowledge.assess_landing(world, site, world.clock_s)
        rows.append(
            {
                "circuit": cid,
                "energization": ck.energization,
                "chargeability": ch.state,
                "expires": _t(ch.expires_s) if ch.expires_s is not None else "",
                "eligible_now": verdict.eligible,
                "why": "; ".join(verdict.reasons),
                "conflict": ", ".join(ck.discrepancy_ids),
                "occupant": occupant.get(cid, ""),
                "evidence": ", ".join(ch.evidence_ids),
            }
        )
    return rows


def coverage_rows(world: World) -> list[dict]:
    """What each lidar reading inspected and missed over its requested target span."""
    return [
        {
            "reading": c.reading_id,
            "segment": c.segment_id,
            "target_km": f"{c.target.start_mm / 1e6:.3f}-{c.target.end_mm / 1e6:.3f}",
            "inspected_km": round(sum(i.end_mm - i.start_mm for i in c.inspected) / 1e6, 3),
            "missing_km": round(sum(i.end_mm - i.start_mm for i in c.missing) / 1e6, 3),
            "epoch": c.epoch,
        }
        for c in world.coverage
    ]


def assessment_rows(world: World) -> list[dict]:
    from grid_ops import assessment

    return [
        {
            "scope": f"{a.scope.segment_id} {a.scope.start_mm / 1e6:.3f}-"
            f"{a.scope.end_mm / 1e6:.3f} km",
            "condition": a.condition,
            "verdict": a.verdict,
            "evidence": ", ".join(a.evidence_ids),
            "assumptions": "; ".join(a.assumptions),
            "explanation": a.explanation,
        }
        for a in assessment.segment_assessments(world)
    ]


def hazard_rows(world: World) -> list[dict]:
    return [
        {
            "hazard": h.id,
            "segment": h.segment_id,
            "interval_km": f"{h.start_mm / 1e6:.3f}-{h.end_mm / 1e6:.3f}",
            "verdict": h.verdict,
            "evidence": ", ".join(h.evidence_ids),
            "note": "; ".join(h.assumptions),
        }
        for h in world.hazards
    ]


def disposition_rows(world: World) -> list[dict]:
    return [
        {
            "time": _t(d.clock_s),
            "reading": d.reading_id,
            "action": d.action,
            "epoch": f"{d.from_epoch}->{d.to_epoch}" if d.to_epoch else "",
            "reason": d.reason,
            "by": d.author,
        }
        for d in world.dispositions
    ]


def proposal_rows(world: World) -> list[dict]:
    return [
        {
            "finding": p.id,
            "author": p.author,
            "snapshot": f"r{p.snapshot_revision}",
            "status": p.status,
            "revisions": len(p.output.revisions),
            "requests": len(getattr(p.output, "measurement_requests", ())),
            "note": p.history[-1][1][:160] if p.history else "",
        }
        for p in world.proposals
    ]


def revision_rows(world: World) -> list[dict]:
    rows = []
    for p in world.proposals:
        for r in p.output.revisions:
            rows.append(
                {
                    "finding": p.id,
                    "status": p.status,
                    "hypothesis": r.hypothesis_id,
                    "change": ", ".join(r.change_kinds),
                    "assessment": f"{r.before.assessment if r.before else '-'} -> "
                    f"{r.after.assessment}",
                    "urgency": f"{r.before.priority if r.before else '-'} -> {r.after.priority}",
                    "scope": "; ".join(
                        f"{s.segment_id} {s.start_km:.2f}-{s.end_km:.2f} km"
                        for s in r.after.scope[:3]
                    ),
                    "finding_summary": r.finding_summary,
                    "update_reason": r.update_reason,
                    "remaining_uncertainty": r.remaining_uncertainty,
                    "trigger": f"{r.trigger_kind} "
                    f"{', '.join(r.trigger_event_ids) or r.operator_event_id or ''}",
                    "evidence": ", ".join(r.contributing_evidence_ids),
                }
            )
    return rows


def recommendation_rows(world: World) -> list[dict]:
    return [
        {
            "finding": p.id,
            "status": p.status,
            "id": rec.id,
            "action": rec.action_kind,
            "urgency": rec.urgency,
            "scope": ""
            if rec.scope is None
            else f"{rec.scope.segment_id} {rec.scope.start_km:.3f}-{rec.scope.end_km:.3f} km",
            "text": rec.text,
            "rationale": rec.rationale,
            "uncertainty": rec.uncertainty,
            "evidence": ", ".join(rec.evidence_ids),
        }
        for p in world.proposals
        for rec in p.output.recommendations
    ]


def hypothesis_rows(world: World) -> list[dict]:
    return [
        {
            "hypothesis": hid,
            "assessment": h["assessment"],
            "urgency": h["priority"],
            "lifecycle": h["lifecycle"],
            "claim": h["claim"],
            "model_change": ", ".join(h["model_change"] or []),
        }
        for hid, h in sorted(world.hypotheses.items())
    ]


def accepted_revision_rows(world: World) -> list[dict]:
    return [
        {
            "revision": r.id,
            "decision": r.decision_id,
            "hypothesis": r.hypothesis_id,
            "change": ", ".join(r.change_kinds),
            "finding": r.finding_summary,
            "reason": r.update_reason,
            "uncertainty": r.remaining_uncertainty,
            "at": _t(r.clock_s),
        }
        for r in world.hypothesis_revisions
    ]


def decision_rows(world: World) -> list[dict]:
    return [
        {
            "decision": o.decision_id,
            "outcome": o.outcome,
            "cycle": str(o.cycle_id) if o.cycle_id is not None else "no hold",
            "fleet": o.fallback,
            "reasons": "; ".join(o.reasons)[:200],
            "at": _t(o.clock_s),
        }
        for o in world.decision_outcomes.values()
    ]


def hold_text(world: World) -> str:
    h = world.hold
    if h is None:
        return "No acceptance hold."
    return (
        f"HOLD cycle {h.cycle_id} since {_t(h.since_s)} ({h.decision_id}): {h.reason}. "
        "Simulated time is frozen until a safe continuation is accepted."
    )


def notice_rows(world: World) -> list[dict]:
    """Pending findings shown beside raw observations, whatever route acceptance did."""
    return [
        {
            "finding": p.id,
            "status": p.status,
            "summary": p.output.summary,
            "history": " | ".join(f"{_t(t)} {n}" for t, n in p.history)[:300],
        }
        for p in world.proposals
        if p.status in ("PROPOSED_NOT_ACCEPTED", "SUPERSEDED", "REJECTED")
    ]


def unavailable_rows(world: World) -> list[dict]:
    return [{"request": u.request_id, "why_no_work": u.reason} for u in world.unavailable_work]


def task_log_rows(world: World) -> list[dict]:
    return [{"time": _t(t), "change": n} for t, n in world.task_log[-30:]]


def score_rows(world: World) -> list[dict]:
    return [
        {
            "task": r["task"],
            "eligible": r["owner_urgency"] is not None,
            "owner_urgency": r["owner_urgency"],
            "points_by_urgency_5_to_1": " / ".join(str(v) for v in r["tier_points_5_to_1"]),
            "notes": r["notes"],
        }
        for r in world.score_preview
    ]


SCORE_ASSUMPTIONS = (
    "Synthetic units, compared lexicographically: urgency tiers 5 to 1, then risk, switching, "
    "airtime, charge. Lidar coverage = 1 point per metre of undecided scope on a fixed 10 m "
    "grid; contact-current and TDR requests = 500 per alternative group, earned once by any "
    "completed valid attempt. Shared evidence is counted once and owned by the highest accepted "
    "urgency. Not a calibrated causal entropy; a preview, not an authorisation."
)


def plan_candidate_rows(result) -> list[dict]:
    return [
        {
            "rank": i + 1,
            "source": c.source,
            "tiers_5_to_1": c.score.tiers,
            "risk": c.score.risk,
            "switching": c.score.switching,
            "airtime_min": round(c.score.airtime_s / 60, 1),
            "charge_min": round(c.score.charge_s / 60, 1),
            "sorties": "; ".join(
                f"{r.drone_id}: {' > '.join(_op_label(s.operation) for s in r.steps) or 'stay'} "
                f"-> {r.final_perch.circuit_id}"
                for r in c.sorties
            )
            or "(no flights)",
        }
        for i, c in enumerate(result.ranked)
    ]


def plan_stage_rows(result) -> list[dict]:
    return [
        {
            "round": s.round,
            "stage": s.name,
            "status": s.status,
            "value": s.objective,
            "bound": s.bound,
            "wall_ms": round(s.wall_s * 1000, 1),
        }
        for s in result.stages
    ]


def plan_unassigned_rows(result) -> list[dict]:
    return [{"task": u.task_id, "why": u.reason} for u in (*result.unassigned, *result.omitted)]


def route_pass_rows(world: World) -> list[dict]:
    return [
        {
            "pass": p.id,
            "at": _t(p.sim_clock_s),
            "trigger": p.trigger,
            "events": ", ".join(p.event_ids),
            "outcome": p.outcome,
            "detail": p.detail,
            "solver": p.solver,
            "latency_ms": round(p.latency_s * 1000),
            "planner_ms": round(p.planner_s * 1000),
        }
        for p in world.route_passes[-30:]
    ]


def coordinator_turn_rows(world: World) -> list[dict]:
    return [
        {
            "turn": t.id,
            "status": t.status,
            "pending_reasoning": t.status == "running",
            "triggers": ", ".join(t.trigger_ids) or "-",
            "snapshot": f"r{t.snapshot_revision}",
            "finding": t.finding_id or "",
            "note": t.note,
            "wall_s": t.wall_s,
        }
        for t in world.coordinator_turns
    ]


def planner_preview_rows(world: World) -> list[dict]:
    return [
        {
            "preview": p.id,
            "snapshot": f"r{p.snapshot_revision}",
            "summary": p.summary,
            "blocked": p.blocked or "",
            "unassigned": len(p.unassigned),
            "authorises_dispatch": p.authorises_dispatch,
        }
        for p in world.planner_previews[-10:]
    ]


def investigation_lines(world: World) -> list[str]:
    from grid_ops import conclusion

    inv = world.investigation
    lines = [f"Incident investigation: {inv.status}"]
    if inv.conclusion and "causal_statement" in inv.conclusion:
        lines.append(f"Concluded: {inv.conclusion['causal_statement']}")
        lines.append(f"Remaining uncertainty: {inv.conclusion['remaining_uncertainty']}")
    if inv.unresolved:
        lines.append(f"UNRESOLVED: {inv.unresolved}")
    for f in inv.reopen_flags:
        lines.append(f"Relevant new evidence since conclusion: {f}")
    rep = conclusion.investigation_report(world)
    lines.append(f"Operating assessment: {rep['operating_assessment']}")
    lines.append(f"Local investigation: {rep['local_investigation']}")
    if isinstance(rep["benchmark"], list):
        for name, met, detail in rep["benchmark"]:
            lines.append(f"Benchmark check {name}: {'MET' if met else 'UNMET'} - {detail}")
    if "deadline" in rep:
        d = rep["deadline"]
        lines.append(
            f"Deadline T+04:00: {d['outcome']}; remaining work: {len(d['remaining_work'])} "
            f"attempt(s); unmet checks: {', '.join(d['unmet_checks']) or 'none'}"
        )
    for t, n in inv.history:
        lines.append(f"{_t(t)} {n}")
    return lines


def resilience_lines(world) -> list[str]:
    """Measured response latency, fallback use and the action required when no safe continuation
    exists. Read-only: derived from recorded route passes."""
    from grid_ops import resilience

    rep = resilience.latency_report(world.route_passes)
    lines = [f"Latency: {rep.summary()}"]
    if rep.n:
        lines.append(
            "Mean stage time: "
            + ", ".join(f"{k.replace('_', ' ')} {v:.3f}s" for k, v in rep.stage_means_s.items())
        )
    for pid, why in rep.fallbacks:
        lines.append(f"Fallback used in {pid}: {why}")
    for pid in rep.budget_misses:
        lines.append(f"BUDGET MISS: {pid} exceeded the response budget")
    actions = resilience.operator_actions(world)
    lines += [f"ACTION REQUIRED: {a}" for a in actions] or ["No operator action required."]
    return lines


# -- reconstruction (P3) ------------------------------------------------------------


def fact_rows(world: World) -> list[dict]:
    """Supported facts by kind. Observed current, inferred service state, fault geometry and
    charge availability are separate records, each with its evidence."""
    belief = world.outage_belief
    if belief is None:
        return []
    return [
        {
            "kind": f.kind,
            "subject": f.subject,
            "statement": f.statement,
            "probability": round(f.probability, 3),
            "status": "SUPPORTED" if f.supported else "not supported",
            "interval": ""
            if f.interval is None
            else f"{f.interval.segment_id} {f.interval.start_mm}-{f.interval.end_mm} mm",
            "evidence": ", ".join(f.evidence_ids),
        }
        for f in belief.supported_facts
    ]


def alternative_rows(world: World) -> list[dict]:
    belief = world.outage_belief
    if belief is None:
        return []
    return [
        {
            "alternatives not yet separated": a.label,
            "candidates": len(a.candidate_ids),
            "posterior mass": round(a.mass, 3),
            "methods that could separate them": ", ".join(a.distinguishing_methods),
        }
        for a in belief.unresolved_classes
    ]


def region_rows(world: World) -> list[dict]:
    belief = world.outage_belief
    if belief is None:
        return []
    return [
        {
            "segment": r.segment_id,
            "fault mass": round(r.fault_mass, 3),
            "live-unloaded mass": round(r.unloaded_mass, 3),
            "current observed (live there)": r.observed_energized,
            "lidar inspected": f"{round(100 * r.lidar_covered_fraction)}%",
        }
        for r in sorted(belief.regions, key=lambda r: (-r.fault_mass, r.segment_id))
    ]


def experiment_rows(world: World) -> list[dict]:
    """Ranked candidate measurements and the value the optimizer was given for open tasks."""
    rows = [
        {
            "ranked candidate": f"{p.method} {p.segment_id}"
            + (f" {p.circuit_id}" if p.circuit_id else "")
            + f" {p.start_mm}"
            + (f"-{p.end_mm}" if p.end_mm != p.start_mm else "")
            + " mm",
            "expected discrimination (bits)": p.eig_bits,
            "joint pick": p.batch_order or "",
        }
        for p in world.experiment_proposals
    ]
    rows += [
        {
            "ranked candidate": f"open task {tid}",
            "expected discrimination (bits)": round(bits, 4),
            "joint pick": "",
        }
        for tid, bits in sorted(world.experiment_values.items())
    ]
    return rows


def reconstruction_lines(world: World) -> list[str]:
    belief, stop = world.outage_belief, world.outage_stop
    if belief is None or stop is None:
        return ["No reconstruction is available in this snapshot."]
    checks = belief.model_checks
    lines = [
        f"Reconstruction ({belief.model_version}): {str(stop.reason).upper()} - {stop.detail}",
        f"Evidence used: {len(belief.evidence_ids)} item(s); broad residual mass "
        f"{checks.residual_mass:.2f}; model mismatch: {'YES' if checks.mismatch else 'no'}.",
    ]
    if belief.truncated:
        lines.append("The candidate pool is TRUNCATED: no claim is unconditional.")
    lines += [f"Model limit: {x}" for x in belief.limits]
    lines += [f"Check: {n}" for n in checks.notes]
    timing = world.outage_timing
    if timing:
        lines.append(
            "Update cost: belief {:.0f} ms, request value {:.0f} ms".format(
                1000 * timing.get("belief_update_s", 0.0), 1000 * timing.get("request_value_s", 0.0)
            )
        )
    if world.experiment_omitted:
        lines.append(
            f"Value omitted under the declared cap for: {', '.join(world.experiment_omitted)}"
        )
    return lines


def reconstruction_status(world: World) -> dict:
    """Map labels from the reconstruction: observed live current, suspected and supported
    fault segments. Zero current never produces a de-energized label."""
    from grid_ops.world import LineStatus

    belief = world.outage_belief
    if belief is None:
        return {}
    out = {}
    supported = {
        f.subject for f in belief.supported_facts if f.kind == "fault_interval" and f.supported
    }
    for r in belief.regions:
        if r.segment_id in supported:
            out[r.segment_id] = LineStatus.FAULT_CONFIRMED
        elif r.fault_mass >= 0.2:
            out[r.segment_id] = LineStatus.SUSPECTED
        elif r.observed_energized:
            out[r.segment_id] = LineStatus.LIVE
    return out
